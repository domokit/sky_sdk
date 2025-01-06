// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/vulkan/context_vk.h"
#include <thread>
#include <unordered_map>

#include "fml/concurrent_message_loop.h"
#include "impeller/core/formats.h"
#include "impeller/core/runtime_types.h"
#include "impeller/renderer/backend/vulkan/command_queue_vk.h"
#include "impeller/renderer/backend/vulkan/descriptor_pool_vk.h"
#include "impeller/renderer/backend/vulkan/render_pass_builder_vk.h"
#include "impeller/renderer/backend/vulkan/workarounds_vk.h"
#include "impeller/renderer/render_target.h"

#ifdef FML_OS_ANDROID
#include <pthread.h>
#include <sys/resource.h>
#include <sys/time.h>
#endif  // FML_OS_ANDROID

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "flutter/fml/cpu_affinity.h"
#include "flutter/fml/trace_event.h"
#include "impeller/base/validation.h"
#include "impeller/renderer/backend/vulkan/allocator_vk.h"
#include "impeller/renderer/backend/vulkan/capabilities_vk.h"
#include "impeller/renderer/backend/vulkan/command_buffer_vk.h"
#include "impeller/renderer/backend/vulkan/command_pool_vk.h"
#include "impeller/renderer/backend/vulkan/command_queue_vk.h"
#include "impeller/renderer/backend/vulkan/debug_report_vk.h"
#include "impeller/renderer/backend/vulkan/descriptor_pool_vk.h"
#include "impeller/renderer/backend/vulkan/fence_waiter_vk.h"
#include "impeller/renderer/backend/vulkan/gpu_tracer_vk.h"
#include "impeller/renderer/backend/vulkan/resource_manager_vk.h"
#include "impeller/renderer/backend/vulkan/surface_context_vk.h"
#include "impeller/renderer/backend/vulkan/yuv_conversion_library_vk.h"
#include "impeller/renderer/capabilities.h"

VULKAN_HPP_DEFAULT_DISPATCH_LOADER_DYNAMIC_STORAGE

namespace impeller {

static bool gHasValidationLayers = false;

bool HasValidationLayers() {
  return gHasValidationLayers;
}

static std::optional<vk::PhysicalDevice> PickPhysicalDevice(
    const CapabilitiesVK& caps,
    const vk::Instance& instance) {
  for (const auto& device : instance.enumeratePhysicalDevices().value) {
    if (caps.GetEnabledDeviceFeatures(device).has_value()) {
      return device;
    }
  }
  return std::nullopt;
}

static std::vector<vk::DeviceQueueCreateInfo> GetQueueCreateInfos(
    std::initializer_list<QueueIndexVK> queues) {
  std::map<size_t /* family */, size_t /* index */> family_index_map;
  for (const auto& queue : queues) {
    family_index_map[queue.family] = 0;
  }
  for (const auto& queue : queues) {
    auto value = family_index_map[queue.family];
    family_index_map[queue.family] = std::max(value, queue.index);
  }

  static float kQueuePriority = 1.0f;
  std::vector<vk::DeviceQueueCreateInfo> infos;
  for (const auto& item : family_index_map) {
    vk::DeviceQueueCreateInfo info;
    info.setQueueFamilyIndex(item.first);
    info.setQueueCount(item.second + 1);
    info.setQueuePriorities(kQueuePriority);
    infos.push_back(info);
  }
  return infos;
}

static std::optional<QueueIndexVK> PickQueue(const vk::PhysicalDevice& device,
                                             vk::QueueFlagBits flags) {
  // This can be modified to ensure that dedicated queues are returned for each
  // queue type depending on support.
  const auto families = device.getQueueFamilyProperties();
  for (size_t i = 0u; i < families.size(); i++) {
    if (!(families[i].queueFlags & flags)) {
      continue;
    }
    return QueueIndexVK{.family = i, .index = 0};
  }
  return std::nullopt;
}

std::shared_ptr<ContextVK> ContextVK::Create(Settings settings) {
  auto context = std::shared_ptr<ContextVK>(new ContextVK());
  context->Setup(std::move(settings));
  if (!context->IsValid()) {
    return nullptr;
  }
  return context;
}

// static
size_t ContextVK::ChooseThreadCountForWorkers(size_t hardware_concurrency) {
  // Never create more than 4 worker threads. Attempt to use up to
  // half of the available concurrency.
  return std::clamp(hardware_concurrency / 2ull, /*lo=*/1ull, /*hi=*/4ull);
}

namespace {
thread_local uint64_t tls_context_count = 0;
uint64_t CalculateHash(void* ptr) {
  // You could make a context once per nanosecond for 584 years on one thread
  // before this overflows.
  return ++tls_context_count;
}
}  // namespace

ContextVK::ContextVK() : hash_(CalculateHash(this)) {}

ContextVK::~ContextVK() {
  if (device_holder_ && device_holder_->device) {
    [[maybe_unused]] auto result = device_holder_->device->waitIdle();
  }
  CommandPoolRecyclerVK::DestroyThreadLocalPools(this);
}

Context::BackendType ContextVK::GetBackendType() const {
  return Context::BackendType::kVulkan;
}

void ContextVK::Setup(Settings settings) {
  TRACE_EVENT0("impeller", "ContextVK::Setup");

  if (!settings.proc_address_callback) {
    return;
  }

  raster_message_loop_ = fml::ConcurrentMessageLoop::Create(
      ChooseThreadCountForWorkers(std::thread::hardware_concurrency()));
  raster_message_loop_->PostTaskToAllWorkers([]() {
    // Currently we only use the worker task pool for small parts of a frame
    // workload, if this changes this setting may need to be adjusted.
    fml::RequestAffinity(fml::CpuAffinity::kNotPerformance);
#ifdef FML_OS_ANDROID
    if (::setpriority(PRIO_PROCESS, gettid(), -5) != 0) {
      FML_LOG(ERROR) << "Failed to set Workers task runner priority";
    }
#endif  // FML_OS_ANDROID
  });

  auto& dispatcher = VULKAN_HPP_DEFAULT_DISPATCHER;
  dispatcher.init(settings.proc_address_callback);

  std::vector<std::string> embedder_instance_extensions;
  std::vector<std::string> embedder_device_extensions;
  if (settings.embedder_data.has_value()) {
    embedder_instance_extensions = settings.embedder_data->instance_extensions;
    embedder_device_extensions = settings.embedder_data->device_extensions;
  }
  auto caps = std::shared_ptr<CapabilitiesVK>(new CapabilitiesVK(
      settings.enable_validation,                                      //
      settings.fatal_missing_validations,                              //
      /*use_embedder_extensions=*/settings.embedder_data.has_value(),  //
      embedder_instance_extensions,                                    //
      embedder_device_extensions                                       //
      ));

  if (!caps->IsValid()) {
    VALIDATION_LOG << "Could not determine device capabilities.";
    return;
  }

  gHasValidationLayers = caps->AreValidationsEnabled();

  auto enabled_layers = caps->GetEnabledLayers();
  auto enabled_extensions = caps->GetEnabledInstanceExtensions();

  if (!enabled_layers.has_value() || !enabled_extensions.has_value()) {
    VALIDATION_LOG << "Device has insufficient capabilities.";
    return;
  }

  vk::InstanceCreateFlags instance_flags = {};

  if (std::find(enabled_extensions.value().begin(),
                enabled_extensions.value().end(),
                "VK_KHR_portability_enumeration") !=
      enabled_extensions.value().end()) {
    instance_flags |= vk::InstanceCreateFlagBits::eEnumeratePortabilityKHR;
  }

  std::vector<const char*> enabled_layers_c;
  std::vector<const char*> enabled_extensions_c;

  for (const auto& layer : enabled_layers.value()) {
    enabled_layers_c.push_back(layer.c_str());
  }

  for (const auto& ext : enabled_extensions.value()) {
    enabled_extensions_c.push_back(ext.c_str());
  }

  vk::ApplicationInfo application_info;
  application_info.setApplicationVersion(VK_API_VERSION_1_0);
  application_info.setApiVersion(VK_API_VERSION_1_1);
  application_info.setEngineVersion(VK_API_VERSION_1_0);
  application_info.setPEngineName("Impeller");
  application_info.setPApplicationName("Impeller");

  vk::StructureChain<vk::InstanceCreateInfo, vk::ValidationFeaturesEXT>
      instance_chain;

  if (!caps->AreValidationsEnabled()) {
    instance_chain.unlink<vk::ValidationFeaturesEXT>();
  }

  std::vector<vk::ValidationFeatureEnableEXT> enabled_validations = {
      vk::ValidationFeatureEnableEXT::eSynchronizationValidation,
  };

  auto validation = instance_chain.get<vk::ValidationFeaturesEXT>();
  validation.setEnabledValidationFeatures(enabled_validations);

  auto instance_info = instance_chain.get<vk::InstanceCreateInfo>();
  instance_info.setPEnabledLayerNames(enabled_layers_c);
  instance_info.setPEnabledExtensionNames(enabled_extensions_c);
  instance_info.setPApplicationInfo(&application_info);
  instance_info.setFlags(instance_flags);

  auto device_holder = std::make_shared<DeviceHolderImpl>();
  if (!settings.embedder_data.has_value()) {
    auto instance = vk::createInstanceUnique(instance_info);
    if (instance.result != vk::Result::eSuccess) {
      VALIDATION_LOG << "Could not create Vulkan instance: "
                     << vk::to_string(instance.result);
      return;
    }
    device_holder->instance = std::move(instance.value);
  } else {
    device_holder->instance.reset(settings.embedder_data->instance);
    device_holder->owned = false;
  }
  dispatcher.init(device_holder->instance.get());

  //----------------------------------------------------------------------------
  /// Setup the debug report.
  ///
  /// Do this as early as possible since we could use the debug report from
  /// initialization issues.
  ///
  auto debug_report =
      std::make_unique<DebugReportVK>(*caps, device_holder->instance.get());

  if (!debug_report->IsValid()) {
    VALIDATION_LOG << "Could not set up debug report.";
    return;
  }

  //----------------------------------------------------------------------------
  /// Pick the physical device.
  ///
  if (!settings.embedder_data.has_value()) {
    auto physical_device =
        PickPhysicalDevice(*caps, device_holder->instance.get());
    if (!physical_device.has_value()) {
      VALIDATION_LOG << "No valid Vulkan device found.";
      return;
    }
    device_holder->physical_device = physical_device.value();
  } else {
    device_holder->physical_device = settings.embedder_data->physical_device;
  }

  //----------------------------------------------------------------------------
  /// Pick device queues.
  ///
  auto graphics_queue =
      PickQueue(device_holder->physical_device, vk::QueueFlagBits::eGraphics);
  auto transfer_queue =
      PickQueue(device_holder->physical_device, vk::QueueFlagBits::eTransfer);
  auto compute_queue =
      PickQueue(device_holder->physical_device, vk::QueueFlagBits::eCompute);

  if (!graphics_queue.has_value()) {
    VALIDATION_LOG << "Could not pick graphics queue.";
    return;
  }
  if (!transfer_queue.has_value()) {
    transfer_queue = graphics_queue.value();
  }
  if (!compute_queue.has_value()) {
    VALIDATION_LOG << "Could not pick compute queue.";
    return;
  }

  //----------------------------------------------------------------------------
  /// Create the logical device.
  ///
  auto enabled_device_extensions =
      caps->GetEnabledDeviceExtensions(device_holder->physical_device);
  if (!enabled_device_extensions.has_value()) {
    // This shouldn't happen since we already did device selection. But
    // doesn't hurt to check again.
    return;
  }

  std::vector<const char*> enabled_device_extensions_c;
  for (const auto& ext : enabled_device_extensions.value()) {
    enabled_device_extensions_c.push_back(ext.c_str());
  }

  const auto queue_create_infos = GetQueueCreateInfos(
      {graphics_queue.value(), compute_queue.value(), transfer_queue.value()});

  const auto enabled_features =
      caps->GetEnabledDeviceFeatures(device_holder->physical_device);
  if (!enabled_features.has_value()) {
    // This shouldn't happen since the device can't be picked if this was not
    // true. But doesn't hurt to check.
    return;
  }

  vk::DeviceCreateInfo device_info;

  device_info.setPNext(&enabled_features.value().get());
  device_info.setQueueCreateInfos(queue_create_infos);
  device_info.setPEnabledExtensionNames(enabled_device_extensions_c);
  // Device layers are deprecated and ignored.

  if (!settings.embedder_data.has_value()) {
    auto device_result =
        device_holder->physical_device.createDeviceUnique(device_info);
    if (device_result.result != vk::Result::eSuccess) {
      VALIDATION_LOG << "Could not create logical device.";
      return;
    }
    device_holder->device = std::move(device_result.value);
  } else {
    device_holder->device.reset(settings.embedder_data->device);
  }

  if (!caps->SetPhysicalDevice(device_holder->physical_device,
                               *enabled_features)) {
    VALIDATION_LOG << "Capabilities could not be updated.";
    return;
  }

  //----------------------------------------------------------------------------
  /// Create the allocator.
  ///
  auto allocator = std::shared_ptr<AllocatorVK>(new AllocatorVK(
      weak_from_this(),                //
      application_info.apiVersion,     //
      device_holder->physical_device,  //
      device_holder,                   //
      device_holder->instance.get(),   //
      *caps                            //
      ));

  if (!allocator->IsValid()) {
    VALIDATION_LOG << "Could not create memory allocator.";
    return;
  }

  //----------------------------------------------------------------------------
  /// Setup the pipeline library.
  ///
  auto pipeline_library = std::shared_ptr<PipelineLibraryVK>(
      new PipelineLibraryVK(device_holder,                         //
                            caps,                                  //
                            std::move(settings.cache_directory),   //
                            raster_message_loop_->GetTaskRunner()  //
                            ));

  if (!pipeline_library->IsValid()) {
    VALIDATION_LOG << "Could not create pipeline library.";
    return;
  }

  auto sampler_library =
      std::shared_ptr<SamplerLibraryVK>(new SamplerLibraryVK(device_holder));

  auto shader_library = std::shared_ptr<ShaderLibraryVK>(
      new ShaderLibraryVK(device_holder,                   //
                          settings.shader_libraries_data)  //
  );

  if (!shader_library->IsValid()) {
    VALIDATION_LOG << "Could not create shader library.";
    return;
  }

  //----------------------------------------------------------------------------
  /// Create the fence waiter.
  ///
  auto fence_waiter =
      std::shared_ptr<FenceWaiterVK>(new FenceWaiterVK(device_holder));

  //----------------------------------------------------------------------------
  /// Create the resource manager and command pool recycler.
  ///
  auto resource_manager = ResourceManagerVK::Create();
  if (!resource_manager) {
    VALIDATION_LOG << "Could not create resource manager.";
    return;
  }

  auto command_pool_recycler =
      std::make_shared<CommandPoolRecyclerVK>(weak_from_this());
  if (!command_pool_recycler) {
    VALIDATION_LOG << "Could not create command pool recycler.";
    return;
  }

  auto descriptor_pool_recycler =
      std::make_shared<DescriptorPoolRecyclerVK>(weak_from_this());
  if (!descriptor_pool_recycler) {
    VALIDATION_LOG << "Could not create descriptor pool recycler.";
    return;
  }

  //----------------------------------------------------------------------------
  /// Fetch the queues.
  ///
  QueuesVK queues;
  if (!settings.embedder_data.has_value()) {
    queues = QueuesVK::FromQueueIndices(device_holder->device.get(),  //
                                        graphics_queue.value(),       //
                                        compute_queue.value(),        //
                                        transfer_queue.value()        //
    );
  } else {
    queues =
        QueuesVK::FromEmbedderQueue(settings.embedder_data->queue,
                                    settings.embedder_data->queue_family_index);
  }
  if (!queues.IsValid()) {
    VALIDATION_LOG << "Could not fetch device queues.";
    return;
  }

  VkPhysicalDeviceProperties physical_device_properties;
  dispatcher.vkGetPhysicalDeviceProperties(device_holder->physical_device,
                                           &physical_device_properties);

  //----------------------------------------------------------------------------
  /// All done!
  ///

  // Apply workarounds for broken drivers.
  auto driver_info =
      std::make_unique<DriverInfoVK>(device_holder->physical_device);
  WorkaroundsVK workarounds = GetWorkarounds(*driver_info);
  caps->ApplyWorkarounds(workarounds);

  device_holder_ = std::move(device_holder);
  idle_waiter_vk_ = std::make_shared<IdleWaiterVK>(device_holder_);
  driver_info_ = std::move(driver_info);
  debug_report_ = std::move(debug_report);
  allocator_ = std::move(allocator);
  shader_library_ = std::move(shader_library);
  sampler_library_ = std::move(sampler_library);
  pipeline_library_ = std::move(pipeline_library);
  yuv_conversion_library_ = std::shared_ptr<YUVConversionLibraryVK>(
      new YUVConversionLibraryVK(device_holder_));
  queues_ = std::move(queues);
  device_capabilities_ = std::move(caps);
  fence_waiter_ = std::move(fence_waiter);
  resource_manager_ = std::move(resource_manager);
  command_pool_recycler_ = std::move(command_pool_recycler);
  descriptor_pool_recycler_ = std::move(descriptor_pool_recycler);
  device_name_ = std::string(physical_device_properties.deviceName);
  command_queue_vk_ = std::make_shared<CommandQueueVK>(weak_from_this());
  should_disable_surface_control_ = settings.disable_surface_control;
  should_batch_cmd_buffers_ = !workarounds.batch_submit_command_buffer_timeout;
  is_valid_ = true;

  // Create the GPU Tracer later because it depends on state from
  // the ContextVK.
  gpu_tracer_ = std::make_shared<GPUTracerVK>(weak_from_this(),
                                              settings.enable_gpu_tracing);
  gpu_tracer_->InitializeQueryPool(*this);

  //----------------------------------------------------------------------------
  /// Label all the relevant objects. This happens after setup so that the
  /// debug messengers have had a chance to be set up.
  ///
  SetDebugName(GetDevice(), device_holder_->device.get(), "ImpellerDevice");
}

void ContextVK::SetOffscreenFormat(PixelFormat pixel_format) {
  CapabilitiesVK::Cast(*device_capabilities_).SetOffscreenFormat(pixel_format);
}

// |Context|
std::string ContextVK::DescribeGpuModel() const {
  return device_name_;
}

bool ContextVK::IsValid() const {
  return is_valid_;
}

std::shared_ptr<Allocator> ContextVK::GetResourceAllocator() const {
  return allocator_;
}

std::shared_ptr<ShaderLibrary> ContextVK::GetShaderLibrary() const {
  return shader_library_;
}

std::shared_ptr<SamplerLibrary> ContextVK::GetSamplerLibrary() const {
  return sampler_library_;
}

std::shared_ptr<PipelineLibrary> ContextVK::GetPipelineLibrary() const {
  return pipeline_library_;
}

std::shared_ptr<CommandBuffer> ContextVK::CreateCommandBuffer() const {
  const auto& recycler = GetCommandPoolRecycler();
  auto tls_pool = recycler->Get();
  if (!tls_pool) {
    return nullptr;
  }

  // look up a cached descriptor pool for the current frame and reuse it
  // if it exists, otherwise create a new pool.
  std::shared_ptr<DescriptorPoolVK> descriptor_pool;
  {
    Lock lock(desc_pool_mutex_);
    DescriptorPoolMap::iterator current_pool =
        cached_descriptor_pool_.find(std::this_thread::get_id());
    if (current_pool == cached_descriptor_pool_.end()) {
      descriptor_pool =
          (cached_descriptor_pool_[std::this_thread::get_id()] =
               std::make_shared<DescriptorPoolVK>(weak_from_this()));
    } else {
      descriptor_pool = current_pool->second;
    }
  }

  auto tracked_objects = std::make_shared<TrackedObjectsVK>(
      weak_from_this(), std::move(tls_pool), std::move(descriptor_pool),
      GetGPUTracer()->CreateGPUProbe());
  auto queue = GetGraphicsQueue();

  if (!tracked_objects || !tracked_objects->IsValid() || !queue) {
    return nullptr;
  }

  vk::CommandBufferBeginInfo begin_info;
  begin_info.flags = vk::CommandBufferUsageFlagBits::eOneTimeSubmit;
  if (tracked_objects->GetCommandBuffer().begin(begin_info) !=
      vk::Result::eSuccess) {
    VALIDATION_LOG << "Could not begin command buffer.";
    return nullptr;
  }

  tracked_objects->GetGPUProbe().RecordCmdBufferStart(
      tracked_objects->GetCommandBuffer());

  return std::shared_ptr<CommandBufferVK>(new CommandBufferVK(
      shared_from_this(),         //
      GetDeviceHolder(),          //
      std::move(tracked_objects)  //
      ));
}

vk::Instance ContextVK::GetInstance() const {
  return *device_holder_->instance;
}

const vk::Device& ContextVK::GetDevice() const {
  return device_holder_->device.get();
}

const std::shared_ptr<fml::ConcurrentTaskRunner>
ContextVK::GetConcurrentWorkerTaskRunner() const {
  return raster_message_loop_->GetTaskRunner();
}

void ContextVK::Shutdown() {
  // There are multiple objects, for example |CommandPoolVK|, that in their
  // destructors make a strong reference to |ContextVK|. Resetting these shared
  // pointers ensures that cleanup happens in a correct order.
  //
  // tl;dr: Without it, we get thread::join failures on shutdown.
  fence_waiter_.reset();
  resource_manager_.reset();

  raster_message_loop_->Terminate();
}

std::shared_ptr<SurfaceContextVK> ContextVK::CreateSurfaceContext() {
  return std::make_shared<SurfaceContextVK>(shared_from_this());
}

const std::shared_ptr<const Capabilities>& ContextVK::GetCapabilities() const {
  return device_capabilities_;
}

const std::shared_ptr<QueueVK>& ContextVK::GetGraphicsQueue() const {
  return queues_.graphics_queue;
}

vk::PhysicalDevice ContextVK::GetPhysicalDevice() const {
  return device_holder_->physical_device;
}

std::shared_ptr<FenceWaiterVK> ContextVK::GetFenceWaiter() const {
  return fence_waiter_;
}

std::shared_ptr<ResourceManagerVK> ContextVK::GetResourceManager() const {
  return resource_manager_;
}

std::shared_ptr<CommandPoolRecyclerVK> ContextVK::GetCommandPoolRecycler()
    const {
  return command_pool_recycler_;
}

std::shared_ptr<GPUTracerVK> ContextVK::GetGPUTracer() const {
  return gpu_tracer_;
}

std::shared_ptr<DescriptorPoolRecyclerVK> ContextVK::GetDescriptorPoolRecycler()
    const {
  return descriptor_pool_recycler_;
}

std::shared_ptr<CommandQueue> ContextVK::GetCommandQueue() const {
  return command_queue_vk_;
}

bool ContextVK::EnqueueCommandBuffer(
    std::shared_ptr<CommandBuffer> command_buffer) {
  if (should_batch_cmd_buffers_) {
    pending_command_buffers_.push_back(std::move(command_buffer));
    return true;
  } else {
    return GetCommandQueue()->Submit({command_buffer}).ok();
  }
}

bool ContextVK::FlushCommandBuffers() {
  if (should_batch_cmd_buffers_) {
    bool result = GetCommandQueue()->Submit(pending_command_buffers_).ok();
    pending_command_buffers_.clear();
    return result;
  } else {
    return true;
  }
}

// Creating a render pass is observed to take an additional 6ms on a Pixel 7
// device as the driver will lazily bootstrap and compile shaders to do so.
// The render pass does not need to be begun or executed.
void ContextVK::InitializeCommonlyUsedShadersIfNeeded() const {
  RenderTargetAllocator rt_allocator(GetResourceAllocator());
  RenderTarget render_target =
      rt_allocator.CreateOffscreenMSAA(*this, {1, 1}, 1);

  RenderPassBuilderVK builder;

  render_target.IterateAllColorAttachments(
      [&builder](size_t index, const ColorAttachment& attachment) -> bool {
        builder.SetColorAttachment(
            index,                                                    //
            attachment.texture->GetTextureDescriptor().format,        //
            attachment.texture->GetTextureDescriptor().sample_count,  //
            attachment.load_action,                                   //
            attachment.store_action                                   //
        );
        return true;
      });

  if (auto depth = render_target.GetDepthAttachment(); depth.has_value()) {
    builder.SetDepthStencilAttachment(
        depth->texture->GetTextureDescriptor().format,        //
        depth->texture->GetTextureDescriptor().sample_count,  //
        depth->load_action,                                   //
        depth->store_action                                   //
    );
  } else if (auto stencil = render_target.GetStencilAttachment();
             stencil.has_value()) {
    builder.SetStencilAttachment(
        stencil->texture->GetTextureDescriptor().format,        //
        stencil->texture->GetTextureDescriptor().sample_count,  //
        stencil->load_action,                                   //
        stencil->store_action                                   //
    );
  }

  auto pass = builder.Build(GetDevice());
}

void ContextVK::DisposeThreadLocalCachedResources() {
  {
    Lock lock(desc_pool_mutex_);
    cached_descriptor_pool_.erase(std::this_thread::get_id());
  }
  command_pool_recycler_->Dispose();
}

const std::shared_ptr<YUVConversionLibraryVK>&
ContextVK::GetYUVConversionLibrary() const {
  return yuv_conversion_library_;
}

const std::unique_ptr<DriverInfoVK>& ContextVK::GetDriverInfo() const {
  return driver_info_;
}

bool ContextVK::GetShouldDisableSurfaceControlSwapchain() const {
  return should_disable_surface_control_;
}

RuntimeStageBackend ContextVK::GetRuntimeStageBackend() const {
  return RuntimeStageBackend::kVulkan;
}

}  // namespace impeller
