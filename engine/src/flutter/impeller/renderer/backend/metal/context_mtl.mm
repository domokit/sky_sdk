// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/metal/context_mtl.h"
#include <Metal/Metal.h>

#include <memory>

#include "flutter/fml/concurrent_message_loop.h"
#include "flutter/fml/file.h"
#include "flutter/fml/logging.h"
#include "flutter/fml/paths.h"
#include "flutter/fml/synchronization/sync_switch.h"
#include "impeller/core/formats.h"
#include "impeller/core/runtime_types.h"
#include "impeller/core/sampler_descriptor.h"
#include "impeller/renderer/backend/metal/gpu_tracer_mtl.h"
#include "impeller/renderer/backend/metal/sampler_library_mtl.h"
#include "impeller/renderer/capabilities.h"

namespace impeller {

static bool DeviceSupportsFramebufferFetch(id<MTLDevice> device) {
  // The iOS simulator lies about supporting framebuffer fetch.
#if FML_OS_IOS_SIMULATOR
  return false;
#else  // FML_OS_IOS_SIMULATOR

  if (@available(macOS 10.15, iOS 13, tvOS 13, *)) {
    return [device supportsFamily:MTLGPUFamilyApple2];
  }
  // According to
  // https://developer.apple.com/metal/Metal-Feature-Set-Tables.pdf , Apple2
  // corresponds to iOS GPU family 2, which supports A8 devices.
#if FML_OS_IOS
  return [device supportsFeatureSet:MTLFeatureSet_iOS_GPUFamily2_v1];
#else
  return false;
#endif  // FML_OS_IOS
#endif  // FML_OS_IOS_SIMULATOR
}

static bool DeviceSupportsComputeSubgroups(id<MTLDevice> device) {
  bool supports_subgroups = false;
  // Refer to the "SIMD-scoped reduction operations" feature in the table
  // below: https://developer.apple.com/metal/Metal-Feature-Set-Tables.pdf
  if (@available(ios 13.0, tvos 13.0, macos 10.15, *)) {
    supports_subgroups = [device supportsFamily:MTLGPUFamilyApple7] ||
                         [device supportsFamily:MTLGPUFamilyMac2];
  }
  return supports_subgroups;
}

static std::unique_ptr<Capabilities> InferMetalCapabilities(
    id<MTLDevice> device,
    PixelFormat color_format) {
  return CapabilitiesBuilder()
      .SetSupportsOffscreenMSAA(true)
      .SetSupportsSSBO(true)
      .SetSupportsTextureToTextureBlits(true)
      .SetSupportsDecalSamplerAddressMode(true)
      .SetSupportsFramebufferFetch(DeviceSupportsFramebufferFetch(device))
      .SetDefaultColorFormat(color_format)
      .SetDefaultStencilFormat(PixelFormat::kS8UInt)
      .SetDefaultDepthStencilFormat(PixelFormat::kD32FloatS8UInt)
      .SetSupportsCompute(true)
      .SetSupportsComputeSubgroups(DeviceSupportsComputeSubgroups(device))
      .SetSupportsReadFromResolve(true)
      .SetSupportsDeviceTransientTextures(true)
      .SetDefaultGlyphAtlasFormat(PixelFormat::kA8UNormInt)
      .SetSupportsTriangleFan(false)
      .SetMaximumRenderPassAttachmentSize(DeviceMaxTextureSizeSupported(device))
      .Build();
}

ContextMTL::ContextMTL(
    id<MTLDevice> device,
    id<MTLCommandQueue> command_queue,
    NSArray<id<MTLLibrary>>* shader_libraries,
    std::shared_ptr<const fml::SyncSwitch> is_gpu_disabled_sync_switch,
    std::optional<PixelFormat> pixel_format_override)
    : device_(device),
      command_queue_(command_queue),
      is_gpu_disabled_sync_switch_(std::move(is_gpu_disabled_sync_switch)) {
  // Validate device.
  if (!device_) {
    VALIDATION_LOG << "Could not set up valid Metal device.";
    return;
  }

  sync_switch_observer_.reset(new SyncSwitchObserver(*this));
  is_gpu_disabled_sync_switch_->AddObserver(sync_switch_observer_.get());

  // Setup the shader library.
  {
    if (shader_libraries == nil) {
      VALIDATION_LOG << "Shader libraries were null.";
      return;
    }

    // std::make_shared disallowed because of private friend ctor.
    auto library = std::shared_ptr<ShaderLibraryMTL>(
        new ShaderLibraryMTL(shader_libraries));
    if (!library->IsValid()) {
      VALIDATION_LOG << "Could not create valid Metal shader library.";
      return;
    }
    shader_library_ = std::move(library);
  }

  // Setup the pipeline library.
  {
    pipeline_library_ =
        std::shared_ptr<PipelineLibraryMTL>(new PipelineLibraryMTL(device_));
  }

  // Setup the sampler library.
  {
    sampler_library_ =
        std::shared_ptr<SamplerLibraryMTL>(new SamplerLibraryMTL(device_));
  }

  // Setup the resource allocator.
  {
    resource_allocator_ = std::shared_ptr<AllocatorMTL>(
        new AllocatorMTL(device_, "Impeller Permanents Allocator"));
    if (!resource_allocator_) {
      VALIDATION_LOG << "Could not set up the resource allocator.";
      return;
    }
  }

  device_capabilities_ =
      InferMetalCapabilities(device_, pixel_format_override.has_value()
                                          ? pixel_format_override.value()
                                          : PixelFormat::kB8G8R8A8UNormInt);
  command_queue_ip_ = std::make_shared<CommandQueue>();
#ifdef IMPELLER_DEBUG
  gpu_tracer_ = std::make_shared<GPUTracerMTL>();
  capture_manager_ = std::make_shared<ImpellerMetalCaptureManager>(device_);
#endif  // IMPELLER_DEBUG
  is_valid_ = true;
}

static NSArray<id<MTLLibrary>>* MTLShaderLibraryFromFilePaths(
    id<MTLDevice> device,
    const std::vector<std::string>& libraries_paths) {
  NSMutableArray<id<MTLLibrary>>* found_libraries = [NSMutableArray array];
  for (const auto& library_path : libraries_paths) {
    if (!fml::IsFile(library_path)) {
      VALIDATION_LOG << "Shader library does not exist at path '"
                     << library_path << "'";
      return nil;
    }
    NSError* shader_library_error = nil;
    auto library = [device newLibraryWithFile:@(library_path.c_str())
                                        error:&shader_library_error];
    if (!library) {
      FML_LOG(ERROR) << "Could not create shader library: "
                     << shader_library_error.localizedDescription.UTF8String;
      return nil;
    }
    [found_libraries addObject:library];
  }
  return found_libraries;
}

static NSArray<id<MTLLibrary>>* MTLShaderLibraryFromFileData(
    id<MTLDevice> device,
    const std::vector<std::shared_ptr<fml::Mapping>>& libraries_data,
    const std::string& label) {
  NSMutableArray<id<MTLLibrary>>* found_libraries = [NSMutableArray array];
  for (const auto& library_data : libraries_data) {
    if (library_data == nullptr) {
      FML_LOG(ERROR) << "Shader library data was null.";
      return nil;
    }

    __block auto data = library_data;

    auto dispatch_data =
        ::dispatch_data_create(library_data->GetMapping(),  // buffer
                               library_data->GetSize(),     // size
                               dispatch_get_main_queue(),   // queue
                               ^() {
                                 // We just need a reference.
                                 data.reset();
                               }  // destructor
        );
    if (!dispatch_data) {
      FML_LOG(ERROR) << "Could not wrap shader data in dispatch data.";
      return nil;
    }

    NSError* shader_library_error = nil;
    auto library = [device newLibraryWithData:dispatch_data
                                        error:&shader_library_error];
    if (!library) {
      FML_LOG(ERROR) << "Could not create shader library: "
                     << shader_library_error.localizedDescription.UTF8String;
      return nil;
    }
    if (!label.empty()) {
      library.label = @(label.c_str());
    }
    [found_libraries addObject:library];
  }
  return found_libraries;
}

static id<MTLDevice> CreateMetalDevice() {
  return ::MTLCreateSystemDefaultDevice();
}

static id<MTLCommandQueue> CreateMetalCommandQueue(id<MTLDevice> device) {
  auto command_queue = device.newCommandQueue;
  if (!command_queue) {
    VALIDATION_LOG << "Could not set up the command queue.";
    return nullptr;
  }
  command_queue.label = @"Impeller Command Queue";
  return command_queue;
}

std::shared_ptr<ContextMTL> ContextMTL::Create(
    const std::vector<std::string>& shader_library_paths,
    std::shared_ptr<const fml::SyncSwitch> is_gpu_disabled_sync_switch) {
  auto device = CreateMetalDevice();
  auto command_queue = CreateMetalCommandQueue(device);
  if (!command_queue) {
    return nullptr;
  }
  auto context = std::shared_ptr<ContextMTL>(new ContextMTL(
      device, command_queue,
      MTLShaderLibraryFromFilePaths(device, shader_library_paths),
      std::move(is_gpu_disabled_sync_switch)));
  if (!context->IsValid()) {
    FML_LOG(ERROR) << "Could not create Metal context.";
    return nullptr;
  }
  return context;
}

std::shared_ptr<ContextMTL> ContextMTL::Create(
    const std::vector<std::shared_ptr<fml::Mapping>>& shader_libraries_data,
    std::shared_ptr<const fml::SyncSwitch> is_gpu_disabled_sync_switch,
    const std::string& library_label,
    std::optional<PixelFormat> pixel_format_override) {
  auto device = CreateMetalDevice();
  auto command_queue = CreateMetalCommandQueue(device);
  if (!command_queue) {
    return nullptr;
  }
  auto context = std::shared_ptr<ContextMTL>(new ContextMTL(
      device, command_queue,
      MTLShaderLibraryFromFileData(device, shader_libraries_data,
                                   library_label),
      std::move(is_gpu_disabled_sync_switch), pixel_format_override));
  if (!context->IsValid()) {
    FML_LOG(ERROR) << "Could not create Metal context.";
    return nullptr;
  }
  return context;
}

std::shared_ptr<ContextMTL> ContextMTL::Create(
    id<MTLDevice> device,
    id<MTLCommandQueue> command_queue,
    const std::vector<std::shared_ptr<fml::Mapping>>& shader_libraries_data,
    std::shared_ptr<const fml::SyncSwitch> is_gpu_disabled_sync_switch,
    const std::string& library_label) {
  auto context = std::shared_ptr<ContextMTL>(
      new ContextMTL(device, command_queue,
                     MTLShaderLibraryFromFileData(device, shader_libraries_data,
                                                  library_label),
                     std::move(is_gpu_disabled_sync_switch)));
  if (!context->IsValid()) {
    FML_LOG(ERROR) << "Could not create Metal context.";
    return nullptr;
  }
  return context;
}

ContextMTL::~ContextMTL() {
  is_gpu_disabled_sync_switch_->RemoveObserver(sync_switch_observer_.get());
}

Context::BackendType ContextMTL::GetBackendType() const {
  return Context::BackendType::kMetal;
}

// |Context|
std::string ContextMTL::DescribeGpuModel() const {
  return std::string([[device_ name] UTF8String]);
}

// |Context|
bool ContextMTL::IsValid() const {
  return is_valid_;
}

// |Context|
std::shared_ptr<ShaderLibrary> ContextMTL::GetShaderLibrary() const {
  return shader_library_;
}

// |Context|
std::shared_ptr<PipelineLibrary> ContextMTL::GetPipelineLibrary() const {
  return pipeline_library_;
}

// |Context|
std::shared_ptr<SamplerLibrary> ContextMTL::GetSamplerLibrary() const {
  return sampler_library_;
}

// |Context|
std::shared_ptr<CommandBuffer> ContextMTL::CreateCommandBuffer() const {
  return CreateCommandBufferInQueue(command_queue_);
}

// |Context|
void ContextMTL::Shutdown() {}

#ifdef IMPELLER_DEBUG
std::shared_ptr<GPUTracerMTL> ContextMTL::GetGPUTracer() const {
  return gpu_tracer_;
}
#endif  // IMPELLER_DEBUG

std::shared_ptr<const fml::SyncSwitch> ContextMTL::GetIsGpuDisabledSyncSwitch()
    const {
  return is_gpu_disabled_sync_switch_;
}

std::shared_ptr<CommandBuffer> ContextMTL::CreateCommandBufferInQueue(
    id<MTLCommandQueue> queue) const {
  if (!IsValid()) {
    return nullptr;
  }

  auto buffer = std::shared_ptr<CommandBufferMTL>(
      new CommandBufferMTL(weak_from_this(), device_, queue));
  if (!buffer->IsValid()) {
    return nullptr;
  }
  return buffer;
}

std::shared_ptr<Allocator> ContextMTL::GetResourceAllocator() const {
  return resource_allocator_;
}

id<MTLDevice> ContextMTL::GetMTLDevice() const {
  return device_;
}

const std::shared_ptr<const Capabilities>& ContextMTL::GetCapabilities() const {
  return device_capabilities_;
}

void ContextMTL::SetCapabilities(
    const std::shared_ptr<const Capabilities>& capabilities) {
  device_capabilities_ = capabilities;
}

// |Context|
bool ContextMTL::UpdateOffscreenLayerPixelFormat(PixelFormat format) {
  device_capabilities_ = InferMetalCapabilities(device_, format);
  return true;
}

id<MTLCommandBuffer> ContextMTL::CreateMTLCommandBuffer(
    const std::string& label) const {
  auto buffer = [command_queue_ commandBuffer];
  if (!label.empty()) {
    [buffer setLabel:@(label.data())];
  }
  return buffer;
}

void ContextMTL::StoreTaskForGPU(const fml::closure& task,
                                 const fml::closure& failure) {
  std::vector<PendingTasks> failed_tasks;
  {
    Lock lock(tasks_awaiting_gpu_mutex_);
    tasks_awaiting_gpu_.push_back(PendingTasks{task, failure});
    int32_t failed_task_count =
        tasks_awaiting_gpu_.size() - kMaxTasksAwaitingGPU;
    if (failed_task_count > 0) {
      failed_tasks.reserve(failed_task_count);
      failed_tasks.insert(failed_tasks.end(),
                          std::make_move_iterator(tasks_awaiting_gpu_.begin()),
                          std::make_move_iterator(tasks_awaiting_gpu_.begin() +
                                                  failed_task_count));
      tasks_awaiting_gpu_.erase(
          tasks_awaiting_gpu_.begin(),
          tasks_awaiting_gpu_.begin() + failed_task_count);
    }
  }
  for (const PendingTasks& task : failed_tasks) {
    if (task.failure) {
      task.failure();
    }
  }
}

void ContextMTL::FlushTasksAwaitingGPU() {
  std::deque<PendingTasks> tasks_awaiting_gpu;
  {
    Lock lock(tasks_awaiting_gpu_mutex_);
    std::swap(tasks_awaiting_gpu, tasks_awaiting_gpu_);
  }
  for (const auto& task : tasks_awaiting_gpu) {
    task.task();
  }
}

ContextMTL::SyncSwitchObserver::SyncSwitchObserver(ContextMTL& parent)
    : parent_(parent) {}

void ContextMTL::SyncSwitchObserver::OnSyncSwitchUpdate(bool new_is_disabled) {
  if (!new_is_disabled) {
    parent_.FlushTasksAwaitingGPU();
  }
}

// |Context|
std::shared_ptr<CommandQueue> ContextMTL::GetCommandQueue() const {
  return command_queue_ip_;
}

// |Context|
RuntimeStageBackend ContextMTL::GetRuntimeStageBackend() const {
  return RuntimeStageBackend::kMetal;
}

#ifdef IMPELLER_DEBUG
const std::shared_ptr<ImpellerMetalCaptureManager>
ContextMTL::GetCaptureManager() const {
  return capture_manager_;
}
#endif  // IMPELLER_DEBUG

ImpellerMetalCaptureManager::ImpellerMetalCaptureManager(id<MTLDevice> device) {
  current_capture_scope_ = [[MTLCaptureManager sharedCaptureManager]
      newCaptureScopeWithDevice:device];
  [current_capture_scope_ setLabel:@"Impeller Frame"];
}

bool ImpellerMetalCaptureManager::CaptureScopeActive() const {
  return scope_active_;
}

void ImpellerMetalCaptureManager::StartCapture() {
  if (scope_active_) {
    return;
  }
  scope_active_ = true;
  [current_capture_scope_ beginScope];
}

void ImpellerMetalCaptureManager::FinishCapture() {
  FML_DCHECK(scope_active_);
  [current_capture_scope_ endScope];
  scope_active_ = false;
}

}  // namespace impeller
