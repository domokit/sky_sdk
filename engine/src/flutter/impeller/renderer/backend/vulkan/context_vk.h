// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_CONTEXT_VK_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_CONTEXT_VK_H_

#include <memory>

#include "flutter/fml/concurrent_message_loop.h"
#include "flutter/fml/mapping.h"
#include "flutter/fml/unique_fd.h"
#include "impeller/base/backend_cast.h"
#include "impeller/base/strings.h"
#include "impeller/core/formats.h"
#include "impeller/core/runtime_types.h"
#include "impeller/renderer/backend/vulkan/command_pool_vk.h"
#include "impeller/renderer/backend/vulkan/device_holder_vk.h"
#include "impeller/renderer/backend/vulkan/driver_info_vk.h"
#include "impeller/renderer/backend/vulkan/pipeline_library_vk.h"
#include "impeller/renderer/backend/vulkan/queue_vk.h"
#include "impeller/renderer/backend/vulkan/sampler_library_vk.h"
#include "impeller/renderer/backend/vulkan/shader_library_vk.h"
#include "impeller/renderer/capabilities.h"
#include "impeller/renderer/command_buffer.h"
#include "impeller/renderer/command_queue.h"
#include "impeller/renderer/context.h"

namespace impeller {

bool HasValidationLayers();

class CommandEncoderFactoryVK;
class CommandEncoderVK;
class CommandPoolRecyclerVK;
class DebugReportVK;
class FenceWaiterVK;
class ResourceManagerVK;
class SurfaceContextVK;
class GPUTracerVK;
class DescriptorPoolRecyclerVK;
class CommandQueueVK;
class DescriptorPoolVK;

class IdleWaiterVK : public IdleWaiter {
 public:
  explicit IdleWaiterVK(std::weak_ptr<DeviceHolderVK> device_holder)
      : device_holder_(std::move(device_holder)) {}

  void WaitIdle() const override {
    std::shared_ptr<DeviceHolderVK> strong_device_holder_ =
        device_holder_.lock();
    if (strong_device_holder_ && strong_device_holder_->GetDevice()) {
      [[maybe_unused]] auto result =
          strong_device_holder_->GetDevice().waitIdle();
    }
  }

 private:
  std::weak_ptr<DeviceHolderVK> device_holder_;
};

class ContextVK final : public Context,
                        public BackendCast<ContextVK, Context>,
                        public std::enable_shared_from_this<ContextVK> {
 public:
  /// Embedder Stuff
  struct EmbedderData {
    VkInstance instance;
    VkPhysicalDevice physical_device;
    VkDevice device;
    uint32_t queue_family_index;
    VkQueue queue;
    std::vector<std::string> instance_extensions;
    std::vector<std::string> device_extensions;
  };

  struct Settings {
    PFN_vkGetInstanceProcAddr proc_address_callback = nullptr;
    std::vector<std::shared_ptr<fml::Mapping>> shader_libraries_data;
    fml::UniqueFD cache_directory;
    bool enable_validation = false;
    bool enable_gpu_tracing = false;
    bool disable_surface_control = false;
    /// If validations are requested but cannot be enabled, log a fatal error.
    bool fatal_missing_validations = false;

    std::optional<EmbedderData> embedder_data;

    Settings() = default;

    Settings(Settings&&) = default;
  };

  /// Choose the number of worker threads the context_vk will create.
  ///
  /// Visible for testing.
  static size_t ChooseThreadCountForWorkers(size_t hardware_concurrency);

  static std::shared_ptr<ContextVK> Create(Settings settings);

  uint64_t GetHash() const { return hash_; }

  // |Context|
  ~ContextVK() override;

  // |Context|
  BackendType GetBackendType() const override;

  // |Context|
  std::string DescribeGpuModel() const override;

  // |Context|
  bool IsValid() const override;

  // |Context|
  std::shared_ptr<Allocator> GetResourceAllocator() const override;

  // |Context|
  std::shared_ptr<ShaderLibrary> GetShaderLibrary() const override;

  // |Context|
  std::shared_ptr<SamplerLibrary> GetSamplerLibrary() const override;

  // |Context|
  std::shared_ptr<PipelineLibrary> GetPipelineLibrary() const override;

  // |Context|
  std::shared_ptr<CommandBuffer> CreateCommandBuffer() const override;

  // |Context|
  const std::shared_ptr<const Capabilities>& GetCapabilities() const override;

  const std::shared_ptr<YUVConversionLibraryVK>& GetYUVConversionLibrary()
      const;

  // |Context|
  void Shutdown() override;

  void SetOffscreenFormat(PixelFormat pixel_format);

  template <typename T>
  bool SetDebugName(T handle, std::string_view label) const {
    return SetDebugName(GetDevice(), handle, label);
  }

  template <typename T>
  bool SetDebugName(T handle,
                    std::string_view label,
                    std::string_view trailing) const {
    if (!HasValidationLayers()) {
      // No-op if validation layers are not enabled.
      return true;
    }
    std::string combined = SPrintF("%s %s", label.data(), trailing.data());
    return SetDebugName(GetDevice(), handle, combined);
  }

  template <typename T>
  static bool SetDebugName(const vk::Device& device,
                           T handle,
                           std::string_view label) {
    if (!HasValidationLayers()) {
      // No-op if validation layers are not enabled.
      return true;
    }

    auto c_handle = static_cast<typename T::CType>(handle);

    vk::DebugUtilsObjectNameInfoEXT info;
    info.objectType = T::objectType;
    info.pObjectName = label.data();
    info.objectHandle = reinterpret_cast<decltype(info.objectHandle)>(c_handle);

    if (device.setDebugUtilsObjectNameEXT(info) != vk::Result::eSuccess) {
      VALIDATION_LOG << "Unable to set debug name: " << label;
      return false;
    }

    return true;
  }

  std::shared_ptr<DeviceHolderVK> GetDeviceHolder() const {
    return device_holder_;
  }

  vk::Instance GetInstance() const;

  const vk::Device& GetDevice() const;

  const std::unique_ptr<DriverInfoVK>& GetDriverInfo() const;

  const std::shared_ptr<fml::ConcurrentTaskRunner>
  GetConcurrentWorkerTaskRunner() const;

  std::shared_ptr<SurfaceContextVK> CreateSurfaceContext();

  const std::shared_ptr<QueueVK>& GetGraphicsQueue() const;

  vk::PhysicalDevice GetPhysicalDevice() const;

  std::shared_ptr<FenceWaiterVK> GetFenceWaiter() const;

  std::shared_ptr<ResourceManagerVK> GetResourceManager() const;

  std::shared_ptr<CommandPoolRecyclerVK> GetCommandPoolRecycler() const;

  std::shared_ptr<DescriptorPoolRecyclerVK> GetDescriptorPoolRecycler() const;

  std::shared_ptr<CommandQueue> GetCommandQueue() const override;

  std::shared_ptr<GPUTracerVK> GetGPUTracer() const;

  void RecordFrameEndTime() const;

  // |Context|
  void InitializeCommonlyUsedShadersIfNeeded() const override;

  // |Context|
  void DisposeThreadLocalCachedResources() override;

  /// @brief Whether the Android Surface control based swapchain should be
  /// disabled, even if the device is capable of supporting it.
  bool GetShouldDisableSurfaceControlSwapchain() const;

  // | Context |
  bool EnqueueCommandBuffer(
      std::shared_ptr<CommandBuffer> command_buffer) override;

  // | Context |
  bool FlushCommandBuffers() override;

  RuntimeStageBackend GetRuntimeStageBackend() const override;

  std::shared_ptr<const IdleWaiter> GetIdleWaiter() const override {
    return idle_waiter_vk_;
  }

 private:
  struct DeviceHolderImpl : public DeviceHolderVK {
    // |DeviceHolder|
    const vk::Device& GetDevice() const override { return device.get(); }
    // |DeviceHolder|
    const vk::PhysicalDevice& GetPhysicalDevice() const override {
      return physical_device;
    }

    ~DeviceHolderImpl() {
      if (!owned) {
        instance.release();
        device.release();
      }
    }

    vk::UniqueInstance instance;
    vk::PhysicalDevice physical_device;
    vk::UniqueDevice device;
    bool owned = true;
  };

  std::shared_ptr<DeviceHolderImpl> device_holder_;
  std::unique_ptr<DriverInfoVK> driver_info_;
  std::unique_ptr<DebugReportVK> debug_report_;
  std::shared_ptr<Allocator> allocator_;
  std::shared_ptr<ShaderLibraryVK> shader_library_;
  std::shared_ptr<SamplerLibraryVK> sampler_library_;
  std::shared_ptr<PipelineLibraryVK> pipeline_library_;
  std::shared_ptr<YUVConversionLibraryVK> yuv_conversion_library_;
  QueuesVK queues_;
  std::shared_ptr<const Capabilities> device_capabilities_;
  std::shared_ptr<FenceWaiterVK> fence_waiter_;
  std::shared_ptr<ResourceManagerVK> resource_manager_;
  std::shared_ptr<DescriptorPoolRecyclerVK> descriptor_pool_recycler_;
  std::shared_ptr<CommandPoolRecyclerVK> command_pool_recycler_;
  std::string device_name_;
  std::shared_ptr<fml::ConcurrentMessageLoop> raster_message_loop_;
  std::shared_ptr<GPUTracerVK> gpu_tracer_;
  std::shared_ptr<CommandQueue> command_queue_vk_;
  std::shared_ptr<const IdleWaiter> idle_waiter_vk_;

  using DescriptorPoolMap =
      std::unordered_map<std::thread::id, std::shared_ptr<DescriptorPoolVK>>;

  mutable Mutex desc_pool_mutex_;
  mutable DescriptorPoolMap IPLR_GUARDED_BY(desc_pool_mutex_)
      cached_descriptor_pool_;
  bool should_disable_surface_control_ = false;
  bool should_batch_cmd_buffers_ = false;
  std::vector<std::shared_ptr<CommandBuffer>> pending_command_buffers_;

  const uint64_t hash_;

  bool is_valid_ = false;

  ContextVK();

  void Setup(Settings settings);

  ContextVK(const ContextVK&) = delete;

  ContextVK& operator=(const ContextVK&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_CONTEXT_VK_H_
