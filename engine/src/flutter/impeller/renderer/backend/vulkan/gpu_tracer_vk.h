// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_GPU_TRACER_VK_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_GPU_TRACER_VK_H_

#include <memory>
#include <thread>

#include "impeller/renderer/backend/vulkan/context_vk.h"
#include "impeller/renderer/backend/vulkan/device_holder_vk.h"
#include "vulkan/vulkan_handles.hpp"

namespace impeller {

class GPUProbe;

/// @brief A class that uses timestamp queries to record the approximate GPU
/// execution time.
///
/// To enable, add the following metadata to the application's Android manifest:
///   <meta-data
///       android:name="io.flutter.embedding.android.EnableVulkanGPUTracing"
///       android:value="false" />
class GPUTracerVK : public std::enable_shared_from_this<GPUTracerVK> {
 public:
  GPUTracerVK(std::weak_ptr<ContextVK> context, bool enable_gpu_tracing);

  ~GPUTracerVK() = default;

  /// @brief Create a GPUProbe to trace the execution of a command buffer on the
  ///        GPU.
  std::unique_ptr<GPUProbe> CreateGPUProbe();

  /// @brief Signal the start of a frame workload.
  ///
  ///        Any cmd buffers that are created after this call and before
  ///        [MarkFrameEnd] will be attributed to the current frame.
  void MarkFrameStart();

  /// @brief Signal the end of a frame workload.
  void MarkFrameEnd();

  // visible for testing.
  bool IsEnabled() const;

  /// Initialize the set of query pools.
  void InitializeQueryPool(const ContextVK& context);

 private:
  friend class GPUProbe;

  static const constexpr size_t kTraceStatesSize = 16u;

  /// @brief Signal that the cmd buffer is completed.
  ///
  ///        If [frame_index] is std::nullopt, this frame recording is ignored.
  void OnFenceComplete(size_t frame);

  /// @brief Record a timestamp query into the provided cmd buffer to record
  ///        start time.
  void RecordCmdBufferStart(const vk::CommandBuffer& buffer, GPUProbe& probe);

  /// @brief Record a timestamp query into the provided cmd buffer to record end
  ///        time.
  void RecordCmdBufferEnd(const vk::CommandBuffer& buffer, GPUProbe& probe);

  std::weak_ptr<ContextVK> context_;

  struct GPUTraceState {
    size_t current_index = 0;
    size_t pending_buffers = 0;
    vk::UniqueQueryPool query_pool;
  };

  mutable Mutex trace_state_mutex_;
  GPUTraceState trace_states_[kTraceStatesSize] IPLR_GUARDED_BY(
      trace_state_mutex_);
  size_t current_state_ IPLR_GUARDED_BY(trace_state_mutex_) = 0u;
  std::vector<size_t> IPLR_GUARDED_BY(trace_state_mutex_) states_to_reset_ = {};

  // The number of nanoseconds for each timestamp unit.
  float timestamp_period_ = 1;

  // If in_frame_ is not true, then this cmd buffer was started as a part of
  // some non-frame workload like image decoding. We should not record this as
  // part of the frame workload, as the gap between this non-frame and a
  // frameworkload may be substantial. For example, support the engine creates a
  // cmd buffer to perform an image upload at timestamp 0 and then 30 ms later
  // actually renders a frame. Without the in_frame_ guard, the GPU frame time
  // would include this 30ms gap during which the engine was idle.
  bool in_frame_ = false;

  // Track the raster thread id to avoid recording mipmap/image cmd buffers
  // that are not guaranteed to start/end according to frame boundaries.
  std::thread::id raster_thread_id_;
  bool enabled_ = false;
};

class GPUProbe {
 public:
  explicit GPUProbe(const std::weak_ptr<GPUTracerVK>& tracer);

  GPUProbe(GPUProbe&&) = delete;
  GPUProbe& operator=(GPUProbe&&) = delete;

  ~GPUProbe();

  /// @brief Record a timestamp query into the provided cmd buffer to record
  ///        start time.
  void RecordCmdBufferStart(const vk::CommandBuffer& buffer);

  /// @brief Record a timestamp query into the provided cmd buffer to record end
  ///        time.
  void RecordCmdBufferEnd(const vk::CommandBuffer& buffer);

 private:
  friend class GPUTracerVK;

  std::weak_ptr<GPUTracerVK> tracer_;
  std::optional<size_t> index_ = std::nullopt;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_GPU_TRACER_VK_H_
