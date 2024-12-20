// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/vulkan/gpu_tracer_vk.h"

#include <memory>
#include <optional>
#include <thread>
#include <utility>

#include "fml/logging.h"
#include "fml/trace_event.h"
#include "impeller/base/validation.h"
#include "impeller/renderer/backend/vulkan/command_buffer_vk.h"
#include "impeller/renderer/backend/vulkan/context_vk.h"

namespace impeller {

static constexpr uint32_t kPoolSize = 128u;

GPUTracerVK::GPUTracerVK(std::weak_ptr<ContextVK> context,
                         bool enable_gpu_tracing)
    : context_(std::move(context)) {
  if (!enable_gpu_tracing) {
    return;
  }
  timestamp_period_ = context_.lock()
                          ->GetDeviceHolder()
                          ->GetPhysicalDevice()
                          .getProperties()
                          .limits.timestampPeriod;
  if (timestamp_period_ <= 0) {
    // The device does not support timestamp queries.
    return;
  }
// Disable tracing in release mode.
#ifdef IMPELLER_DEBUG
  enabled_ = true;
#endif  // IMPELLER_DEBUG
}

void GPUTracerVK::InitializeQueryPool(const ContextVK& context) {
  if (!enabled_) {
    return;
  }
  Lock lock(trace_state_mutex_);
  std::shared_ptr<CommandBuffer> buffer = context.CreateCommandBuffer();
  CommandBufferVK& buffer_vk = CommandBufferVK::Cast(*buffer);

  for (auto i = 0u; i < kTraceStatesSize; i++) {
    vk::QueryPoolCreateInfo info;
    info.queryCount = kPoolSize;
    info.queryType = vk::QueryType::eTimestamp;

    auto [status, pool] = context.GetDevice().createQueryPoolUnique(info);
    if (status != vk::Result::eSuccess) {
      VALIDATION_LOG << "Failed to create query pool.";
      return;
    }
    trace_states_[i].query_pool = std::move(pool);
    buffer_vk.GetCommandBuffer().resetQueryPool(
        trace_states_[i].query_pool.get(), 0, kPoolSize);
  }
  if (!context.GetCommandQueue()->Submit({buffer}).ok()) {
    VALIDATION_LOG << "Failed to reset query pool for trace events.";
    enabled_ = false;
  }
}

bool GPUTracerVK::IsEnabled() const {
  return enabled_;
}

void GPUTracerVK::MarkFrameStart() {
  if (!enabled_) {
    return;
  }
  FML_DCHECK(!in_frame_);
  in_frame_ = true;
  raster_thread_id_ = std::this_thread::get_id();
}

void GPUTracerVK::MarkFrameEnd() {
  in_frame_ = false;

  if (!enabled_) {
    return;
  }

  Lock lock(trace_state_mutex_);
  current_state_ = (current_state_ + 1) % kTraceStatesSize;

  auto& state = trace_states_[current_state_];
  // If there are still pending buffers on the trace state we're switching to,
  // that means that a cmd buffer we were relying on to signal this likely
  // never finished. This shouldn't happen unless there is a bug in the
  // encoder logic. We set it to zero anyway to prevent a validation error
  // from becoming a memory leak.
  FML_DCHECK(state.pending_buffers == 0u);
  state.pending_buffers = 0;
  state.current_index = 0;
}

std::unique_ptr<GPUProbe> GPUTracerVK::CreateGPUProbe() {
  return std::make_unique<GPUProbe>(weak_from_this());
}

void GPUTracerVK::RecordCmdBufferStart(const vk::CommandBuffer& buffer,
                                       GPUProbe& probe) {
  if (!enabled_ || std::this_thread::get_id() != raster_thread_id_ ||
      !in_frame_) {
    return;
  }
  Lock lock(trace_state_mutex_);
  auto& state = trace_states_[current_state_];

  // Reset previously completed queries.
  if (!states_to_reset_.empty()) {
    for (auto i = 0u; i < states_to_reset_.size(); i++) {
      buffer.resetQueryPool(trace_states_[states_to_reset_[i]].query_pool.get(),
                            0, kPoolSize);
    }
    states_to_reset_.clear();
  }

  // We size the query pool to kPoolSize, but Flutter applications can create an
  // unbounded amount of work per frame. If we encounter this, stop recording
  // cmds.
  if (state.current_index >= kPoolSize) {
    return;
  }

  buffer.writeTimestamp(vk::PipelineStageFlagBits::eTopOfPipe,
                        trace_states_[current_state_].query_pool.get(),
                        state.current_index);
  state.current_index += 1;
  probe.index_ = current_state_;
  state.pending_buffers += 1;
}

void GPUTracerVK::RecordCmdBufferEnd(const vk::CommandBuffer& buffer,
                                     GPUProbe& probe) {
  if (!enabled_ || std::this_thread::get_id() != raster_thread_id_ ||
      !in_frame_ || !probe.index_.has_value()) {
    return;
  }
  Lock lock(trace_state_mutex_);
  GPUTraceState& state = trace_states_[current_state_];

  if (state.current_index >= kPoolSize) {
    return;
  }

  buffer.writeTimestamp(vk::PipelineStageFlagBits::eBottomOfPipe,
                        state.query_pool.get(), state.current_index);

  state.current_index += 1;
}

void GPUTracerVK::OnFenceComplete(size_t frame_index) {
  if (!enabled_) {
    return;
  }

  size_t pending = 0;
  size_t query_count = 0;
  vk::QueryPool pool;
  {
    Lock lock(trace_state_mutex_);
    GPUTraceState& state = trace_states_[frame_index];

    FML_DCHECK(state.pending_buffers > 0);
    state.pending_buffers -= 1;
    pending = state.pending_buffers;
    query_count = state.current_index;
    pool = state.query_pool.get();
  }

  if (pending == 0) {
    std::vector<uint64_t> bits(query_count);
    std::shared_ptr<ContextVK> context = context_.lock();
    if (!context) {
      return;
    }

    auto result = context->GetDevice().getQueryPoolResults(
        pool, 0, query_count, query_count * sizeof(uint64_t), bits.data(),
        sizeof(uint64_t), vk::QueryResultFlagBits::e64);
    // This may return VK_NOT_READY if the query couldn't be completed, or if
    // there are queries still pending. From local testing, this happens
    // occassionally on very expensive frames. Its unclear if we can do anything
    // about this, because by design this should only signal after all cmd
    // buffers have signaled. Adding VK_QUERY_RESULT_WAIT_BIT to the flags
    // passed to getQueryPoolResults seems like it would fix this, but actually
    // seems to result in more stuck query errors. Better to just drop them and
    // move on.
    if (result == vk::Result::eSuccess) {
      uint64_t smallest_timestamp = std::numeric_limits<uint64_t>::max();
      uint64_t largest_timestamp = 0;
      for (auto i = 0u; i < bits.size(); i++) {
        smallest_timestamp = std::min(smallest_timestamp, bits[i]);
        largest_timestamp = std::max(largest_timestamp, bits[i]);
      }
      auto gpu_ms =
          (((largest_timestamp - smallest_timestamp) * timestamp_period_) /
           1000000);
      FML_TRACE_COUNTER("flutter", "GPUTracer",
                        reinterpret_cast<int64_t>(this),  // Trace Counter ID
                        "FrameTimeMS", gpu_ms);
    }

    // Record this query to be reset the next time a command is recorded.
    Lock lock(trace_state_mutex_);
    states_to_reset_.push_back(frame_index);
  }
}

GPUProbe::GPUProbe(const std::weak_ptr<GPUTracerVK>& tracer)
    : tracer_(tracer) {}

GPUProbe::~GPUProbe() {
  if (!index_.has_value()) {
    return;
  }
  auto tracer = tracer_.lock();
  if (!tracer) {
    return;
  }
  tracer->OnFenceComplete(index_.value());
}

void GPUProbe::RecordCmdBufferStart(const vk::CommandBuffer& buffer) {
  auto tracer = tracer_.lock();
  if (!tracer) {
    return;
  }
  tracer->RecordCmdBufferStart(buffer, *this);
}

void GPUProbe::RecordCmdBufferEnd(const vk::CommandBuffer& buffer) {
  auto tracer = tracer_.lock();
  if (!tracer) {
    return;
  }
  tracer->RecordCmdBufferEnd(buffer, *this);
}

}  // namespace impeller
