// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "fml/status.h"

#include "impeller/renderer/backend/vulkan/command_queue_vk.h"

#include "impeller/base/validation.h"
#include "impeller/renderer/backend/vulkan/command_buffer_vk.h"
#include "impeller/renderer/backend/vulkan/context_vk.h"
#include "impeller/renderer/backend/vulkan/fence_waiter_vk.h"
#include "impeller/renderer/backend/vulkan/tracked_objects_vk.h"
#include "impeller/renderer/command_buffer.h"

namespace impeller {

CommandQueueVK::CommandQueueVK(const std::weak_ptr<ContextVK>& context)
    : context_(context) {}

CommandQueueVK::~CommandQueueVK() = default;

fml::Status CommandQueueVK::Submit(
    const std::vector<std::shared_ptr<CommandBuffer>>& buffers,
    const CompletionCallback& completion_callback) {
  if (buffers.empty()) {
    return fml::Status(fml::StatusCode::kInvalidArgument,
                       "No command buffers provided.");
  }
  // Success or failure, you only get to submit once.
  fml::ScopedCleanupClosure reset([&]() {
    if (completion_callback) {
      completion_callback(CommandBuffer::Status::kError);
    }
  });

  std::vector<vk::CommandBuffer> vk_buffers;
  std::vector<std::shared_ptr<TrackedObjectsVK>> tracked_objects;
  vk_buffers.reserve(buffers.size());
  tracked_objects.reserve(buffers.size());
  for (const std::shared_ptr<CommandBuffer>& buffer : buffers) {
    CommandBufferVK& command_buffer = CommandBufferVK::Cast(*buffer);
    if (!command_buffer.EndCommandBuffer()) {
      return fml::Status(fml::StatusCode::kCancelled,
                         "Failed to end command buffer.");
    }
    vk_buffers.push_back(command_buffer.GetCommandBuffer());
    tracked_objects.push_back(std::move(command_buffer.tracked_objects_));
  }

  auto context = context_.lock();
  if (!context) {
    VALIDATION_LOG << "Device lost.";
    return fml::Status(fml::StatusCode::kCancelled, "Device lost.");
  }
  auto [fence_result, fence] = context->GetDevice().createFenceUnique({});
  if (fence_result != vk::Result::eSuccess) {
    VALIDATION_LOG << "Failed to create fence: " << vk::to_string(fence_result);
    return fml::Status(fml::StatusCode::kCancelled, "Failed to create fence.");
  }

  vk::SubmitInfo submit_info;
  submit_info.setCommandBuffers(vk_buffers);
  auto status = context->GetGraphicsQueue()->Submit(submit_info, *fence);
  if (status != vk::Result::eSuccess) {
    VALIDATION_LOG << "Failed to submit queue: " << vk::to_string(status);
    return fml::Status(fml::StatusCode::kCancelled, "Failed to submit queue: ");
  }

  // Submit will proceed, call callback with true when it is done and do not
  // call when `reset` is collected.
  auto added_fence = context->GetFenceWaiter()->AddFence(
      std::move(fence), [completion_callback, tracked_objects = std::move(
                                                  tracked_objects)]() mutable {
        // Ensure tracked objects are destructed before calling any final
        // callbacks.
        tracked_objects.clear();
        if (completion_callback) {
          completion_callback(CommandBuffer::Status::kCompleted);
        }
      });
  if (!added_fence) {
    return fml::Status(fml::StatusCode::kCancelled, "Failed to add fence.");
  }
  reset.Release();
  return fml::Status();
}

}  // namespace impeller
