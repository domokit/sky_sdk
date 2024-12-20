// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/vulkan/queue_vk.h"

#include <utility>

#include "impeller/renderer/backend/vulkan/context_vk.h"

namespace impeller {

QueueVK::QueueVK(QueueIndexVK index, vk::Queue queue)
    : index_(index), queue_(queue) {}

QueueVK::~QueueVK() = default;

const QueueIndexVK& QueueVK::GetIndex() const {
  return index_;
}

vk::Result QueueVK::Submit(const vk::SubmitInfo& submit_info,
                           const vk::Fence& fence) const {
  Lock lock(queue_mutex_);
  return queue_.submit(submit_info, fence);
}

vk::Result QueueVK::Submit(const vk::Fence& fence) const {
  Lock lock(queue_mutex_);
  return queue_.submit({}, fence);
}

vk::Result QueueVK::Present(const vk::PresentInfoKHR& present_info) {
  Lock lock(queue_mutex_);
  return queue_.presentKHR(present_info);
}

void QueueVK::InsertDebugMarker(std::string_view label) const {
  if (!HasValidationLayers()) {
    return;
  }
  vk::DebugUtilsLabelEXT label_info;
  label_info.pLabelName = label.data();
  Lock lock(queue_mutex_);
  queue_.insertDebugUtilsLabelEXT(label_info);
}

QueuesVK::QueuesVK() = default;

QueuesVK::QueuesVK(std::shared_ptr<QueueVK> graphics_queue,
                   std::shared_ptr<QueueVK> compute_queue,
                   std::shared_ptr<QueueVK> transfer_queue)
    : graphics_queue(std::move(graphics_queue)),
      compute_queue(std::move(compute_queue)),
      transfer_queue(std::move(transfer_queue)) {}

// static
QueuesVK QueuesVK::FromEmbedderQueue(vk::Queue queue,
                                     uint32_t queue_family_index) {
  auto graphics_queue = std::make_shared<QueueVK>(
      QueueIndexVK{.family = queue_family_index, .index = 0}, queue);

  return QueuesVK(graphics_queue, graphics_queue, graphics_queue);
}

// static
QueuesVK QueuesVK::FromQueueIndices(const vk::Device& device,
                                    QueueIndexVK graphics,
                                    QueueIndexVK compute,
                                    QueueIndexVK transfer) {
  auto vk_graphics = device.getQueue(graphics.family, graphics.index);
  auto vk_compute = device.getQueue(compute.family, compute.index);
  auto vk_transfer = device.getQueue(transfer.family, transfer.index);

  // Always set up the graphics queue.
  auto graphics_queue = std::make_shared<QueueVK>(graphics, vk_graphics);
  ContextVK::SetDebugName(device, vk_graphics, "ImpellerGraphicsQ");

  // Setup the compute queue if its different from the graphics queue.
  std::shared_ptr<QueueVK> compute_queue;
  if (compute == graphics) {
    compute_queue = graphics_queue;
  } else {
    compute_queue = std::make_shared<QueueVK>(compute, vk_compute);
    ContextVK::SetDebugName(device, vk_compute, "ImpellerComputeQ");
  }

  // Setup the transfer queue if its different from the graphics or compute
  // queues.
  std::shared_ptr<QueueVK> transfer_queue;
  if (transfer == graphics) {
    transfer_queue = graphics_queue;
  } else if (transfer == compute) {
    transfer_queue = compute_queue;
  } else {
    transfer_queue = std::make_shared<QueueVK>(transfer, vk_transfer);
    ContextVK::SetDebugName(device, vk_transfer, "ImpellerTransferQ");
  }

  return QueuesVK(std::move(graphics_queue), std::move(compute_queue),
                  std::move(transfer_queue));
}

bool QueuesVK::IsValid() const {
  return graphics_queue && compute_queue && transfer_queue;
}

}  // namespace impeller
