// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_QUEUE_VK_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_QUEUE_VK_H_

#include <memory>

#include "impeller/base/thread.h"
#include "impeller/renderer/backend/vulkan/vk.h"

namespace impeller {

struct QueueIndexVK {
  size_t family = 0;
  size_t index = 0;

  constexpr bool operator==(const QueueIndexVK& other) const {
    return family == other.family && index == other.index;
  }
};

//------------------------------------------------------------------------------
/// @brief      A thread safe object that can be used to access device queues.
///             If multiple objects are created with the same underlying queue,
///             then the external synchronization guarantees of Vulkan queues
///             cannot be met. So care must be taken the same device queue
///             doesn't form the basis of multiple `QueueVK`s.
///
class QueueVK {
 public:
  QueueVK(QueueIndexVK index, vk::Queue queue);

  ~QueueVK();

  const QueueIndexVK& GetIndex() const;

  vk::Result Submit(const vk::SubmitInfo& submit_info,
                    const vk::Fence& fence) const;

  vk::Result Submit(const vk::Fence& fence) const;

  vk::Result Present(const vk::PresentInfoKHR& present_info);

  void InsertDebugMarker(std::string_view label) const;

 private:
  mutable Mutex queue_mutex_;

  const QueueIndexVK index_;
  const vk::Queue queue_ IPLR_GUARDED_BY(queue_mutex_);

  QueueVK(const QueueVK&) = delete;

  QueueVK& operator=(const QueueVK&) = delete;
};

//------------------------------------------------------------------------------
/// @brief      The collection of queues used by the context. The queues may all
///             be the same.
///
struct QueuesVK {
  std::shared_ptr<QueueVK> graphics_queue;
  std::shared_ptr<QueueVK> compute_queue;
  std::shared_ptr<QueueVK> transfer_queue;

  QueuesVK();

  QueuesVK(std::shared_ptr<QueueVK> graphics_queue,
           std::shared_ptr<QueueVK> compute_queue,
           std::shared_ptr<QueueVK> transfer_queue);

  static QueuesVK FromEmbedderQueue(vk::Queue queue,
                                    uint32_t queue_family_index);

  static QueuesVK FromQueueIndices(const vk::Device& device,
                                   QueueIndexVK graphics,
                                   QueueIndexVK compute,
                                   QueueIndexVK transfer);

  bool IsValid() const;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_QUEUE_VK_H_
