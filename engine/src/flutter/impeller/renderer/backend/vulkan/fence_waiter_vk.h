// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_FENCE_WAITER_VK_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_FENCE_WAITER_VK_H_

#include <condition_variable>
#include <memory>
#include <thread>
#include <vector>

#include "flutter/fml/closure.h"
#include "impeller/renderer/backend/vulkan/device_holder_vk.h"

namespace impeller {

class ContextVK;
class WaitSetEntry;

using WaitSet = std::vector<std::shared_ptr<WaitSetEntry>>;

class FenceWaiterVK {
 public:
  ~FenceWaiterVK();

  bool IsValid() const;

  void Terminate();

  bool AddFence(vk::UniqueFence fence, const fml::closure& callback);

 private:
  friend class ContextVK;

  std::weak_ptr<DeviceHolderVK> device_holder_;
  std::unique_ptr<std::thread> waiter_thread_;
  std::mutex wait_set_mutex_;
  std::condition_variable wait_set_cv_;
  WaitSet wait_set_;
  bool terminate_ = false;

  explicit FenceWaiterVK(std::weak_ptr<DeviceHolderVK> device_holder);

  void Main();

  bool Wait();
  void WaitUntilEmpty();

  FenceWaiterVK(const FenceWaiterVK&) = delete;

  FenceWaiterVK& operator=(const FenceWaiterVK&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_FENCE_WAITER_VK_H_
