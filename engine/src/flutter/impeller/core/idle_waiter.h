// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_CORE_IDLE_WAITER_H_
#define FLUTTER_IMPELLER_CORE_IDLE_WAITER_H_

namespace impeller {

/// Abstraction over waiting for the GPU to be idle.
///
/// This is important for platforms like Vulkan where we need to make sure
/// we aren't deleting resources while the GPU is using them.
class IdleWaiter {
 public:
  virtual ~IdleWaiter() = default;

  /// Wait for the GPU tasks to finish.
  /// This is a noop on some platforms, it's important for Vulkan.
  virtual void WaitIdle() const = 0;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_CORE_IDLE_WAITER_H_
