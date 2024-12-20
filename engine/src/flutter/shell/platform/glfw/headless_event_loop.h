// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_GLFW_HEADLESS_EVENT_LOOP_H_
#define FLUTTER_SHELL_PLATFORM_GLFW_HEADLESS_EVENT_LOOP_H_

#include <condition_variable>

#include "flutter/shell/platform/glfw/event_loop.h"

namespace flutter {

// An event loop implementation that only handles Flutter Engine task
// scheduling.
class HeadlessEventLoop : public EventLoop {
 public:
  using TaskExpiredCallback = std::function<void(const FlutterTask*)>;
  HeadlessEventLoop(std::thread::id main_thread_id,
                    const TaskExpiredCallback& on_task_expired);

  ~HeadlessEventLoop();

  // Disallow copy.
  HeadlessEventLoop(const HeadlessEventLoop&) = delete;
  HeadlessEventLoop& operator=(const HeadlessEventLoop&) = delete;

 private:
  // |EventLoop|
  void WaitUntil(const TaskTimePoint& time) override;

  // |EventLoop|
  void Wake() override;

  std::condition_variable task_queue_condition_;
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_GLFW_HEADLESS_EVENT_LOOP_H_
