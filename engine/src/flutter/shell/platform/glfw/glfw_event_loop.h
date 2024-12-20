// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_GLFW_GLFW_EVENT_LOOP_H_
#define FLUTTER_SHELL_PLATFORM_GLFW_GLFW_EVENT_LOOP_H_

#include "flutter/shell/platform/glfw/event_loop.h"

namespace flutter {

// An event loop implementation that supports Flutter Engine tasks scheduling in
// the GLFW event loop.
class GLFWEventLoop : public EventLoop {
 public:
  GLFWEventLoop(std::thread::id main_thread_id,
                const TaskExpiredCallback& on_task_expired);

  virtual ~GLFWEventLoop();

  // Prevent copying.
  GLFWEventLoop(const GLFWEventLoop&) = delete;
  GLFWEventLoop& operator=(const GLFWEventLoop&) = delete;

 private:
  // |EventLoop|
  void WaitUntil(const TaskTimePoint& time) override;

  // |EventLoop|
  void Wake() override;
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_GLFW_GLFW_EVENT_LOOP_H_
