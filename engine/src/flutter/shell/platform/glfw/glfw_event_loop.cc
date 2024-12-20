// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/glfw/glfw_event_loop.h"

#include <GLFW/glfw3.h>

#include <atomic>
#include <utility>

namespace flutter {

GLFWEventLoop::GLFWEventLoop(std::thread::id main_thread_id,
                             const TaskExpiredCallback& on_task_expired)
    : EventLoop(main_thread_id, on_task_expired) {}

GLFWEventLoop::~GLFWEventLoop() = default;

void GLFWEventLoop::WaitUntil(const TaskTimePoint& time) {
  const auto now = TaskTimePoint::clock::now();

  // Make sure the seconds are not integral.
  using Seconds = std::chrono::duration<double, std::ratio<1>>;
  const auto duration_to_wait = std::chrono::duration_cast<Seconds>(time - now);

  if (duration_to_wait.count() > 0.0) {
    ::glfwWaitEventsTimeout(duration_to_wait.count());
  } else {
    // Avoid engine task priority inversion by making sure GLFW events are
    // always processed even when there is no need to wait for pending engine
    // tasks.
    ::glfwPollEvents();
  }
}

void GLFWEventLoop::Wake() {
  ::glfwPostEmptyEvent();
}

}  // namespace flutter
