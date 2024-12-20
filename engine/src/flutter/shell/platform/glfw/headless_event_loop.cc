// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/glfw/headless_event_loop.h"

#include <atomic>
#include <utility>

namespace flutter {

HeadlessEventLoop::HeadlessEventLoop(std::thread::id main_thread_id,
                                     const TaskExpiredCallback& on_task_expired)
    : EventLoop(main_thread_id, on_task_expired) {}

HeadlessEventLoop::~HeadlessEventLoop() = default;

void HeadlessEventLoop::WaitUntil(const TaskTimePoint& time) {
  std::mutex& mutex = GetTaskQueueMutex();
  std::unique_lock<std::mutex> lock(mutex);
  task_queue_condition_.wait_until(lock, time);
}

void HeadlessEventLoop::Wake() {
  task_queue_condition_.notify_one();
}

}  // namespace flutter
