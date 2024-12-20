// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_GLFW_EVENT_LOOP_H_
#define FLUTTER_SHELL_PLATFORM_GLFW_EVENT_LOOP_H_

#include <chrono>
#include <deque>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>

#include "flutter/shell/platform/embedder/embedder.h"

namespace flutter {

// An abstract event loop.
class EventLoop {
 public:
  using TaskExpiredCallback = std::function<void(const FlutterTask*)>;

  // Creates an event loop running on the given thread, calling
  // |on_task_expired| to run tasks.
  EventLoop(std::thread::id main_thread_id,
            const TaskExpiredCallback& on_task_expired);

  virtual ~EventLoop();

  // Disallow copy.
  EventLoop(const EventLoop&) = delete;
  EventLoop& operator=(const EventLoop&) = delete;

  // Returns if the current thread is the thread used by this event loop.
  bool RunsTasksOnCurrentThread() const;

  // Waits for the next event, processes it, and returns.
  //
  // Expired engine events, if any, are processed as well. The optional
  // timeout should only be used when events not managed by this loop need to be
  // processed in a polling manner.
  void WaitForEvents(
      std::chrono::nanoseconds max_wait = std::chrono::nanoseconds::max());

  // Posts a Flutter engine task to the event loop for delayed execution.
  void PostTask(FlutterTask flutter_task, uint64_t flutter_target_time_nanos);

 protected:
  using TaskTimePoint = std::chrono::steady_clock::time_point;

  // Returns the timepoint corresponding to a Flutter task time.
  static TaskTimePoint TimePointFromFlutterTime(
      uint64_t flutter_target_time_nanos);

  // Returns the mutex used to control the task queue. Subclasses may safely
  // lock this mutex in the abstract methods below.
  std::mutex& GetTaskQueueMutex() { return task_queue_mutex_; }

  // Waits until the given time, or a Wake() call.
  virtual void WaitUntil(const TaskTimePoint& time) = 0;

  // Wakes the main thread from a WaitUntil call.
  virtual void Wake() = 0;

  struct Task {
    uint64_t order;
    TaskTimePoint fire_time;
    FlutterTask task;

    struct Comparer {
      bool operator()(const Task& a, const Task& b) {
        if (a.fire_time == b.fire_time) {
          return a.order > b.order;
        }
        return a.fire_time > b.fire_time;
      }
    };
  };
  std::thread::id main_thread_id_;
  TaskExpiredCallback on_task_expired_;
  std::mutex task_queue_mutex_;
  std::priority_queue<Task, std::deque<Task>, Task::Comparer> task_queue_;
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_GLFW_EVENT_LOOP_H_
