// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/concurrent_message_loop.h"

#include <algorithm>

#include "flutter/fml/thread.h"
#include "flutter/fml/trace_event.h"

namespace fml {

ConcurrentMessageLoop::ConcurrentMessageLoop(size_t worker_count)
    : worker_count_(std::max<size_t>(worker_count, 1ul)) {
  for (size_t i = 0; i < worker_count_; ++i) {
    workers_.emplace_back([i, this]() {
      fml::Thread::SetCurrentThreadName(fml::Thread::ThreadConfig(
          std::string{"io.worker." + std::to_string(i + 1)}));
      WorkerMain();
    });
  }

  for (const auto& worker : workers_) {
    worker_thread_ids_.emplace_back(worker.get_id());
  }
}

ConcurrentMessageLoop::~ConcurrentMessageLoop() {
  Terminate();
  for (auto& worker : workers_) {
    FML_DCHECK(worker.joinable());
    worker.join();
  }
}

size_t ConcurrentMessageLoop::GetWorkerCount() const {
  return worker_count_;
}

std::shared_ptr<ConcurrentTaskRunner> ConcurrentMessageLoop::GetTaskRunner() {
  return std::make_shared<ConcurrentTaskRunner>(weak_from_this());
}

void ConcurrentMessageLoop::PostTask(const fml::closure& task) {
  if (!task) {
    return;
  }

  std::unique_lock lock(tasks_mutex_);

  // Don't just drop tasks on the floor in case of shutdown.
  if (shutdown_) {
    FML_DLOG(WARNING)
        << "Tried to post a task to shutdown concurrent message "
           "loop. The task will be executed on the callers thread.";
    lock.unlock();
    ExecuteTask(task);
    return;
  }

  tasks_.push(task);

  // Unlock the mutex before notifying the condition variable because that mutex
  // has to be acquired on the other thread anyway. Waiting in this scope till
  // it is acquired there is a pessimization.
  lock.unlock();

  tasks_condition_.notify_one();
}

void ConcurrentMessageLoop::WorkerMain() {
  while (true) {
    std::unique_lock lock(tasks_mutex_);
    tasks_condition_.wait(lock, [&]() {
      return !tasks_.empty() || shutdown_ || HasThreadTasksLocked();
    });

    // Shutdown cannot be read with the task mutex unlocked.
    bool shutdown_now = shutdown_;
    fml::closure task;
    std::vector<fml::closure> thread_tasks;

    if (!tasks_.empty()) {
      task = tasks_.front();
      tasks_.pop();
    }

    if (HasThreadTasksLocked()) {
      thread_tasks = GetThreadTasksLocked();
      FML_DCHECK(!HasThreadTasksLocked());
    }

    // Don't hold onto the mutex while tasks are being executed as they could
    // themselves try to post more tasks to the message loop.
    lock.unlock();

    TRACE_EVENT0("flutter", "ConcurrentWorkerWake");
    // Execute the primary task we woke up for.
    if (task) {
      ExecuteTask(task);
    }

    // Execute any thread tasks.
    for (const auto& thread_task : thread_tasks) {
      ExecuteTask(thread_task);
    }

    if (shutdown_now) {
      break;
    }
  }
}

void ConcurrentMessageLoop::ExecuteTask(const fml::closure& task) {
  task();
}

void ConcurrentMessageLoop::Terminate() {
  std::scoped_lock lock(tasks_mutex_);
  shutdown_ = true;
  tasks_condition_.notify_all();
}

void ConcurrentMessageLoop::PostTaskToAllWorkers(const fml::closure& task) {
  if (!task) {
    return;
  }

  std::scoped_lock lock(tasks_mutex_);
  for (const auto& worker_thread_id : worker_thread_ids_) {
    thread_tasks_[worker_thread_id].emplace_back(task);
  }
  tasks_condition_.notify_all();
}

bool ConcurrentMessageLoop::HasThreadTasksLocked() const {
  return thread_tasks_.count(std::this_thread::get_id()) > 0;
}

std::vector<fml::closure> ConcurrentMessageLoop::GetThreadTasksLocked() {
  auto found = thread_tasks_.find(std::this_thread::get_id());
  FML_DCHECK(found != thread_tasks_.end());
  std::vector<fml::closure> pending_tasks;
  std::swap(pending_tasks, found->second);
  thread_tasks_.erase(found);
  return pending_tasks;
}

ConcurrentTaskRunner::ConcurrentTaskRunner(
    std::weak_ptr<ConcurrentMessageLoop> weak_loop)
    : weak_loop_(std::move(weak_loop)) {}

ConcurrentTaskRunner::~ConcurrentTaskRunner() = default;

void ConcurrentTaskRunner::PostTask(const fml::closure& task) {
  if (!task) {
    return;
  }

  if (auto loop = weak_loop_.lock()) {
    loop->PostTask(task);
    return;
  }

  FML_DLOG(WARNING)
      << "Tried to post to a concurrent message loop that has already died. "
         "Executing the task on the callers thread.";
  task();
}

bool ConcurrentMessageLoop::RunsTasksOnCurrentThread() {
  std::scoped_lock lock(tasks_mutex_);
  for (const auto& worker_thread_id : worker_thread_ids_) {
    if (worker_thread_id == std::this_thread::get_id()) {
      return true;
    }
  }
  return false;
}

}  // namespace fml
