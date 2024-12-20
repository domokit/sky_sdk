// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#define FML_USED_ON_EMBEDDER

#include "flutter/fml/task_runner.h"
#include "flutter/fml/memory/task_runner_checker.h"

#include <utility>

#include "flutter/fml/logging.h"
#include "flutter/fml/message_loop.h"
#include "flutter/fml/message_loop_impl.h"
#include "flutter/fml/message_loop_task_queues.h"

namespace fml {

TaskRunner::TaskRunner(fml::RefPtr<MessageLoopImpl> loop)
    : loop_(std::move(loop)) {}

TaskRunner::~TaskRunner() = default;

void TaskRunner::PostTask(const fml::closure& task) {
  loop_->PostTask(task, fml::TimePoint::Now());
}

void TaskRunner::PostTaskForTime(const fml::closure& task,
                                 fml::TimePoint target_time) {
  loop_->PostTask(task, target_time);
}

void TaskRunner::PostDelayedTask(const fml::closure& task,
                                 fml::TimeDelta delay) {
  loop_->PostTask(task, fml::TimePoint::Now() + delay);
}

TaskQueueId TaskRunner::GetTaskQueueId() {
  FML_DCHECK(loop_);
  return loop_->GetTaskQueueId();
}

bool TaskRunner::RunsTasksOnCurrentThread() {
  if (!fml::MessageLoop::IsInitializedForCurrentThread()) {
    return false;
  }

  const auto current_queue_id = MessageLoop::GetCurrentTaskQueueId();
  const auto loop_queue_id = loop_->GetTaskQueueId();

  return TaskRunnerChecker::RunsOnTheSameThread(current_queue_id,
                                                loop_queue_id);
}

// static
void TaskRunner::RunNowOrPostTask(const fml::RefPtr<fml::TaskRunner>& runner,
                                  const fml::closure& task) {
  FML_DCHECK(runner);
  if (runner->RunsTasksOnCurrentThread()) {
    task();
  } else {
    runner->PostTask(task);
  }
}

// static
void TaskRunner::RunNowAndFlushMessages(
    const fml::RefPtr<fml::TaskRunner>& runner,
    const fml::closure& task) {
  FML_DCHECK(runner);
  if (runner->RunsTasksOnCurrentThread()) {
    task();
    // Post an empty task to make the UI message loop run its task observers.
    // The observers will execute any Dart microtasks queued by the platform
    // message handler.
    runner->PostTask([] {});
  } else {
    runner->PostTask(task);
  }
}

}  // namespace fml
