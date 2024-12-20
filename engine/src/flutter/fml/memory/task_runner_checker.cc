// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/memory/task_runner_checker.h"

namespace fml {

TaskRunnerChecker::TaskRunnerChecker()
    : initialized_queue_id_(InitTaskQueueId()),
      subsumed_queue_ids_(
          MessageLoopTaskQueues::GetInstance()->GetSubsumedTaskQueueId(
              initialized_queue_id_)){};

TaskRunnerChecker::~TaskRunnerChecker() = default;

bool TaskRunnerChecker::RunsOnCreationTaskRunner() const {
  FML_CHECK(fml::MessageLoop::IsInitializedForCurrentThread());
  const auto current_queue_id = MessageLoop::GetCurrentTaskQueueId();
  if (RunsOnTheSameThread(current_queue_id, initialized_queue_id_)) {
    return true;
  }
  for (auto& subsumed : subsumed_queue_ids_) {
    if (RunsOnTheSameThread(current_queue_id, subsumed)) {
      return true;
    }
  }
  return false;
};

bool TaskRunnerChecker::RunsOnTheSameThread(TaskQueueId queue_a,
                                            TaskQueueId queue_b) {
  if (queue_a == queue_b) {
    return true;
  }

  auto queues = MessageLoopTaskQueues::GetInstance();
  if (queues->Owns(queue_a, queue_b)) {
    return true;
  }
  if (queues->Owns(queue_b, queue_a)) {
    return true;
  }
  return false;
};

TaskQueueId TaskRunnerChecker::InitTaskQueueId() {
  MessageLoop::EnsureInitializedForCurrentThread();
  return MessageLoop::GetCurrentTaskQueueId();
};

}  // namespace fml
