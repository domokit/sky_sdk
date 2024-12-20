// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/embedder/embedder_task_runner.h"

#include "flutter/fml/message_loop_impl.h"
#include "flutter/fml/message_loop_task_queues.h"

namespace flutter {

EmbedderTaskRunner::EmbedderTaskRunner(DispatchTable table,
                                       size_t embedder_identifier)
    : TaskRunner(nullptr /* loop implemenation*/),
      embedder_identifier_(embedder_identifier),
      dispatch_table_(std::move(table)),
      placeholder_id_(
          fml::MessageLoopTaskQueues::GetInstance()->CreateTaskQueue()) {
  FML_DCHECK(dispatch_table_.post_task_callback);
  FML_DCHECK(dispatch_table_.runs_task_on_current_thread_callback);
  FML_DCHECK(dispatch_table_.destruction_callback);
}

EmbedderTaskRunner::~EmbedderTaskRunner() {
  dispatch_table_.destruction_callback();
}

size_t EmbedderTaskRunner::GetEmbedderIdentifier() const {
  return embedder_identifier_;
}

void EmbedderTaskRunner::PostTask(const fml::closure& task) {
  PostTaskForTime(task, fml::TimePoint::Now());
}

void EmbedderTaskRunner::PostTaskForTime(const fml::closure& task,
                                         fml::TimePoint target_time) {
  if (!task) {
    return;
  }

  uint64_t baton = 0;

  {
    // Release the lock before the jump via the dispatch table.
    std::scoped_lock lock(tasks_mutex_);
    baton = ++last_baton_;
    pending_tasks_[baton] = task;
  }

  dispatch_table_.post_task_callback(this, baton, target_time);
}

void EmbedderTaskRunner::PostDelayedTask(const fml::closure& task,
                                         fml::TimeDelta delay) {
  PostTaskForTime(task, fml::TimePoint::Now() + delay);
}

bool EmbedderTaskRunner::RunsTasksOnCurrentThread() {
  return dispatch_table_.runs_task_on_current_thread_callback();
}

bool EmbedderTaskRunner::PostTask(uint64_t baton) {
  fml::closure task;

  {
    std::scoped_lock lock(tasks_mutex_);
    auto found = pending_tasks_.find(baton);
    if (found == pending_tasks_.end()) {
      FML_LOG(ERROR) << "Embedder attempted to post an unknown task.";
      return false;
    }
    task = found->second;
    pending_tasks_.erase(found);

    // Let go of the tasks mutex befor executing the task.
  }

  FML_DCHECK(task);
  task();
  return true;
}

// |fml::TaskRunner|
fml::TaskQueueId EmbedderTaskRunner::GetTaskQueueId() {
  return placeholder_id_;
}

}  // namespace flutter
