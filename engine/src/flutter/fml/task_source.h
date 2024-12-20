// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FML_TASK_SOURCE_H_
#define FLUTTER_FML_TASK_SOURCE_H_

#include "flutter/fml/delayed_task.h"
#include "flutter/fml/task_queue_id.h"
#include "flutter/fml/task_source_grade.h"

namespace fml {

class MessageLoopTaskQueues;

/**
 * A Source of tasks for the `MessageLoopTaskQueues` task dispatcher. This is a
 * wrapper around a primary and secondary task heap with the difference between
 * them being that the secondary task heap can be paused and resumed by the task
 * dispatcher. `TaskSourceGrade` determines what task heap the task is assigned
 * to.
 *
 * Registering Tasks
 * -----------------
 * The task dispatcher associates a task source with each `TaskQueueID`. When
 * the user of the task dispatcher registers a task, the task is in-turn
 * registered with the `TaskSource` corresponding to the `TaskQueueID`.
 *
 * Processing Tasks
 * ----------------
 * Task dispatcher provides the event loop a way to acquire tasks to run via
 * `GetNextTaskToRun`. Task dispatcher asks the underlying `TaskSource` for the
 * next task.
 */
class TaskSource {
 public:
  struct TopTask {
    TaskQueueId task_queue_id;
    const DelayedTask& task;
  };

  /// Construts a TaskSource with the given `task_queue_id`.
  explicit TaskSource(TaskQueueId task_queue_id);

  ~TaskSource();

  /// Drops the pending tasks from both primary and secondary task heaps.
  void ShutDown();

  /// Adds a task to the corresponding task heap as dictated by the
  /// `TaskSourceGrade` of the `DelayedTask`.
  void RegisterTask(const DelayedTask& task);

  /// Pops the task heap corresponding to the `TaskSourceGrade`.
  void PopTask(TaskSourceGrade grade);

  /// Returns the number of pending tasks. Excludes the tasks from the secondary
  /// heap if it's paused.
  size_t GetNumPendingTasks() const;

  /// Returns true if `GetNumPendingTasks` is zero.
  bool IsEmpty() const;

  /// Returns the top task based on scheduled time, taking into account whether
  /// the secondary heap has been paused or not.
  TopTask Top() const;

  /// Pause providing tasks from secondary task heap.
  void PauseSecondary();

  /// Resume providing tasks from secondary task heap.
  void ResumeSecondary();

 private:
  const fml::TaskQueueId task_queue_id_;
  fml::DelayedTaskQueue primary_task_queue_;
  fml::DelayedTaskQueue secondary_task_queue_;
  int secondary_pause_requests_ = 0;

  FML_DISALLOW_COPY_ASSIGN_AND_MOVE(TaskSource);
};

}  // namespace fml

#endif  // FLUTTER_FML_TASK_SOURCE_H_
