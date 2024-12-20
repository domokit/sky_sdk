// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FML_MESSAGE_LOOP_TASK_QUEUES_H_
#define FLUTTER_FML_MESSAGE_LOOP_TASK_QUEUES_H_

#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <vector>

#include "flutter/fml/closure.h"
#include "flutter/fml/delayed_task.h"
#include "flutter/fml/macros.h"
#include "flutter/fml/memory/ref_counted.h"
#include "flutter/fml/task_queue_id.h"
#include "flutter/fml/task_source.h"
#include "flutter/fml/wakeable.h"

namespace fml {

static const TaskQueueId kUnmerged = TaskQueueId(TaskQueueId::kUnmerged);

/// A collection of tasks and observers associated with one TaskQueue.
///
/// Often a TaskQueue has a one-to-one relationship with a fml::MessageLoop,
/// this isn't the case when TaskQueues are merged via
/// \p fml::MessageLoopTaskQueues::Merge.
class TaskQueueEntry {
 public:
  using TaskObservers = std::map<intptr_t, fml::closure>;
  Wakeable* wakeable;
  TaskObservers task_observers;
  std::unique_ptr<TaskSource> task_source;

  /// Set of the TaskQueueIds which is owned by this TaskQueue. If the set is
  /// empty, this TaskQueue does not own any other TaskQueues.
  std::set<TaskQueueId> owner_of;

  /// Identifies the TaskQueue that subsumes this TaskQueue. If it is kUnmerged
  /// it indicates that this TaskQueue is not owned by any other TaskQueue.
  TaskQueueId subsumed_by;

  TaskQueueId created_for;

  explicit TaskQueueEntry(TaskQueueId created_for);

 private:
  FML_DISALLOW_COPY_ASSIGN_AND_MOVE(TaskQueueEntry);
};

enum class FlushType {
  kSingle,
  kAll,
};

/// A singleton container for all tasks and observers associated with all
/// fml::MessageLoops.
///
/// This also wakes up the loop at the required times.
/// \see fml::MessageLoop
/// \see fml::Wakeable
class MessageLoopTaskQueues {
 public:
  // Lifecycle.

  static MessageLoopTaskQueues* GetInstance();

  TaskQueueId CreateTaskQueue();

  void Dispose(TaskQueueId queue_id);

  void DisposeTasks(TaskQueueId queue_id);

  // Tasks methods.

  void RegisterTask(TaskQueueId queue_id,
                    const fml::closure& task,
                    fml::TimePoint target_time,
                    fml::TaskSourceGrade task_source_grade =
                        fml::TaskSourceGrade::kUnspecified);

  bool HasPendingTasks(TaskQueueId queue_id) const;

  fml::closure GetNextTaskToRun(TaskQueueId queue_id, fml::TimePoint from_time);

  size_t GetNumPendingTasks(TaskQueueId queue_id) const;

  static TaskSourceGrade GetCurrentTaskSourceGrade();

  // Observers methods.

  void AddTaskObserver(TaskQueueId queue_id,
                       intptr_t key,
                       const fml::closure& callback);

  void RemoveTaskObserver(TaskQueueId queue_id, intptr_t key);

  std::vector<fml::closure> GetObserversToNotify(TaskQueueId queue_id) const;

  // Misc.

  void SetWakeable(TaskQueueId queue_id, fml::Wakeable* wakeable);

  // Invariants for merge and un-merge
  //  1. RegisterTask will always submit to the queue_id that is passed
  //     to it. It is not aware of whether a queue is merged or not. Same with
  //     task observers.
  //  2. When we get the tasks to run now, we look at both the queue_ids
  //     for the owner and the subsumed task queues.
  //  3. One TaskQueue can subsume multiple other TaskQueues. A TaskQueue can be
  //     in exactly one of the following three states:
  //     a. Be an owner of multiple other TaskQueues.
  //     b. Be subsumed by a TaskQueue (an owner can never be subsumed).
  //     c. Be independent, i.e, neither owner nor be subsumed.
  //
  //  Methods currently aware of the merged state of the queues:
  //  HasPendingTasks, GetNextTaskToRun, GetNumPendingTasks
  bool Merge(TaskQueueId owner, TaskQueueId subsumed);

  // Will return false if the owner has not been merged before, or owner was
  // subsumed by others, or subsumed wasn't subsumed by others, or owner
  // didn't own the given subsumed queue id.
  bool Unmerge(TaskQueueId owner, TaskQueueId subsumed);

  /// Returns \p true if \p owner owns the \p subsumed task queue.
  bool Owns(TaskQueueId owner, TaskQueueId subsumed) const;

  // Returns the subsumed task queue if any or |TaskQueueId::kUnmerged|
  // otherwise.
  std::set<TaskQueueId> GetSubsumedTaskQueueId(TaskQueueId owner) const;

  void PauseSecondarySource(TaskQueueId queue_id);

  void ResumeSecondarySource(TaskQueueId queue_id);

 private:
  class MergedQueuesRunner;

  MessageLoopTaskQueues();

  ~MessageLoopTaskQueues();

  void WakeUpUnlocked(TaskQueueId queue_id, fml::TimePoint time) const;

  bool HasPendingTasksUnlocked(TaskQueueId queue_id) const;

  TaskSource::TopTask PeekNextTaskUnlocked(TaskQueueId owner) const;

  fml::TimePoint GetNextWakeTimeUnlocked(TaskQueueId queue_id) const;

  mutable std::mutex queue_mutex_;
  std::map<TaskQueueId, std::unique_ptr<TaskQueueEntry>> queue_entries_;

  size_t task_queue_id_counter_ = 0;

  std::atomic_int order_;

  FML_DISALLOW_COPY_ASSIGN_AND_MOVE(MessageLoopTaskQueues);
};

}  // namespace fml

#endif  // FLUTTER_FML_MESSAGE_LOOP_TASK_QUEUES_H_
