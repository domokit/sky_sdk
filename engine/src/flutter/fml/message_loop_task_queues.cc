// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#define FML_USED_ON_EMBEDDER

#include "flutter/fml/message_loop_task_queues.h"

#include <algorithm>
#include <iostream>
#include <memory>
#include <optional>

#include "flutter/fml/make_copyable.h"
#include "flutter/fml/task_source.h"

namespace fml {

const size_t TaskQueueId::kUnmerged = ULONG_MAX;

namespace {

// iOS prior to version 9 prevents c++11 thread_local and __thread specifier,
// having us resort to boxed enum containers.
class TaskSourceGradeHolder {
 public:
  TaskSourceGrade task_source_grade;

  explicit TaskSourceGradeHolder(TaskSourceGrade task_source_grade_arg)
      : task_source_grade(task_source_grade_arg) {}
};
}  // namespace

static thread_local std::unique_ptr<TaskSourceGradeHolder>
    tls_task_source_grade;

TaskQueueEntry::TaskQueueEntry(TaskQueueId created_for_arg)
    : subsumed_by(kUnmerged), created_for(created_for_arg) {
  wakeable = NULL;
  task_observers = TaskObservers();
  task_source = std::make_unique<TaskSource>(created_for);
}

MessageLoopTaskQueues* MessageLoopTaskQueues::GetInstance() {
  static MessageLoopTaskQueues* instance = new MessageLoopTaskQueues;
  return instance;
}

TaskQueueId MessageLoopTaskQueues::CreateTaskQueue() {
  std::lock_guard guard(queue_mutex_);
  TaskQueueId loop_id = TaskQueueId(task_queue_id_counter_);
  ++task_queue_id_counter_;
  queue_entries_[loop_id] = std::make_unique<TaskQueueEntry>(loop_id);
  return loop_id;
}

MessageLoopTaskQueues::MessageLoopTaskQueues() : order_(0) {
  tls_task_source_grade.reset(
      new TaskSourceGradeHolder{TaskSourceGrade::kUnspecified});
}

MessageLoopTaskQueues::~MessageLoopTaskQueues() = default;

void MessageLoopTaskQueues::Dispose(TaskQueueId queue_id) {
  std::lock_guard guard(queue_mutex_);
  const auto& queue_entry = queue_entries_.at(queue_id);
  FML_DCHECK(queue_entry->subsumed_by == kUnmerged);
  auto& subsumed_set = queue_entry->owner_of;
  for (auto& subsumed : subsumed_set) {
    queue_entries_.erase(subsumed);
  }
  // Erase owner queue_id at last to avoid &subsumed_set from being invalid
  queue_entries_.erase(queue_id);
}

void MessageLoopTaskQueues::DisposeTasks(TaskQueueId queue_id) {
  std::lock_guard guard(queue_mutex_);
  const auto& queue_entry = queue_entries_.at(queue_id);
  FML_DCHECK(queue_entry->subsumed_by == kUnmerged);
  auto& subsumed_set = queue_entry->owner_of;
  queue_entry->task_source->ShutDown();
  for (auto& subsumed : subsumed_set) {
    queue_entries_.at(subsumed)->task_source->ShutDown();
  }
}

TaskSourceGrade MessageLoopTaskQueues::GetCurrentTaskSourceGrade() {
  return tls_task_source_grade.get()->task_source_grade;
}

void MessageLoopTaskQueues::RegisterTask(
    TaskQueueId queue_id,
    const fml::closure& task,
    fml::TimePoint target_time,
    fml::TaskSourceGrade task_source_grade) {
  std::lock_guard guard(queue_mutex_);
  size_t order = order_++;
  const auto& queue_entry = queue_entries_.at(queue_id);
  queue_entry->task_source->RegisterTask(
      {order, task, target_time, task_source_grade});
  TaskQueueId loop_to_wake = queue_id;
  if (queue_entry->subsumed_by != kUnmerged) {
    loop_to_wake = queue_entry->subsumed_by;
  }

  // This can happen when the secondary tasks are paused.
  if (HasPendingTasksUnlocked(loop_to_wake)) {
    WakeUpUnlocked(loop_to_wake, GetNextWakeTimeUnlocked(loop_to_wake));
  }
}

bool MessageLoopTaskQueues::HasPendingTasks(TaskQueueId queue_id) const {
  std::lock_guard guard(queue_mutex_);
  return HasPendingTasksUnlocked(queue_id);
}

fml::closure MessageLoopTaskQueues::GetNextTaskToRun(TaskQueueId queue_id,
                                                     fml::TimePoint from_time) {
  std::lock_guard guard(queue_mutex_);
  if (!HasPendingTasksUnlocked(queue_id)) {
    return nullptr;
  }
  TaskSource::TopTask top = PeekNextTaskUnlocked(queue_id);

  if (!HasPendingTasksUnlocked(queue_id)) {
    WakeUpUnlocked(queue_id, fml::TimePoint::Max());
  } else {
    WakeUpUnlocked(queue_id, GetNextWakeTimeUnlocked(queue_id));
  }

  if (top.task.GetTargetTime() > from_time) {
    return nullptr;
  }
  fml::closure invocation = top.task.GetTask();
  const auto task_source_grade = top.task.GetTaskSourceGrade();
  queue_entries_.at(top.task_queue_id)->task_source->PopTask(task_source_grade);
  tls_task_source_grade.reset(new TaskSourceGradeHolder{task_source_grade});
  return invocation;
}

void MessageLoopTaskQueues::WakeUpUnlocked(TaskQueueId queue_id,
                                           fml::TimePoint time) const {
  if (queue_entries_.at(queue_id)->wakeable) {
    queue_entries_.at(queue_id)->wakeable->WakeUp(time);
  }
}

size_t MessageLoopTaskQueues::GetNumPendingTasks(TaskQueueId queue_id) const {
  std::lock_guard guard(queue_mutex_);
  const auto& queue_entry = queue_entries_.at(queue_id);
  if (queue_entry->subsumed_by != kUnmerged) {
    return 0;
  }

  size_t total_tasks = 0;
  total_tasks += queue_entry->task_source->GetNumPendingTasks();

  auto& subsumed_set = queue_entry->owner_of;
  for (auto& subsumed : subsumed_set) {
    const auto& subsumed_entry = queue_entries_.at(subsumed);
    total_tasks += subsumed_entry->task_source->GetNumPendingTasks();
  }
  return total_tasks;
}

void MessageLoopTaskQueues::AddTaskObserver(TaskQueueId queue_id,
                                            intptr_t key,
                                            const fml::closure& callback) {
  std::lock_guard guard(queue_mutex_);
  FML_DCHECK(callback != nullptr) << "Observer callback must be non-null.";
  queue_entries_.at(queue_id)->task_observers[key] = callback;
}

void MessageLoopTaskQueues::RemoveTaskObserver(TaskQueueId queue_id,
                                               intptr_t key) {
  std::lock_guard guard(queue_mutex_);
  queue_entries_.at(queue_id)->task_observers.erase(key);
}

std::vector<fml::closure> MessageLoopTaskQueues::GetObserversToNotify(
    TaskQueueId queue_id) const {
  std::lock_guard guard(queue_mutex_);
  std::vector<fml::closure> observers;

  if (queue_entries_.at(queue_id)->subsumed_by != kUnmerged) {
    return observers;
  }

  for (const auto& observer : queue_entries_.at(queue_id)->task_observers) {
    observers.push_back(observer.second);
  }

  auto& subsumed_set = queue_entries_.at(queue_id)->owner_of;
  for (auto& subsumed : subsumed_set) {
    for (const auto& observer : queue_entries_.at(subsumed)->task_observers) {
      observers.push_back(observer.second);
    }
  }

  return observers;
}

void MessageLoopTaskQueues::SetWakeable(TaskQueueId queue_id,
                                        fml::Wakeable* wakeable) {
  std::lock_guard guard(queue_mutex_);
  FML_CHECK(!queue_entries_.at(queue_id)->wakeable)
      << "Wakeable can only be set once.";
  queue_entries_.at(queue_id)->wakeable = wakeable;
}

bool MessageLoopTaskQueues::Merge(TaskQueueId owner, TaskQueueId subsumed) {
  if (owner == subsumed) {
    return true;
  }
  std::lock_guard guard(queue_mutex_);
  auto& owner_entry = queue_entries_.at(owner);
  auto& subsumed_entry = queue_entries_.at(subsumed);
  auto& subsumed_set = owner_entry->owner_of;
  if (subsumed_set.find(subsumed) != subsumed_set.end()) {
    return true;
  }

  // Won't check owner_entry->owner_of, because it may contains items when
  // merged with other different queues.

  // Ensure owner_entry->subsumed_by being kUnmerged
  if (owner_entry->subsumed_by != kUnmerged) {
    FML_LOG(WARNING) << "Thread merging failed: owner_entry was already "
                        "subsumed by others, owner="
                     << owner << ", subsumed=" << subsumed
                     << ", owner->subsumed_by=" << owner_entry->subsumed_by;
    return false;
  }
  // Ensure subsumed_entry->owner_of being empty
  if (!subsumed_entry->owner_of.empty()) {
    FML_LOG(WARNING)
        << "Thread merging failed: subsumed_entry already owns others, owner="
        << owner << ", subsumed=" << subsumed
        << ", subsumed->owner_of.size()=" << subsumed_entry->owner_of.size();
    return false;
  }
  // Ensure subsumed_entry->subsumed_by being kUnmerged
  if (subsumed_entry->subsumed_by != kUnmerged) {
    FML_LOG(WARNING) << "Thread merging failed: subsumed_entry was already "
                        "subsumed by others, owner="
                     << owner << ", subsumed=" << subsumed
                     << ", subsumed->subsumed_by="
                     << subsumed_entry->subsumed_by;
    return false;
  }
  // All checking is OK, set merged state.
  owner_entry->owner_of.insert(subsumed);
  subsumed_entry->subsumed_by = owner;

  if (HasPendingTasksUnlocked(owner)) {
    WakeUpUnlocked(owner, GetNextWakeTimeUnlocked(owner));
  }

  return true;
}

bool MessageLoopTaskQueues::Unmerge(TaskQueueId owner, TaskQueueId subsumed) {
  std::lock_guard guard(queue_mutex_);
  const auto& owner_entry = queue_entries_.at(owner);
  if (owner_entry->owner_of.empty()) {
    FML_LOG(WARNING)
        << "Thread unmerging failed: owner_entry doesn't own anyone, owner="
        << owner << ", subsumed=" << subsumed;
    return false;
  }
  if (owner_entry->subsumed_by != kUnmerged) {
    FML_LOG(WARNING)
        << "Thread unmerging failed: owner_entry was subsumed by others, owner="
        << owner << ", subsumed=" << subsumed
        << ", owner_entry->subsumed_by=" << owner_entry->subsumed_by;
    return false;
  }
  if (queue_entries_.at(subsumed)->subsumed_by == kUnmerged) {
    FML_LOG(WARNING) << "Thread unmerging failed: subsumed_entry wasn't "
                        "subsumed by others, owner="
                     << owner << ", subsumed=" << subsumed;
    return false;
  }
  if (owner_entry->owner_of.find(subsumed) == owner_entry->owner_of.end()) {
    FML_LOG(WARNING) << "Thread unmerging failed: owner_entry didn't own the "
                        "given subsumed queue id, owner="
                     << owner << ", subsumed=" << subsumed;
    return false;
  }

  queue_entries_.at(subsumed)->subsumed_by = kUnmerged;
  owner_entry->owner_of.erase(subsumed);

  if (HasPendingTasksUnlocked(owner)) {
    WakeUpUnlocked(owner, GetNextWakeTimeUnlocked(owner));
  }

  if (HasPendingTasksUnlocked(subsumed)) {
    WakeUpUnlocked(subsumed, GetNextWakeTimeUnlocked(subsumed));
  }

  return true;
}

bool MessageLoopTaskQueues::Owns(TaskQueueId owner,
                                 TaskQueueId subsumed) const {
  std::lock_guard guard(queue_mutex_);
  if (owner == kUnmerged || subsumed == kUnmerged) {
    return false;
  }
  auto& subsumed_set = queue_entries_.at(owner)->owner_of;
  return subsumed_set.find(subsumed) != subsumed_set.end();
}

std::set<TaskQueueId> MessageLoopTaskQueues::GetSubsumedTaskQueueId(
    TaskQueueId owner) const {
  std::lock_guard guard(queue_mutex_);
  return queue_entries_.at(owner)->owner_of;
}

void MessageLoopTaskQueues::PauseSecondarySource(TaskQueueId queue_id) {
  std::lock_guard guard(queue_mutex_);
  queue_entries_.at(queue_id)->task_source->PauseSecondary();
}

void MessageLoopTaskQueues::ResumeSecondarySource(TaskQueueId queue_id) {
  std::lock_guard guard(queue_mutex_);
  queue_entries_.at(queue_id)->task_source->ResumeSecondary();
  // Schedule a wake as needed.
  if (HasPendingTasksUnlocked(queue_id)) {
    WakeUpUnlocked(queue_id, GetNextWakeTimeUnlocked(queue_id));
  }
}

// Subsumed queues will never have pending tasks.
// Owning queues will consider both their and their subsumed tasks.
bool MessageLoopTaskQueues::HasPendingTasksUnlocked(
    TaskQueueId queue_id) const {
  const auto& entry = queue_entries_.at(queue_id);
  bool is_subsumed = entry->subsumed_by != kUnmerged;
  if (is_subsumed) {
    return false;
  }

  if (!entry->task_source->IsEmpty()) {
    return true;
  }

  auto& subsumed_set = entry->owner_of;
  return std::any_of(
      subsumed_set.begin(), subsumed_set.end(), [&](const auto& subsumed) {
        return !queue_entries_.at(subsumed)->task_source->IsEmpty();
      });
}

fml::TimePoint MessageLoopTaskQueues::GetNextWakeTimeUnlocked(
    TaskQueueId queue_id) const {
  return PeekNextTaskUnlocked(queue_id).task.GetTargetTime();
}

TaskSource::TopTask MessageLoopTaskQueues::PeekNextTaskUnlocked(
    TaskQueueId owner) const {
  FML_DCHECK(HasPendingTasksUnlocked(owner));
  const auto& entry = queue_entries_.at(owner);
  if (entry->owner_of.empty()) {
    FML_CHECK(!entry->task_source->IsEmpty());
    return entry->task_source->Top();
  }

  // Use optional for the memory of TopTask object.
  std::optional<TaskSource::TopTask> top_task;

  std::function<void(const TaskSource*)> top_task_updater =
      [&top_task](const TaskSource* source) {
        if (source && !source->IsEmpty()) {
          TaskSource::TopTask other_task = source->Top();
          if (!top_task.has_value() || top_task->task > other_task.task) {
            top_task.emplace(other_task);
          }
        }
      };

  TaskSource* owner_tasks = entry->task_source.get();
  top_task_updater(owner_tasks);

  for (TaskQueueId subsumed : entry->owner_of) {
    TaskSource* subsumed_tasks = queue_entries_.at(subsumed)->task_source.get();
    top_task_updater(subsumed_tasks);
  }
  // At least one task at the top because PeekNextTaskUnlocked() is called after
  // HasPendingTasksUnlocked()
  FML_CHECK(top_task.has_value());
  // Covered by FML_CHECK.
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access)
  return top_task.value();
}

}  // namespace fml
