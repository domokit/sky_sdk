// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FML_TASK_RUNNER_H_
#define FLUTTER_FML_TASK_RUNNER_H_

#include "flutter/fml/closure.h"
#include "flutter/fml/macros.h"
#include "flutter/fml/memory/ref_counted.h"
#include "flutter/fml/memory/ref_ptr.h"
#include "flutter/fml/message_loop_task_queues.h"
#include "flutter/fml/time/time_point.h"

namespace fml {

class MessageLoopImpl;

/// An interface over the ability to schedule tasks on a \p TaskRunner.
class BasicTaskRunner {
 public:
  /// Schedules \p task to be executed on the TaskRunner's associated event
  /// loop.
  virtual void PostTask(const fml::closure& task) = 0;
};

/// The object for scheduling tasks on a \p fml::MessageLoop.
///
/// Typically there is one \p TaskRunner associated with each thread.  When one
/// wants to execute an operation on that thread they post a task to the
/// TaskRunner.
///
/// \see fml::MessageLoop
class TaskRunner : public fml::RefCountedThreadSafe<TaskRunner>,
                   public BasicTaskRunner {
 public:
  virtual ~TaskRunner();

  virtual void PostTask(const fml::closure& task) override;

  virtual void PostTaskForTime(const fml::closure& task,
                               fml::TimePoint target_time);

  /// Schedules a task to be run on the MessageLoop after the time \p delay has
  /// passed.
  /// \note There is latency between when the task is schedule and actually
  /// executed so that the actual execution time is: now + delay +
  /// message_loop_latency, where message_loop_latency is undefined and could be
  /// tens of milliseconds.
  virtual void PostDelayedTask(const fml::closure& task, fml::TimeDelta delay);

  /// Returns \p true when the current executing thread's TaskRunner matches
  /// this instance.
  virtual bool RunsTasksOnCurrentThread();

  /// Returns the unique identifier associated with the TaskRunner.
  /// \see fml::MessageLoopTaskQueues
  ///
  /// Will be TaskQueueId::kInvalid for embedder supplied task runners
  /// that are not associated with a task queue.
  virtual TaskQueueId GetTaskQueueId();

  /// Executes the \p task directly if the TaskRunner \p runner is the
  /// TaskRunner associated with the current executing thread.
  static void RunNowOrPostTask(const fml::RefPtr<fml::TaskRunner>& runner,
                               const fml::closure& task);

  /// Like RunNowOrPostTask, except that if the task can be immediately
  /// executed, an empty task will still be posted to the runner afterwards.
  ///
  /// This is used to ensure that messages posted to Dart from the platform
  /// thread always flush the Dart event loop.
  static void RunNowAndFlushMessages(const fml::RefPtr<fml::TaskRunner>& runner,
                                     const fml::closure& task);

 protected:
  explicit TaskRunner(fml::RefPtr<MessageLoopImpl> loop);

 private:
  fml::RefPtr<MessageLoopImpl> loop_;

  FML_FRIEND_MAKE_REF_COUNTED(TaskRunner);
  FML_FRIEND_REF_COUNTED_THREAD_SAFE(TaskRunner);
  FML_DISALLOW_COPY_AND_ASSIGN(TaskRunner);
};

}  // namespace fml

#endif  // FLUTTER_FML_TASK_RUNNER_H_
