// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FML_MESSAGE_LOOP_IMPL_H_
#define FLUTTER_FML_MESSAGE_LOOP_IMPL_H_

#include <atomic>
#include <deque>
#include <map>
#include <mutex>
#include <queue>
#include <utility>

#include "flutter/fml/closure.h"
#include "flutter/fml/delayed_task.h"
#include "flutter/fml/macros.h"
#include "flutter/fml/memory/ref_counted.h"
#include "flutter/fml/message_loop.h"
#include "flutter/fml/message_loop_task_queues.h"
#include "flutter/fml/time/time_point.h"
#include "flutter/fml/wakeable.h"

namespace fml {

/// An abstract class that represents the differences in implementation of a \p
/// fml::MessageLoop depending on the platform.
/// \see fml::MessageLoop
/// \see fml::MessageLoopAndroid
/// \see fml::MessageLoopDarwin
class MessageLoopImpl : public Wakeable,
                        public fml::RefCountedThreadSafe<MessageLoopImpl> {
 public:
  static fml::RefPtr<MessageLoopImpl> Create();

  virtual ~MessageLoopImpl();

  virtual void Run() = 0;

  virtual void Terminate() = 0;

  void PostTask(const fml::closure& task, fml::TimePoint target_time);

  void AddTaskObserver(intptr_t key, const fml::closure& callback);

  void RemoveTaskObserver(intptr_t key);

  void DoRun();

  void DoTerminate();

  virtual TaskQueueId GetTaskQueueId() const;

 protected:
  // Exposed for the embedder shell which allows clients to poll for events
  // instead of dedicating a thread to the message loop.
  friend class MessageLoop;

  void RunExpiredTasksNow();

  void RunSingleExpiredTaskNow();

 protected:
  MessageLoopImpl();

 private:
  fml::MessageLoopTaskQueues* task_queue_;
  TaskQueueId queue_id_;

  std::atomic_bool terminated_;

  void FlushTasks(FlushType type);

  FML_DISALLOW_COPY_AND_ASSIGN(MessageLoopImpl);
};

}  // namespace fml

#endif  // FLUTTER_FML_MESSAGE_LOOP_IMPL_H_
