// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FML_TASK_QUEUE_ID_H_
#define FLUTTER_FML_TASK_QUEUE_ID_H_

#include "flutter/fml/logging.h"

namespace fml {

/**
 * `MessageLoopTaskQueues` task dispatcher's internal task queue identifier.
 */
class TaskQueueId {
 public:
  /// This constant indicates whether a task queue has been subsumed by a task
  /// runner.
  static const size_t kUnmerged;

  /// This constant indicates an invalid task queue. Used in embedder
  /// supplied task runners not associated with a task queue.
  static const size_t kInvalid;

  /// Intializes a task queue with the given value as it's ID.
  explicit TaskQueueId(size_t value) : value_(value) {}

  operator size_t() const {  // NOLINT(google-explicit-constructor)
    return value_;
  }

  bool is_valid() const { return value_ != kInvalid; }

 private:
  size_t value_ = kUnmerged;
};

}  // namespace fml

#endif  // FLUTTER_FML_TASK_QUEUE_ID_H_
