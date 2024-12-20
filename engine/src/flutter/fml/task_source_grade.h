// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FML_TASK_SOURCE_GRADE_H_
#define FLUTTER_FML_TASK_SOURCE_GRADE_H_

namespace fml {

/**
 * Categories of work dispatched to `MessageLoopTaskQueues` dispatcher. By
 * specifying the `TaskSourceGrade`, you indicate the task's importance to the
 * dispatcher.
 */
enum class TaskSourceGrade {
  /// This `TaskSourceGrade` indicates that a task is critical to user
  /// interaction.
  kUserInteraction,
  /// This `TaskSourceGrade` indicates that a task corresponds to servicing a
  /// dart event loop task. These aren't critical to user interaction.
  kDartEventLoop,
  /// The absence of a specialized `TaskSourceGrade`.
  kUnspecified,
};

}  // namespace fml

#endif  // FLUTTER_FML_TASK_SOURCE_GRADE_H_
