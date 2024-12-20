// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <atomic>
#include <thread>

#include "flutter/fml/macros.h"
#include "flutter/fml/task_source.h"
#include "flutter/fml/time/chrono_timestamp_provider.h"
#include "flutter/fml/time/time_delta.h"
#include "flutter/fml/time/time_point.h"
#include "gtest/gtest.h"

namespace fml {
namespace testing {

TEST(TaskSourceTests, SimpleInitialization) {
  TaskSource task_source = TaskSource(TaskQueueId(1));
  task_source.RegisterTask(
      {1, [] {}, ChronoTicksSinceEpoch(), TaskSourceGrade::kUnspecified});
  ASSERT_EQ(task_source.GetNumPendingTasks(), 1u);
}

TEST(TaskSourceTests, MultipleTaskGrades) {
  TaskSource task_source = TaskSource(TaskQueueId(1));
  task_source.RegisterTask(
      {1, [] {}, ChronoTicksSinceEpoch(), TaskSourceGrade::kUnspecified});
  task_source.RegisterTask(
      {2, [] {}, ChronoTicksSinceEpoch(), TaskSourceGrade::kUserInteraction});
  task_source.RegisterTask(
      {3, [] {}, ChronoTicksSinceEpoch(), TaskSourceGrade::kDartEventLoop});
  ASSERT_EQ(task_source.GetNumPendingTasks(), 3u);
}

TEST(TaskSourceTests, SimpleOrdering) {
  TaskSource task_source = TaskSource(TaskQueueId(1));
  auto time_stamp = ChronoTicksSinceEpoch();
  int value = 0;
  task_source.RegisterTask(
      {1, [&] { value = 1; }, time_stamp, TaskSourceGrade::kUnspecified});
  task_source.RegisterTask({2, [&] { value = 7; },
                            time_stamp + fml::TimeDelta::FromMilliseconds(1),
                            TaskSourceGrade::kUnspecified});
  task_source.Top().task.GetTask()();
  task_source.PopTask(TaskSourceGrade::kUnspecified);
  ASSERT_EQ(value, 1);
  task_source.Top().task.GetTask()();
  task_source.PopTask(TaskSourceGrade::kUnspecified);
  ASSERT_EQ(value, 7);
}

TEST(TaskSourceTests, SimpleOrderingMultiTaskHeaps) {
  TaskSource task_source = TaskSource(TaskQueueId(1));
  auto time_stamp = ChronoTicksSinceEpoch();
  int value = 0;
  task_source.RegisterTask(
      {1, [&] { value = 1; }, time_stamp, TaskSourceGrade::kDartEventLoop});
  task_source.RegisterTask({2, [&] { value = 7; },
                            time_stamp + fml::TimeDelta::FromMilliseconds(1),
                            TaskSourceGrade::kUserInteraction});
  auto top_task = task_source.Top();
  top_task.task.GetTask()();
  task_source.PopTask(top_task.task.GetTaskSourceGrade());
  ASSERT_EQ(value, 1);

  auto second_task = task_source.Top();
  second_task.task.GetTask()();
  task_source.PopTask(second_task.task.GetTaskSourceGrade());
  ASSERT_EQ(value, 7);
}

TEST(TaskSourceTests, OrderingMultiTaskHeapsSecondaryPaused) {
  TaskSource task_source = TaskSource(TaskQueueId(1));
  auto time_stamp = ChronoTicksSinceEpoch();
  int value = 0;
  task_source.RegisterTask(
      {1, [&] { value = 1; }, time_stamp, TaskSourceGrade::kDartEventLoop});
  task_source.RegisterTask({2, [&] { value = 7; },
                            time_stamp + fml::TimeDelta::FromMilliseconds(1),
                            TaskSourceGrade::kUserInteraction});

  task_source.PauseSecondary();

  auto top_task = task_source.Top();
  top_task.task.GetTask()();
  task_source.PopTask(top_task.task.GetTaskSourceGrade());
  ASSERT_EQ(value, 7);

  ASSERT_TRUE(task_source.IsEmpty());

  task_source.ResumeSecondary();

  auto second_task = task_source.Top();
  second_task.task.GetTask()();
  task_source.PopTask(second_task.task.GetTaskSourceGrade());
  ASSERT_EQ(value, 1);
}

}  // namespace testing
}  // namespace fml
