// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#define FML_USED_ON_EMBEDDER

#include "flutter/fml/message_loop_task_queues.h"

#include <algorithm>
#include <cstdlib>
#include <thread>
#include <utility>

#include "flutter/fml/synchronization/count_down_latch.h"
#include "flutter/fml/synchronization/waitable_event.h"
#include "flutter/fml/time/chrono_timestamp_provider.h"
#include "gtest/gtest.h"

namespace fml {
namespace testing {

class TestWakeable : public fml::Wakeable {
 public:
  using WakeUpCall = std::function<void(const fml::TimePoint)>;

  explicit TestWakeable(WakeUpCall call) : wake_up_call_(std::move(call)) {}

  void WakeUp(fml::TimePoint time_point) override { wake_up_call_(time_point); }

 private:
  WakeUpCall wake_up_call_;
};

TEST(MessageLoopTaskQueue, StartsWithNoPendingTasks) {
  auto task_queue = fml::MessageLoopTaskQueues::GetInstance();
  auto queue_id = task_queue->CreateTaskQueue();
  ASSERT_FALSE(task_queue->HasPendingTasks(queue_id));
}

TEST(MessageLoopTaskQueue, RegisterOneTask) {
  const auto time = fml::TimePoint::Max();

  auto task_queue = fml::MessageLoopTaskQueues::GetInstance();
  auto queue_id = task_queue->CreateTaskQueue();
  auto wakeable = std::make_unique<TestWakeable>(
      [&time](fml::TimePoint wake_time) { ASSERT_TRUE(wake_time == time); });
  task_queue->SetWakeable(queue_id, wakeable.get());

  task_queue->RegisterTask(queue_id, [] {}, time);
  ASSERT_TRUE(task_queue->HasPendingTasks(queue_id));
  ASSERT_TRUE(task_queue->GetNumPendingTasks(queue_id) == 1);
}

TEST(MessageLoopTaskQueue, RegisterTwoTasksAndCount) {
  auto task_queue = fml::MessageLoopTaskQueues::GetInstance();
  auto queue_id = task_queue->CreateTaskQueue();
  task_queue->RegisterTask(queue_id, [] {}, ChronoTicksSinceEpoch());
  task_queue->RegisterTask(queue_id, [] {}, fml::TimePoint::Max());
  ASSERT_TRUE(task_queue->HasPendingTasks(queue_id));
  ASSERT_TRUE(task_queue->GetNumPendingTasks(queue_id) == 2);
}

TEST(MessageLoopTaskQueue, RegisterTasksOnMergedQueuesAndCount) {
  auto task_queue = fml::MessageLoopTaskQueues::GetInstance();
  auto platform_queue = task_queue->CreateTaskQueue();
  auto raster_queue = task_queue->CreateTaskQueue();
  // A task in platform_queue
  task_queue->RegisterTask(platform_queue, []() {}, fml::TimePoint::Now());
  // A task in raster_queue
  task_queue->RegisterTask(raster_queue, []() {}, fml::TimePoint::Now());
  ASSERT_TRUE(task_queue->GetNumPendingTasks(platform_queue) == 1);
  ASSERT_TRUE(task_queue->GetNumPendingTasks(raster_queue) == 1);

  ASSERT_FALSE(task_queue->Owns(platform_queue, raster_queue));
  task_queue->Merge(platform_queue, raster_queue);
  ASSERT_TRUE(task_queue->Owns(platform_queue, raster_queue));

  ASSERT_TRUE(task_queue->HasPendingTasks(platform_queue));
  ASSERT_TRUE(task_queue->GetNumPendingTasks(platform_queue) == 2);
  // The task count of subsumed queue is 0
  ASSERT_FALSE(task_queue->HasPendingTasks(raster_queue));
  ASSERT_TRUE(task_queue->GetNumPendingTasks(raster_queue) == 0);

  task_queue->Unmerge(platform_queue, raster_queue);
  ASSERT_FALSE(task_queue->Owns(platform_queue, raster_queue));
  ASSERT_TRUE(task_queue->GetNumPendingTasks(platform_queue) == 1);
  ASSERT_TRUE(task_queue->GetNumPendingTasks(raster_queue) == 1);
}

TEST(MessageLoopTaskQueue, PreserveTaskOrdering) {
  auto task_queue = fml::MessageLoopTaskQueues::GetInstance();
  auto queue_id = task_queue->CreateTaskQueue();
  int test_val = 0;

  // order: 0
  task_queue->RegisterTask(
      queue_id, [&test_val]() { test_val = 1; }, ChronoTicksSinceEpoch());

  // order: 1
  task_queue->RegisterTask(
      queue_id, [&test_val]() { test_val = 2; }, ChronoTicksSinceEpoch());

  const auto now = ChronoTicksSinceEpoch();
  int expected_value = 1;
  while (true) {
    fml::closure invocation = task_queue->GetNextTaskToRun(queue_id, now);
    if (!invocation) {
      break;
    }
    invocation();
    ASSERT_TRUE(test_val == expected_value);
    expected_value++;
  }
}

TEST(MessageLoopTaskQueue, RegisterTasksOnMergedQueuesPreserveTaskOrdering) {
  auto task_queue = fml::MessageLoopTaskQueues::GetInstance();
  auto platform_queue = task_queue->CreateTaskQueue();
  auto raster1_queue = task_queue->CreateTaskQueue();
  auto raster2_queue = task_queue->CreateTaskQueue();
  int test_val = 0;

  // order 0 in raster1_queue
  task_queue->RegisterTask(
      raster1_queue, [&test_val]() { test_val = 0; }, fml::TimePoint::Now());

  // order 1 in platform_queue
  task_queue->RegisterTask(
      platform_queue, [&test_val]() { test_val = 1; }, fml::TimePoint::Now());

  // order 2 in raster2_queue
  task_queue->RegisterTask(
      raster2_queue, [&test_val]() { test_val = 2; }, fml::TimePoint::Now());

  task_queue->Merge(platform_queue, raster1_queue);
  ASSERT_TRUE(task_queue->Owns(platform_queue, raster1_queue));
  task_queue->Merge(platform_queue, raster2_queue);
  ASSERT_TRUE(task_queue->Owns(platform_queue, raster2_queue));
  const auto now = fml::TimePoint::Now();
  int expected_value = 0;
  // Right order:
  // "test_val = 0" in raster1_queue
  // "test_val = 1" in platform_queue
  // "test_val = 2" in raster2_queue
  while (true) {
    fml::closure invocation = task_queue->GetNextTaskToRun(platform_queue, now);
    if (!invocation) {
      break;
    }
    invocation();
    ASSERT_TRUE(test_val == expected_value);
    expected_value++;
  }
}

TEST(MessageLoopTaskQueue, UnmergeRespectTheOriginalTaskOrderingInQueues) {
  auto task_queue = fml::MessageLoopTaskQueues::GetInstance();
  auto platform_queue = task_queue->CreateTaskQueue();
  auto raster_queue = task_queue->CreateTaskQueue();
  int test_val = 0;

  // order 0 in platform_queue
  task_queue->RegisterTask(
      platform_queue, [&test_val]() { test_val = 0; }, fml::TimePoint::Now());
  // order 1 in platform_queue
  task_queue->RegisterTask(
      platform_queue, [&test_val]() { test_val = 1; }, fml::TimePoint::Now());
  // order 2 in raster_queue
  task_queue->RegisterTask(
      raster_queue, [&test_val]() { test_val = 2; }, fml::TimePoint::Now());
  // order 3 in raster_queue
  task_queue->RegisterTask(
      raster_queue, [&test_val]() { test_val = 3; }, fml::TimePoint::Now());
  // order 4 in platform_queue
  task_queue->RegisterTask(
      platform_queue, [&test_val]() { test_val = 4; }, fml::TimePoint::Now());
  // order 5 in raster_queue
  task_queue->RegisterTask(
      raster_queue, [&test_val]() { test_val = 5; }, fml::TimePoint::Now());

  ASSERT_TRUE(task_queue->Merge(platform_queue, raster_queue));
  ASSERT_TRUE(task_queue->Owns(platform_queue, raster_queue));
  const auto now = fml::TimePoint::Now();
  // The right order after merged and consumed 3 tasks:
  // "test_val = 0" in platform_queue
  // "test_val = 1" in platform_queue
  // "test_val = 2" in raster_queue (running on platform)
  for (int i = 0; i < 3; i++) {
    fml::closure invocation = task_queue->GetNextTaskToRun(platform_queue, now);
    ASSERT_FALSE(!invocation);
    invocation();
    ASSERT_TRUE(test_val == i);
  }
  ASSERT_TRUE(task_queue->GetNumPendingTasks(platform_queue) == 3);
  ASSERT_TRUE(task_queue->GetNumPendingTasks(raster_queue) == 0);

  ASSERT_TRUE(task_queue->Unmerge(platform_queue, raster_queue));
  ASSERT_FALSE(task_queue->Owns(platform_queue, raster_queue));

  // The right order after unmerged and left 3 tasks:
  // "test_val = 3" in raster_queue
  // "test_val = 4" in platform_queue
  // "test_val = 5" in raster_queue

  // platform_queue has 1 task left: "test_val = 4"
  {
    ASSERT_TRUE(task_queue->GetNumPendingTasks(platform_queue) == 1);
    fml::closure invocation = task_queue->GetNextTaskToRun(platform_queue, now);
    ASSERT_FALSE(!invocation);
    invocation();
    ASSERT_TRUE(test_val == 4);
    ASSERT_TRUE(task_queue->GetNumPendingTasks(platform_queue) == 0);
  }

  // raster_queue has 2 tasks left: "test_val = 3" and "test_val = 5"
  {
    ASSERT_TRUE(task_queue->GetNumPendingTasks(raster_queue) == 2);
    fml::closure invocation = task_queue->GetNextTaskToRun(raster_queue, now);
    ASSERT_FALSE(!invocation);
    invocation();
    ASSERT_TRUE(test_val == 3);
  }
  {
    ASSERT_TRUE(task_queue->GetNumPendingTasks(raster_queue) == 1);
    fml::closure invocation = task_queue->GetNextTaskToRun(raster_queue, now);
    ASSERT_FALSE(!invocation);
    invocation();
    ASSERT_TRUE(test_val == 5);
    ASSERT_TRUE(task_queue->GetNumPendingTasks(raster_queue) == 0);
  }
}

void TestNotifyObservers(fml::TaskQueueId queue_id) {
  auto task_queue = fml::MessageLoopTaskQueues::GetInstance();
  std::vector<fml::closure> observers =
      task_queue->GetObserversToNotify(queue_id);
  for (const auto& observer : observers) {
    observer();
  }
}

TEST(MessageLoopTaskQueue, AddRemoveNotifyObservers) {
  auto task_queue = fml::MessageLoopTaskQueues::GetInstance();
  auto queue_id = task_queue->CreateTaskQueue();

  int test_val = 0;
  intptr_t key = 123;

  task_queue->AddTaskObserver(queue_id, key, [&test_val]() { test_val = 1; });
  TestNotifyObservers(queue_id);
  ASSERT_TRUE(test_val == 1);

  test_val = 0;
  task_queue->RemoveTaskObserver(queue_id, key);
  TestNotifyObservers(queue_id);
  ASSERT_TRUE(test_val == 0);
}

TEST(MessageLoopTaskQueue, WakeUpIndependentOfTime) {
  auto task_queue = fml::MessageLoopTaskQueues::GetInstance();
  auto queue_id = task_queue->CreateTaskQueue();

  int num_wakes = 0;
  auto wakeable = std::make_unique<TestWakeable>(
      [&num_wakes](fml::TimePoint wake_time) { ++num_wakes; });
  task_queue->SetWakeable(queue_id, wakeable.get());

  task_queue->RegisterTask(queue_id, []() {}, ChronoTicksSinceEpoch());
  task_queue->RegisterTask(queue_id, []() {}, fml::TimePoint::Max());

  ASSERT_TRUE(num_wakes == 2);
}

TEST(MessageLoopTaskQueue, WokenUpWithNewerTime) {
  auto task_queue = fml::MessageLoopTaskQueues::GetInstance();
  auto queue_id = task_queue->CreateTaskQueue();
  fml::CountDownLatch latch(2);

  fml::TimePoint expected = fml::TimePoint::Max();

  auto wakeable = std::make_unique<TestWakeable>(
      [&latch, &expected](fml::TimePoint wake_time) {
        ASSERT_TRUE(wake_time == expected);
        latch.CountDown();
      });

  task_queue->SetWakeable(queue_id, wakeable.get());

  task_queue->RegisterTask(queue_id, []() {}, fml::TimePoint::Max());

  const auto now = ChronoTicksSinceEpoch();
  expected = now;
  task_queue->RegisterTask(queue_id, []() {}, now);

  latch.Wait();
}

TEST(MessageLoopTaskQueue, NotifyObserversWhileCreatingQueues) {
  auto task_queues = fml::MessageLoopTaskQueues::GetInstance();
  fml::TaskQueueId queue_id = task_queues->CreateTaskQueue();
  fml::AutoResetWaitableEvent first_observer_executing, before_second_observer;

  task_queues->AddTaskObserver(queue_id, queue_id + 1, [&]() {
    first_observer_executing.Signal();
    before_second_observer.Wait();
  });

  for (int i = 0; i < 100; i++) {
    task_queues->AddTaskObserver(queue_id, queue_id + i + 2, [] {});
  }

  std::thread notify_observers([&]() { TestNotifyObservers(queue_id); });

  first_observer_executing.Wait();

  for (int i = 0; i < 100; i++) {
    task_queues->CreateTaskQueue();
  }

  before_second_observer.Signal();
  notify_observers.join();
}

TEST(MessageLoopTaskQueue, QueueDoNotOwnItself) {
  auto task_queue = fml::MessageLoopTaskQueues::GetInstance();
  auto queue_id = task_queue->CreateTaskQueue();
  ASSERT_FALSE(task_queue->Owns(queue_id, queue_id));
}

TEST(MessageLoopTaskQueue, QueueDoNotOwnUnmergedTaskQueueId) {
  auto task_queue = fml::MessageLoopTaskQueues::GetInstance();
  ASSERT_FALSE(task_queue->Owns(task_queue->CreateTaskQueue(), kUnmerged));
  ASSERT_FALSE(task_queue->Owns(kUnmerged, task_queue->CreateTaskQueue()));
  ASSERT_FALSE(task_queue->Owns(kUnmerged, kUnmerged));
}

TEST(MessageLoopTaskQueue, QueueOwnsMergedTaskQueueId) {
  auto task_queue = fml::MessageLoopTaskQueues::GetInstance();
  auto platform_queue = task_queue->CreateTaskQueue();
  auto raster_queue = task_queue->CreateTaskQueue();
  ASSERT_FALSE(task_queue->Owns(platform_queue, raster_queue));
  ASSERT_FALSE(task_queue->Owns(raster_queue, platform_queue));
  task_queue->Merge(platform_queue, raster_queue);
  ASSERT_TRUE(task_queue->Owns(platform_queue, raster_queue));
  ASSERT_FALSE(task_queue->Owns(raster_queue, platform_queue));
}

//------------------------------------------------------------------------------
/// Verifies that tasks can be added to task queues concurrently.
///
TEST(MessageLoopTaskQueue, ConcurrentQueueAndTaskCreatingCounts) {
  auto task_queues = fml::MessageLoopTaskQueues::GetInstance();

  // kThreadCount threads post kThreadTaskCount tasks each to kTaskQueuesCount
  // task queues. Each thread picks a task queue randomly for each task.
  constexpr size_t kThreadCount = 4;
  constexpr size_t kTaskQueuesCount = 2;
  constexpr size_t kThreadTaskCount = 500;

  std::vector<TaskQueueId> task_queue_ids;
  for (size_t i = 0; i < kTaskQueuesCount; ++i) {
    task_queue_ids.emplace_back(task_queues->CreateTaskQueue());
  }

  ASSERT_EQ(task_queue_ids.size(), kTaskQueuesCount);

  fml::CountDownLatch tasks_posted_latch(kThreadCount);

  auto thread_main = [&]() {
    for (size_t i = 0; i < kThreadTaskCount; i++) {
      const auto current_task_queue_id =
          task_queue_ids[std::rand() % kTaskQueuesCount];
      const auto empty_task = []() {};
      // The timepoint doesn't matter as the queue is never going to be drained.
      const auto task_timepoint = ChronoTicksSinceEpoch();

      task_queues->RegisterTask(current_task_queue_id, empty_task,
                                task_timepoint);
    }

    tasks_posted_latch.CountDown();
  };

  std::vector<std::thread> threads;

  for (size_t i = 0; i < kThreadCount; i++) {
    threads.emplace_back(std::thread{thread_main});
  }

  ASSERT_EQ(threads.size(), kThreadCount);

  for (size_t i = 0; i < kThreadCount; i++) {
    threads[i].join();
  }

  // All tasks have been posted by now. Check that they are all pending.

  size_t pending_tasks = 0u;
  std::for_each(task_queue_ids.begin(), task_queue_ids.end(),
                [&](const auto& queue) {
                  pending_tasks += task_queues->GetNumPendingTasks(queue);
                });

  ASSERT_EQ(pending_tasks, kThreadCount * kThreadTaskCount);
}

TEST(MessageLoopTaskQueue, RegisterTaskWakesUpOwnerQueue) {
  auto task_queue = fml::MessageLoopTaskQueues::GetInstance();
  auto platform_queue = task_queue->CreateTaskQueue();
  auto raster_queue = task_queue->CreateTaskQueue();

  std::vector<fml::TimePoint> wakes;

  auto wakeable1 = std::make_unique<TestWakeable>(
      [&wakes](fml::TimePoint wake_time) { wakes.push_back(wake_time); });
  auto wakeable2 = std::make_unique<TestWakeable>([](fml::TimePoint wake_time) {
    // The raster queue is owned by the platform queue.
    ASSERT_FALSE(true);
  });

  task_queue->SetWakeable(platform_queue, wakeable1.get());
  task_queue->SetWakeable(raster_queue, wakeable2.get());

  auto time1 = ChronoTicksSinceEpoch() + fml::TimeDelta::FromMilliseconds(1);
  auto time2 = ChronoTicksSinceEpoch() + fml::TimeDelta::FromMilliseconds(2);

  ASSERT_EQ(0UL, wakes.size());

  task_queue->RegisterTask(platform_queue, []() {}, time1);

  ASSERT_EQ(1UL, wakes.size());
  ASSERT_EQ(time1, wakes[0]);

  task_queue->Merge(platform_queue, raster_queue);

  task_queue->RegisterTask(raster_queue, []() {}, time2);

  ASSERT_EQ(3UL, wakes.size());
  ASSERT_EQ(time1, wakes[1]);
  ASSERT_EQ(time1, wakes[2]);
}

}  // namespace testing
}  // namespace fml
