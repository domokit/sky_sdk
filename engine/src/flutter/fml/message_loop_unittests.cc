// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#define FML_USED_ON_EMBEDDER

#include "flutter/fml/message_loop.h"

#include <iostream>
#include <thread>

#include "flutter/fml/build_config.h"
#include "flutter/fml/concurrent_message_loop.h"
#include "flutter/fml/synchronization/count_down_latch.h"
#include "flutter/fml/synchronization/waitable_event.h"
#include "flutter/fml/task_runner.h"
#include "flutter/fml/time/chrono_timestamp_provider.h"
#include "gtest/gtest.h"

TEST(MessageLoop, GetCurrent) {
  std::thread thread([]() {
    fml::MessageLoop::EnsureInitializedForCurrentThread();
    ASSERT_TRUE(fml::MessageLoop::GetCurrent().GetTaskRunner());
  });
  thread.join();
}

TEST(MessageLoop, DifferentThreadsHaveDifferentLoops) {
  fml::MessageLoop* loop1 = nullptr;
  fml::AutoResetWaitableEvent latch1;
  fml::AutoResetWaitableEvent term1;
  std::thread thread1([&loop1, &latch1, &term1]() {
    fml::MessageLoop::EnsureInitializedForCurrentThread();
    loop1 = &fml::MessageLoop::GetCurrent();
    latch1.Signal();
    term1.Wait();
  });

  fml::MessageLoop* loop2 = nullptr;
  fml::AutoResetWaitableEvent latch2;
  fml::AutoResetWaitableEvent term2;
  std::thread thread2([&loop2, &latch2, &term2]() {
    fml::MessageLoop::EnsureInitializedForCurrentThread();
    loop2 = &fml::MessageLoop::GetCurrent();
    latch2.Signal();
    term2.Wait();
  });
  latch1.Wait();
  latch2.Wait();
  ASSERT_FALSE(loop1 == loop2);
  term1.Signal();
  term2.Signal();
  thread1.join();
  thread2.join();
}

TEST(MessageLoop, CanRunAndTerminate) {
  bool started = false;
  bool terminated = false;
  std::thread thread([&started, &terminated]() {
    fml::MessageLoop::EnsureInitializedForCurrentThread();
    auto& loop = fml::MessageLoop::GetCurrent();
    ASSERT_TRUE(loop.GetTaskRunner());
    loop.GetTaskRunner()->PostTask([&terminated]() {
      fml::MessageLoop::GetCurrent().Terminate();
      terminated = true;
    });
    loop.Run();
    started = true;
  });
  thread.join();
  ASSERT_TRUE(started);
  ASSERT_TRUE(terminated);
}

TEST(MessageLoop, NonDelayedTasksAreRunInOrder) {
  const size_t count = 100;
  bool started = false;
  bool terminated = false;
  std::thread thread([&started, &terminated, count]() {
    fml::MessageLoop::EnsureInitializedForCurrentThread();
    auto& loop = fml::MessageLoop::GetCurrent();
    size_t current = 0;
    for (size_t i = 0; i < count; i++) {
      loop.GetTaskRunner()->PostTask([&terminated, i, &current]() {
        ASSERT_EQ(current, i);
        current++;
        if (count == i + 1) {
          fml::MessageLoop::GetCurrent().Terminate();
          terminated = true;
        }
      });
    }
    loop.Run();
    ASSERT_EQ(current, count);
    started = true;
  });
  thread.join();
  ASSERT_TRUE(started);
  ASSERT_TRUE(terminated);
}

TEST(MessageLoop, DelayedTasksAtSameTimeAreRunInOrder) {
  const size_t count = 100;
  bool started = false;
  bool terminated = false;
  std::thread thread([&started, &terminated, count]() {
    fml::MessageLoop::EnsureInitializedForCurrentThread();
    auto& loop = fml::MessageLoop::GetCurrent();
    size_t current = 0;
    const auto now_plus_some =
        fml::ChronoTicksSinceEpoch() + fml::TimeDelta::FromMilliseconds(2);
    for (size_t i = 0; i < count; i++) {
      loop.GetTaskRunner()->PostTaskForTime(
          [&terminated, i, &current]() {
            ASSERT_EQ(current, i);
            current++;
            if (count == i + 1) {
              fml::MessageLoop::GetCurrent().Terminate();
              terminated = true;
            }
          },
          now_plus_some);
    }
    loop.Run();
    ASSERT_EQ(current, count);
    started = true;
  });
  thread.join();
  ASSERT_TRUE(started);
  ASSERT_TRUE(terminated);
}

TEST(MessageLoop, CheckRunsTaskOnCurrentThread) {
  fml::RefPtr<fml::TaskRunner> runner;
  fml::AutoResetWaitableEvent latch;
  std::thread thread([&runner, &latch]() {
    fml::MessageLoop::EnsureInitializedForCurrentThread();
    auto& loop = fml::MessageLoop::GetCurrent();
    runner = loop.GetTaskRunner();
    latch.Signal();
    ASSERT_TRUE(loop.GetTaskRunner()->RunsTasksOnCurrentThread());
  });
  latch.Wait();
  ASSERT_TRUE(runner);
  ASSERT_FALSE(runner->RunsTasksOnCurrentThread());
  thread.join();
}

TEST(MessageLoop, TaskObserverFire) {
  bool started = false;
  bool terminated = false;
  std::thread thread([&started, &terminated]() {
    fml::MessageLoop::EnsureInitializedForCurrentThread();
    const size_t count = 25;
    auto& loop = fml::MessageLoop::GetCurrent();
    size_t task_count = 0;
    size_t obs_count = 0;
    auto obs = [&obs_count]() { obs_count++; };
    for (size_t i = 0; i < count; i++) {
      loop.GetTaskRunner()->PostTask([&terminated, i, &task_count]() {
        ASSERT_EQ(task_count, i);
        task_count++;
        if (count == i + 1) {
          fml::MessageLoop::GetCurrent().Terminate();
          terminated = true;
        }
      });
    }
    loop.AddTaskObserver(0, obs);
    loop.Run();
    ASSERT_EQ(task_count, count);
    ASSERT_EQ(obs_count, count);
    started = true;
  });
  thread.join();
  ASSERT_TRUE(started);
  ASSERT_TRUE(terminated);
}

TEST(MessageLoop, ConcurrentMessageLoopHasNonZeroWorkers) {
  auto loop = fml::ConcurrentMessageLoop::Create(
      0u /* explicitly specify zero workers */);
  ASSERT_GT(loop->GetWorkerCount(), 0u);
}

TEST(MessageLoop, CanCreateAndShutdownConcurrentMessageLoopsOverAndOver) {
  for (size_t i = 0; i < 10; ++i) {
    auto loop = fml::ConcurrentMessageLoop::Create(i + 1);
    ASSERT_EQ(loop->GetWorkerCount(), i + 1);
  }
}

TEST(MessageLoop, CanCreateConcurrentMessageLoop) {
  auto loop = fml::ConcurrentMessageLoop::Create();
  auto task_runner = loop->GetTaskRunner();
  const size_t kCount = 10;
  fml::CountDownLatch latch(kCount);
  std::mutex thread_ids_mutex;
  std::set<std::thread::id> thread_ids;
  for (size_t i = 0; i < kCount; ++i) {
    task_runner->PostTask([&]() {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      std::cout << "Ran on thread: " << std::this_thread::get_id() << std::endl;
      {
        std::scoped_lock lock(thread_ids_mutex);
        thread_ids.insert(std::this_thread::get_id());
      }
      latch.CountDown();
    });
  }
  latch.Wait();
  ASSERT_GE(thread_ids.size(), 1u);
}
