// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#define FML_USED_ON_EMBEDDER

#include "flutter/fml/memory/task_runner_checker.h"

#include <thread>

#include "flutter/fml/message_loop.h"
#include "flutter/fml/raster_thread_merger.h"
#include "flutter/fml/synchronization/count_down_latch.h"
#include "flutter/fml/synchronization/waitable_event.h"
#include "gtest/gtest.h"

namespace fml {
namespace testing {

TEST(TaskRunnerCheckerTests, RunsOnCurrentTaskRunner) {
  TaskRunnerChecker checker;
  EXPECT_EQ(checker.RunsOnCreationTaskRunner(), true);
}

TEST(TaskRunnerCheckerTests, FailsTheCheckIfOnDifferentTaskRunner) {
  TaskRunnerChecker checker;
  EXPECT_EQ(checker.RunsOnCreationTaskRunner(), true);
  fml::MessageLoop* loop = nullptr;
  fml::AutoResetWaitableEvent latch;
  std::thread another_thread([&]() {
    fml::MessageLoop::EnsureInitializedForCurrentThread();
    loop = &fml::MessageLoop::GetCurrent();
    loop->GetTaskRunner()->PostTask([&]() {
      EXPECT_EQ(checker.RunsOnCreationTaskRunner(), false);
      latch.Signal();
    });
    loop->Run();
  });
  latch.Wait();
  loop->Terminate();
  another_thread.join();
  EXPECT_EQ(checker.RunsOnCreationTaskRunner(), true);
}

TEST(TaskRunnerCheckerTests, SameTaskRunnerRunsOnTheSameThread) {
  fml::MessageLoop::EnsureInitializedForCurrentThread();
  fml::MessageLoop& loop1 = fml::MessageLoop::GetCurrent();
  fml::MessageLoop& loop2 = fml::MessageLoop::GetCurrent();
  TaskQueueId a = loop1.GetTaskRunner()->GetTaskQueueId();
  TaskQueueId b = loop2.GetTaskRunner()->GetTaskQueueId();
  EXPECT_EQ(TaskRunnerChecker::RunsOnTheSameThread(a, b), true);
}

TEST(TaskRunnerCheckerTests, RunsOnDifferentThreadsReturnsFalse) {
  fml::MessageLoop::EnsureInitializedForCurrentThread();
  fml::MessageLoop& loop1 = fml::MessageLoop::GetCurrent();
  TaskQueueId a = loop1.GetTaskRunner()->GetTaskQueueId();
  fml::AutoResetWaitableEvent latch;
  std::thread another_thread([&]() {
    fml::MessageLoop::EnsureInitializedForCurrentThread();
    fml::MessageLoop& loop2 = fml::MessageLoop::GetCurrent();
    TaskQueueId b = loop2.GetTaskRunner()->GetTaskQueueId();
    EXPECT_EQ(TaskRunnerChecker::RunsOnTheSameThread(a, b), false);
    latch.Signal();
  });
  latch.Wait();
  another_thread.join();
}

TEST(TaskRunnerCheckerTests, MergedTaskRunnersRunsOnTheSameThread) {
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
  fml::TaskQueueId qid1 = loop1->GetTaskRunner()->GetTaskQueueId();
  fml::TaskQueueId qid2 = loop2->GetTaskRunner()->GetTaskQueueId();
  const auto raster_thread_merger =
      fml::MakeRefCounted<fml::RasterThreadMerger>(qid1, qid2);
  const size_t kNumFramesMerged = 5;

  raster_thread_merger->MergeWithLease(kNumFramesMerged);

  // merged, running on the same thread
  EXPECT_EQ(TaskRunnerChecker::RunsOnTheSameThread(qid1, qid2), true);

  for (size_t i = 0; i < kNumFramesMerged; i++) {
    ASSERT_TRUE(raster_thread_merger->IsMerged());
    raster_thread_merger->DecrementLease();
  }

  ASSERT_FALSE(raster_thread_merger->IsMerged());

  // un-merged, not running on the same thread
  EXPECT_EQ(TaskRunnerChecker::RunsOnTheSameThread(qid1, qid2), false);

  term1.Signal();
  term2.Signal();
  thread1.join();
  thread2.join();
}

TEST(TaskRunnerCheckerTests,
     PassesRunsOnCreationTaskRunnerIfOnDifferentTaskRunner) {
  fml::MessageLoop* loop1 = nullptr;
  fml::AutoResetWaitableEvent latch1;
  std::thread thread1([&]() {
    fml::MessageLoop::EnsureInitializedForCurrentThread();
    loop1 = &fml::MessageLoop::GetCurrent();
    latch1.Signal();
    loop1->Run();
  });

  fml::MessageLoop* loop2 = nullptr;
  fml::AutoResetWaitableEvent latch2;
  std::thread thread2([&]() {
    fml::MessageLoop::EnsureInitializedForCurrentThread();
    loop2 = &fml::MessageLoop::GetCurrent();
    latch2.Signal();
    loop2->Run();
  });

  latch1.Wait();
  latch2.Wait();

  fml::TaskQueueId qid1 = loop1->GetTaskRunner()->GetTaskQueueId();
  fml::TaskQueueId qid2 = loop2->GetTaskRunner()->GetTaskQueueId();
  fml::MessageLoopTaskQueues::GetInstance()->Merge(qid1, qid2);

  std::unique_ptr<TaskRunnerChecker> checker;

  fml::AutoResetWaitableEvent latch3;
  loop2->GetTaskRunner()->PostTask([&]() {
    checker = std::make_unique<TaskRunnerChecker>();
    EXPECT_EQ(checker->RunsOnCreationTaskRunner(), true);
    latch3.Signal();
  });
  latch3.Wait();

  fml::MessageLoopTaskQueues::GetInstance()->Unmerge(qid1, qid2);

  fml::AutoResetWaitableEvent latch4;
  loop2->GetTaskRunner()->PostTask([&]() {
    EXPECT_EQ(checker->RunsOnCreationTaskRunner(), true);
    latch4.Signal();
  });
  latch4.Wait();

  loop1->Terminate();
  loop2->Terminate();
  thread1.join();
  thread2.join();
}

}  // namespace testing
}  // namespace fml
