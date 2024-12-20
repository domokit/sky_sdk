// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#define FML_USED_ON_EMBEDDER

#include "flutter/fml/raster_thread_merger.h"

#include <thread>

#include "flutter/fml/memory/ref_ptr.h"
#include "flutter/fml/message_loop.h"
#include "flutter/fml/synchronization/count_down_latch.h"
#include "flutter/fml/synchronization/waitable_event.h"
#include "flutter/fml/task_runner.h"
#include "flutter/fml/thread.h"
#include "gtest/gtest.h"

namespace fml {
namespace testing {

/// A mock task queue NOT calling MessageLoop->Run() in thread
struct TaskQueueWrapper {
  fml::MessageLoop* loop = nullptr;

  /// The waiter for message loop initialized ok
  fml::AutoResetWaitableEvent latch;

  /// The waiter for thread finished
  fml::AutoResetWaitableEvent term;

  /// This field must below latch and term member, because
  /// cpp standard reference:
  /// non-static data members are initialized in the order they were declared in
  /// the class definition
  std::thread thread;

  TaskQueueWrapper()
      : thread([this]() {
          fml::MessageLoop::EnsureInitializedForCurrentThread();
          loop = &fml::MessageLoop::GetCurrent();
          latch.Signal();
          term.Wait();
        }) {
    latch.Wait();
  }

  ~TaskQueueWrapper() {
    term.Signal();
    thread.join();
  }

  fml::TaskQueueId GetTaskQueueId() const {
    return loop->GetTaskRunner()->GetTaskQueueId();
  }
};

TEST(RasterThreadMerger, RemainMergedTillLeaseExpires) {
  TaskQueueWrapper queue1;
  TaskQueueWrapper queue2;
  fml::TaskQueueId qid1 = queue1.GetTaskQueueId();
  fml::TaskQueueId qid2 = queue2.GetTaskQueueId();
  const auto raster_thread_merger =
      fml::MakeRefCounted<fml::RasterThreadMerger>(qid1, qid2);
  const size_t kNumFramesMerged = 5;

  ASSERT_FALSE(raster_thread_merger->IsMerged());

  raster_thread_merger->MergeWithLease(kNumFramesMerged);

  for (size_t i = 0; i < kNumFramesMerged; i++) {
    ASSERT_TRUE(raster_thread_merger->IsMerged());
    raster_thread_merger->DecrementLease();
  }

  ASSERT_FALSE(raster_thread_merger->IsMerged());
}

TEST(RasterThreadMerger, IsNotOnRasterizingThread) {
  fml::MessageLoop* loop1 = nullptr;
  fml::AutoResetWaitableEvent latch1;
  std::thread thread1([&loop1, &latch1]() {
    fml::MessageLoop::EnsureInitializedForCurrentThread();
    loop1 = &fml::MessageLoop::GetCurrent();
    loop1->GetTaskRunner()->PostTask([&]() { latch1.Signal(); });
    loop1->Run();
  });

  fml::MessageLoop* loop2 = nullptr;
  fml::AutoResetWaitableEvent latch2;
  std::thread thread2([&loop2, &latch2]() {
    fml::MessageLoop::EnsureInitializedForCurrentThread();
    loop2 = &fml::MessageLoop::GetCurrent();
    loop2->GetTaskRunner()->PostTask([&]() { latch2.Signal(); });
    loop2->Run();
  });

  latch1.Wait();
  latch2.Wait();

  fml::TaskQueueId qid1 = loop1->GetTaskRunner()->GetTaskQueueId();
  fml::TaskQueueId qid2 = loop2->GetTaskRunner()->GetTaskQueueId();
  const auto raster_thread_merger =
      fml::MakeRefCounted<fml::RasterThreadMerger>(qid1, qid2);

  fml::CountDownLatch pre_merge(2), post_merge(2), post_unmerge(2);

  loop1->GetTaskRunner()->PostTask([&]() {
    ASSERT_FALSE(raster_thread_merger->IsOnRasterizingThread());
    ASSERT_TRUE(raster_thread_merger->IsOnPlatformThread());
    ASSERT_EQ(fml::MessageLoop::GetCurrentTaskQueueId(), qid1);
    pre_merge.CountDown();
  });

  loop2->GetTaskRunner()->PostTask([&]() {
    ASSERT_TRUE(raster_thread_merger->IsOnRasterizingThread());
    ASSERT_FALSE(raster_thread_merger->IsOnPlatformThread());
    ASSERT_EQ(fml::MessageLoop::GetCurrentTaskQueueId(), qid2);
    pre_merge.CountDown();
  });

  pre_merge.Wait();

  raster_thread_merger->MergeWithLease(1);

  loop1->GetTaskRunner()->PostTask([&]() {
    ASSERT_TRUE(raster_thread_merger->IsOnRasterizingThread());
    ASSERT_TRUE(raster_thread_merger->IsOnPlatformThread());
    ASSERT_EQ(fml::MessageLoop::GetCurrentTaskQueueId(), qid1);
    post_merge.CountDown();
  });

  loop2->GetTaskRunner()->PostTask([&]() {
    // this will be false since this is going to be run
    // on loop1 really.
    ASSERT_TRUE(raster_thread_merger->IsOnRasterizingThread());
    ASSERT_TRUE(raster_thread_merger->IsOnPlatformThread());
    ASSERT_EQ(fml::MessageLoop::GetCurrentTaskQueueId(), qid1);
    post_merge.CountDown();
  });

  post_merge.Wait();

  raster_thread_merger->DecrementLease();

  loop1->GetTaskRunner()->PostTask([&]() {
    ASSERT_FALSE(raster_thread_merger->IsOnRasterizingThread());
    ASSERT_TRUE(raster_thread_merger->IsOnPlatformThread());
    ASSERT_EQ(fml::MessageLoop::GetCurrentTaskQueueId(), qid1);
    post_unmerge.CountDown();
  });

  loop2->GetTaskRunner()->PostTask([&]() {
    ASSERT_TRUE(raster_thread_merger->IsOnRasterizingThread());
    ASSERT_FALSE(raster_thread_merger->IsOnPlatformThread());
    ASSERT_EQ(fml::MessageLoop::GetCurrentTaskQueueId(), qid2);
    post_unmerge.CountDown();
  });

  post_unmerge.Wait();

  loop1->GetTaskRunner()->PostTask([&]() { loop1->Terminate(); });

  loop2->GetTaskRunner()->PostTask([&]() { loop2->Terminate(); });

  thread1.join();
  thread2.join();
}

TEST(RasterThreadMerger, LeaseExtension) {
  TaskQueueWrapper queue1;
  TaskQueueWrapper queue2;

  fml::TaskQueueId qid1 = queue1.GetTaskQueueId();
  fml::TaskQueueId qid2 = queue2.GetTaskQueueId();
  const auto raster_thread_merger =
      fml::MakeRefCounted<fml::RasterThreadMerger>(qid1, qid2);
  const size_t kNumFramesMerged = 5;

  ASSERT_FALSE(raster_thread_merger->IsMerged());

  raster_thread_merger->MergeWithLease(kNumFramesMerged);

  // let there be one more turn till the leases expire.
  for (size_t i = 0; i < kNumFramesMerged - 1; i++) {
    ASSERT_TRUE(raster_thread_merger->IsMerged());
    raster_thread_merger->DecrementLease();
  }

  // extend the lease once.
  raster_thread_merger->ExtendLeaseTo(kNumFramesMerged);

  // we will NOT last for 1 extra turn, we just set it.
  for (size_t i = 0; i < kNumFramesMerged; i++) {
    ASSERT_TRUE(raster_thread_merger->IsMerged());
    raster_thread_merger->DecrementLease();
  }

  ASSERT_FALSE(raster_thread_merger->IsMerged());
}

TEST(RasterThreadMerger, WaitUntilMerged) {
  fml::RefPtr<fml::RasterThreadMerger> raster_thread_merger;

  fml::AutoResetWaitableEvent create_thread_merger_latch;
  fml::MessageLoop* loop_platform = nullptr;
  fml::AutoResetWaitableEvent latch_platform;
  fml::AutoResetWaitableEvent term_platform;
  fml::AutoResetWaitableEvent latch_merged;
  std::thread thread_platform([&]() {
    fml::MessageLoop::EnsureInitializedForCurrentThread();
    loop_platform = &fml::MessageLoop::GetCurrent();
    latch_platform.Signal();
    create_thread_merger_latch.Wait();
    raster_thread_merger->WaitUntilMerged();
    latch_merged.Signal();
    term_platform.Wait();
  });

  const size_t kNumFramesMerged = 5;
  fml::MessageLoop* loop_raster = nullptr;
  fml::AutoResetWaitableEvent term_raster;
  std::thread thread_raster([&]() {
    fml::MessageLoop::EnsureInitializedForCurrentThread();
    loop_raster = &fml::MessageLoop::GetCurrent();
    latch_platform.Wait();
    fml::TaskQueueId qid_platform =
        loop_platform->GetTaskRunner()->GetTaskQueueId();
    fml::TaskQueueId qid_raster =
        loop_raster->GetTaskRunner()->GetTaskQueueId();
    raster_thread_merger =
        fml::MakeRefCounted<fml::RasterThreadMerger>(qid_platform, qid_raster);
    ASSERT_FALSE(raster_thread_merger->IsMerged());
    create_thread_merger_latch.Signal();
    raster_thread_merger->MergeWithLease(kNumFramesMerged);
    term_raster.Wait();
  });

  latch_merged.Wait();
  ASSERT_TRUE(raster_thread_merger->IsMerged());

  for (size_t i = 0; i < kNumFramesMerged; i++) {
    ASSERT_TRUE(raster_thread_merger->IsMerged());
    raster_thread_merger->DecrementLease();
  }

  ASSERT_FALSE(raster_thread_merger->IsMerged());

  term_platform.Signal();
  term_raster.Signal();
  thread_platform.join();
  thread_raster.join();
}

TEST(RasterThreadMerger, HandleTaskQueuesAreTheSame) {
  TaskQueueWrapper queue;
  fml::TaskQueueId qid1 = queue.GetTaskQueueId();
  fml::TaskQueueId qid2 = qid1;
  const auto raster_thread_merger =
      fml::MakeRefCounted<fml::RasterThreadMerger>(qid1, qid2);
  // Statically merged.
  ASSERT_TRUE(raster_thread_merger->IsMerged());

  // Test decrement lease and unmerge are both no-ops.
  // The task queues should be always merged.
  const size_t kNumFramesMerged = 5;
  raster_thread_merger->MergeWithLease(kNumFramesMerged);

  for (size_t i = 0; i < kNumFramesMerged; i++) {
    ASSERT_TRUE(raster_thread_merger->IsMerged());
    raster_thread_merger->DecrementLease();
  }

  ASSERT_TRUE(raster_thread_merger->IsMerged());

  // Wait until merged should also return immediately.
  raster_thread_merger->WaitUntilMerged();
  ASSERT_TRUE(raster_thread_merger->IsMerged());
}

TEST(RasterThreadMerger, Enable) {
  TaskQueueWrapper queue1;
  TaskQueueWrapper queue2;
  fml::TaskQueueId qid1 = queue1.GetTaskQueueId();
  fml::TaskQueueId qid2 = queue2.GetTaskQueueId();
  const auto raster_thread_merger =
      fml::MakeRefCounted<fml::RasterThreadMerger>(qid1, qid2);

  raster_thread_merger->Disable();
  raster_thread_merger->MergeWithLease(1);
  ASSERT_FALSE(raster_thread_merger->IsMerged());

  raster_thread_merger->Enable();
  ASSERT_FALSE(raster_thread_merger->IsMerged());

  raster_thread_merger->MergeWithLease(1);
  ASSERT_TRUE(raster_thread_merger->IsMerged());

  raster_thread_merger->DecrementLease();
  ASSERT_FALSE(raster_thread_merger->IsMerged());
}

TEST(RasterThreadMerger, Disable) {
  TaskQueueWrapper queue1;
  TaskQueueWrapper queue2;
  fml::TaskQueueId qid1 = queue1.GetTaskQueueId();
  fml::TaskQueueId qid2 = queue2.GetTaskQueueId();
  const auto raster_thread_merger =
      fml::MakeRefCounted<fml::RasterThreadMerger>(qid1, qid2);

  raster_thread_merger->Disable();
  ASSERT_FALSE(raster_thread_merger->IsMerged());

  raster_thread_merger->MergeWithLease(1);
  ASSERT_FALSE(raster_thread_merger->IsMerged());

  raster_thread_merger->Enable();
  raster_thread_merger->MergeWithLease(1);
  ASSERT_TRUE(raster_thread_merger->IsMerged());

  raster_thread_merger->Disable();
  raster_thread_merger->UnMergeNowIfLastOne();
  ASSERT_TRUE(raster_thread_merger->IsMerged());

  {
    auto decrement_result = raster_thread_merger->DecrementLease();
    ASSERT_EQ(fml::RasterThreadStatus::kRemainsMerged, decrement_result);
  }

  ASSERT_TRUE(raster_thread_merger->IsMerged());

  raster_thread_merger->Enable();
  raster_thread_merger->UnMergeNowIfLastOne();
  ASSERT_FALSE(raster_thread_merger->IsMerged());

  raster_thread_merger->MergeWithLease(1);

  ASSERT_TRUE(raster_thread_merger->IsMerged());

  {
    auto decrement_result = raster_thread_merger->DecrementLease();
    ASSERT_EQ(fml::RasterThreadStatus::kUnmergedNow, decrement_result);
  }

  ASSERT_FALSE(raster_thread_merger->IsMerged());
}

TEST(RasterThreadMerger, IsEnabled) {
  TaskQueueWrapper queue1;
  TaskQueueWrapper queue2;
  fml::TaskQueueId qid1 = queue1.GetTaskQueueId();
  fml::TaskQueueId qid2 = queue2.GetTaskQueueId();
  const auto raster_thread_merger =
      fml::MakeRefCounted<fml::RasterThreadMerger>(qid1, qid2);
  ASSERT_TRUE(raster_thread_merger->IsEnabled());

  raster_thread_merger->Disable();
  ASSERT_FALSE(raster_thread_merger->IsEnabled());

  raster_thread_merger->Enable();
  ASSERT_TRUE(raster_thread_merger->IsEnabled());
}

TEST(RasterThreadMerger, TwoMergersWithSameThreadPairShareEnabledState) {
  TaskQueueWrapper queue1;
  TaskQueueWrapper queue2;
  fml::TaskQueueId qid1 = queue1.GetTaskQueueId();
  fml::TaskQueueId qid2 = queue2.GetTaskQueueId();
  const auto merger1 =
      RasterThreadMerger::CreateOrShareThreadMerger(nullptr, qid1, qid2);
  const auto merger2 =
      RasterThreadMerger::CreateOrShareThreadMerger(merger1, qid1, qid2);
  ASSERT_TRUE(merger1->IsEnabled());
  ASSERT_TRUE(merger2->IsEnabled());

  merger1->Disable();
  ASSERT_FALSE(merger1->IsEnabled());
  ASSERT_FALSE(merger2->IsEnabled());

  merger2->Enable();
  ASSERT_TRUE(merger1->IsEnabled());
  ASSERT_TRUE(merger2->IsEnabled());
}

TEST(RasterThreadMerger, RunExpiredTasksWhileFirstTaskMergesThreads) {
  fml::MessageLoop* loop_platform = nullptr;
  fml::AutoResetWaitableEvent latch1;
  std::thread thread_platform([&loop_platform, &latch1]() {
    fml::MessageLoop::EnsureInitializedForCurrentThread();
    loop_platform = &fml::MessageLoop::GetCurrent();
    loop_platform->GetTaskRunner()->PostTask([&]() { latch1.Signal(); });
    loop_platform->Run();
  });

  fml::MessageLoop* loop_raster = nullptr;
  fml::AutoResetWaitableEvent latch2;
  std::thread thread_raster([&loop_raster, &loop_platform, &latch1, &latch2]() {
    latch1.Wait();

    fml::MessageLoop::EnsureInitializedForCurrentThread();
    loop_raster = &fml::MessageLoop::GetCurrent();
    fml::TaskQueueId qid_platform =
        loop_platform->GetTaskRunner()->GetTaskQueueId();
    fml::TaskQueueId qid_raster =
        loop_raster->GetTaskRunner()->GetTaskQueueId();
    fml::CountDownLatch post_merge(2);
    const auto raster_thread_merger =
        fml::MakeRefCounted<fml::RasterThreadMerger>(qid_platform, qid_raster);
    loop_raster->GetTaskRunner()->PostTask([&]() {
      ASSERT_TRUE(raster_thread_merger->IsOnRasterizingThread());
      ASSERT_FALSE(raster_thread_merger->IsOnPlatformThread());
      ASSERT_EQ(fml::MessageLoop::GetCurrentTaskQueueId(), qid_raster);
      raster_thread_merger->MergeWithLease(1);
      post_merge.CountDown();
    });

    loop_raster->GetTaskRunner()->PostTask([&]() {
      ASSERT_TRUE(raster_thread_merger->IsOnRasterizingThread());
      ASSERT_TRUE(raster_thread_merger->IsOnPlatformThread());
      ASSERT_EQ(fml::MessageLoop::GetCurrentTaskQueueId(), qid_platform);
      raster_thread_merger->DecrementLease();
      post_merge.CountDown();
    });

    loop_raster->RunExpiredTasksNow();
    post_merge.Wait();
    latch2.Signal();
  });

  latch2.Wait();
  loop_platform->GetTaskRunner()->PostTask(
      [&]() { loop_platform->Terminate(); });

  thread_platform.join();
  thread_raster.join();
}

TEST(RasterThreadMerger, RunExpiredTasksWhileFirstTaskUnMergesThreads) {
  fml::Thread platform_thread("test_platform_thread");

  fml::AutoResetWaitableEvent raster_latch;
  std::thread thread_raster([&]() {
    fml::MessageLoop::EnsureInitializedForCurrentThread();
    fml::MessageLoop* loop_raster = &fml::MessageLoop::GetCurrent();

    fml::TaskQueueId qid_platform =
        platform_thread.GetTaskRunner()->GetTaskQueueId();
    fml::TaskQueueId qid_raster =
        loop_raster->GetTaskRunner()->GetTaskQueueId();

    fml::AutoResetWaitableEvent merge_latch;
    const auto raster_thread_merger =
        fml::MakeRefCounted<fml::RasterThreadMerger>(qid_platform, qid_raster);
    loop_raster->GetTaskRunner()->PostTask([&]() {
      raster_thread_merger->MergeWithLease(1);
      merge_latch.Signal();
    });

    loop_raster->RunExpiredTasksNow();
    merge_latch.Wait();

    // threads should be merged at this point.
    fml::AutoResetWaitableEvent unmerge_latch;
    loop_raster->GetTaskRunner()->PostTask([&]() {
      ASSERT_TRUE(raster_thread_merger->IsOnRasterizingThread());
      ASSERT_TRUE(raster_thread_merger->IsOnPlatformThread());
      ASSERT_EQ(fml::MessageLoop::GetCurrentTaskQueueId(), qid_platform);
      raster_thread_merger->DecrementLease();
      unmerge_latch.Signal();
    });

    fml::AutoResetWaitableEvent post_unmerge_latch;
    loop_raster->GetTaskRunner()->PostTask([&]() {
      ASSERT_TRUE(raster_thread_merger->IsOnRasterizingThread());
      ASSERT_FALSE(raster_thread_merger->IsOnPlatformThread());
      ASSERT_EQ(fml::MessageLoop::GetCurrentTaskQueueId(), qid_raster);
      post_unmerge_latch.Signal();
    });

    unmerge_latch.Wait();
    loop_raster->RunExpiredTasksNow();

    post_unmerge_latch.Wait();
    raster_latch.Signal();
  });

  raster_latch.Wait();
  thread_raster.join();
}

TEST(RasterThreadMerger, SetMergeUnmergeCallback) {
  TaskQueueWrapper queue1;
  TaskQueueWrapper queue2;
  fml::TaskQueueId qid1 = queue1.GetTaskQueueId();
  fml::TaskQueueId qid2 = queue2.GetTaskQueueId();

  const auto raster_thread_merger =
      fml::MakeRefCounted<fml::RasterThreadMerger>(qid1, qid2);

  int callbacks = 0;
  raster_thread_merger->SetMergeUnmergeCallback(
      [&callbacks]() { callbacks++; });

  ASSERT_EQ(0, callbacks);

  raster_thread_merger->MergeWithLease(1);
  ASSERT_EQ(1, callbacks);

  raster_thread_merger->DecrementLease();
  ASSERT_EQ(2, callbacks);
}

TEST(RasterThreadMerger, MultipleMergersCanMergeSameThreadPair) {
  TaskQueueWrapper queue1;
  TaskQueueWrapper queue2;
  fml::TaskQueueId qid1 = queue1.GetTaskQueueId();
  fml::TaskQueueId qid2 = queue2.GetTaskQueueId();
  // Two mergers will share one same inner merger
  const auto raster_thread_merger1 =
      fml::RasterThreadMerger::CreateOrShareThreadMerger(nullptr, qid1, qid2);
  const auto raster_thread_merger2 =
      fml::RasterThreadMerger::CreateOrShareThreadMerger(raster_thread_merger1,
                                                         qid1, qid2);
  const size_t kNumFramesMerged = 5;
  ASSERT_FALSE(raster_thread_merger1->IsMerged());
  ASSERT_FALSE(raster_thread_merger2->IsMerged());

  // Merge using the first merger
  raster_thread_merger1->MergeWithLease(kNumFramesMerged);

  ASSERT_TRUE(raster_thread_merger1->IsMerged());
  ASSERT_TRUE(raster_thread_merger2->IsMerged());

  // let there be one more turn till the leases expire.
  for (size_t i = 0; i < kNumFramesMerged - 1; i++) {
    // Check merge state using the two merger
    ASSERT_TRUE(raster_thread_merger1->IsMerged());
    ASSERT_TRUE(raster_thread_merger2->IsMerged());
    raster_thread_merger1->DecrementLease();
  }

  ASSERT_TRUE(raster_thread_merger1->IsMerged());
  ASSERT_TRUE(raster_thread_merger2->IsMerged());

  // extend the lease once with the first merger
  raster_thread_merger1->ExtendLeaseTo(kNumFramesMerged);

  // we will NOT last for 1 extra turn, we just set it.
  for (size_t i = 0; i < kNumFramesMerged; i++) {
    // Check merge state using the two merger
    ASSERT_TRUE(raster_thread_merger1->IsMerged());
    ASSERT_TRUE(raster_thread_merger2->IsMerged());
    raster_thread_merger1->DecrementLease();
  }

  ASSERT_FALSE(raster_thread_merger1->IsMerged());
  ASSERT_FALSE(raster_thread_merger2->IsMerged());
}

TEST(RasterThreadMerger, TheLastCallerOfMultipleMergersCanUnmergeNow) {
  TaskQueueWrapper queue1;
  TaskQueueWrapper queue2;
  fml::TaskQueueId qid1 = queue1.GetTaskQueueId();
  fml::TaskQueueId qid2 = queue2.GetTaskQueueId();
  // Two mergers will share one same inner merger
  const auto raster_thread_merger1 =
      fml::RasterThreadMerger::CreateOrShareThreadMerger(nullptr, qid1, qid2);
  const auto raster_thread_merger2 =
      fml::RasterThreadMerger::CreateOrShareThreadMerger(raster_thread_merger1,
                                                         qid1, qid2);
  const size_t kNumFramesMerged = 5;
  ASSERT_FALSE(raster_thread_merger1->IsMerged());
  ASSERT_FALSE(raster_thread_merger2->IsMerged());

  // Merge using the mergers
  raster_thread_merger1->MergeWithLease(kNumFramesMerged);
  ASSERT_TRUE(raster_thread_merger1->IsMerged());
  ASSERT_TRUE(raster_thread_merger2->IsMerged());
  // Extend the second merger's lease
  raster_thread_merger2->ExtendLeaseTo(kNumFramesMerged);
  ASSERT_TRUE(raster_thread_merger1->IsMerged());
  ASSERT_TRUE(raster_thread_merger2->IsMerged());

  // Two callers state becomes one caller left.
  raster_thread_merger1->UnMergeNowIfLastOne();
  // Check if still merged
  ASSERT_TRUE(raster_thread_merger1->IsMerged());
  ASSERT_TRUE(raster_thread_merger2->IsMerged());

  // One caller state becomes no callers left.
  raster_thread_merger2->UnMergeNowIfLastOne();
  // Check if unmerged
  ASSERT_FALSE(raster_thread_merger1->IsMerged());
  ASSERT_FALSE(raster_thread_merger2->IsMerged());
}

TEST(RasterThreadMerger, TheLastMergedCallerOfMultipleMergersCanUnmergeNow) {
  TaskQueueWrapper queue1;
  TaskQueueWrapper queue2;
  fml::TaskQueueId qid1 = queue1.GetTaskQueueId();
  fml::TaskQueueId qid2 = queue2.GetTaskQueueId();
  // Two mergers will share one same inner merger
  const auto raster_thread_merger1 =
      fml::RasterThreadMerger::CreateOrShareThreadMerger(nullptr, qid1, qid2);

  const size_t kNumFramesMerged = 1;
  ASSERT_FALSE(raster_thread_merger1->IsMerged());

  // Merge using the mergers
  raster_thread_merger1->MergeWithLease(kNumFramesMerged);
  ASSERT_TRUE(raster_thread_merger1->IsMerged());

  for (size_t i = 0; i < kNumFramesMerged; i++) {
    // Un-merge thread merger 1.
    raster_thread_merger1->DecrementLease();
  }
  ASSERT_FALSE(raster_thread_merger1->IsMerged());

  const auto raster_thread_merger2 =
      fml::RasterThreadMerger::CreateOrShareThreadMerger(raster_thread_merger1,
                                                         qid1, qid2);
  ASSERT_FALSE(raster_thread_merger2->IsMerged());
  raster_thread_merger2->MergeWithLease(kNumFramesMerged);
  ASSERT_TRUE(raster_thread_merger2->IsMerged());

  // One caller state becomes no callers left.
  raster_thread_merger2->UnMergeNowIfLastOne();
  // Check if unmerged
  ASSERT_FALSE(raster_thread_merger1->IsMerged());
  ASSERT_FALSE(raster_thread_merger2->IsMerged());
}

/// This case tests multiple standalone engines using independent merger to
/// merge two different raster threads into the same platform thread.
TEST(RasterThreadMerger,
     TwoIndependentMergersCanMergeTwoDifferentThreadsIntoSamePlatformThread) {
  TaskQueueWrapper queue1;
  TaskQueueWrapper queue2;
  TaskQueueWrapper queue3;
  fml::TaskQueueId qid1 = queue1.GetTaskQueueId();
  fml::TaskQueueId qid2 = queue2.GetTaskQueueId();
  fml::TaskQueueId qid3 = queue3.GetTaskQueueId();

  // Two mergers will NOT share same inner merger
  const auto merger_from_2_to_1 =
      fml::RasterThreadMerger::CreateOrShareThreadMerger(nullptr, qid1, qid2);
  const auto merger_from_3_to_1 =
      fml::RasterThreadMerger::CreateOrShareThreadMerger(merger_from_2_to_1,
                                                         qid1, qid3);
  const size_t kNumFramesMerged = 5;
  ASSERT_FALSE(merger_from_2_to_1->IsMerged());
  ASSERT_FALSE(merger_from_3_to_1->IsMerged());

  // Merge thread2 into thread1
  merger_from_2_to_1->MergeWithLease(kNumFramesMerged);
  // Merge thread3 into thread1
  merger_from_3_to_1->MergeWithLease(kNumFramesMerged);

  ASSERT_TRUE(merger_from_2_to_1->IsMerged());
  ASSERT_TRUE(merger_from_3_to_1->IsMerged());

  for (size_t i = 0; i < kNumFramesMerged; i++) {
    ASSERT_TRUE(merger_from_2_to_1->IsMerged());
    merger_from_2_to_1->DecrementLease();
  }

  ASSERT_FALSE(merger_from_2_to_1->IsMerged());
  ASSERT_TRUE(merger_from_3_to_1->IsMerged());

  for (size_t i = 0; i < kNumFramesMerged; i++) {
    ASSERT_TRUE(merger_from_3_to_1->IsMerged());
    merger_from_3_to_1->DecrementLease();
  }

  ASSERT_FALSE(merger_from_2_to_1->IsMerged());
  ASSERT_FALSE(merger_from_3_to_1->IsMerged());

  merger_from_2_to_1->MergeWithLease(kNumFramesMerged);
  ASSERT_TRUE(merger_from_2_to_1->IsMerged());
  ASSERT_FALSE(merger_from_3_to_1->IsMerged());
  merger_from_3_to_1->MergeWithLease(kNumFramesMerged);
  ASSERT_TRUE(merger_from_2_to_1->IsMerged());
  ASSERT_TRUE(merger_from_3_to_1->IsMerged());

  // Can unmerge independently
  merger_from_2_to_1->UnMergeNowIfLastOne();
  ASSERT_FALSE(merger_from_2_to_1->IsMerged());
  ASSERT_TRUE(merger_from_3_to_1->IsMerged());

  // Can unmerge independently
  merger_from_3_to_1->UnMergeNowIfLastOne();
  ASSERT_FALSE(merger_from_2_to_1->IsMerged());
  ASSERT_FALSE(merger_from_3_to_1->IsMerged());
}

}  // namespace testing
}  // namespace fml
