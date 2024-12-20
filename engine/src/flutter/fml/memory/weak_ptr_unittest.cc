// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#define FML_USED_ON_EMBEDDER

#include "flutter/fml/memory/weak_ptr.h"

#include <thread>
#include <utility>

#include "flutter/fml/message_loop.h"
#include "flutter/fml/raster_thread_merger.h"
#include "flutter/fml/synchronization/count_down_latch.h"
#include "gtest/gtest.h"

namespace fml {
namespace {

TEST(WeakPtrTest, Basic) {
  int data = 0;
  WeakPtrFactory<int> factory(&data);
  WeakPtr<int> ptr = factory.GetWeakPtr();
  EXPECT_EQ(&data, ptr.get());
}

TEST(WeakPtrTest, CopyConstruction) {
  int data = 0;
  WeakPtrFactory<int> factory(&data);
  WeakPtr<int> ptr = factory.GetWeakPtr();
  const WeakPtr<int>& ptr2(ptr);
  EXPECT_EQ(&data, ptr.get());
  EXPECT_EQ(&data, ptr2.get());
}

TEST(WeakPtrTest, MoveConstruction) {
  int data = 0;
  WeakPtrFactory<int> factory(&data);
  WeakPtr<int> ptr = factory.GetWeakPtr();
  WeakPtr<int> ptr2(std::move(ptr));
  // The clang linter flags the method called on the moved-from reference, but
  // this is testing the move implementation, so it is marked NOLINT.
  EXPECT_EQ(nullptr, ptr.get());  // NOLINT
  EXPECT_EQ(&data, ptr2.get());
}

TEST(WeakPtrTest, CopyAssignment) {
  int data = 0;
  WeakPtrFactory<int> factory(&data);
  WeakPtr<int> ptr = factory.GetWeakPtr();
  WeakPtr<int> ptr2;
  EXPECT_EQ(nullptr, ptr2.get());
  ptr2 = ptr;
  EXPECT_EQ(&data, ptr.get());
  EXPECT_EQ(&data, ptr2.get());
}

TEST(WeakPtrTest, MoveAssignment) {
  int data = 0;
  WeakPtrFactory<int> factory(&data);
  WeakPtr<int> ptr = factory.GetWeakPtr();
  WeakPtr<int> ptr2;
  EXPECT_EQ(nullptr, ptr2.get());
  ptr2 = std::move(ptr);
  // The clang linter flags the method called on the moved-from reference, but
  // this is testing the move implementation, so it is marked NOLINT.
  EXPECT_EQ(nullptr, ptr.get());  // NOLINT
  EXPECT_EQ(&data, ptr2.get());
}

TEST(WeakPtrTest, Testable) {
  int data = 0;
  WeakPtrFactory<int> factory(&data);
  WeakPtr<int> ptr;
  EXPECT_EQ(nullptr, ptr.get());
  EXPECT_FALSE(ptr);
  ptr = factory.GetWeakPtr();
  EXPECT_EQ(&data, ptr.get());
  EXPECT_TRUE(ptr);
}

TEST(WeakPtrTest, OutOfScope) {
  WeakPtr<int> ptr;
  EXPECT_EQ(nullptr, ptr.get());
  {
    int data = 0;
    WeakPtrFactory<int> factory(&data);
    ptr = factory.GetWeakPtr();
  }
  EXPECT_EQ(nullptr, ptr.get());
}

TEST(WeakPtrTest, Multiple) {
  WeakPtr<int> a;
  WeakPtr<int> b;
  {
    int data = 0;
    WeakPtrFactory<int> factory(&data);
    a = factory.GetWeakPtr();
    b = factory.GetWeakPtr();
    EXPECT_EQ(&data, a.get());
    EXPECT_EQ(&data, b.get());
  }
  EXPECT_EQ(nullptr, a.get());
  EXPECT_EQ(nullptr, b.get());
}

TEST(WeakPtrTest, MultipleStaged) {
  WeakPtr<int> a;
  {
    int data = 0;
    WeakPtrFactory<int> factory(&data);
    a = factory.GetWeakPtr();
    { WeakPtr<int> b = factory.GetWeakPtr(); }
    EXPECT_NE(a.get(), nullptr);
  }
  EXPECT_EQ(nullptr, a.get());
}

struct Base {
  double member = 0.;
};
struct Derived : public Base {};

TEST(WeakPtrTest, Dereference) {
  Base data;
  data.member = 123456.;
  WeakPtrFactory<Base> factory(&data);
  WeakPtr<Base> ptr = factory.GetWeakPtr();
  EXPECT_EQ(&data, ptr.get());
  EXPECT_EQ(data.member, (*ptr).member);
  EXPECT_EQ(data.member, ptr->member);
}

TEST(WeakPtrTest, UpcastCopyConstruction) {
  Derived data;
  WeakPtrFactory<Derived> factory(&data);
  WeakPtr<Derived> ptr = factory.GetWeakPtr();
  WeakPtr<Base> ptr2(ptr);
  EXPECT_EQ(&data, ptr.get());
  EXPECT_EQ(&data, ptr2.get());
}

TEST(WeakPtrTest, UpcastMoveConstruction) {
  Derived data;
  WeakPtrFactory<Derived> factory(&data);
  WeakPtr<Derived> ptr = factory.GetWeakPtr();
  WeakPtr<Base> ptr2(std::move(ptr));
  // The clang linter flags the method called on the moved-from reference, but
  // this is testing the move implementation, so it is marked NOLINT.
  EXPECT_EQ(nullptr, ptr.get());  // NOLINT
  EXPECT_EQ(&data, ptr2.get());
}

TEST(WeakPtrTest, UpcastCopyAssignment) {
  Derived data;
  WeakPtrFactory<Derived> factory(&data);
  WeakPtr<Derived> ptr = factory.GetWeakPtr();
  WeakPtr<Base> ptr2;
  EXPECT_EQ(nullptr, ptr2.get());
  ptr2 = ptr;
  EXPECT_EQ(&data, ptr.get());
  EXPECT_EQ(&data, ptr2.get());
}

TEST(WeakPtrTest, UpcastMoveAssignment) {
  Derived data;
  WeakPtrFactory<Derived> factory(&data);
  WeakPtr<Derived> ptr = factory.GetWeakPtr();
  WeakPtr<Base> ptr2;
  EXPECT_EQ(nullptr, ptr2.get());
  ptr2 = std::move(ptr);
  // The clang linter flags the method called on the moved-from reference, but
  // this is testing the move implementation, so it is marked NOLINT.
  EXPECT_EQ(nullptr, ptr.get());  // NOLINT
  EXPECT_EQ(&data, ptr2.get());
}

TEST(TaskRunnerAffineWeakPtrTest, ShouldNotCrashIfRunningOnTheSameTaskRunner) {
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
  fml::AutoResetWaitableEvent loop2_task_finish_latch;
  fml::AutoResetWaitableEvent loop2_task_start_latch;
  std::thread thread2([&loop2, &latch2, &term2, &loop2_task_finish_latch,
                       &loop2_task_start_latch]() {
    fml::MessageLoop::EnsureInitializedForCurrentThread();
    int data = 0;
    TaskRunnerAffineWeakPtrFactory<int> factory(&data);
    loop2 = &fml::MessageLoop::GetCurrent();

    loop2->GetTaskRunner()->PostTask([&]() {
      latch2.Signal();
      loop2_task_start_latch.Wait();
      TaskRunnerAffineWeakPtr<int> ptr = factory.GetWeakPtr();
      EXPECT_EQ(*ptr, data);
      loop2_task_finish_latch.Signal();
    });
    loop2->Run();
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

  loop2_task_start_latch.Signal();
  loop2_task_finish_latch.Wait();

  for (size_t i = 0; i < kNumFramesMerged; i++) {
    ASSERT_TRUE(raster_thread_merger->IsMerged());
    raster_thread_merger->DecrementLease();
  }

  ASSERT_FALSE(raster_thread_merger->IsMerged());
  loop2->Terminate();

  term1.Signal();
  term2.Signal();
  thread1.join();
  thread2.join();
}

}  // namespace
}  // namespace fml
