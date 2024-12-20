// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/testing/testing.h"
#include "impeller/base/mask.h"
#include "impeller/base/promise.h"
#include "impeller/base/strings.h"
#include "impeller/base/thread.h"

namespace impeller {

enum class MyMaskBits : uint32_t {
  kFoo = 0,
  kBar = 1 << 0,
  kBaz = 1 << 1,
  kBang = 1 << 2,
};

using MyMask = Mask<MyMaskBits>;

IMPELLER_ENUM_IS_MASK(MyMaskBits);

namespace testing {

struct Foo {
  Mutex mtx;
  int a IPLR_GUARDED_BY(mtx);
};

struct RWFoo {
  RWMutex mtx;
  int a IPLR_GUARDED_BY(mtx);
};

TEST(ThreadTest, CanCreateMutex) {
  Foo f = {};

  // f.a = 100; <--- Static analysis error.
  f.mtx.Lock();
  f.a = 100;
  f.mtx.Unlock();
}

TEST(ThreadTest, CanCreateMutexLock) {
  Foo f = {};

  // f.a = 100; <--- Static analysis error.
  auto a = Lock(f.mtx);
  f.a = 100;
}

TEST(ThreadTest, CanCreateRWMutex) {
  RWFoo f = {};

  // f.a = 100; <--- Static analysis error.
  f.mtx.LockWriter();
  f.a = 100;
  f.mtx.UnlockWriter();
  // int b = f.a; <--- Static analysis error.
  f.mtx.LockReader();
  [[maybe_unused]] int b = f.a;  // NOLINT(clang-analyzer-deadcode.DeadStores)
  f.mtx.UnlockReader();
}

TEST(ThreadTest, CanCreateRWMutexLock) {
  RWFoo f = {};

  // f.a = 100; <--- Static analysis error.
  {
    auto write_lock = WriterLock{f.mtx};
    f.a = 100;
  }

  // int b = f.a; <--- Static analysis error.
  {
    auto read_lock = ReaderLock(f.mtx);
    [[maybe_unused]] int b = f.a;  // NOLINT(clang-analyzer-deadcode.DeadStores)
  }

  // f.mtx.UnlockReader(); <--- Static analysis error.
}

TEST(StringsTest, CanSPrintF) {
  ASSERT_EQ(SPrintF("%sx%d", "Hello", 12), "Hellox12");
  ASSERT_EQ(SPrintF(""), "");
  ASSERT_EQ(SPrintF("Hello"), "Hello");
  ASSERT_EQ(SPrintF("%sx%.2f", "Hello", 12.122222), "Hellox12.12");
}

struct CVTest {
  Mutex mutex;
  ConditionVariable cv;
  uint32_t rando_ivar IPLR_GUARDED_BY(mutex) = 0;
};

TEST(ConditionVariableTest, WaitUntil) {
  CVTest test;
  // test.rando_ivar = 12; // <--- Static analysis error
  for (size_t i = 0; i < 2; ++i) {
    test.mutex.Lock();  // <--- Static analysis error without this.
    auto result = test.cv.WaitUntil(
        test.mutex,
        std::chrono::high_resolution_clock::now() +
            std::chrono::milliseconds{10},
        [&]() IPLR_REQUIRES(test.mutex) {
          test.rando_ivar = 12;  // <-- Static analysics error without the
                                 // IPLR_REQUIRES on the pred.
          return false;
        });
    test.mutex.Unlock();
    ASSERT_FALSE(result);
  }
  Lock lock(test.mutex);  // <--- Static analysis error without this.
  // The predicate never returns true. So return has to be due to a non-spurious
  // wake.
  ASSERT_EQ(test.rando_ivar, 12u);
}

TEST(ConditionVariableTest, WaitFor) {
  CVTest test;
  // test.rando_ivar = 12; // <--- Static analysis error
  for (size_t i = 0; i < 2; ++i) {
    test.mutex.Lock();  // <--- Static analysis error without this.
    auto result = test.cv.WaitFor(
        test.mutex, std::chrono::milliseconds{10},
        [&]() IPLR_REQUIRES(test.mutex) {
          test.rando_ivar = 12;  // <-- Static analysics error without the
                                 // IPLR_REQUIRES on the pred.
          return false;
        });
    test.mutex.Unlock();
    ASSERT_FALSE(result);
  }
  Lock lock(test.mutex);  // <--- Static analysis error without this.
  // The predicate never returns true. So return has to be due to a non-spurious
  // wake.
  ASSERT_EQ(test.rando_ivar, 12u);
}

TEST(ConditionVariableTest, WaitForever) {
  CVTest test;
  // test.rando_ivar = 12; // <--- Static analysis error
  for (size_t i = 0; i < 2; ++i) {
    test.mutex.Lock();  // <--- Static analysis error without this.
    test.cv.Wait(test.mutex, [&]() IPLR_REQUIRES(test.mutex) {
      test.rando_ivar = 12;  // <-- Static analysics error without
                             // the IPLR_REQUIRES on the pred.
      return true;
    });
    test.mutex.Unlock();
  }
  Lock lock(test.mutex);  // <--- Static analysis error without this.
  // The wake only happens when the predicate returns true.
  ASSERT_EQ(test.rando_ivar, 12u);
}

TEST(ConditionVariableTest, TestsCriticalSectionAfterWaitForUntil) {
  std::vector<std::thread> threads;
  const auto kThreadCount = 10u;

  Mutex mtx;
  ConditionVariable cv;
  size_t sum = 0u;

  std::condition_variable start_cv;
  std::mutex start_mtx;
  bool start = false;
  auto start_predicate = [&start]() { return start; };
  auto thread_main = [&]() {
    {
      std::unique_lock start_lock(start_mtx);
      start_cv.wait(start_lock, start_predicate);
    }

    mtx.Lock();
    cv.WaitFor(mtx, std::chrono::milliseconds{0u}, []() { return true; });
    auto old_val = sum;
    std::this_thread::sleep_for(std::chrono::milliseconds{100u});
    sum = old_val + 1u;
    mtx.Unlock();
  };
  // Launch all threads. They will wait for the start CV to be signaled.
  for (size_t i = 0; i < kThreadCount; i++) {
    threads.emplace_back(thread_main);
  }
  // Notify all threads that the test may start.
  {
    {
      std::scoped_lock start_lock(start_mtx);
      start = true;
    }
    start_cv.notify_all();
  }
  // Join all threads.
  ASSERT_EQ(threads.size(), kThreadCount);
  for (size_t i = 0; i < kThreadCount; i++) {
    threads[i].join();
  }
  ASSERT_EQ(sum, kThreadCount);
}

TEST(ConditionVariableTest, TestsCriticalSectionAfterWait) {
  std::vector<std::thread> threads;
  const auto kThreadCount = 10u;

  Mutex mtx;
  ConditionVariable cv;
  size_t sum = 0u;

  std::condition_variable start_cv;
  std::mutex start_mtx;
  bool start = false;
  auto start_predicate = [&start]() { return start; };
  auto thread_main = [&]() {
    {
      std::unique_lock start_lock(start_mtx);
      start_cv.wait(start_lock, start_predicate);
    }

    mtx.Lock();
    cv.Wait(mtx, []() { return true; });
    auto old_val = sum;
    std::this_thread::sleep_for(std::chrono::milliseconds{100u});
    sum = old_val + 1u;
    mtx.Unlock();
  };
  // Launch all threads. They will wait for the start CV to be signaled.
  for (size_t i = 0; i < kThreadCount; i++) {
    threads.emplace_back(thread_main);
  }
  // Notify all threads that the test may start.
  {
    {
      std::scoped_lock start_lock(start_mtx);
      start = true;
    }
    start_cv.notify_all();
  }
  // Join all threads.
  ASSERT_EQ(threads.size(), kThreadCount);
  for (size_t i = 0; i < kThreadCount; i++) {
    threads[i].join();
  }
  ASSERT_EQ(sum, kThreadCount);
}

TEST(BaseTest, NoExceptionPromiseValue) {
  NoExceptionPromise<int> wrapper;
  std::future future = wrapper.get_future();
  wrapper.set_value(123);
  ASSERT_EQ(future.get(), 123);
}

TEST(BaseTest, NoExceptionPromiseEmpty) {
  auto wrapper = std::make_shared<NoExceptionPromise<int>>();
  std::future future = wrapper->get_future();

  // Destroy the empty promise with the future still pending. Verify that the
  // process does not abort while destructing the promise.
  wrapper.reset();
}

TEST(BaseTest, CanUseTypedMasks) {
  {
    MyMask mask;
    ASSERT_EQ(static_cast<uint32_t>(mask), 0u);
    ASSERT_FALSE(mask);
  }

  {
    MyMask mask(MyMaskBits::kBar);
    ASSERT_EQ(static_cast<uint32_t>(mask), 1u);
    ASSERT_TRUE(mask);
  }

  {
    MyMask mask2(MyMaskBits::kBar);
    MyMask mask(mask2);
    ASSERT_EQ(static_cast<uint32_t>(mask), 1u);
    ASSERT_TRUE(mask);
  }

  {
    MyMask mask2(MyMaskBits::kBar);
    MyMask mask(std::move(mask2));  // NOLINT
    ASSERT_EQ(static_cast<uint32_t>(mask), 1u);
    ASSERT_TRUE(mask);
  }

  ASSERT_LT(MyMaskBits::kBar, MyMaskBits::kBaz);
  ASSERT_LE(MyMaskBits::kBar, MyMaskBits::kBaz);
  ASSERT_GT(MyMaskBits::kBaz, MyMaskBits::kBar);
  ASSERT_GE(MyMaskBits::kBaz, MyMaskBits::kBar);
  ASSERT_EQ(MyMaskBits::kBaz, MyMaskBits::kBaz);
  ASSERT_NE(MyMaskBits::kBaz, MyMaskBits::kBang);

  {
    MyMask m1(MyMaskBits::kBar);
    MyMask m2(MyMaskBits::kBaz);
    ASSERT_EQ(static_cast<uint32_t>(m1 & m2), 0u);
    ASSERT_FALSE(m1 & m2);
  }

  {
    MyMask m1(MyMaskBits::kBar);
    MyMask m2(MyMaskBits::kBaz);
    ASSERT_EQ(static_cast<uint32_t>(m1 | m2), ((1u << 0u) | (1u << 1u)));
    ASSERT_TRUE(m1 | m2);
  }

  {
    MyMask m1(MyMaskBits::kBar);
    MyMask m2(MyMaskBits::kBaz);
    ASSERT_EQ(static_cast<uint32_t>(m1 ^ m2), ((1u << 0u) ^ (1u << 1u)));
    ASSERT_TRUE(m1 ^ m2);
  }

  {
    MyMask m1(MyMaskBits::kBar);
    ASSERT_EQ(static_cast<uint32_t>(~m1), (~(1u << 0u)));
    ASSERT_TRUE(m1);
  }

  {
    MyMask m1 = MyMaskBits::kBar;
    MyMask m2 = MyMaskBits::kBaz;
    m2 = m1;
    ASSERT_EQ(m2, MyMaskBits::kBar);
  }

  {
    MyMask m = MyMaskBits::kBar | MyMaskBits::kBaz;
    ASSERT_TRUE(m);
  }

  {
    MyMask m = MyMaskBits::kBar & MyMaskBits::kBaz;
    ASSERT_FALSE(m);
  }

  {
    MyMask m = MyMaskBits::kBar ^ MyMaskBits::kBaz;
    ASSERT_TRUE(m);
  }

  {
    MyMask m = ~MyMaskBits::kBar;
    ASSERT_TRUE(m);
  }

  {
    MyMask m1 = MyMaskBits::kBar;
    MyMask m2 = MyMaskBits::kBaz;
    m2 |= m1;
    ASSERT_EQ(m1, MyMaskBits::kBar);
    MyMask pred = MyMaskBits::kBar | MyMaskBits::kBaz;
    ASSERT_EQ(m2, pred);
  }

  {
    MyMask m1 = MyMaskBits::kBar;
    MyMask m2 = MyMaskBits::kBaz;
    m2 &= m1;
    ASSERT_EQ(m1, MyMaskBits::kBar);
    MyMask pred = MyMaskBits::kBar & MyMaskBits::kBaz;
    ASSERT_EQ(m2, pred);
  }

  {
    MyMask m1 = MyMaskBits::kBar;
    MyMask m2 = MyMaskBits::kBaz;
    m2 ^= m1;
    ASSERT_EQ(m1, MyMaskBits::kBar);
    MyMask pred = MyMaskBits::kBar ^ MyMaskBits::kBaz;
    ASSERT_EQ(m2, pred);
  }

  {
    MyMask x = MyMaskBits::kBar;
    MyMask m = x | MyMaskBits::kBaz;
    ASSERT_TRUE(m);
  }

  {
    MyMask x = MyMaskBits::kBar;
    MyMask m = MyMaskBits::kBaz | x;
    ASSERT_TRUE(m);
  }

  {
    MyMask x = MyMaskBits::kBar;
    MyMask m = x & MyMaskBits::kBaz;
    ASSERT_FALSE(m);
  }

  {
    MyMask x = MyMaskBits::kBar;
    MyMask m = MyMaskBits::kBaz & x;
    ASSERT_FALSE(m);
  }

  {
    MyMask x = MyMaskBits::kBar;
    MyMask m = x ^ MyMaskBits::kBaz;
    ASSERT_TRUE(m);
  }

  {
    MyMask x = MyMaskBits::kBar;
    MyMask m = MyMaskBits::kBaz ^ x;
    ASSERT_TRUE(m);
  }

  {
    MyMask x = MyMaskBits::kBar;
    MyMask m = ~x;
    ASSERT_TRUE(m);
  }

  {
    MyMaskBits x = MyMaskBits::kBar;
    MyMask m = MyMaskBits::kBaz;
    ASSERT_TRUE(x < m);
    ASSERT_TRUE(x <= m);
  }

  {
    MyMaskBits x = MyMaskBits::kBar;
    MyMask m = MyMaskBits::kBaz;
    ASSERT_FALSE(x == m);
  }

  {
    MyMaskBits x = MyMaskBits::kBar;
    MyMask m = MyMaskBits::kBaz;
    ASSERT_TRUE(x != m);
  }

  {
    MyMaskBits x = MyMaskBits::kBar;
    MyMask m = MyMaskBits::kBaz;
    ASSERT_FALSE(x > m);
    ASSERT_FALSE(x >= m);
  }
}

}  // namespace testing
}  // namespace impeller
