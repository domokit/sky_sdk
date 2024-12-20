// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/synchronization/waitable_event.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <thread>
#include <type_traits>
#include <vector>

#include "flutter/fml/macros.h"
#include "gtest/gtest.h"

// rand() is only used for tests in this file.
// NOLINTBEGIN(clang-analyzer-security.insecureAPI.rand)

namespace fml {
namespace {

constexpr TimeDelta kEpsilonTimeout = TimeDelta::FromMilliseconds(20);
constexpr TimeDelta kTinyTimeout = TimeDelta::FromMilliseconds(100);
constexpr TimeDelta kActionTimeout = TimeDelta::FromMilliseconds(10000);

// Sleeps for a "very small" amount of time.

void SleepFor(TimeDelta duration) {
  std::this_thread::sleep_for(
      std::chrono::nanoseconds(duration.ToNanoseconds()));
}

void EpsilonRandomSleep() {
  TimeDelta duration =
      TimeDelta::FromMilliseconds(static_cast<unsigned>(rand()) % 20u);
  SleepFor(duration);
}

// AutoResetWaitableEvent ------------------------------------------------------

TEST(AutoResetWaitableEventTest, Basic) {
  AutoResetWaitableEvent ev;
  EXPECT_FALSE(ev.IsSignaledForTest());
  ev.Signal();
  EXPECT_TRUE(ev.IsSignaledForTest());
  ev.Wait();
  EXPECT_FALSE(ev.IsSignaledForTest());
  ev.Reset();
  EXPECT_FALSE(ev.IsSignaledForTest());
  ev.Signal();
  EXPECT_TRUE(ev.IsSignaledForTest());
  ev.Reset();
  EXPECT_FALSE(ev.IsSignaledForTest());
  EXPECT_TRUE(ev.WaitWithTimeout(TimeDelta::Zero()));
  EXPECT_FALSE(ev.IsSignaledForTest());
  EXPECT_TRUE(ev.WaitWithTimeout(TimeDelta::FromMilliseconds(1)));
  EXPECT_FALSE(ev.IsSignaledForTest());
  ev.Signal();
  EXPECT_TRUE(ev.IsSignaledForTest());
  EXPECT_FALSE(ev.WaitWithTimeout(TimeDelta::Zero()));
  EXPECT_FALSE(ev.IsSignaledForTest());
  EXPECT_TRUE(ev.WaitWithTimeout(TimeDelta::FromMilliseconds(1)));
  EXPECT_FALSE(ev.IsSignaledForTest());
  ev.Signal();
  EXPECT_FALSE(ev.WaitWithTimeout(TimeDelta::FromMilliseconds(1)));
  EXPECT_FALSE(ev.IsSignaledForTest());
}

TEST(AutoResetWaitableEventTest, MultipleWaiters) {
  AutoResetWaitableEvent ev;

  for (size_t i = 0u; i < 5u; i++) {
    std::atomic_uint wake_count(0u);
    std::vector<std::thread> threads;
    for (size_t j = 0u; j < 4u; j++) {
      threads.push_back(std::thread([&ev, &wake_count]() {
        if (rand() % 2 == 0) {
          ev.Wait();
        } else {
          EXPECT_FALSE(ev.WaitWithTimeout(kActionTimeout));
        }
        wake_count.fetch_add(1u);
        // Note: We can't say anything about the signaled state of |ev| here,
        // since the main thread may have already signaled it again.
      }));
    }

    // Unfortunately, we can't really wait for the threads to be waiting, so we
    // just sleep for a bit, and count on them having started and advanced to
    // waiting.
    SleepFor(kTinyTimeout + kTinyTimeout);

    for (size_t j = 0u; j < threads.size(); j++) {
      unsigned old_wake_count = wake_count.load();
      EXPECT_EQ(j, old_wake_count);

      // Each |Signal()| should wake exactly one thread.
      ev.Signal();

      // Poll for |wake_count| to change.
      while (wake_count.load() == old_wake_count) {
        SleepFor(kEpsilonTimeout);
      }

      EXPECT_FALSE(ev.IsSignaledForTest());

      // And once it's changed, wait a little longer, to see if any other
      // threads are awoken (they shouldn't be).
      SleepFor(kEpsilonTimeout);

      EXPECT_EQ(old_wake_count + 1u, wake_count.load());

      EXPECT_FALSE(ev.IsSignaledForTest());
    }

    // Having done that, if we signal |ev| now, it should stay signaled.
    ev.Signal();
    SleepFor(kEpsilonTimeout);
    EXPECT_TRUE(ev.IsSignaledForTest());

    for (auto& thread : threads) {
      thread.join();
    }

    ev.Reset();
  }
}

// ManualResetWaitableEvent ----------------------------------------------------

TEST(ManualResetWaitableEventTest, Basic) {
  ManualResetWaitableEvent ev;
  EXPECT_FALSE(ev.IsSignaledForTest());
  ev.Signal();
  EXPECT_TRUE(ev.IsSignaledForTest());
  ev.Wait();
  EXPECT_TRUE(ev.IsSignaledForTest());
  ev.Reset();
  EXPECT_FALSE(ev.IsSignaledForTest());
  EXPECT_TRUE(ev.WaitWithTimeout(TimeDelta::Zero()));
  EXPECT_FALSE(ev.IsSignaledForTest());
  EXPECT_TRUE(ev.WaitWithTimeout(TimeDelta::FromMilliseconds(1)));
  EXPECT_FALSE(ev.IsSignaledForTest());
  ev.Signal();
  EXPECT_TRUE(ev.IsSignaledForTest());
  EXPECT_FALSE(ev.WaitWithTimeout(TimeDelta::Zero()));
  EXPECT_TRUE(ev.IsSignaledForTest());
  EXPECT_FALSE(ev.WaitWithTimeout(TimeDelta::FromMilliseconds(1)));
  EXPECT_TRUE(ev.IsSignaledForTest());
}

TEST(ManualResetWaitableEventTest, SignalMultiple) {
  ManualResetWaitableEvent ev;

  for (size_t i = 0u; i < 10u; i++) {
    for (size_t num_waiters = 1u; num_waiters < 5u; num_waiters++) {
      std::vector<std::thread> threads;
      for (size_t j = 0u; j < num_waiters; j++) {
        threads.push_back(std::thread([&ev]() {
          EpsilonRandomSleep();

          if (rand() % 2 == 0) {
            ev.Wait();
          } else {
            EXPECT_FALSE(ev.WaitWithTimeout(kActionTimeout));
          }
        }));
      }

      EpsilonRandomSleep();

      ev.Signal();

      // The threads will only terminate once they've successfully waited (or
      // timed out).
      for (auto& thread : threads) {
        thread.join();
      }

      ev.Reset();
    }
  }
}

}  // namespace
}  // namespace fml

// NOLINTEND(clang-analyzer-security.insecureAPI.rand)
