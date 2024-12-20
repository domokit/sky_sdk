// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/synchronization/waitable_event.h"

#include <cerrno>
#include <ctime>

#include "flutter/fml/logging.h"
#include "flutter/fml/time/time_delta.h"
#include "flutter/fml/time/time_point.h"

namespace fml {

// Waits with a timeout on |condition()|. Returns true on timeout, or false if
// |condition()| ever returns true. |condition()| should have no side effects
// (and will always be called with |*mutex| held).
template <typename ConditionFn>
bool WaitWithTimeoutImpl(std::unique_lock<std::mutex>* locker,
                         std::condition_variable* cv,
                         ConditionFn condition,
                         TimeDelta timeout) {
  FML_DCHECK(locker->owns_lock());

  if (condition()) {
    return false;
  }

  // We may get spurious wakeups.
  TimeDelta wait_remaining = timeout;
  TimePoint start = TimePoint::Now();
  while (true) {
    if (std::cv_status::timeout ==
        cv->wait_for(*locker, std::chrono::nanoseconds(
                                  wait_remaining.ToNanoseconds()))) {
      return true;  // Definitely timed out.
    }

    // We may have been awoken.
    if (condition()) {
      return false;
    }

    // Or the wakeup may have been spurious.
    TimePoint now = TimePoint::Now();
    FML_DCHECK(now >= start);
    TimeDelta elapsed = now - start;
    // It's possible that we may have timed out anyway.
    if (elapsed >= timeout) {
      return true;
    }

    // Otherwise, recalculate the amount that we have left to wait.
    wait_remaining = timeout - elapsed;
  }
}

// AutoResetWaitableEvent ------------------------------------------------------

void AutoResetWaitableEvent::Signal() {
  std::scoped_lock locker(mutex_);
  signaled_ = true;
  cv_.notify_one();
}

void AutoResetWaitableEvent::Reset() {
  std::scoped_lock locker(mutex_);
  signaled_ = false;
}

void AutoResetWaitableEvent::Wait() {
  std::unique_lock<std::mutex> locker(mutex_);
  while (!signaled_) {
    cv_.wait(locker);
  }
  signaled_ = false;
}

bool AutoResetWaitableEvent::WaitWithTimeout(TimeDelta timeout) {
  std::unique_lock<std::mutex> locker(mutex_);

  if (signaled_) {
    signaled_ = false;
    return false;
  }

  // We may get spurious wakeups.
  TimeDelta wait_remaining = timeout;
  TimePoint start = TimePoint::Now();
  while (true) {
    if (std::cv_status::timeout ==
        cv_.wait_for(
            locker, std::chrono::nanoseconds(wait_remaining.ToNanoseconds()))) {
      return true;  // Definitely timed out.
    }

    // We may have been awoken.
    if (signaled_) {
      break;
    }

    // Or the wakeup may have been spurious.
    TimePoint now = TimePoint::Now();
    FML_DCHECK(now >= start);
    TimeDelta elapsed = now - start;
    // It's possible that we may have timed out anyway.
    if (elapsed >= timeout) {
      return true;
    }

    // Otherwise, recalculate the amount that we have left to wait.
    wait_remaining = timeout - elapsed;
  }

  signaled_ = false;
  return false;
}

bool AutoResetWaitableEvent::IsSignaledForTest() {
  std::scoped_lock locker(mutex_);
  return signaled_;
}

// ManualResetWaitableEvent ----------------------------------------------------

void ManualResetWaitableEvent::Signal() {
  std::scoped_lock locker(mutex_);
  signaled_ = true;
  signal_id_++;
  cv_.notify_all();
}

void ManualResetWaitableEvent::Reset() {
  std::scoped_lock locker(mutex_);
  signaled_ = false;
}

void ManualResetWaitableEvent::Wait() {
  std::unique_lock<std::mutex> locker(mutex_);

  if (signaled_) {
    return;
  }

  auto last_signal_id = signal_id_;
  do {
    cv_.wait(locker);
  } while (signal_id_ == last_signal_id);
}

bool ManualResetWaitableEvent::WaitWithTimeout(TimeDelta timeout) {
  std::unique_lock<std::mutex> locker(mutex_);

  auto last_signal_id = signal_id_;
  // Disable thread-safety analysis for the lambda: We could annotate it with
  // |FML_EXCLUSIVE_LOCKS_REQUIRED(mutex_)|, but then the analyzer currently
  // isn't able to figure out that |WaitWithTimeoutImpl()| calls it while
  // holding |mutex_|.
  bool rv = WaitWithTimeoutImpl(
      &locker, &cv_,
      [this, last_signal_id]() {
        // Also check |signaled_| in case we're already signaled.
        return signaled_ || signal_id_ != last_signal_id;
      },
      timeout);
  FML_DCHECK(rv || signaled_ || signal_id_ != last_signal_id);
  return rv;
}

bool ManualResetWaitableEvent::IsSignaledForTest() {
  std::scoped_lock locker(mutex_);
  return signaled_;
}

}  // namespace fml
