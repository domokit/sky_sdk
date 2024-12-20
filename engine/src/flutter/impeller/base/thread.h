// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_BASE_THREAD_H_
#define FLUTTER_IMPELLER_BASE_THREAD_H_

#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <thread>

#include "impeller/base/thread_safety.h"

namespace impeller {

class ConditionVariable;

class IPLR_CAPABILITY("mutex") Mutex {
 public:
  Mutex() = default;

  ~Mutex() = default;

  void Lock() IPLR_ACQUIRE() { mutex_.lock(); }

  void Unlock() IPLR_RELEASE() { mutex_.unlock(); }

 private:
  friend class ConditionVariable;

  std::mutex mutex_;

  Mutex(const Mutex&) = delete;

  Mutex(Mutex&&) = delete;

  Mutex& operator=(const Mutex&) = delete;

  Mutex& operator=(Mutex&&) = delete;
};

class IPLR_CAPABILITY("mutex") RWMutex {
 public:
  RWMutex() = default;

  ~RWMutex() = default;

  void LockWriter() IPLR_ACQUIRE() { mutex_.lock(); }

  void UnlockWriter() IPLR_RELEASE() { mutex_.unlock(); }

  void LockReader() IPLR_ACQUIRE_SHARED() { mutex_.lock_shared(); }

  void UnlockReader() IPLR_RELEASE_SHARED() { mutex_.unlock_shared(); }

 private:
  std::shared_mutex mutex_;

  RWMutex(const RWMutex&) = delete;

  RWMutex(RWMutex&&) = delete;

  RWMutex& operator=(const RWMutex&) = delete;

  RWMutex& operator=(RWMutex&&) = delete;
};

class IPLR_SCOPED_CAPABILITY Lock {
 public:
  explicit Lock(Mutex& mutex) IPLR_ACQUIRE(mutex) : mutex_(mutex) {
    mutex_.Lock();
  }

  ~Lock() IPLR_RELEASE() { mutex_.Unlock(); }

 private:
  Mutex& mutex_;

  Lock(const Lock&) = delete;

  Lock(Lock&&) = delete;

  Lock& operator=(const Lock&) = delete;

  Lock& operator=(Lock&&) = delete;
};

class IPLR_SCOPED_CAPABILITY ReaderLock {
 public:
  explicit ReaderLock(RWMutex& mutex) IPLR_ACQUIRE_SHARED(mutex)
      : mutex_(mutex) {
    mutex_.LockReader();
  }

  ~ReaderLock() IPLR_RELEASE() { mutex_.UnlockReader(); }

 private:
  RWMutex& mutex_;

  ReaderLock(const ReaderLock&) = delete;

  ReaderLock(ReaderLock&&) = delete;

  ReaderLock& operator=(const ReaderLock&) = delete;

  ReaderLock& operator=(ReaderLock&&) = delete;
};

class IPLR_SCOPED_CAPABILITY WriterLock {
 public:
  explicit WriterLock(RWMutex& mutex) IPLR_ACQUIRE(mutex) : mutex_(mutex) {
    mutex_.LockWriter();
  }

  ~WriterLock() IPLR_RELEASE() { mutex_.UnlockWriter(); }

 private:
  RWMutex& mutex_;

  WriterLock(const WriterLock&) = delete;

  WriterLock(WriterLock&&) = delete;

  WriterLock& operator=(const WriterLock&) = delete;

  WriterLock& operator=(WriterLock&&) = delete;
};

//------------------------------------------------------------------------------
/// @brief      A condition variable exactly similar to the one in libcxx with
///             two major differences:
///
///             * On the Wait, WaitFor, and WaitUntil calls, static analysis
///               annotation are respected.
///             * There is no ability to wait on a condition variable and also
///               be susceptible to spurious wakes. This is because the
///               predicate is mandatory.
///
class ConditionVariable {
 public:
  ConditionVariable() = default;

  ~ConditionVariable() = default;

  ConditionVariable(const ConditionVariable&) = delete;

  ConditionVariable& operator=(const ConditionVariable&) = delete;

  void NotifyOne() { cv_.notify_one(); }

  void NotifyAll() { cv_.notify_all(); }

  using Predicate = std::function<bool()>;

  //----------------------------------------------------------------------------
  /// @brief      Atomically unlocks the mutex and waits on the condition
  ///             variable up to a specified time point. Lock will be reacquired
  ///             when the wait exits. Spurious wakes may happen before the time
  ///             point is reached. In such cases the predicate is invoked and
  ///             it must return `false` for the wait to continue. The predicate
  ///             will be invoked with the mutex locked.
  ///
  /// @note       Since the predicate is invoked with the mutex locked, if it
  ///             accesses other guarded resources, the predicate itself must be
  ///             decorated with the IPLR_REQUIRES directive. For instance,
  ///
  ///             ```c++
  ///                [] () IPLR_REQUIRES(mutex) {
  ///                return my_guarded_resource.should_stop_waiting;
  ///              }
  ///              ```
  ///
  /// @param      mutex                The mutex.
  /// @param[in]  time_point           The time point to wait to.
  /// @param[in]  should_stop_waiting  The predicate invoked on spurious wakes.
  ///                                  Must return false for the wait to
  ///                                  continue.
  ///
  /// @tparam     Clock                The clock type.
  /// @tparam     Duration             The duration type.
  ///
  /// @return     The value of the predicate at the end of the wait.
  ///
  template <class Clock, class Duration>
  bool WaitUntil(Mutex& mutex,
                 const std::chrono::time_point<Clock, Duration>& time_point,
                 const Predicate& should_stop_waiting) IPLR_REQUIRES(mutex) {
    std::unique_lock lock(mutex.mutex_, std::adopt_lock);
    const auto result = cv_.wait_until(lock, time_point, should_stop_waiting);
    lock.release();
    return result;
  }

  //----------------------------------------------------------------------------
  /// @brief      Atomically unlocks the mutex and waits on the condition
  ///             variable for a designated duration. Lock will be reacquired
  ///             when the wait exits. Spurious wakes may happen before the time
  ///             point is reached. In such cases the predicate is invoked and
  ///             it must return `false` for the wait to continue. The predicate
  ///             will be invoked with the mutex locked.
  ///
  /// @note       Since the predicate is invoked with the mutex locked, if it
  ///             accesses other guarded resources, the predicate itself must be
  ///             decorated with the IPLR_REQUIRES directive. For instance,
  ///
  ///             ```c++
  ///                [] () IPLR_REQUIRES(mutex) {
  ///                return my_guarded_resource.should_stop_waiting;
  ///              }
  ///              ```
  ///
  /// @param      mutex                The mutex.
  /// @param[in]  duration             The duration to wait for.
  /// @param[in]  should_stop_waiting  The predicate invoked on spurious wakes.
  ///                                  Must return false for the wait to
  ///                                  continue.
  ///
  /// @tparam     Representation       The duration representation type.
  /// @tparam     Period               The duration period type.
  ///
  /// @return     The value of the predicate at the end of the wait.
  ///
  template <class Representation, class Period>
  bool WaitFor(Mutex& mutex,
               const std::chrono::duration<Representation, Period>& duration,
               const Predicate& should_stop_waiting) IPLR_REQUIRES(mutex) {
    return WaitUntil(mutex, std::chrono::steady_clock::now() + duration,
                     should_stop_waiting);
  }

  //----------------------------------------------------------------------------
  /// @brief      Atomically unlocks the mutex and waits on the condition
  ///             variable indefinitely till the predicate determines that the
  ///             wait must end. Lock will be reacquired when the wait exits.
  ///             Spurious wakes may happen before the time point is reached. In
  ///             such cases the predicate is invoked and it must return `false`
  ///             for the wait to continue. The predicate will be invoked with
  ///             the mutex locked.
  ///
  /// @note       Since the predicate is invoked with the mutex locked, if it
  ///             accesses other guarded resources, the predicate itself must be
  ///             decorated with the IPLR_REQUIRES directive. For instance,
  ///
  ///             ```c++
  ///                [] () IPLR_REQUIRES(mutex) {
  ///                return my_guarded_resource.should_stop_waiting;
  ///              }
  ///              ```
  ///
  /// @param      mutex                The mutex
  /// @param[in]  should_stop_waiting  The should stop waiting
  ///
  void Wait(Mutex& mutex, const Predicate& should_stop_waiting)
      IPLR_REQUIRES(mutex) {
    std::unique_lock lock(mutex.mutex_, std::adopt_lock);
    cv_.wait(lock, should_stop_waiting);
    lock.release();
  }

 private:
  std::condition_variable cv_;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_BASE_THREAD_H_
