// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// Provides classes with functionality analogous to (but much more limited than)
// Chromium's |base::WaitableEvent|, which in turn provides functionality
// analogous to Windows's Event. (Unlike these two, we have separate types for
// the manual- and auto-reset versions.)

#ifndef FLUTTER_FML_SYNCHRONIZATION_WAITABLE_EVENT_H_
#define FLUTTER_FML_SYNCHRONIZATION_WAITABLE_EVENT_H_

#include <condition_variable>
#include <mutex>

#include "flutter/fml/macros.h"
#include "flutter/fml/time/time_delta.h"

namespace fml {

// AutoResetWaitableEvent ------------------------------------------------------

// An event that can be signaled and waited on. This version automatically
// returns to the unsignaled state after unblocking one waiter. (This is similar
// to Windows's auto-reset Event, which is also imitated by Chromium's
// auto-reset |base::WaitableEvent|. However, there are some limitations -- see
// |Signal()|.) This class is thread-safe.
class AutoResetWaitableEvent final {
 public:
  AutoResetWaitableEvent() {}
  ~AutoResetWaitableEvent() {}

  // Put the event in the signaled state. Exactly one |Wait()| will be unblocked
  // and the event will be returned to the unsignaled state.
  //
  // Notes (these are arguably bugs, but not worth working around):
  // * That |Wait()| may be one that occurs on the calling thread, *after* the
  //   call to |Signal()|.
  // * A |Signal()|, followed by a |Reset()|, may cause *no* waiting thread to
  //   be unblocked.
  // * We rely on pthreads's queueing for picking which waiting thread to
  //   unblock, rather than enforcing FIFO ordering.
  void Signal();

  // Put the event into the unsignaled state. Generally, this is not recommended
  // on an auto-reset event (see notes above).
  void Reset();

  // Blocks the calling thread until the event is signaled. Upon unblocking, the
  // event is returned to the unsignaled state, so that (unless |Reset()| is
  // called) each |Signal()| unblocks exactly one |Wait()|.
  void Wait();

  // Like |Wait()|, but with a timeout. Also unblocks if |timeout| expires
  // without being signaled in which case it returns true (otherwise, it returns
  // false).
  bool WaitWithTimeout(TimeDelta timeout);

  // Returns whether this event is in a signaled state or not. For use in tests
  // only (in general, this is racy). Note: Unlike
  // |base::WaitableEvent::IsSignaled()|, this doesn't reset the signaled state.
  bool IsSignaledForTest();

 private:
  std::condition_variable cv_;
  std::mutex mutex_;

  // True if this event is in the signaled state.
  bool signaled_ = false;

  FML_DISALLOW_COPY_AND_ASSIGN(AutoResetWaitableEvent);
};

// ManualResetWaitableEvent ----------------------------------------------------

// An event that can be signaled and waited on. This version remains signaled
// until explicitly reset. (This is similar to Windows's manual-reset Event,
// which is also imitated by Chromium's manual-reset |base::WaitableEvent|.)
// This class is thread-safe.
class ManualResetWaitableEvent final {
 public:
  ManualResetWaitableEvent() {}
  ~ManualResetWaitableEvent() {}

  // Put the event into the unsignaled state.
  void Reset();

  // Put the event in the signaled state. If this is a manual-reset event, it
  // wakes all waiting threads (blocked on |Wait()| or |WaitWithTimeout()|).
  // Otherwise, it wakes a single waiting thread (and returns to the unsignaled
  // state), if any; if there are none, it remains signaled.
  void Signal();

  // Blocks the calling thread until the event is signaled.
  void Wait();

  // Like |Wait()|, but with a timeout. Also unblocks if |timeout| expires
  // without being signaled in which case it returns true (otherwise, it returns
  // false).
  bool WaitWithTimeout(TimeDelta timeout);

  // Returns whether this event is in a signaled state or not. For use in tests
  // only (in general, this is racy).
  bool IsSignaledForTest();

 private:
  std::condition_variable cv_;
  std::mutex mutex_;

  // True if this event is in the signaled state.
  bool signaled_ = false;

  // While |std::condition_variable::notify_all()| (|pthread_cond_broadcast()|)
  // will wake all waiting threads, one has to deal with spurious wake-ups.
  // Checking |signaled_| isn't sufficient, since another thread may have been
  // awoken and (manually) reset |signaled_|. This is a counter that is
  // incremented in |Signal()| before calling
  // |std::condition_variable::notify_all()|. A waiting thread knows it was
  // awoken if |signal_id_| is different from when it started waiting.
  unsigned signal_id_ = 0u;

  FML_DISALLOW_COPY_AND_ASSIGN(ManualResetWaitableEvent);
};

}  // namespace fml

#endif  // FLUTTER_FML_SYNCHRONIZATION_WAITABLE_EVENT_H_
