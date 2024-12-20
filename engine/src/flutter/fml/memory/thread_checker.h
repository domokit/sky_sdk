// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// A class for checking that the current thread is/isn't the same as an initial
// thread.

#ifndef FLUTTER_FML_MEMORY_THREAD_CHECKER_H_
#define FLUTTER_FML_MEMORY_THREAD_CHECKER_H_

#include "flutter/fml/build_config.h"
#include "flutter/fml/logging.h"
#include "flutter/fml/macros.h"

#if defined(FML_OS_WIN)
#include <windows.h>
#else
#include <pthread.h>
#endif

namespace fml {

// A simple class that records the identity of the thread that it was created
// on, and at later points can tell if the current thread is the same as its
// creation thread. This class is thread-safe.
//
// Note: Unlike Chromium's |base::ThreadChecker|, this is *not* Debug-only (so
// #ifdef it out if you want something Debug-only). (Rationale: Having a
// |CalledOnValidThread()| that lies in Release builds seems bad. Moreover,
// there's a small space cost to having even an empty class. )
class ThreadChecker final {
 public:
  static void DisableNextThreadCheckFailure() { disable_next_failure_ = true; }

 private:
  static thread_local bool disable_next_failure_;

 public:
#if defined(FML_OS_WIN)
  ThreadChecker() : self_(GetCurrentThreadId()) {}
  ~ThreadChecker() {}

  bool IsCreationThreadCurrent() const {
    bool result = GetCurrentThreadId() == self_;
    if (!result && disable_next_failure_) {
      disable_next_failure_ = false;
      return true;
    }
    return result;
  }

 private:
  DWORD self_;

#else
  ThreadChecker() : self_(pthread_self()) {}
  ~ThreadChecker() {}

  // Returns true if the current thread is the thread this object was created
  // on and false otherwise.
  bool IsCreationThreadCurrent() const {
    pthread_t current_thread = pthread_self();
    bool is_creation_thread_current = !!pthread_equal(current_thread, self_);
    if (disable_next_failure_ && !is_creation_thread_current) {
      disable_next_failure_ = false;
      return true;
    }
#ifdef __APPLE__
    // TODO(https://github.com/flutter/flutter/issues/45272): Implement for
    // other platforms.
    if (!is_creation_thread_current) {
      static const int buffer_length = 128;
      char expected_thread[buffer_length];
      char actual_thread[buffer_length];
      if (0 == pthread_getname_np(current_thread, actual_thread,
                                  buffer_length) &&
          0 == pthread_getname_np(self_, expected_thread, buffer_length)) {
        FML_DLOG(ERROR) << "Object referenced on a thread other than the one "
                           "on which it was created. Expected thread: '"
                        << expected_thread << "'. Actual thread: '"
                        << actual_thread << "'.";
      }
    }
#endif  // __APPLE__
    return is_creation_thread_current;
  }

 private:
  pthread_t self_;
#endif
};

#if !defined(NDEBUG)
#define FML_DECLARE_THREAD_CHECKER(c) fml::ThreadChecker c
#define FML_DCHECK_CREATION_THREAD_IS_CURRENT(c) \
  FML_DCHECK((c).IsCreationThreadCurrent())
#else
#define FML_DECLARE_THREAD_CHECKER(c)
#define FML_DCHECK_CREATION_THREAD_IS_CURRENT(c) ((void)0)
#endif

}  // namespace fml

#endif  // FLUTTER_FML_MEMORY_THREAD_CHECKER_H_
