// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/platform/win/message_loop_win.h"

#include <VersionHelpers.h>
#include <timeapi.h>

#include "flutter/fml/logging.h"

constexpr uint32_t kHighResolutionTimer = 1;  // 1 ms
constexpr uint32_t kLowResolutionTimer = 15;  // 15 ms

namespace fml {

MessageLoopWin::MessageLoopWin()
    : timer_(CreateWaitableTimer(NULL, FALSE, NULL)) {
  FML_CHECK(timer_.is_valid());
  // Flutter uses timers to schedule frames. By default, Windows timers do
  // not have the precision to reliably schedule frame rates greater than
  // 60hz. We can increase the precision, but on versions of Windows before
  // 10, this would globally increase timer precision leading to increased
  // resource usage. This would be particularly problematic on a laptop or
  // mobile device.
  if (IsWindows10OrGreater()) {
    timer_resolution_ = kHighResolutionTimer;
  } else {
    timer_resolution_ = kLowResolutionTimer;
  }
  timeBeginPeriod(timer_resolution_);
}

MessageLoopWin::~MessageLoopWin() = default;

void MessageLoopWin::Run() {
  running_ = true;

  while (running_) {
    FML_CHECK(WaitForSingleObject(timer_.get(), INFINITE) == 0);
    RunExpiredTasksNow();
  }
}

void MessageLoopWin::Terminate() {
  running_ = false;
  WakeUp(fml::TimePoint::Now());
  timeEndPeriod(timer_resolution_);
}

void MessageLoopWin::WakeUp(fml::TimePoint time_point) {
  LARGE_INTEGER due_time = {0};
  fml::TimePoint now = fml::TimePoint::Now();
  if (time_point > now) {
    due_time.QuadPart = (time_point - now).ToNanoseconds() / -100;
  }
  FML_CHECK(SetWaitableTimer(timer_.get(), &due_time, 0, NULL, NULL, FALSE));
}

}  // namespace fml
