// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/platform/darwin/message_loop_darwin.h"

#include <CoreFoundation/CFRunLoop.h>
#include <Foundation/Foundation.h>

#include "flutter/fml/logging.h"

namespace fml {

static constexpr CFTimeInterval kDistantFuture = 1.0e10;

CFStringRef MessageLoopDarwin::kMessageLoopCFRunLoopMode = CFSTR("fmlMessageLoop");

MessageLoopDarwin::MessageLoopDarwin()
    : running_(false), loop_((CFRunLoopRef)CFRetain(CFRunLoopGetCurrent())) {
  FML_DCHECK(loop_ != nullptr);

  // Setup the delayed wake source.
  CFRunLoopTimerContext timer_context = {
      .info = this,
  };
  delayed_wake_timer_.Reset(
      CFRunLoopTimerCreate(kCFAllocatorDefault, kDistantFuture /* fire date */,
                           HUGE_VAL /* interval */, 0 /* flags */, 0 /* order */,
                           reinterpret_cast<CFRunLoopTimerCallBack>(&MessageLoopDarwin::OnTimerFire)
                           /* callout */,
                           &timer_context /* context */));
  FML_DCHECK(delayed_wake_timer_ != nullptr);
  CFRunLoopAddTimer(loop_, delayed_wake_timer_, kCFRunLoopCommonModes);
  // This mode will be used by FlutterKeyboardManager.
  CFRunLoopAddTimer(loop_, delayed_wake_timer_, kMessageLoopCFRunLoopMode);
}

MessageLoopDarwin::~MessageLoopDarwin() {
  CFRunLoopTimerInvalidate(delayed_wake_timer_);
  CFRunLoopRemoveTimer(loop_, delayed_wake_timer_, kCFRunLoopCommonModes);
  CFRunLoopRemoveTimer(loop_, delayed_wake_timer_, kMessageLoopCFRunLoopMode);
}

void MessageLoopDarwin::Run() {
  FML_DCHECK(loop_ == CFRunLoopGetCurrent());

  running_ = true;

  while (running_) {
    @autoreleasepool {
      int result = CFRunLoopRunInMode(kCFRunLoopDefaultMode, kDistantFuture, YES);
      if (result == kCFRunLoopRunStopped || result == kCFRunLoopRunFinished) {
        // This handles the case where the loop is terminated using
        // CoreFoundation APIs.
        @autoreleasepool {
          RunExpiredTasksNow();
        }
        running_ = false;
      }
    }
  }
}

void MessageLoopDarwin::Terminate() {
  running_ = false;

  // Ensure that the CFRunLoop remains alive through the end of this function
  // even if the loop's thread exits and drops its reference to the loop.
  CFRef<CFRunLoopRef> local_loop(loop_);

  CFRunLoopStop(local_loop);
}

void MessageLoopDarwin::WakeUp(fml::TimePoint time_point) {
  // Rearm the timer. The time bases used by CoreFoundation and FXL are
  // different and must be accounted for.
  CFRunLoopTimerSetNextFireDate(
      delayed_wake_timer_,
      CFAbsoluteTimeGetCurrent() + (time_point - fml::TimePoint::Now()).ToSecondsF());
}

void MessageLoopDarwin::OnTimerFire(CFRunLoopTimerRef timer, MessageLoopDarwin* loop) {
  @autoreleasepool {
    // RunExpiredTasksNow rearms the timer as appropriate via a call to WakeUp.
    loop->RunExpiredTasksNow();
  }
}

}  // namespace fml
