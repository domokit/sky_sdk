// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/ios/framework/Source/vsync_waiter_ios.h"

#include <utility>

#include <Foundation/Foundation.h>
#include <UIKit/UIKit.h>
#include <mach/mach_time.h>

#include "flutter/common/task_runners.h"
#include "flutter/fml/logging.h"
#include "flutter/fml/memory/task_runner_checker.h"
#include "flutter/fml/trace_event.h"
#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterMacros.h"

FLUTTER_ASSERT_ARC

NSString* const kCADisableMinimumFrameDurationOnPhoneKey = @"CADisableMinimumFrameDurationOnPhone";

@interface VSyncClient ()
@property(nonatomic, assign, readonly) double refreshRate;
@end

// When calculating refresh rate diffrence, anything within 0.1 fps is ignored.
const static double kRefreshRateDiffToIgnore = 0.1;

namespace flutter {

VsyncWaiterIOS::VsyncWaiterIOS(const flutter::TaskRunners& task_runners)
    : VsyncWaiter(task_runners) {
  auto callback = [this](std::unique_ptr<flutter::FrameTimingsRecorder> recorder) {
    const fml::TimePoint start_time = recorder->GetVsyncStartTime();
    const fml::TimePoint target_time = recorder->GetVsyncTargetTime();
    FireCallback(start_time, target_time, true);
  };
  client_ = [[VSyncClient alloc] initWithTaskRunner:task_runners_.GetUITaskRunner()
                                           callback:callback];
  max_refresh_rate_ = DisplayLinkManager.displayRefreshRate;
}

VsyncWaiterIOS::~VsyncWaiterIOS() {
  // This way, we will get no more callbacks from the display link that holds a weak (non-nilling)
  // reference to this C++ object.
  [client_ invalidate];
}

void VsyncWaiterIOS::AwaitVSync() {
  double new_max_refresh_rate = DisplayLinkManager.displayRefreshRate;
  if (fabs(new_max_refresh_rate - max_refresh_rate_) > kRefreshRateDiffToIgnore) {
    max_refresh_rate_ = new_max_refresh_rate;
    [client_ setMaxRefreshRate:max_refresh_rate_];
  }
  [client_ await];
}

// |VariableRefreshRateReporter|
double VsyncWaiterIOS::GetRefreshRate() const {
  return client_.refreshRate;
}

}  // namespace flutter

@implementation VSyncClient {
  flutter::VsyncWaiter::Callback _callback;
  CADisplayLink* _displayLink;
}

- (instancetype)initWithTaskRunner:(fml::RefPtr<fml::TaskRunner>)task_runner
                          callback:(flutter::VsyncWaiter::Callback)callback {
  FML_DCHECK(task_runner);

  if (self = [super init]) {
    _refreshRate = DisplayLinkManager.displayRefreshRate;
    _allowPauseAfterVsync = YES;
    _callback = std::move(callback);
    _displayLink = [CADisplayLink displayLinkWithTarget:self selector:@selector(onDisplayLink:)];
    _displayLink.paused = YES;

    [self setMaxRefreshRate:DisplayLinkManager.displayRefreshRate];

    // Strongly retain the the captured link until it is added to the runloop.
    CADisplayLink* localDisplayLink = _displayLink;
    task_runner->PostTask([localDisplayLink]() {
      [localDisplayLink addToRunLoop:NSRunLoop.currentRunLoop forMode:NSRunLoopCommonModes];
    });
  }

  return self;
}

- (void)setMaxRefreshRate:(double)refreshRate {
  if (!DisplayLinkManager.maxRefreshRateEnabledOnIPhone) {
    return;
  }
  double maxFrameRate = fmax(refreshRate, 60);
  double minFrameRate = fmax(maxFrameRate / 2, 60);
  if (@available(iOS 15.0, *)) {
    _displayLink.preferredFrameRateRange =
        CAFrameRateRangeMake(minFrameRate, maxFrameRate, maxFrameRate);
  } else {
    _displayLink.preferredFramesPerSecond = maxFrameRate;
  }
}

- (void)await {
  _displayLink.paused = NO;
}

- (void)pause {
  _displayLink.paused = YES;
}

- (void)onDisplayLink:(CADisplayLink*)link {
  CFTimeInterval delay = CACurrentMediaTime() - link.timestamp;
  fml::TimePoint frame_start_time = fml::TimePoint::Now() - fml::TimeDelta::FromSecondsF(delay);

  CFTimeInterval duration = link.targetTimestamp - link.timestamp;
  fml::TimePoint frame_target_time = frame_start_time + fml::TimeDelta::FromSecondsF(duration);

  TRACE_EVENT2_INT("flutter", "PlatformVsync", "frame_start_time",
                   frame_start_time.ToEpochDelta().ToMicroseconds(), "frame_target_time",
                   frame_target_time.ToEpochDelta().ToMicroseconds());

  std::unique_ptr<flutter::FrameTimingsRecorder> recorder =
      std::make_unique<flutter::FrameTimingsRecorder>();

  _refreshRate = round(1 / (frame_target_time - frame_start_time).ToSecondsF());

  recorder->RecordVsync(frame_start_time, frame_target_time);
  if (_allowPauseAfterVsync) {
    link.paused = YES;
  }
  _callback(std::move(recorder));
}

- (void)invalidate {
  [_displayLink invalidate];
  _displayLink = nil;  // Break retain cycle.
}

- (CADisplayLink*)getDisplayLink {
  return _displayLink;
}

@end

@implementation DisplayLinkManager

+ (double)displayRefreshRate {
  CADisplayLink* displayLink = [CADisplayLink displayLinkWithTarget:[[[self class] alloc] init]
                                                           selector:@selector(onDisplayLink:)];
  displayLink.paused = YES;
  auto preferredFPS = displayLink.preferredFramesPerSecond;

  // From Docs:
  // The default value for preferredFramesPerSecond is 0. When this value is 0, the preferred
  // frame rate is equal to the maximum refresh rate of the display, as indicated by the
  // maximumFramesPerSecond property.

  if (preferredFPS != 0) {
    return preferredFPS;
  }

  return UIScreen.mainScreen.maximumFramesPerSecond;
}

- (void)onDisplayLink:(CADisplayLink*)link {
  // no-op.
}

+ (BOOL)maxRefreshRateEnabledOnIPhone {
  return [[NSBundle.mainBundle objectForInfoDictionaryKey:kCADisableMinimumFrameDurationOnPhoneKey]
      boolValue];
}

@end
