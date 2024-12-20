// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_VSYNC_WAITER_IOS_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_VSYNC_WAITER_IOS_H_

#include <QuartzCore/CADisplayLink.h>

#include "flutter/fml/macros.h"
#include "flutter/shell/common/variable_refresh_rate_reporter.h"
#include "flutter/shell/common/vsync_waiter.h"

//------------------------------------------------------------------------------
/// @brief      Info.plist key enabling the full range of ProMotion refresh rates for CADisplayLink
///             callbacks and CAAnimation animations in the app.
///
/// @see
/// https://developer.apple.com/documentation/quartzcore/optimizing_promotion_refresh_rates_for_iphone_13_pro_and_ipad_pro#3885321
///
extern NSString* const kCADisableMinimumFrameDurationOnPhoneKey;

@interface DisplayLinkManager : NSObject

//------------------------------------------------------------------------------
/// @brief      Whether the max refresh rate on iPhone ProMotion devices are enabled. This reflects
///             the value of `CADisableMinimumFrameDurationOnPhone` in the info.plist file. On iPads
///             that support ProMotion, the max refresh rate is always enabled.
///
/// @return     YES if the max refresh rate on ProMotion devices is enabled.
///
@property(class, nonatomic, readonly) BOOL maxRefreshRateEnabledOnIPhone;

//------------------------------------------------------------------------------
/// @brief      The display refresh rate used for reporting purposes. The engine does not care
///             about this for frame scheduling. It is only used by tools for instrumentation. The
///             engine uses the duration field of the link per frame for frame scheduling.
///
/// @attention  Do not use the this call in frame scheduling. It is only meant for reporting.
///
/// @return     The refresh rate in frames per second.
///
@property(class, nonatomic, readonly) double displayRefreshRate;

@end

@interface VSyncClient : NSObject

//------------------------------------------------------------------------------
/// @brief      Default value is YES. Vsync client will pause vsync callback after receiving
///             a vsync signal. Setting this property to NO can avoid this and vsync client
///             will trigger vsync callback continuously.
///
///
/// @param allowPauseAfterVsync Allow vsync client to pause after receiving a vsync signal.
///
@property(nonatomic, assign) BOOL allowPauseAfterVsync;

- (instancetype)initWithTaskRunner:(fml::RefPtr<fml::TaskRunner>)task_runner
                          callback:(flutter::VsyncWaiter::Callback)callback;

- (void)await;

- (void)pause;

//------------------------------------------------------------------------------
/// @brief      Call invalidate before releasing this object to remove from runloops.
///
- (void)invalidate;

- (void)setMaxRefreshRate:(double)refreshRate;

@end

namespace flutter {

class VsyncWaiterIOS final : public VsyncWaiter, public VariableRefreshRateReporter {
 public:
  explicit VsyncWaiterIOS(const flutter::TaskRunners& task_runners);

  ~VsyncWaiterIOS() override;

  // |VariableRefreshRateReporter|
  double GetRefreshRate() const override;

  // |VsyncWaiter|
  // Made public for testing.
  void AwaitVSync() override;

 private:
  VSyncClient* client_;
  double max_refresh_rate_;

  FML_DISALLOW_COPY_AND_ASSIGN(VsyncWaiterIOS);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_VSYNC_WAITER_IOS_H_
