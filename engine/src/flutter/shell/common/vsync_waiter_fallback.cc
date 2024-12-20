// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/common/vsync_waiter_fallback.h"

#include <memory>

#include "flutter/fml/logging.h"
#include "flutter/fml/message_loop.h"
#include "flutter/fml/trace_event.h"

namespace flutter {
namespace {

static fml::TimePoint SnapToNextTick(fml::TimePoint value,
                                     fml::TimePoint tick_phase,
                                     fml::TimeDelta tick_interval) {
  fml::TimeDelta offset = (tick_phase - value) % tick_interval;
  if (offset != fml::TimeDelta::Zero()) {
    offset = offset + tick_interval;
  }
  return value + offset;
}

}  // namespace

VsyncWaiterFallback::VsyncWaiterFallback(const TaskRunners& task_runners,
                                         bool for_testing)
    : VsyncWaiter(task_runners),
      phase_(fml::TimePoint::Now()),
      for_testing_(for_testing) {}

VsyncWaiterFallback::~VsyncWaiterFallback() = default;

// |VsyncWaiter|
void VsyncWaiterFallback::AwaitVSync() {
  constexpr fml::TimeDelta kSingleFrameInterval =
      fml::TimeDelta::FromSecondsF(1.0 / 60.0);
  auto frame_start_time =
      SnapToNextTick(fml::TimePoint::Now(), phase_, kSingleFrameInterval);
  auto frame_target_time = frame_start_time + kSingleFrameInterval;

  TRACE_EVENT2_INT("flutter", "PlatformVsync", "frame_start_time",
                   frame_start_time.ToEpochDelta().ToMicroseconds(),
                   "frame_target_time",
                   frame_target_time.ToEpochDelta().ToMicroseconds());

  std::weak_ptr<VsyncWaiterFallback> weak_this =
      std::static_pointer_cast<VsyncWaiterFallback>(shared_from_this());

  task_runners_.GetUITaskRunner()->PostTaskForTime(
      [frame_start_time, frame_target_time, weak_this]() {
        if (auto vsync_waiter = weak_this.lock()) {
          vsync_waiter->FireCallback(frame_start_time, frame_target_time,
                                     !vsync_waiter->for_testing_);
        }
      },
      frame_start_time);
}

}  // namespace flutter
