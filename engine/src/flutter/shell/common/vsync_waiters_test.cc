// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#define FML_USED_ON_EMBEDDER

#include "flutter/shell/common/shell_test.h"

#include "flutter/flow/layers/layer_tree.h"
#include "flutter/flow/layers/transform_layer.h"
#include "flutter/fml/make_copyable.h"
#include "flutter/fml/mapping.h"
#include "flutter/runtime/dart_vm.h"
#include "flutter/testing/testing.h"

namespace flutter {
namespace testing {

void ShellTestVsyncClock::SimulateVSync() {
  std::scoped_lock lock(mutex_);
  if (vsync_issued_ >= vsync_promised_.size()) {
    vsync_promised_.emplace_back();
  }
  FML_CHECK(vsync_issued_ < vsync_promised_.size());
  vsync_promised_[vsync_issued_].set_value(vsync_issued_);
  vsync_issued_ += 1;
}

std::future<int> ShellTestVsyncClock::NextVSync() {
  std::scoped_lock lock(mutex_);
  vsync_promised_.emplace_back();
  return vsync_promised_.back().get_future();
}

void ShellTestVsyncWaiter::AwaitVSync() {
  FML_DCHECK(task_runners_.GetUITaskRunner()->RunsTasksOnCurrentThread());
  auto vsync_future = clock_->NextVSync();

  auto async_wait = std::async([&vsync_future, this]() {
    vsync_future.wait();

    // Post the `FireCallback` to the Platform thread so earlier Platform tasks
    // (specifically, the `VSyncFlush` call) will be finished before
    // `FireCallback` is executed. This is only needed for our unit tests.
    //
    // Without this, the repeated VSYNC signals in `VSyncFlush` may start both
    // the current frame in the UI thread and the next frame in the secondary
    // callback (both of them are waiting for VSYNCs). That breaks the unit
    // test's assumption that each frame's VSYNC must be issued by different
    // `VSyncFlush` call (which resets the `will_draw_new_frame` bit).
    //
    // For example, HandlesActualIphoneXsInputEvents will fail without this.
    task_runners_.GetPlatformTaskRunner()->PostTask([this]() {
      FireCallback(fml::TimePoint::Now(), fml::TimePoint::Now());
    });
  });
}

void ConstantFiringVsyncWaiter::AwaitVSync() {
  FML_DCHECK(task_runners_.GetUITaskRunner()->RunsTasksOnCurrentThread());
  auto async_wait = std::async([this]() {
    task_runners_.GetPlatformTaskRunner()->PostTask(
        [this]() { FireCallback(kFrameBeginTime, kFrameTargetTime); });
  });
}

TestRefreshRateReporter::TestRefreshRateReporter(double refresh_rate)
    : refresh_rate_(refresh_rate) {}

void TestRefreshRateReporter::UpdateRefreshRate(double refresh_rate) {
  refresh_rate_ = refresh_rate;
}

double TestRefreshRateReporter::GetRefreshRate() const {
  return refresh_rate_;
}

}  // namespace testing
}  // namespace flutter
