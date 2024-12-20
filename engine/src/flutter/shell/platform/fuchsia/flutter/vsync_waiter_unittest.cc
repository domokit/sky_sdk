// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <gtest/gtest.h>

#include <string>

#include "flutter/fml/task_runner.h"
#include "flutter/shell/common/thread_host.h"
#include "fml/make_copyable.h"
#include "fml/message_loop.h"
#include "fml/synchronization/waitable_event.h"
#include "fml/time/time_delta.h"
#include "fml/time/time_point.h"
#include "vsync_waiter.h"

namespace flutter_runner {

TEST(VSyncWaiterFuchsia, FrameScheduledForStartTime) {
  using flutter::ThreadHost;
  std::string prefix = "vsync_waiter_test";

  fml::MessageLoop::EnsureInitializedForCurrentThread();
  auto platform_task_runner = fml::MessageLoop::GetCurrent().GetTaskRunner();

  ThreadHost thread_host =
      ThreadHost(prefix, flutter::ThreadHost::Type::kRaster |
                             flutter::ThreadHost::Type::kUi |
                             flutter::ThreadHost::Type::kIo);
  const flutter::TaskRunners task_runners(
      prefix,                                      // Dart thread labels
      platform_task_runner,                        // platform
      thread_host.raster_thread->GetTaskRunner(),  // raster
      thread_host.ui_thread->GetTaskRunner(),      // ui
      thread_host.io_thread->GetTaskRunner()       // io
  );

  // await vsync will invoke the callback right away, but vsync waiter will post
  // the task for frame_start time.
  VsyncWaiter vsync_waiter(
      [](FireCallbackCallback callback) {
        const auto now = fml::TimePoint::Now();
        const auto frame_start = now + fml::TimeDelta::FromMilliseconds(20);
        const auto frame_end = now + fml::TimeDelta::FromMilliseconds(36);
        callback(frame_start, frame_end);
      },
      /*secondary callback*/ nullptr, task_runners);

  fml::AutoResetWaitableEvent latch;
  task_runners.GetUITaskRunner()->PostTask([&]() {
    vsync_waiter.AsyncWaitForVsync(
        [&](std::unique_ptr<flutter::FrameTimingsRecorder> recorder) {
          const auto now = fml::TimePoint::Now();
          EXPECT_GT(now, recorder->GetVsyncStartTime());
          latch.Signal();
        });
  });

  latch.Wait();
}

}  // namespace flutter_runner
