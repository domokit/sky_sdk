// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "vsync_waiter.h"

#include <cstdint>

#include <lib/async/default.h>

#include "flutter/fml/logging.h"
#include "flutter/fml/make_copyable.h"
#include "flutter/fml/synchronization/waitable_event.h"
#include "flutter/fml/time/time_delta.h"
#include "flutter/fml/trace_event.h"

#include "flutter_runner_product_configuration.h"

namespace flutter_runner {

VsyncWaiter::VsyncWaiter(AwaitVsyncCallback await_vsync_callback,
                         AwaitVsyncForSecondaryCallbackCallback
                             await_vsync_for_secondary_callback_callback,
                         flutter::TaskRunners task_runners)
    : flutter::VsyncWaiter(task_runners),
      await_vsync_callback_(await_vsync_callback),
      await_vsync_for_secondary_callback_callback_(
          await_vsync_for_secondary_callback_callback),
      weak_factory_ui_(nullptr),
      weak_factory_(this) {
  fire_callback_callback_ = [this](fml::TimePoint frame_start,
                                   fml::TimePoint frame_end) {
    task_runners_.GetUITaskRunner()->PostTaskForTime(
        [frame_start, frame_end, weak_this = weak_ui_]() {
          if (weak_this) {
            // Note: It is VERY important to set |pause_secondary_tasks| to
            // false, else Animator will almost immediately crash on Fuchsia.
            // FML_LOG(INFO) << "CRASH:: VsyncWaiter about to FireCallback";
            weak_this->FireCallback(frame_start, frame_end,
                                    /*pause_secondary_tasks*/ false);
          }
        },
        frame_start);
  };

  // Generate a WeakPtrFactory for use with the UI thread. This does not need
  // to wait on a latch because we only ever use the WeakPtrFactory on the UI
  // thread so we have ordering guarantees (see ::AwaitVSync())
  fml::TaskRunner::RunNowOrPostTask(
      task_runners_.GetUITaskRunner(), fml::MakeCopyable([this]() mutable {
        weak_factory_ui_ =
            std::make_unique<fml::WeakPtrFactory<VsyncWaiter>>(this);
        weak_ui_ = weak_factory_ui_->GetWeakPtr();
      }));
}

VsyncWaiter::~VsyncWaiter() {
  fml::AutoResetWaitableEvent ui_latch;
  fml::TaskRunner::RunNowOrPostTask(
      task_runners_.GetUITaskRunner(),
      fml::MakeCopyable(
          [weak_factory_ui = std::move(weak_factory_ui_), &ui_latch]() mutable {
            weak_factory_ui.reset();
            ui_latch.Signal();
          }));
  ui_latch.Wait();
}

void VsyncWaiter::AwaitVSync() {
  await_vsync_callback_(fire_callback_callback_);
}

void VsyncWaiter::AwaitVSyncForSecondaryCallback() {
  await_vsync_for_secondary_callback_callback_(fire_callback_callback_);
}

}  // namespace flutter_runner
