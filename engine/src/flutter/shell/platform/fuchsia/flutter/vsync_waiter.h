// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_VSYNC_WAITER_H_
#define FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_VSYNC_WAITER_H_

#include <lib/async/cpp/wait.h>

#include "flutter/fml/macros.h"
#include "flutter/fml/memory/weak_ptr.h"
#include "flutter/fml/time/time_delta.h"
#include "flutter/fml/time/time_point.h"
#include "flutter/shell/common/vsync_waiter.h"
#include "flutter_runner_product_configuration.h"

namespace flutter_runner {

using FireCallbackCallback =
    std::function<void(fml::TimePoint, fml::TimePoint)>;

using AwaitVsyncCallback = std::function<void(FireCallbackCallback)>;

using AwaitVsyncForSecondaryCallbackCallback =
    std::function<void(FireCallbackCallback)>;

class VsyncWaiter final : public flutter::VsyncWaiter {
 public:
  VsyncWaiter(AwaitVsyncCallback await_vsync_callback,
              AwaitVsyncForSecondaryCallbackCallback
                  await_vsync_for_secondary_callback_callback,
              flutter::TaskRunners task_runners);

  ~VsyncWaiter() override;

 private:
  // |flutter::VsyncWaiter|
  void AwaitVSync() override;

  // |flutter::VsyncWaiter|
  void AwaitVSyncForSecondaryCallback() override;

  FireCallbackCallback fire_callback_callback_;

  AwaitVsyncCallback await_vsync_callback_;
  AwaitVsyncForSecondaryCallbackCallback
      await_vsync_for_secondary_callback_callback_;

  fml::WeakPtr<VsyncWaiter> weak_ui_;
  std::unique_ptr<fml::WeakPtrFactory<VsyncWaiter>> weak_factory_ui_;
  fml::WeakPtrFactory<VsyncWaiter> weak_factory_;

  FML_DISALLOW_COPY_AND_ASSIGN(VsyncWaiter);
};

}  // namespace flutter_runner

#endif  // FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_VSYNC_WAITER_H_
