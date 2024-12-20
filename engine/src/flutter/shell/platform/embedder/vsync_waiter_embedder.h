// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_EMBEDDER_VSYNC_WAITER_EMBEDDER_H_
#define FLUTTER_SHELL_PLATFORM_EMBEDDER_VSYNC_WAITER_EMBEDDER_H_

#include "flutter/fml/macros.h"
#include "flutter/shell/common/vsync_waiter.h"

namespace flutter {

class VsyncWaiterEmbedder final : public VsyncWaiter {
 public:
  using VsyncCallback = std::function<void(intptr_t)>;

  VsyncWaiterEmbedder(const VsyncCallback& callback,
                      const flutter::TaskRunners& task_runners);

  ~VsyncWaiterEmbedder() override;

  static bool OnEmbedderVsync(const flutter::TaskRunners& task_runners,
                              intptr_t baton,
                              fml::TimePoint frame_start_time,
                              fml::TimePoint frame_target_time);

 private:
  const VsyncCallback vsync_callback_;

  // |VsyncWaiter|
  void AwaitVSync() override;

  FML_DISALLOW_COPY_AND_ASSIGN(VsyncWaiterEmbedder);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_EMBEDDER_VSYNC_WAITER_EMBEDDER_H_
