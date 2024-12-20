// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_ANDROID_VSYNC_WAITER_ANDROID_H_
#define FLUTTER_SHELL_PLATFORM_ANDROID_VSYNC_WAITER_ANDROID_H_

#include <jni.h>

#include <memory>

#include "flutter/fml/macros.h"
#include "flutter/shell/common/vsync_waiter.h"

namespace flutter {

class AndroidChoreographer;

class VsyncWaiterAndroid final : public VsyncWaiter {
 public:
  static bool Register(JNIEnv* env);

  explicit VsyncWaiterAndroid(const flutter::TaskRunners& task_runners);

  ~VsyncWaiterAndroid() override;

 private:
  // |VsyncWaiter|
  void AwaitVSync() override;

  static void OnVsyncFromNDK(int64_t frame_nanos, void* data);

  static void OnVsyncFromJava(JNIEnv* env,
                              jclass jcaller,
                              jlong frameDelayNanos,
                              jlong refreshPeriodNanos,
                              jlong java_baton);

  static void ConsumePendingCallback(std::weak_ptr<VsyncWaiter>* weak_this,
                                     fml::TimePoint frame_start_time,
                                     fml::TimePoint frame_target_time);

  static void OnUpdateRefreshRate(JNIEnv* env,
                                  jclass jcaller,
                                  jfloat refresh_rate);

  FML_DISALLOW_COPY_AND_ASSIGN(VsyncWaiterAndroid);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_ANDROID_VSYNC_WAITER_ANDROID_H_
