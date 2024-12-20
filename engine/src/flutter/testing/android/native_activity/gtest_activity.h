// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_TESTING_ANDROID_NATIVE_ACTIVITY_GTEST_ACTIVITY_H_
#define FLUTTER_TESTING_ANDROID_NATIVE_ACTIVITY_GTEST_ACTIVITY_H_

#include "flutter/fml/macros.h"
#include "flutter/fml/thread.h"
#include "flutter/testing/android/native_activity/native_activity.h"

namespace flutter {

//------------------------------------------------------------------------------
/// @brief      A native activity subclass an in implementation of
///             `flutter::NativeActivityMain` that return it.
///
///             This class runs a Google Test harness on a background thread and
///             redirects progress updates to `logcat` instead of STDOUT.
///
class GTestActivity final : public NativeActivity {
 public:
  explicit GTestActivity(ANativeActivity* activity);

  ~GTestActivity() override;

  GTestActivity(const GTestActivity&) = delete;

  GTestActivity& operator=(const GTestActivity&) = delete;

  // |NativeActivity|
  void OnNativeWindowCreated(ANativeWindow* window) override;

 private:
  fml::Thread background_thread_;
};

}  // namespace flutter

#endif  // FLUTTER_TESTING_ANDROID_NATIVE_ACTIVITY_GTEST_ACTIVITY_H_
