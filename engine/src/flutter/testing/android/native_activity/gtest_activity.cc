// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/testing/android/native_activity/gtest_activity.h"

#include "flutter/impeller/toolkit/android/native_window.h"
#include "flutter/testing/logger_listener.h"
#include "flutter/testing/test_timeout_listener.h"

namespace flutter {

GTestActivity::GTestActivity(ANativeActivity* activity)
    : NativeActivity(activity) {}

GTestActivity::~GTestActivity() = default;

static void StartTestSuite(const impeller::android::NativeWindow& window) {
  auto timeout_listener = new flutter::testing::TestTimeoutListener(
      fml::TimeDelta::FromSeconds(120u));
  auto logger_listener = new flutter::testing::LoggerListener();

  auto& listeners = ::testing::UnitTest::GetInstance()->listeners();

  listeners.Append(timeout_listener);
  listeners.Append(logger_listener);

  int result = RUN_ALL_TESTS();

  delete listeners.Release(timeout_listener);
  delete listeners.Release(logger_listener);

  FML_CHECK(result == 0);
}

// |NativeActivity|
void GTestActivity::OnNativeWindowCreated(ANativeWindow* window) {
  auto handle = std::make_shared<impeller::android::NativeWindow>(window);
  background_thread_.GetTaskRunner()->PostTask(
      [handle]() { StartTestSuite(*handle); });
}

std::unique_ptr<NativeActivity> NativeActivityMain(
    ANativeActivity* activity,
    std::unique_ptr<fml::Mapping> saved_state) {
  return std::make_unique<GTestActivity>(activity);
}

}  // namespace flutter
