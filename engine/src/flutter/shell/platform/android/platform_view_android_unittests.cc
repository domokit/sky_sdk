// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "shell/platform/android/flutter_main.h"
#include "third_party/googletest/googlemock/include/gmock/gmock-nice-strict.h"

namespace flutter {
namespace testing {

// TODO(matanlurey): Re-enable.
//
// This test (and the entire suite) was skipped on CI (see
// https://github.com/flutter/flutter/issues/163742) and has since bit rotted
// (we fallback to OpenGLES on emulators for performance reasons); either fix
// the test, or remove it.
TEST(AndroidPlatformView, DISABLED_SelectsVulkanBasedOnApiLevel) {
  Settings settings;
  settings.enable_software_rendering = false;
  settings.enable_impeller = true;

  EXPECT_EQ(FlutterMain::SelectedRenderingAPI(settings, 29),
            AndroidRenderingAPI::kImpellerVulkan);
  EXPECT_EQ(FlutterMain::SelectedRenderingAPI(settings, 24),
            AndroidRenderingAPI::kImpellerOpenGLES);
}

TEST(AndroidPlatformView, SoftwareRenderingNotSupportedWithImpeller) {
  Settings settings;
  settings.enable_software_rendering = true;
  settings.enable_impeller = true;

  ASSERT_DEATH(FlutterMain::SelectedRenderingAPI(settings, 29), "");
}

TEST(AndroidPlatformView, FallsBackToGLESonEmulator) {
  std::string emulator_product = "gphone_x64";
  std::string device_product = "smg1234";

  EXPECT_TRUE(FlutterMain::IsDeviceEmulator(emulator_product));
  EXPECT_FALSE(FlutterMain::IsDeviceEmulator(device_product));
}

TEST(AndroidPlatformView, FallsBackToGLESonMostExynos) {
  std::string exynos_board = "exynos7870";
  std::string snap_board = "smg1234";

  EXPECT_TRUE(FlutterMain::IsKnownBadSOC(exynos_board));
  EXPECT_FALSE(FlutterMain::IsKnownBadSOC(snap_board));
}

}  // namespace testing
}  // namespace flutter
