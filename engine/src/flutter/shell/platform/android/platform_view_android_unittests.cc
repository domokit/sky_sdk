// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "shell/platform/android/flutter_main.h"
#include "third_party/googletest/googlemock/include/gmock/gmock-nice-strict.h"

namespace flutter {
namespace testing {

TEST(AndroidPlatformView, SelectsVulkanBasedOnApiLevel) {
  Settings settings;
  settings.enable_software_rendering = false;
  settings.enable_impeller = true;

  int api_level = android_get_device_api_level();
  EXPECT_GT(api_level, 0);
  if (api_level >= 29) {
    EXPECT_EQ(FlutterMain::SelectedRenderingAPI(settings),
              AndroidRenderingAPI::kImpellerVulkan);
  } else {
    EXPECT_EQ(FlutterMain::SelectedRenderingAPI(settings),
              AndroidRenderingAPI::kImpellerOpenGLES);
  }
}

TEST(AndroidPlatformView, SoftwareRenderingNotSupportedWithImpeller) {
  Settings settings;
  settings.enable_software_rendering = true;
  settings.enable_impeller = true;

  ASSERT_DEATH(FlutterMain::SelectedRenderingAPI(settings), "");
}

}  // namespace testing
}  // namespace flutter
