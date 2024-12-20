// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_TESTING_MOCK_SETTINGS_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_TESTING_MOCK_SETTINGS_H_

#include "flutter/shell/platform/linux/fl_settings.h"

#include "gmock/gmock.h"

namespace flutter {
namespace testing {

// Mock for FlSettings.
class MockSettings {
 public:
  MockSettings();
  ~MockSettings();

  // This was an existing use of operator overloading. It's against our style
  // guide but enabling clang tidy on header files is a higher priority than
  // fixing this.
  // NOLINTNEXTLINE(google-explicit-constructor)
  operator FlSettings*();

  MOCK_METHOD(FlClockFormat,
              fl_settings_get_clock_format,
              (FlSettings * settings));

  MOCK_METHOD(FlColorScheme,
              fl_settings_get_color_scheme,
              (FlSettings * settings));

  MOCK_METHOD(bool, fl_settings_get_enable_animations, (FlSettings * settings));

  MOCK_METHOD(bool, fl_settings_get_high_contrast, (FlSettings * settings));

  MOCK_METHOD(gdouble,
              fl_settings_get_text_scaling_factor,
              (FlSettings * settings));

 private:
  FlSettings* instance_ = nullptr;
};

}  // namespace testing
}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_TESTING_MOCK_SETTINGS_H_
