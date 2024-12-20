// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_TESTING_MOCK_WINDOW_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_TESTING_MOCK_WINDOW_H_

#include "gmock/gmock.h"

#include <gtk/gtk.h>

namespace flutter {
namespace testing {

class MockWindow {
 public:
  MockWindow();

  MOCK_METHOD(GdkWindowState, gdk_window_get_state, (GdkWindow * window));
};

}  // namespace testing
}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_TESTING_MOCK_WINDOW_H_
