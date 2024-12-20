// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_WINDOWS_TESTING_MOCK_WINDOW_BINDING_HANDLER_H_
#define FLUTTER_SHELL_PLATFORM_WINDOWS_TESTING_MOCK_WINDOW_BINDING_HANDLER_H_

#include "flutter/fml/macros.h"
#include "flutter/shell/platform/windows/window_binding_handler.h"
#include "flutter/third_party/accessibility/ax/platform/ax_platform_node_win.h"
#include "gmock/gmock.h"

namespace flutter {
namespace testing {

/// Mock for the |Window| base class.
class MockWindowBindingHandler : public WindowBindingHandler {
 public:
  MockWindowBindingHandler();
  virtual ~MockWindowBindingHandler();

  MOCK_METHOD(void, SetView, (WindowBindingHandlerDelegate * view), (override));
  MOCK_METHOD(HWND, GetWindowHandle, (), (override));
  MOCK_METHOD(float, GetDpiScale, (), (override));
  MOCK_METHOD(PhysicalWindowBounds, GetPhysicalWindowBounds, (), (override));
  MOCK_METHOD(void,
              UpdateFlutterCursor,
              (const std::string& cursor_name),
              (override));
  MOCK_METHOD(void, SetFlutterCursor, (HCURSOR cursor_name), (override));
  MOCK_METHOD(void, OnCursorRectUpdated, (const Rect& rect), (override));
  MOCK_METHOD(void, OnResetImeComposing, (), (override));
  MOCK_METHOD(bool, OnBitmapSurfaceCleared, (), (override));
  MOCK_METHOD(bool,
              OnBitmapSurfaceUpdated,
              (const void* allocation, size_t row_bytes, size_t height),
              (override));
  MOCK_METHOD(PointerLocation, GetPrimaryPointerLocation, (), (override));
  MOCK_METHOD(AlertPlatformNodeDelegate*, GetAlertDelegate, (), (override));
  MOCK_METHOD(ui::AXPlatformNodeWin*, GetAlert, (), (override));

 private:
  FML_DISALLOW_COPY_AND_ASSIGN(MockWindowBindingHandler);
};

}  // namespace testing
}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_WINDOWS_TESTING_MOCK_WINDOW_BINDING_HANDLER_H_
