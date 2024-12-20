// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <memory>
#include <string>

#include "flutter/shell/platform/windows/client_wrapper/include/flutter/flutter_view.h"
#include "flutter/shell/platform/windows/client_wrapper/testing/stub_flutter_windows_api.h"
#include "gtest/gtest.h"

namespace flutter {

namespace {

// Stub implementation to validate calls to the API.
class TestWindowsApi : public testing::StubFlutterWindowsApi {
  HWND ViewGetHWND() override { return reinterpret_cast<HWND>(7); }

  IDXGIAdapter* ViewGetGraphicsAdapter() override {
    return reinterpret_cast<IDXGIAdapter*>(8);
  }
};

}  // namespace

TEST(FlutterViewTest, HwndAccessPassesThrough) {
  testing::ScopedStubFlutterWindowsApi scoped_api_stub(
      std::make_unique<TestWindowsApi>());
  auto test_api = static_cast<TestWindowsApi*>(scoped_api_stub.stub());
  FlutterView view(reinterpret_cast<FlutterDesktopViewRef>(2));
  EXPECT_EQ(view.GetNativeWindow(), reinterpret_cast<HWND>(7));
}

TEST(FlutterViewTest, GraphicsAdapterAccessPassesThrough) {
  testing::ScopedStubFlutterWindowsApi scoped_api_stub(
      std::make_unique<TestWindowsApi>());
  auto test_api = static_cast<TestWindowsApi*>(scoped_api_stub.stub());
  FlutterView view(reinterpret_cast<FlutterDesktopViewRef>(2));

  IDXGIAdapter* adapter = view.GetGraphicsAdapter();
  EXPECT_EQ(adapter, reinterpret_cast<IDXGIAdapter*>(8));
}

}  // namespace flutter
