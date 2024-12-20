// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/glfw/client_wrapper/include/flutter/flutter_window.h"

#include <memory>
#include <string>

#include "flutter/shell/platform/glfw/client_wrapper/testing/stub_flutter_glfw_api.h"
#include "gtest/gtest.h"

namespace flutter {

namespace {

// Stub implementation to validate calls to the API.
class TestGlfwApi : public testing::StubFlutterGlfwApi {
 public:
  // |flutter::testing::StubFlutterGlfwApi|
  void SetSizeLimits(FlutterDesktopSize minimum_size,
                     FlutterDesktopSize maximum_size) override {
    set_size_limits_called_ = true;
  }

  bool set_size_limits_called() { return set_size_limits_called_; }

 private:
  bool set_size_limits_called_ = false;
};

}  // namespace

TEST(FlutterWindowTest, SetSizeLimits) {
  const std::string icu_data_path = "fake/path/to/icu";
  testing::ScopedStubFlutterGlfwApi scoped_api_stub(
      std::make_unique<TestGlfwApi>());
  auto test_api = static_cast<TestGlfwApi*>(scoped_api_stub.stub());
  // This is not actually used so any non-zero value works.
  auto raw_window = reinterpret_cast<FlutterDesktopWindowRef>(1);

  auto window = std::make_unique<FlutterWindow>(raw_window);

  FlutterDesktopSize minimum_size = {};
  minimum_size.width = 100;
  minimum_size.height = 100;

  FlutterDesktopSize maximum_size = {};
  maximum_size.width = -1;
  maximum_size.height = -1;

  window->SetSizeLimits(minimum_size, maximum_size);

  EXPECT_EQ(test_api->set_size_limits_called(), true);
}

}  // namespace flutter
