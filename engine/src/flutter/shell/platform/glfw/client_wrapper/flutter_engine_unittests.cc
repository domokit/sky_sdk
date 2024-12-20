// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <memory>
#include <string>

#include "flutter/shell/platform/glfw/client_wrapper/include/flutter/flutter_engine.h"
#include "flutter/shell/platform/glfw/client_wrapper/testing/stub_flutter_glfw_api.h"
#include "gtest/gtest.h"

namespace flutter {

namespace {

// Stub implementation to validate calls to the API.
class TestGlfwApi : public testing::StubFlutterGlfwApi {
 public:
  // |flutter::testing::StubFlutterGlfwApi|
  FlutterDesktopEngineRef RunEngine(
      const FlutterDesktopEngineProperties& properties) override {
    run_called_ = true;
    return reinterpret_cast<FlutterDesktopEngineRef>(1);
  }

  // |flutter::testing::StubFlutterGlfwApi|
  void RunEngineEventLoopWithTimeout(uint32_t millisecond_timeout) override {
    last_run_loop_timeout_ = millisecond_timeout;
  }

  // |flutter::testing::StubFlutterGlfwApi|
  bool ShutDownEngine() override {
    shut_down_called_ = true;
    return true;
  }

  bool run_called() { return run_called_; }

  bool shut_down_called() { return shut_down_called_; }

  uint32_t last_run_loop_timeout() { return last_run_loop_timeout_; }

 private:
  bool run_called_ = false;
  bool shut_down_called_ = false;
  uint32_t last_run_loop_timeout_ = 0;
};

}  // namespace

TEST(FlutterEngineTest, CreateDestroy) {
  const std::string icu_data_path = "fake/path/to/icu";
  const std::string assets_path = "fake/path/to/assets";
  testing::ScopedStubFlutterGlfwApi scoped_api_stub(
      std::make_unique<TestGlfwApi>());
  auto test_api = static_cast<TestGlfwApi*>(scoped_api_stub.stub());
  {
    FlutterEngine engine;
    engine.Start(icu_data_path, assets_path, {});
    EXPECT_EQ(test_api->run_called(), true);
    EXPECT_EQ(test_api->shut_down_called(), false);
  }
  // Destroying should implicitly shut down if it hasn't been done manually.
  EXPECT_EQ(test_api->shut_down_called(), true);
}

TEST(FlutterEngineTest, ExplicitShutDown) {
  const std::string icu_data_path = "fake/path/to/icu";
  const std::string assets_path = "fake/path/to/assets";
  testing::ScopedStubFlutterGlfwApi scoped_api_stub(
      std::make_unique<TestGlfwApi>());
  auto test_api = static_cast<TestGlfwApi*>(scoped_api_stub.stub());

  FlutterEngine engine;
  engine.Start(icu_data_path, assets_path, {});
  EXPECT_EQ(test_api->run_called(), true);
  EXPECT_EQ(test_api->shut_down_called(), false);
  engine.ShutDown();
  EXPECT_EQ(test_api->shut_down_called(), true);
}

TEST(FlutterEngineTest, RunloopTimeoutTranslation) {
  const std::string icu_data_path = "fake/path/to/icu";
  const std::string assets_path = "fake/path/to/assets";
  testing::ScopedStubFlutterGlfwApi scoped_api_stub(
      std::make_unique<TestGlfwApi>());
  auto test_api = static_cast<TestGlfwApi*>(scoped_api_stub.stub());

  FlutterEngine engine;
  engine.Start(icu_data_path, assets_path, {});

  engine.RunEventLoopWithTimeout(std::chrono::milliseconds(100));
  EXPECT_EQ(test_api->last_run_loop_timeout(), 100U);

  engine.RunEventLoopWithTimeout(std::chrono::milliseconds::max() -
                                 std::chrono::milliseconds(1));
  EXPECT_EQ(test_api->last_run_loop_timeout(), UINT32_MAX);

  engine.RunEventLoopWithTimeout(std::chrono::milliseconds::max());
  EXPECT_EQ(test_api->last_run_loop_timeout(), 0U);
}

}  // namespace flutter
