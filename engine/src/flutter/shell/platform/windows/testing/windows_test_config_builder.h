// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_WINDOWS_TESTING_WINDOWS_TEST_CONFIG_BUILDER_H_
#define FLUTTER_SHELL_PLATFORM_WINDOWS_TESTING_WINDOWS_TEST_CONFIG_BUILDER_H_

#include <string>
#include <string_view>
#include <vector>

#include "flutter/fml/macros.h"
#include "flutter/fml/unique_object.h"
#include "flutter/shell/platform/windows/public/flutter_windows.h"
#include "flutter/shell/platform/windows/testing/windows_test_context.h"

namespace flutter {
namespace testing {

// Deleter for FlutterDesktopEngineRef objects.
struct EngineDeleter {
  typedef FlutterDesktopEngineRef pointer;
  void operator()(FlutterDesktopEngineRef engine) {
    FML_CHECK(FlutterDesktopEngineDestroy(engine));
  }
};

// Unique pointer wrapper for FlutterDesktopEngineRef.
using EnginePtr = std::unique_ptr<FlutterDesktopEngine, EngineDeleter>;

// Deleter for FlutterViewControllerRef objects.
struct ViewControllerDeleter {
  typedef FlutterDesktopViewControllerRef pointer;
  void operator()(FlutterDesktopViewControllerRef controller) {
    FlutterDesktopViewControllerDestroy(controller);
  }
};

// Unique pointer wrapper for FlutterDesktopViewControllerRef.
using ViewControllerPtr =
    std::unique_ptr<FlutterDesktopViewController, ViewControllerDeleter>;

// Test configuration builder for WindowsTests.
//
// Utility class for configuring engine and view controller launch arguments,
// and launching the engine to run a test fixture.
class WindowsConfigBuilder {
 public:
  explicit WindowsConfigBuilder(WindowsTestContext& context);
  ~WindowsConfigBuilder();

  // Returns the desktop engine properties configured for this test.
  FlutterDesktopEngineProperties GetEngineProperties() const;

  // Sets the Dart entrypoint to the specified value.
  //
  // If not set, the default entrypoint (main) is used. Custom Dart entrypoints
  // must be decorated with `@pragma('vm:entry-point')`.
  void SetDartEntrypoint(std::string_view entrypoint);

  // Set the UI Thread policy for the engine.
  // If not set defaults to FlutterDesktopUIThreadPolicy::Default;
  void SetUIThreadPolicy(FlutterDesktopUIThreadPolicy policy);

  // Adds an argument to the Dart entrypoint arguments List<String>.
  void AddDartEntrypointArgument(std::string_view arg);

  void SetGpuPreference(FlutterDesktopGpuPreference gpu_preference);

  // Returns a configured and initialized engine.
  EnginePtr InitializeEngine() const;

  // Returns a configured and initialized engine that runs the configured Dart
  // entrypoint.
  //
  // Returns null on failure.
  EnginePtr RunHeadless() const;

  // Returns a configured and initialized view controller that runs the
  // configured Dart entrypoint and owns its engine.
  //
  // Returns null on failure.
  ViewControllerPtr Run() const;

 private:
  // Initialize COM, so that it is available for use in the library and/or
  // plugins.
  void InitializeCOM() const;

  WindowsTestContext& context_;
  std::string dart_entrypoint_;
  std::vector<std::string> dart_entrypoint_arguments_;
  FlutterDesktopUIThreadPolicy ui_thread_policy_ =
      FlutterDesktopUIThreadPolicy::Default;

  FlutterDesktopGpuPreference gpu_preference_ =
      FlutterDesktopGpuPreference::NoPreference;

  FML_DISALLOW_COPY_AND_ASSIGN(WindowsConfigBuilder);
};

}  // namespace testing
}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_WINDOWS_TESTING_WINDOWS_TEST_CONFIG_BUILDER_H_
