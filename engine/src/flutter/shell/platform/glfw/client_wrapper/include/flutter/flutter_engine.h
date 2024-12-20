// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_GLFW_CLIENT_WRAPPER_INCLUDE_FLUTTER_FLUTTER_ENGINE_H_
#define FLUTTER_SHELL_PLATFORM_GLFW_CLIENT_WRAPPER_INCLUDE_FLUTTER_FLUTTER_ENGINE_H_

#include <flutter_glfw.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "plugin_registrar.h"
#include "plugin_registry.h"

namespace flutter {

// An engine for running a headless Flutter application.
class FlutterEngine : public PluginRegistry {
 public:
  explicit FlutterEngine();

  virtual ~FlutterEngine();

  // Prevent copying.
  FlutterEngine(FlutterEngine const&) = delete;
  FlutterEngine& operator=(FlutterEngine const&) = delete;

  // Starts running the engine with the given parameters, returning true if
  // successful.
  bool Start(const std::string& icu_data_path,
             const std::string& assets_path,
             const std::vector<std::string>& arguments,
             const std::string& aot_library_path = "");

  // Terminates the running engine.
  void ShutDown();

  // Processes the next event for the engine, or returns early if |timeout| is
  // reached before the next event.
  void RunEventLoopWithTimeout(
      std::chrono::milliseconds timeout = std::chrono::milliseconds::max());

  // flutter::PluginRegistry:
  FlutterDesktopPluginRegistrarRef GetRegistrarForPlugin(
      const std::string& plugin_name) override;

 private:
  using UniqueEnginePtr = std::unique_ptr<FlutterDesktopEngineState,
                                          bool (*)(FlutterDesktopEngineState*)>;

  // Handle for interacting with the C API's engine reference.
  UniqueEnginePtr engine_ =
      UniqueEnginePtr(nullptr, FlutterDesktopShutDownEngine);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_GLFW_CLIENT_WRAPPER_INCLUDE_FLUTTER_FLUTTER_ENGINE_H_
