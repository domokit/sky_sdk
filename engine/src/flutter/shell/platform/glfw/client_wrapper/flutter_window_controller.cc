// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "include/flutter/flutter_window_controller.h"

#include <algorithm>
#include <iostream>

namespace flutter {

FlutterWindowController::FlutterWindowController(
    const std::string& icu_data_path)
    : icu_data_path_(icu_data_path) {
  init_succeeded_ = FlutterDesktopInit();
}

FlutterWindowController::~FlutterWindowController() {
  if (controller_) {
    FlutterDesktopDestroyWindow(controller_);
  }
  if (init_succeeded_) {
    FlutterDesktopTerminate();
  }
}

bool FlutterWindowController::CreateWindow(
    const WindowProperties& window_properties,
    const std::string& assets_path,
    const std::vector<std::string>& arguments,
    const std::string& aot_library_path) {
  if (!init_succeeded_) {
    std::cerr << "Could not create window; FlutterDesktopInit failed."
              << std::endl;
    return false;
  }

  if (controller_) {
    std::cerr << "Only one Flutter window can exist at a time." << std::endl;
    return false;
  }

  FlutterDesktopWindowProperties c_window_properties = {};
  c_window_properties.title = window_properties.title.c_str();
  c_window_properties.width = window_properties.width;
  c_window_properties.height = window_properties.height;
  c_window_properties.prevent_resize = window_properties.prevent_resize;

  FlutterDesktopEngineProperties c_engine_properties = {};
  c_engine_properties.assets_path = assets_path.c_str();
  c_engine_properties.aot_library_path = aot_library_path.c_str();
  c_engine_properties.icu_data_path = icu_data_path_.c_str();
  std::vector<const char*> engine_switches;
  std::transform(
      arguments.begin(), arguments.end(), std::back_inserter(engine_switches),
      [](const std::string& arg) -> const char* { return arg.c_str(); });
  if (!engine_switches.empty()) {
    c_engine_properties.switches = &engine_switches[0];
    c_engine_properties.switches_count = engine_switches.size();
  }

  controller_ =
      FlutterDesktopCreateWindow(c_window_properties, c_engine_properties);
  if (!controller_) {
    std::cerr << "Failed to create window." << std::endl;
    return false;
  }
  window_ =
      std::make_unique<FlutterWindow>(FlutterDesktopGetWindow(controller_));
  return true;
}

void FlutterWindowController::DestroyWindow() {
  if (controller_) {
    FlutterDesktopDestroyWindow(controller_);
    controller_ = nullptr;
    window_ = nullptr;
  }
}

FlutterDesktopPluginRegistrarRef FlutterWindowController::GetRegistrarForPlugin(
    const std::string& plugin_name) {
  if (!controller_) {
    std::cerr << "Cannot get plugin registrar without a window; call "
                 "CreateWindow first."
              << std::endl;
    return nullptr;
  }
  return FlutterDesktopGetPluginRegistrar(FlutterDesktopGetEngine(controller_),
                                          plugin_name.c_str());
}

bool FlutterWindowController::RunEventLoopWithTimeout(
    std::chrono::milliseconds timeout) {
  if (!controller_) {
    std::cerr << "Cannot run event loop without a window window; call "
                 "CreateWindow first."
              << std::endl;
    return false;
  }
  uint32_t timeout_milliseconds;
  if (timeout == std::chrono::milliseconds::max()) {
    // The C API uses 0 to represent no timeout, so convert |max| to 0.
    timeout_milliseconds = 0;
  } else if (timeout.count() > UINT32_MAX) {
    timeout_milliseconds = UINT32_MAX;
  } else {
    timeout_milliseconds = static_cast<uint32_t>(timeout.count());
  }
  bool still_running = FlutterDesktopRunWindowEventLoopWithTimeout(
      controller_, timeout_milliseconds);
  if (!still_running) {
    DestroyWindow();
  }
  return still_running;
}

void FlutterWindowController::RunEventLoop() {
  while (RunEventLoopWithTimeout()) {
  }
}

}  // namespace flutter
