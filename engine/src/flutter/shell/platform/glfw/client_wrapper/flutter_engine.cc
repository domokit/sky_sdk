// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "include/flutter/flutter_engine.h"

#include <algorithm>
#include <iostream>

namespace flutter {

FlutterEngine::FlutterEngine() {}

FlutterEngine::~FlutterEngine() {}

bool FlutterEngine::Start(const std::string& icu_data_path,
                          const std::string& assets_path,
                          const std::vector<std::string>& arguments,
                          const std::string& aot_library_path) {
  if (engine_) {
    std::cerr << "Cannot run an already running engine. Create a new instance "
                 "or call ShutDown first."
              << std::endl;
    return false;
  }

  FlutterDesktopEngineProperties c_engine_properties = {};
  c_engine_properties.assets_path = assets_path.c_str();
  c_engine_properties.icu_data_path = icu_data_path.c_str();
  c_engine_properties.aot_library_path = aot_library_path.c_str();
  std::vector<const char*> engine_switches;
  std::transform(
      arguments.begin(), arguments.end(), std::back_inserter(engine_switches),
      [](const std::string& arg) -> const char* { return arg.c_str(); });
  if (!engine_switches.empty()) {
    c_engine_properties.switches = &engine_switches[0];
    c_engine_properties.switches_count = engine_switches.size();
  }

  engine_ = UniqueEnginePtr(FlutterDesktopRunEngine(c_engine_properties),
                            FlutterDesktopShutDownEngine);
  if (!engine_) {
    std::cerr << "Failed to start engine." << std::endl;
    return false;
  }
  return true;
}

void FlutterEngine::ShutDown() {
  engine_ = nullptr;
}

FlutterDesktopPluginRegistrarRef FlutterEngine::GetRegistrarForPlugin(
    const std::string& plugin_name) {
  if (!engine_) {
    std::cerr << "Cannot get plugin registrar on an engine that isn't running; "
                 "call Run first."
              << std::endl;
    return nullptr;
  }
  return FlutterDesktopGetPluginRegistrar(engine_.get(), plugin_name.c_str());
}

void FlutterEngine::RunEventLoopWithTimeout(std::chrono::milliseconds timeout) {
  if (!engine_) {
    std::cerr << "Cannot run event loop without a running engine; call "
                 "Run first."
              << std::endl;
    return;
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
  FlutterDesktopRunEngineEventLoopWithTimeout(engine_.get(),
                                              timeout_milliseconds);
}

}  // namespace flutter
