// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "include/flutter/flutter_engine.h"

#include <algorithm>
#include <iostream>

#include "binary_messenger_impl.h"
#include "flutter_windows.h"

namespace flutter {

FlutterEngine::FlutterEngine(const DartProject& project) {
  FlutterDesktopEngineProperties c_engine_properties = {};
  c_engine_properties.assets_path = project.assets_path().c_str();
  c_engine_properties.icu_data_path = project.icu_data_path().c_str();
  c_engine_properties.aot_library_path = project.aot_library_path().c_str();
  c_engine_properties.dart_entrypoint = project.dart_entrypoint().c_str();
  c_engine_properties.gpu_preference =
      static_cast<FlutterDesktopGpuPreference>(project.gpu_preference());
  c_engine_properties.ui_thread_policy =
      static_cast<FlutterDesktopUIThreadPolicy>(project.ui_thread_policy());

  const std::vector<std::string>& entrypoint_args =
      project.dart_entrypoint_arguments();
  std::vector<const char*> entrypoint_argv;
  std::transform(
      entrypoint_args.begin(), entrypoint_args.end(),
      std::back_inserter(entrypoint_argv),
      [](const std::string& arg) -> const char* { return arg.c_str(); });

  c_engine_properties.dart_entrypoint_argc =
      static_cast<int>(entrypoint_argv.size());
  c_engine_properties.dart_entrypoint_argv =
      entrypoint_argv.empty() ? nullptr : entrypoint_argv.data();

  engine_ = FlutterDesktopEngineCreate(&c_engine_properties);

  auto core_messenger = FlutterDesktopEngineGetMessenger(engine_);
  messenger_ = std::make_unique<BinaryMessengerImpl>(core_messenger);
}

FlutterEngine::~FlutterEngine() {
  ShutDown();
}

bool FlutterEngine::Run() {
  return Run(nullptr);
}

bool FlutterEngine::Run(const char* entry_point) {
  if (!engine_) {
    std::cerr << "Cannot run an engine that failed creation." << std::endl;
    return false;
  }
  if (run_succeeded_) {
    std::cerr << "Cannot run an engine more than once." << std::endl;
    return false;
  }
  bool run_succeeded = FlutterDesktopEngineRun(engine_, entry_point);
  if (!run_succeeded) {
    std::cerr << "Failed to start engine." << std::endl;
  }
  run_succeeded_ = true;
  return run_succeeded;
}

void FlutterEngine::ShutDown() {
  if (engine_ && owns_engine_) {
    FlutterDesktopEngineDestroy(engine_);
  }
  engine_ = nullptr;
}

std::chrono::nanoseconds FlutterEngine::ProcessMessages() {
  return std::chrono::nanoseconds(FlutterDesktopEngineProcessMessages(engine_));
}

void FlutterEngine::ReloadSystemFonts() {
  FlutterDesktopEngineReloadSystemFonts(engine_);
}

FlutterDesktopPluginRegistrarRef FlutterEngine::GetRegistrarForPlugin(
    const std::string& plugin_name) {
  if (!engine_) {
    std::cerr << "Cannot get plugin registrar on an engine that isn't running; "
                 "call Run first."
              << std::endl;
    return nullptr;
  }
  return FlutterDesktopEngineGetPluginRegistrar(engine_, plugin_name.c_str());
}

void FlutterEngine::SetNextFrameCallback(std::function<void()> callback) {
  next_frame_callback_ = std::move(callback);
  FlutterDesktopEngineSetNextFrameCallback(
      engine_,
      [](void* user_data) {
        FlutterEngine* self = static_cast<FlutterEngine*>(user_data);
        self->next_frame_callback_();
        self->next_frame_callback_ = nullptr;
      },
      this);
}

std::optional<LRESULT> FlutterEngine::ProcessExternalWindowMessage(
    HWND hwnd,
    UINT message,
    WPARAM wparam,
    LPARAM lparam) {
  LRESULT result;
  if (FlutterDesktopEngineProcessExternalWindowMessage(
          engine_, hwnd, message, wparam, lparam, &result)) {
    return result;
  }
  return std::nullopt;
}

FlutterDesktopEngineRef FlutterEngine::RelinquishEngine() {
  owns_engine_ = false;
  return engine_;
}

}  // namespace flutter
