// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_WINDOWS_TESTING_ENGINE_MODIFIER_H_
#define FLUTTER_SHELL_PLATFORM_WINDOWS_TESTING_ENGINE_MODIFIER_H_

#include "flutter/shell/platform/windows/flutter_windows_engine.h"

#include <chrono>

#include "flutter/fml/macros.h"

namespace flutter {

// A test utility class providing the ability to access and alter various
// private fields in an Engine instance.
//
// This simply provides a way to access the normally-private embedder proc
// table, so the lifetime of any changes made to the proc table is that of the
// engine object, not this helper.
class EngineModifier {
 public:
  explicit EngineModifier(FlutterWindowsEngine* engine) : engine_(engine) {}

  // Returns the engine's embedder API proc table, allowing for modification.
  //
  // Modifications are to the engine, and will last for the lifetime of the
  // engine unless overwritten again.
  FlutterEngineProcTable& embedder_api() { return engine_->embedder_api_; }

  // Override the EGL manager used by the engine.
  //
  // Modifications are to the engine, and will last for the lifetime of the
  // engine unless overwritten again.
  void SetEGLManager(std::unique_ptr<egl::Manager> egl_manager) {
    engine_->egl_manager_ = std::move(egl_manager);
  }

  // Override the engine's implicit view. This is the "default" view
  // that Flutter apps render to.
  void SetImplicitView(FlutterWindowsView* view) {
    engine_->views_[kImplicitViewId] = view;
  }

  /// Reset the start_time field that is used to align vsync events.
  void SetStartTime(uint64_t start_time_nanos) {
    engine_->start_time_ = std::chrono::nanoseconds(start_time_nanos);
  }

  /// Override the frame interval to the provided nanosecond interval.
  ///
  /// This will prevent the windows engine from delegating to dwm to
  /// discover the true frame interval, which can vary across machines.
  void SetFrameInterval(uint64_t frame_interval_nanos) {
    engine_->frame_interval_override_ =
        std::optional<std::chrono::nanoseconds>(frame_interval_nanos);
  }

  // Explicitly releases the egl::Manager being used by the
  // FlutterWindowsEngine instance. This should be used if SetEGLManager is
  // used to explicitly set to a non-null value (but not a valid object) to test
  // a successful ANGLE initialization.
  //
  // Modifications are to the engine, and will last for the lifetime of the
  // engine unless overwritten again.
  void ReleaseEGLManager() { engine_->egl_manager_.release(); }

  // Run the FlutterWindowsEngine's handler that runs right before an engine
  // restart. This resets the keyboard's state if it exists.
  void Restart() { engine_->OnPreEngineRestart(); }

  // Initialize they keyboard and text input subsystems or reset them them if
  // they are already initialized.
  void InitializeKeyboard() { engine_->InitializeKeyboard(); }

  void SetLifecycleManager(std::unique_ptr<WindowsLifecycleManager>&& handler) {
    engine_->lifecycle_manager_ = std::move(handler);
  }

  void SetPlatformViewPlugin(std::unique_ptr<PlatformViewPlugin>&& manager) {
    engine_->platform_view_plugin_ = std::move(manager);
  }

 private:
  FlutterWindowsEngine* engine_;

  FML_DISALLOW_COPY_AND_ASSIGN(EngineModifier);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_WINDOWS_TESTING_ENGINE_MODIFIER_H_
