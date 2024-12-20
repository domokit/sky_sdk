// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_GLFW_CLIENT_WRAPPER_INCLUDE_FLUTTER_PLUGIN_REGISTRAR_GLFW_H_
#define FLUTTER_SHELL_PLATFORM_GLFW_CLIENT_WRAPPER_INCLUDE_FLUTTER_PLUGIN_REGISTRAR_GLFW_H_

#include <flutter_glfw.h>

#include <memory>

#include "flutter_window.h"
#include "plugin_registrar.h"

namespace flutter {

// An extension to PluginRegistrar providing access to GLFW-shell-specific
// functionality.
class PluginRegistrarGlfw : public PluginRegistrar {
 public:
  // Creates a new PluginRegistrar. |core_registrar| and the messenger it
  // provides must remain valid as long as this object exists.
  explicit PluginRegistrarGlfw(FlutterDesktopPluginRegistrarRef core_registrar)
      : PluginRegistrar(core_registrar) {
    window_ = std::make_unique<FlutterWindow>(
        FlutterDesktopPluginRegistrarGetWindow(core_registrar));
  }

  virtual ~PluginRegistrarGlfw() {
    // Must be the first call.
    ClearPlugins();
    // Explicitly cleared to facilitate destruction order testing.
    window_.reset();
  }

  // Prevent copying.
  PluginRegistrarGlfw(PluginRegistrarGlfw const&) = delete;
  PluginRegistrarGlfw& operator=(PluginRegistrarGlfw const&) = delete;

  FlutterWindow* window() { return window_.get(); }

  // Enables input blocking on the given channel name.
  //
  // If set, then the parent window should disable input callbacks
  // while waiting for the handler for messages on that channel to run.
  void EnableInputBlockingForChannel(const std::string& channel) {
    FlutterDesktopPluginRegistrarEnableInputBlocking(registrar(),
                                                     channel.c_str());
  }

 private:
  // The owned FlutterWindow, if any.
  std::unique_ptr<FlutterWindow> window_;
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_GLFW_CLIENT_WRAPPER_INCLUDE_FLUTTER_PLUGIN_REGISTRAR_GLFW_H_
