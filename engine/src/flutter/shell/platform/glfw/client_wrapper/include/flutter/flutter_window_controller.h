// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_GLFW_CLIENT_WRAPPER_INCLUDE_FLUTTER_FLUTTER_WINDOW_CONTROLLER_H_
#define FLUTTER_SHELL_PLATFORM_GLFW_CLIENT_WRAPPER_INCLUDE_FLUTTER_FLUTTER_WINDOW_CONTROLLER_H_

#include <flutter_glfw.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "flutter_window.h"
#include "plugin_registrar.h"
#include "plugin_registry.h"

namespace flutter {

// Properties for Flutter window creation.
struct WindowProperties {
  // The display title.
  std::string title;
  // Width in screen coordinates.
  int32_t width;
  // Height in screen coordinates.
  int32_t height;
  // Whether or not the user is prevented from resizing the window.
  // Reversed so that the default for a cleared struct is to allow resizing.
  bool prevent_resize;
};

// A controller for a window displaying Flutter content.
//
// This is the primary wrapper class for the desktop C API.
// If you use this class, you should not call any of the setup or teardown
// methods in the C API directly, as this class will do that internally.
//
// Note: This is an early implementation (using GLFW internally) which
// requires control of the application's event loop, and is thus useful
// primarily for building a simple one-window shell hosting a Flutter
// application. The final implementation and API will be very different.
class FlutterWindowController : public PluginRegistry {
 public:
  // There must be only one instance of this class in an application at any
  // given time, as Flutter does not support multiple engines in one process,
  // or multiple views in one engine.
  //
  // |icu_data_path| is the path to the icudtl.dat file for the version of
  // Flutter you are using.
  explicit FlutterWindowController(const std::string& icu_data_path);

  virtual ~FlutterWindowController();

  // Prevent copying.
  FlutterWindowController(FlutterWindowController const&) = delete;
  FlutterWindowController& operator=(FlutterWindowController const&) = delete;

  // Creates and displays a window for displaying Flutter content.
  //
  // The |assets_path| is the path to the flutter_assets folder for the Flutter
  // application to be run.
  //
  // The |arguments| are passed to the Flutter engine. See:
  // https://github.com/flutter/engine/blob/main/shell/common/switches.h for
  // details. Not all arguments will apply to desktop.
  //
  // The |aot_library_path| is the path to the libapp.so file for the Flutter
  // application to be run. While this parameter is only required in AOT mode,
  // it is perfectly safe to provide the path in non-AOT mode too.
  //
  // Only one Flutter window can exist at a time; see constructor comment.
  bool CreateWindow(const WindowProperties& window_properties,
                    const std::string& assets_path,
                    const std::vector<std::string>& arguments,
                    const std::string& aot_library_path = "");

  // Destroys the current window, if any.
  //
  // Because only one window can exist at a time, this method must be called
  // between calls to CreateWindow, or the second one will fail.
  void DestroyWindow();

  // The FlutterWindow managed by this controller, if any. Returns nullptr
  // before CreateWindow is called, after DestroyWindow is called, and after
  // RunEventLoop returns;
  FlutterWindow* window() { return window_.get(); }

  // Processes the next event on this window, or returns early if |timeout| is
  // reached before the next event.
  //
  // Returns false if the window was closed as a result of event processing.
  bool RunEventLoopWithTimeout(
      std::chrono::milliseconds timeout = std::chrono::milliseconds::max());

  // Deprecated. Use RunEventLoopWithTimeout.
  void RunEventLoop();

  // flutter::PluginRegistry:
  FlutterDesktopPluginRegistrarRef GetRegistrarForPlugin(
      const std::string& plugin_name) override;

 private:
  // The path to the ICU data file. Set at creation time since it is the same
  // for any window created.
  std::string icu_data_path_;

  // Whether or not FlutterDesktopInit succeeded at creation time.
  bool init_succeeded_ = false;

  // The owned FlutterWindow, if any.
  std::unique_ptr<FlutterWindow> window_;

  // Handle for interacting with the C API's window controller, if any.
  FlutterDesktopWindowControllerRef controller_ = nullptr;
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_GLFW_CLIENT_WRAPPER_INCLUDE_FLUTTER_FLUTTER_WINDOW_CONTROLLER_H_
