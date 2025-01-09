// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_WINDOWS_WINDOWING_HANDLER_H_
#define FLUTTER_SHELL_PLATFORM_WINDOWS_WINDOWING_HANDLER_H_

#include "flutter/fml/macros.h"
#include "flutter/shell/platform/common/client_wrapper/include/flutter/method_channel.h"
#include "flutter/shell/platform/windows/flutter_host_window_controller.h"

namespace flutter {

// Handler for the windowing channel.
class WindowingHandler {
 public:
  explicit WindowingHandler(flutter::BinaryMessenger* messenger,
                            flutter::FlutterHostWindowController* controller);

 private:
  // Handler for method calls received on |channel_|. Messages are
  // redirected to either HandleCreateWindow or HandleDestroyWindow.
  void HandleMethodCall(
      const flutter::MethodCall<EncodableValue>& method_call,
      std::unique_ptr<flutter::MethodResult<EncodableValue>> result);

  // Handles the creation of windows.
  void HandleCreateWindow(flutter::WindowArchetype archetype,
                          flutter::MethodCall<> const& call,
                          flutter::MethodResult<>& result);
  // Handles the destruction of windows.
  void HandleDestroyWindow(flutter::MethodCall<> const& call,
                           flutter::MethodResult<>& result);

  // The MethodChannel used for communication with the Flutter engine.
  std::shared_ptr<flutter::MethodChannel<EncodableValue>> channel_;

  // The controller of the host windows.
  flutter::FlutterHostWindowController* controller_;

  FML_DISALLOW_COPY_AND_ASSIGN(WindowingHandler);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_WINDOWS_WINDOWING_HANDLER_H_
