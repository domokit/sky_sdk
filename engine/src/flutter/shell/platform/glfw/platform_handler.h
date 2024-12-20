// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_GLFW_PLATFORM_HANDLER_H_
#define FLUTTER_SHELL_PLATFORM_GLFW_PLATFORM_HANDLER_H_

#include <GLFW/glfw3.h>

#include "flutter/shell/platform/common/client_wrapper/include/flutter/binary_messenger.h"
#include "flutter/shell/platform/common/client_wrapper/include/flutter/method_channel.h"
#include "flutter/shell/platform/glfw/public/flutter_glfw.h"
#include "rapidjson/document.h"

namespace flutter {

// Handler for internal system channels.
class PlatformHandler {
 public:
  explicit PlatformHandler(flutter::BinaryMessenger* messenger,
                           GLFWwindow* window);

 private:
  // Called when a method is called on |channel_|;
  void HandleMethodCall(
      const flutter::MethodCall<rapidjson::Document>& method_call,
      std::unique_ptr<flutter::MethodResult<rapidjson::Document>> result);

  // The MethodChannel used for communication with the Flutter engine.
  std::unique_ptr<flutter::MethodChannel<rapidjson::Document>> channel_;

  // A reference to the GLFW window, if any. Null in headless mode.
  GLFWwindow* window_;
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_GLFW_PLATFORM_HANDLER_H_
