// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_WINDOWS_CURSOR_HANDLER_H_
#define FLUTTER_SHELL_PLATFORM_WINDOWS_CURSOR_HANDLER_H_

#include <unordered_map>

#include "flutter/fml/macros.h"
#include "flutter/shell/platform/common/client_wrapper/include/flutter/binary_messenger.h"
#include "flutter/shell/platform/common/client_wrapper/include/flutter/encodable_value.h"
#include "flutter/shell/platform/common/client_wrapper/include/flutter/method_channel.h"
#include "flutter/shell/platform/windows/public/flutter_windows.h"
#include "flutter/shell/platform/windows/window_binding_handler.h"

namespace flutter {

class FlutterWindowsEngine;

// Handler for the cursor system channel.
class CursorHandler {
 public:
  explicit CursorHandler(flutter::BinaryMessenger* messenger,
                         flutter::FlutterWindowsEngine* engine);

 private:
  // Called when a method is called on |channel_|;
  void HandleMethodCall(
      const flutter::MethodCall<EncodableValue>& method_call,
      std::unique_ptr<flutter::MethodResult<EncodableValue>> result);

  // The MethodChannel used for communication with the Flutter engine.
  std::unique_ptr<flutter::MethodChannel<EncodableValue>> channel_;

  // The Flutter engine that will be notified for cursor updates.
  FlutterWindowsEngine* engine_;

  // The cache map for custom cursors.
  std::unordered_map<std::string, HCURSOR> custom_cursors_;

  FML_DISALLOW_COPY_AND_ASSIGN(CursorHandler);
};

// Create a cursor from a rawBGRA buffer and the cursor info.
HCURSOR GetCursorFromBuffer(const std::vector<uint8_t>& buffer,
                            double hot_x,
                            double hot_y,
                            int width,
                            int height);

// Get the corresponding mask bitmap from the source bitmap.
void GetMaskBitmaps(HBITMAP bitmap, HBITMAP& mask_bitmap);

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_WINDOWS_CURSOR_HANDLER_H_
