// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_WINDOWS_ACCESSIBILITY_PLUGIN_H_
#define FLUTTER_SHELL_PLATFORM_WINDOWS_ACCESSIBILITY_PLUGIN_H_

#include <string_view>

#include "flutter/fml/macros.h"
#include "flutter/shell/platform/common/client_wrapper/include/flutter/binary_messenger.h"

namespace flutter {

class FlutterWindowsEngine;

// Handles messages on the flutter/accessibility channel.
//
// See:
// https://api.flutter.dev/flutter/semantics/SemanticsService-class.html
class AccessibilityPlugin {
 public:
  explicit AccessibilityPlugin(FlutterWindowsEngine* engine);

  // Begin handling accessibility messages on the `binary_messenger`.
  static void SetUp(BinaryMessenger* binary_messenger,
                    AccessibilityPlugin* plugin);

  // Announce a message through the assistive technology.
  virtual void Announce(const std::string_view message);

 private:
  // The engine that owns this plugin.
  FlutterWindowsEngine* engine_ = nullptr;

  FML_DISALLOW_COPY_AND_ASSIGN(AccessibilityPlugin);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_WINDOWS_ACCESSIBILITY_PLUGIN_H_
