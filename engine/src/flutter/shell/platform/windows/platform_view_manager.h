// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_WINDOWS_PLATFORM_VIEW_MANAGER_H_
#define FLUTTER_SHELL_PLATFORM_WINDOWS_PLATFORM_VIEW_MANAGER_H_

#include <memory>

#include "flutter/shell/platform/common/client_wrapper/include/flutter/method_channel.h"
#include "flutter/shell/platform/windows/flutter_windows_internal.h"
#include "flutter/shell/platform/windows/task_runner.h"

namespace flutter {

// Possible reasons for change of keyboard focus.
enum class FocusChangeDirection {
  kProgrammatic,  // Un-directed focus change.
  kForward,       // Keyboard focus moves forwards, e.g. TAB key.
  kBackward       // Keyboard focus moves backwards, e.g. Shift+TAB.
};

// The platform method handler for platform view related communication between
// the engine and the framework. This base class is derived by a concrete class
// (i.e. PlatformViewPlugin) to provide implementation of its abstract virtual
// methods.
class PlatformViewManager {
 public:
  PlatformViewManager(BinaryMessenger* binary_messenger);

  virtual ~PlatformViewManager();

  // Add a new platform view instance to be lazily instantiated when it is next
  // composited. The manager will invoke Success when this method returns true,
  // and invoke Error otherwise.
  virtual bool AddPlatformView(PlatformViewId id,
                               std::string_view type_name) = 0;

  // The framework may invoke this method when keyboard focus must be given to
  // the platform view. The manager will invoke Success when this method returns
  // true, and invoke Error otherwise.
  virtual bool FocusPlatformView(PlatformViewId id,
                                 FocusChangeDirection direction,
                                 bool focus) = 0;

 private:
  std::unique_ptr<MethodChannel<EncodableValue>> channel_;
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_WINDOWS_PLATFORM_VIEW_MANAGER_H_
