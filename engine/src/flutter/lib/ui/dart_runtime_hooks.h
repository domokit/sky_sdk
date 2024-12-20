// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_DART_RUNTIME_HOOKS_H_
#define FLUTTER_LIB_UI_DART_RUNTIME_HOOKS_H_

#include "flutter/fml/macros.h"
#include "third_party/dart/runtime/include/dart_api.h"
#include "third_party/tonic/dart_library_natives.h"

namespace flutter {

class DartRuntimeHooks {
 public:
  static void Install(bool is_ui_isolate, const std::string& script_uri);

  static void Logger_PrintDebugString(const std::string& message);

  static void Logger_PrintString(const std::string& message);

  static void ScheduleMicrotask(Dart_Handle closure);

  static Dart_Handle GetCallbackHandle(Dart_Handle func);

  static Dart_Handle GetCallbackFromHandle(int64_t handle);

 private:
  FML_DISALLOW_IMPLICIT_CONSTRUCTORS(DartRuntimeHooks);
};

void DartPluginRegistrant_EnsureInitialized();

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_DART_RUNTIME_HOOKS_H_
