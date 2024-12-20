// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/windows/platform_view_plugin.h"

namespace flutter {

PlatformViewPlugin::PlatformViewPlugin(BinaryMessenger* messenger,
                                       TaskRunner* task_runner)
    : PlatformViewManager(messenger), task_runner_(task_runner) {}

PlatformViewPlugin::~PlatformViewPlugin() {}

// TODO(schectman): Impelement platform view lookup.
// https://github.com/flutter/flutter/issues/143375
std::optional<HWND> PlatformViewPlugin::GetNativeHandleForId(
    PlatformViewId id) const {
  return std::nullopt;
}

// TODO(schectman): Impelement platform view type registration.
// https://github.com/flutter/flutter/issues/143375
void PlatformViewPlugin::RegisterPlatformViewType(
    std::string_view type_name,
    const FlutterPlatformViewTypeEntry& type) {}

// TODO(schectman): Impelement platform view instantiation.
// https://github.com/flutter/flutter/issues/143375
void PlatformViewPlugin::InstantiatePlatformView(PlatformViewId id) {}

// TODO(schectman): Impelement platform view addition.
// https://github.com/flutter/flutter/issues/143375
bool PlatformViewPlugin::AddPlatformView(PlatformViewId id,
                                         std::string_view type_name) {
  return true;
}

// TODO(schectman): Impelement platform view focus request.
// https://github.com/flutter/flutter/issues/143375
bool PlatformViewPlugin::FocusPlatformView(PlatformViewId id,
                                           FocusChangeDirection direction,
                                           bool focus) {
  return true;
}

}  // namespace flutter
