// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_COMMON_WINDOWING_H_
#define FLUTTER_SHELL_PLATFORM_COMMON_WINDOWING_H_

#include <optional>

#include "flutter/fml/logging.h"
#include "geometry.h"

namespace flutter {

// A unique identifier for a view.
using FlutterViewId = int64_t;

// Types of windows.
enum class WindowArchetype {
  // Regular top-level window.
  kRegular,
};

// Possible states a window can be in.
enum class WindowState {
  // Normal state, neither maximized, nor minimized.
  kRestored,
  // Maximized, occupying the full screen but still showing the system UI.
  kMaximized,
  // Minimized and not visible on the screen.
  kMinimized,
};

// Converts a |flutter::WindowState| to the string representation of
// |WindowState| as defined in the framework.
inline std::string WindowStateToString(WindowState state) {
  switch (state) {
    case WindowState::kRestored:
      return "WindowState.restored";
    case WindowState::kMaximized:
      return "WindowState.maximized";
    case WindowState::kMinimized:
      return "WindowState.minimized";
    default:
      FML_UNREACHABLE();
  }
}

// Converts the string representation of |WindowState| defined in the framework
// to a |flutter::WindowState|. Returns std::nullopt if the given string is
// invalid.
inline std::optional<WindowState> StringToWindowState(std::string_view str) {
  if (str == "WindowState.restored")
    return WindowState::kRestored;
  if (str == "WindowState.maximized")
    return WindowState::kMaximized;
  if (str == "WindowState.minimized")
    return WindowState::kMinimized;
  return std::nullopt;
}

// Settings used for creating a Flutter window.
struct WindowCreationSettings {
  // Type of the window.
  WindowArchetype archetype = WindowArchetype::kRegular;
  // Requested size of the window's client area, in logical coordinates.
  Size size;
  // Minimum size of the window's client area, in logical coordinates.
  std::optional<Size> min_size;
  // Maximum size of the window's client area, in logical coordinates.
  std::optional<Size> max_size;
  // Window title.
  std::optional<std::string> title;
  // Initial state of the window.
  std::optional<WindowState> state;
};

// Settings for modifying a Flutter window.
struct WindowModificationSettings {
  // The new requested size, in logical coordinates.
  std::optional<Size> size;
  // The new window title.
  std::optional<std::string> title;
  // The new window state.
  std::optional<WindowState> state;
};

// Window metadata returned as the result of creating a Flutter window.
struct WindowMetadata {
  // The ID of the view used for this window, which is unique to each window.
  FlutterViewId view_id = 0;
  // The type of the window.
  WindowArchetype archetype = WindowArchetype::kRegular;
  // Size of the created window, in logical coordinates.
  Size size;
  // The ID of the view used by the parent window. If not set, the window is
  // assumed a top-level window.
  std::optional<FlutterViewId> parent_id;
  // The initial state of the window, or std::nullopt if not a regular window.
  std::optional<WindowState> state;
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_COMMON_WINDOWING_H_
