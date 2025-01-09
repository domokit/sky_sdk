// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_COMMON_WINDOWING_H_
#define FLUTTER_SHELL_PLATFORM_COMMON_WINDOWING_H_

#include <optional>

namespace flutter {

// A unique identifier for a view.
using FlutterViewId = int64_t;

// A point in 2D space for window positioning using integer coordinates.
struct WindowPoint {
  int x = 0;
  int y = 0;

  friend WindowPoint operator+(WindowPoint const& lhs, WindowPoint const& rhs) {
    return {lhs.x + rhs.x, lhs.y + rhs.y};
  }

  friend WindowPoint operator-(WindowPoint const& lhs, WindowPoint const& rhs) {
    return {lhs.x - rhs.x, lhs.y - rhs.y};
  }

  friend bool operator==(WindowPoint const& lhs, WindowPoint const& rhs) {
    return lhs.x == rhs.x && lhs.y == rhs.y;
  }
};

// A 2D size using integer dimensions.
struct WindowSize {
  int width = 0;
  int height = 0;

  explicit operator WindowPoint() const { return {width, height}; }

  friend bool operator==(WindowSize const& lhs, WindowSize const& rhs) {
    return lhs.width == rhs.width && lhs.height == rhs.height;
  }
};

// A rectangular area defined by a top-left point and size.
struct WindowRectangle {
  WindowPoint top_left;
  WindowSize size;

  // Checks if this rectangle fully contains |rect|.
  // Note: An empty rectangle can still contain other empty rectangles,
  // which are treated as points or lines of thickness zero
  bool contains(WindowRectangle const& rect) const {
    return rect.top_left.x >= top_left.x &&
           rect.top_left.x + rect.size.width <= top_left.x + size.width &&
           rect.top_left.y >= top_left.y &&
           rect.top_left.y + rect.size.height <= top_left.y + size.height;
  }

  friend bool operator==(WindowRectangle const& lhs,
                         WindowRectangle const& rhs) {
    return lhs.top_left == rhs.top_left && lhs.size == rhs.size;
  }
};

// Types of windows.
enum class WindowArchetype {
  // Regular top-level window.
  regular,
};

// Window metadata returned as the result of creating a Flutter window.
struct WindowMetadata {
  // The ID of the view used for this window, which is unique to each window.
  FlutterViewId view_id = 0;
  // The type of the window.
  WindowArchetype archetype = WindowArchetype::regular;
  // Size of the created window, in logical coordinates.
  WindowSize size;
  // The ID of the view used by the parent window. If not set, the window is
  // assumed a top-level window.
  std::optional<FlutterViewId> parent_id;
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_COMMON_WINDOWING_H_
