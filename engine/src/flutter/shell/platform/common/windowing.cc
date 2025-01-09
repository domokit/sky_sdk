// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/common/windowing.h"

#include <algorithm>
#include <iostream>

namespace flutter {

namespace {

WindowPoint offset_for(WindowSize const& size,
                       WindowPositioner::Anchor anchor) {
  switch (anchor) {
    case WindowPositioner::Anchor::top_left:
      return {0, 0};
    case WindowPositioner::Anchor::top:
      return {-size.width / 2, 0};
    case WindowPositioner::Anchor::top_right:
      return {-1 * size.width, 0};
    case WindowPositioner::Anchor::left:
      return {0, -size.height / 2};
    case WindowPositioner::Anchor::center:
      return {-size.width / 2, -size.height / 2};
    case WindowPositioner::Anchor::right:
      return {-1 * size.width, -size.height / 2};
    case WindowPositioner::Anchor::bottom_left:
      return {0, -1 * size.height};
    case WindowPositioner::Anchor::bottom:
      return {-size.width / 2, -1 * size.height};
    case WindowPositioner::Anchor::bottom_right:
      return {-1 * size.width, -1 * size.height};
    default:
      std::cerr << "Unknown anchor value: " << static_cast<int>(anchor) << '\n';
      std::abort();
  }
}

WindowPoint anchor_position_for(WindowRectangle const& rect,
                                WindowPositioner::Anchor anchor) {
  switch (anchor) {
    case WindowPositioner::Anchor::top_left:
      return rect.top_left;
    case WindowPositioner::Anchor::top:
      return rect.top_left + WindowPoint{rect.size.width / 2, 0};
    case WindowPositioner::Anchor::top_right:
      return rect.top_left + WindowPoint{rect.size.width, 0};
    case WindowPositioner::Anchor::left:
      return rect.top_left + WindowPoint{0, rect.size.height / 2};
    case WindowPositioner::Anchor::center:
      return rect.top_left +
             WindowPoint{rect.size.width / 2, rect.size.height / 2};
    case WindowPositioner::Anchor::right:
      return rect.top_left + WindowPoint{rect.size.width, rect.size.height / 2};
    case WindowPositioner::Anchor::bottom_left:
      return rect.top_left + WindowPoint{0, rect.size.height};
    case WindowPositioner::Anchor::bottom:
      return rect.top_left + WindowPoint{rect.size.width / 2, rect.size.height};
    case WindowPositioner::Anchor::bottom_right:
      return rect.top_left + WindowPoint{rect.size.width, rect.size.height};
    default:
      std::cerr << "Unknown anchor value: " << static_cast<int>(anchor) << '\n';
      std::abort();
  }
}

WindowPoint constrain_to(WindowRectangle const& r, WindowPoint const& p) {
  return {std::clamp(p.x, r.top_left.x, r.top_left.x + r.size.width),
          std::clamp(p.y, r.top_left.y, r.top_left.y + r.size.height)};
}

WindowPositioner::Anchor flip_anchor_x(WindowPositioner::Anchor anchor) {
  switch (anchor) {
    case WindowPositioner::Anchor::top_left:
      return WindowPositioner::Anchor::top_right;
    case WindowPositioner::Anchor::top_right:
      return WindowPositioner::Anchor::top_left;
    case WindowPositioner::Anchor::left:
      return WindowPositioner::Anchor::right;
    case WindowPositioner::Anchor::right:
      return WindowPositioner::Anchor::left;
    case WindowPositioner::Anchor::bottom_left:
      return WindowPositioner::Anchor::bottom_right;
    case WindowPositioner::Anchor::bottom_right:
      return WindowPositioner::Anchor::bottom_left;
    default:
      return anchor;
  }
}

WindowPositioner::Anchor flip_anchor_y(WindowPositioner::Anchor anchor) {
  switch (anchor) {
    case WindowPositioner::Anchor::top_left:
      return WindowPositioner::Anchor::bottom_left;
    case WindowPositioner::Anchor::top:
      return WindowPositioner::Anchor::bottom;
    case WindowPositioner::Anchor::top_right:
      return WindowPositioner::Anchor::bottom_right;
    case WindowPositioner::Anchor::bottom_left:
      return WindowPositioner::Anchor::top_left;
    case WindowPositioner::Anchor::bottom:
      return WindowPositioner::Anchor::top;
    case WindowPositioner::Anchor::bottom_right:
      return WindowPositioner::Anchor::top_right;
    default:
      return anchor;
  }
}

WindowPoint flip_offset_x(WindowPoint const& p) {
  return {-1 * p.x, p.y};
}

WindowPoint flip_offset_y(WindowPoint const& p) {
  return {p.x, -1 * p.y};
}

}  // namespace

WindowRectangle PlaceWindow(WindowPositioner const& positioner,
                            WindowSize child_size,
                            WindowRectangle const& anchor_rect,
                            WindowRectangle const& parent_rect,
                            WindowRectangle const& output_rect) {
  WindowRectangle default_result;

  {
    WindowPoint const result =
        constrain_to(parent_rect, anchor_position_for(
                                      anchor_rect, positioner.parent_anchor) +
                                      positioner.offset) +
        offset_for(child_size, positioner.child_anchor);

    if (output_rect.contains({result, child_size})) {
      return WindowRectangle{result, child_size};
    }

    default_result = WindowRectangle{result, child_size};
  }

  if (static_cast<int>(positioner.constraint_adjustment) &
      static_cast<int>(WindowPositioner::ConstraintAdjustment::flip_x)) {
    WindowPoint const result =
        constrain_to(parent_rect,
                     anchor_position_for(
                         anchor_rect, flip_anchor_x(positioner.parent_anchor)) +
                         flip_offset_x(positioner.offset)) +
        offset_for(child_size, flip_anchor_x(positioner.child_anchor));

    if (output_rect.contains({result, child_size})) {
      return WindowRectangle{result, child_size};
    }
  }

  if (static_cast<int>(positioner.constraint_adjustment) &
      static_cast<int>(WindowPositioner::ConstraintAdjustment::flip_y)) {
    WindowPoint const result =
        constrain_to(parent_rect,
                     anchor_position_for(
                         anchor_rect, flip_anchor_y(positioner.parent_anchor)) +
                         flip_offset_y(positioner.offset)) +
        offset_for(child_size, flip_anchor_y(positioner.child_anchor));

    if (output_rect.contains({result, child_size})) {
      return WindowRectangle{result, child_size};
    }
  }

  if (static_cast<int>(positioner.constraint_adjustment) &
          static_cast<int>(WindowPositioner::ConstraintAdjustment::flip_x) &&
      static_cast<int>(positioner.constraint_adjustment) &
          static_cast<int>(WindowPositioner::ConstraintAdjustment::flip_y)) {
    WindowPoint const result =
        constrain_to(
            parent_rect,
            anchor_position_for(anchor_rect, flip_anchor_x(flip_anchor_y(
                                                 positioner.parent_anchor))) +
                flip_offset_x(flip_offset_y(positioner.offset))) +
        offset_for(child_size,
                   flip_anchor_x(flip_anchor_y(positioner.child_anchor)));

    if (output_rect.contains({result, child_size})) {
      return WindowRectangle{result, child_size};
    }
  }

  {
    WindowPoint result =
        constrain_to(parent_rect, anchor_position_for(
                                      anchor_rect, positioner.parent_anchor) +
                                      positioner.offset) +
        offset_for(child_size, positioner.child_anchor);

    if (static_cast<int>(positioner.constraint_adjustment) &
        static_cast<int>(WindowPositioner::ConstraintAdjustment::slide_x)) {
      int const left_overhang = result.x - output_rect.top_left.x;
      int const right_overhang =
          (result.x + child_size.width) -
          (output_rect.top_left.x + output_rect.size.width);

      if (left_overhang < 0) {
        result.x -= left_overhang;
      } else if (right_overhang > 0) {
        result.x -= right_overhang;
      }
    }

    if (static_cast<int>(positioner.constraint_adjustment) &
        static_cast<int>(WindowPositioner::ConstraintAdjustment::slide_y)) {
      int const top_overhang = result.y - output_rect.top_left.y;
      int const bot_overhang =
          (result.y + child_size.height) -
          (output_rect.top_left.y + output_rect.size.height);

      if (top_overhang < 0) {
        result.y -= top_overhang;
      } else if (bot_overhang > 0) {
        result.y -= bot_overhang;
      }
    }

    if (output_rect.contains({result, child_size})) {
      return WindowRectangle{result, child_size};
    }
  }

  {
    WindowPoint result =
        constrain_to(parent_rect, anchor_position_for(
                                      anchor_rect, positioner.parent_anchor) +
                                      positioner.offset) +
        offset_for(child_size, positioner.child_anchor);

    if (static_cast<int>(positioner.constraint_adjustment) &
        static_cast<int>(WindowPositioner::ConstraintAdjustment::resize_x)) {
      int const left_overhang = result.x - output_rect.top_left.x;
      int const right_overhang =
          (result.x + child_size.width) -
          (output_rect.top_left.x + output_rect.size.width);

      if (left_overhang < 0) {
        result.x -= left_overhang;
        child_size.width += left_overhang;
      }

      if (right_overhang > 0) {
        child_size.width -= right_overhang;
      }
    }

    if (static_cast<int>(positioner.constraint_adjustment) &
        static_cast<int>(WindowPositioner::ConstraintAdjustment::resize_y)) {
      int const top_overhang = result.y - output_rect.top_left.y;
      int const bot_overhang =
          (result.y + child_size.height) -
          (output_rect.top_left.y + output_rect.size.height);

      if (top_overhang < 0) {
        result.y -= top_overhang;
        child_size.height += top_overhang;
      }

      if (bot_overhang > 0) {
        child_size.height -= bot_overhang;
      }
    }

    if (output_rect.contains({result, child_size})) {
      return WindowRectangle{result, child_size};
    }
  }

  return default_result;
}

}  // namespace flutter
