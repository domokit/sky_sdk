// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_PLAYGROUND_WIDGETS_H_
#define FLUTTER_IMPELLER_PLAYGROUND_WIDGETS_H_

#include <optional>
#include <tuple>

#include "impeller/base/strings.h"
#include "impeller/geometry/color.h"
#include "impeller/geometry/point.h"
#include "third_party/imgui/imgui.h"

namespace impeller {
struct PlaygroundPoint {
  PlaygroundPoint(Point default_position, Scalar p_radius, Color p_color)
      : position(default_position),
        reset_position(default_position),
        radius(p_radius),
        color(p_color) {}
  Point position;
  Point reset_position;
  bool dragging = false;
  std::optional<Point> prev_mouse_pos;
  Scalar radius;
  Color color;
  PlaygroundPoint(const PlaygroundPoint&) = delete;
  PlaygroundPoint(PlaygroundPoint&&) = delete;
  PlaygroundPoint& operator=(const PlaygroundPoint&) = delete;
};

Point DrawPlaygroundPoint(PlaygroundPoint& point);

std::tuple<Point, Point> DrawPlaygroundLine(PlaygroundPoint& point_a,
                                            PlaygroundPoint& point_b);
}  // namespace impeller
#endif  // FLUTTER_IMPELLER_PLAYGROUND_WIDGETS_H_
