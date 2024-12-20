// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/lib/ui/window/viewport_metrics.h"

#include "flutter/fml/logging.h"

namespace flutter {

ViewportMetrics::ViewportMetrics() = default;

ViewportMetrics::ViewportMetrics(double p_device_pixel_ratio,
                                 double p_physical_width,
                                 double p_physical_height,
                                 double p_physical_touch_slop,
                                 size_t p_display_id)
    : device_pixel_ratio(p_device_pixel_ratio),
      physical_width(p_physical_width),
      physical_height(p_physical_height),
      physical_touch_slop(p_physical_touch_slop),
      display_id(p_display_id) {}

ViewportMetrics::ViewportMetrics(
    double p_device_pixel_ratio,
    double p_physical_width,
    double p_physical_height,
    double p_physical_padding_top,
    double p_physical_padding_right,
    double p_physical_padding_bottom,
    double p_physical_padding_left,
    double p_physical_view_inset_top,
    double p_physical_view_inset_right,
    double p_physical_view_inset_bottom,
    double p_physical_view_inset_left,
    double p_physical_system_gesture_inset_top,
    double p_physical_system_gesture_inset_right,
    double p_physical_system_gesture_inset_bottom,
    double p_physical_system_gesture_inset_left,
    double p_physical_touch_slop,
    const std::vector<double>& p_physical_display_features_bounds,
    const std::vector<int>& p_physical_display_features_type,
    const std::vector<int>& p_physical_display_features_state,
    size_t p_display_id)
    : device_pixel_ratio(p_device_pixel_ratio),
      physical_width(p_physical_width),
      physical_height(p_physical_height),
      physical_padding_top(p_physical_padding_top),
      physical_padding_right(p_physical_padding_right),
      physical_padding_bottom(p_physical_padding_bottom),
      physical_padding_left(p_physical_padding_left),
      physical_view_inset_top(p_physical_view_inset_top),
      physical_view_inset_right(p_physical_view_inset_right),
      physical_view_inset_bottom(p_physical_view_inset_bottom),
      physical_view_inset_left(p_physical_view_inset_left),
      physical_system_gesture_inset_top(p_physical_system_gesture_inset_top),
      physical_system_gesture_inset_right(
          p_physical_system_gesture_inset_right),
      physical_system_gesture_inset_bottom(
          p_physical_system_gesture_inset_bottom),
      physical_system_gesture_inset_left(p_physical_system_gesture_inset_left),
      physical_touch_slop(p_physical_touch_slop),
      physical_display_features_bounds(p_physical_display_features_bounds),
      physical_display_features_type(p_physical_display_features_type),
      physical_display_features_state(p_physical_display_features_state),
      display_id(p_display_id) {}

bool operator==(const ViewportMetrics& a, const ViewportMetrics& b) {
  return a.device_pixel_ratio == b.device_pixel_ratio &&
         a.physical_width == b.physical_width &&
         a.physical_height == b.physical_height &&
         a.physical_padding_top == b.physical_padding_top &&
         a.physical_padding_right == b.physical_padding_right &&
         a.physical_padding_bottom == b.physical_padding_bottom &&
         a.physical_padding_left == b.physical_padding_left &&
         a.physical_view_inset_top == b.physical_view_inset_top &&
         a.physical_view_inset_right == b.physical_view_inset_right &&
         a.physical_view_inset_bottom == b.physical_view_inset_bottom &&
         a.physical_view_inset_left == b.physical_view_inset_left &&
         a.physical_system_gesture_inset_top ==
             b.physical_system_gesture_inset_top &&
         a.physical_system_gesture_inset_right ==
             b.physical_system_gesture_inset_right &&
         a.physical_system_gesture_inset_bottom ==
             b.physical_system_gesture_inset_bottom &&
         a.physical_system_gesture_inset_left ==
             b.physical_system_gesture_inset_left &&
         a.physical_touch_slop == b.physical_touch_slop &&
         a.physical_display_features_bounds ==
             b.physical_display_features_bounds &&
         a.physical_display_features_type == b.physical_display_features_type &&
         a.physical_display_features_state ==
             b.physical_display_features_state &&
         a.display_id == b.display_id;
}

std::ostream& operator<<(std::ostream& os, const ViewportMetrics& a) {
  os << "DPR: " << a.device_pixel_ratio << " " << "Size: [" << a.physical_width
     << "W " << a.physical_height << "H] " << "Padding: ["
     << a.physical_padding_top << "T " << a.physical_padding_right << "R "
     << a.physical_padding_bottom << "B " << a.physical_padding_left << "L] "
     << "Insets: [" << a.physical_view_inset_top << "T "
     << a.physical_view_inset_right << "R " << a.physical_view_inset_bottom
     << "B " << a.physical_view_inset_left << "L] " << "Gesture Insets: ["
     << a.physical_system_gesture_inset_top << "T "
     << a.physical_system_gesture_inset_right << "R "
     << a.physical_system_gesture_inset_bottom << "B "
     << a.physical_system_gesture_inset_left << "L] "
     << "Display Features: " << a.physical_display_features_type.size() << " "
     << "Display ID: " << a.display_id;
  return os;
}

}  // namespace flutter
