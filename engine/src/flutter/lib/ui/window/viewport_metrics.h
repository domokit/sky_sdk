// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_WINDOW_VIEWPORT_METRICS_H_
#define FLUTTER_LIB_UI_WINDOW_VIEWPORT_METRICS_H_

#include <ostream>
#include <vector>

namespace flutter {

struct ViewportMetrics {
  ViewportMetrics();
  ViewportMetrics(double p_device_pixel_ratio,
                  double p_physical_width,
                  double p_physical_height,
                  double p_physical_touch_slop,
                  size_t display_id);
  ViewportMetrics(double p_device_pixel_ratio,
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
                  size_t p_display_id);

  double device_pixel_ratio = 1.0;
  double physical_width = 0;
  double physical_height = 0;
  double physical_padding_top = 0;
  double physical_padding_right = 0;
  double physical_padding_bottom = 0;
  double physical_padding_left = 0;
  double physical_view_inset_top = 0;
  double physical_view_inset_right = 0;
  double physical_view_inset_bottom = 0;
  double physical_view_inset_left = 0;
  double physical_system_gesture_inset_top = 0;
  double physical_system_gesture_inset_right = 0;
  double physical_system_gesture_inset_bottom = 0;
  double physical_system_gesture_inset_left = 0;
  double physical_touch_slop = -1.0;
  std::vector<double> physical_display_features_bounds;
  std::vector<int> physical_display_features_type;
  std::vector<int> physical_display_features_state;
  size_t display_id = 0;
};

bool operator==(const ViewportMetrics& a, const ViewportMetrics& b);
std::ostream& operator<<(std::ostream& os, const ViewportMetrics& a);

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_WINDOW_VIEWPORT_METRICS_H_
