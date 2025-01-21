// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_DISPLAY_LIST_UTILS_DL_ACCUMULATION_RECT_H_
#define FLUTTER_DISPLAY_LIST_UTILS_DL_ACCUMULATION_RECT_H_

#include "flutter/display_list/geometry/dl_geometry_types.h"

namespace flutter {

// Utility class to collect bounds from a bunch of rectangles and points
// while also noting if there might be any overlap between any of the data
// point/rects. Note that the overlap protection is not sophisticated,
// simply noting if the new data intersects with the already accumulated
// bounds. This can successfully detect non-overlap of a linear sequence
// of non-overlapping objects, or even a cross of non-overlapping objects
// as long as they are built out from the center in the right order. True
// detection of non-overlapping objects would require much more time and/or
// space.
class AccumulationRect {
 public:
  AccumulationRect() { reset(); }

  void accumulate(DlScalar x, DlScalar y);
  void accumulate(DlPoint p) { accumulate(p.x, p.y); }
  void accumulate(DlRect r);
  void accumulate(AccumulationRect& ar);

  bool is_empty() const { return min_x_ >= max_x_ || min_y_ >= max_y_; }
  bool is_not_empty() const { return min_x_ < max_x_ && min_y_ < max_y_; }

  DlRect GetBounds() const;

  void reset();

  bool overlap_detected() const { return overlap_detected_; }
  void record_overlapping_bounds() { overlap_detected_ = true; }

 private:
  DlScalar min_x_;
  DlScalar min_y_;
  DlScalar max_x_;
  DlScalar max_y_;
  bool overlap_detected_;
};

}  // namespace flutter

#endif  // FLUTTER_DISPLAY_LIST_UTILS_DL_ACCUMULATION_RECT_H_
