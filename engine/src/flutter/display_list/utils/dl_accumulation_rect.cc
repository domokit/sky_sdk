// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/utils/dl_accumulation_rect.h"

namespace flutter {

void AccumulationRect::accumulate(DlScalar x, DlScalar y) {
  if (!std::isfinite(x) || !std::isfinite(y)) {
    return;
  }
  if (x >= min_x_ && x < max_x_ && y >= min_y_ && y < max_y_) {
    record_overlapping_bounds();
    return;
  }
  if (min_x_ > x) {
    min_x_ = x;
  }
  if (min_y_ > y) {
    min_y_ = y;
  }
  if (max_x_ < x) {
    max_x_ = x;
  }
  if (max_y_ < y) {
    max_y_ = y;
  }
}

void AccumulationRect::accumulate(DlRect r) {
  if (r.IsEmpty()) {
    return;
  }
  if (r.GetLeft() < max_x_ && r.GetRight() > min_x_ &&  //
      r.GetTop() < max_y_ && r.GetBottom() > min_y_) {
    record_overlapping_bounds();
  }
  if (min_x_ > r.GetLeft()) {
    min_x_ = r.GetLeft();
  }
  if (min_y_ > r.GetTop()) {
    min_y_ = r.GetTop();
  }
  if (max_x_ < r.GetRight()) {
    max_x_ = r.GetRight();
  }
  if (max_y_ < r.GetBottom()) {
    max_y_ = r.GetBottom();
  }
}

void AccumulationRect::accumulate(AccumulationRect& ar) {
  if (ar.is_empty()) {
    return;
  }
  if (ar.min_x_ < max_x_ && ar.max_x_ > min_x_ &&  //
      ar.min_y_ < max_y_ && ar.max_y_ > min_y_) {
    record_overlapping_bounds();
  }
  if (min_x_ > ar.min_x_) {
    min_x_ = ar.min_x_;
  }
  if (min_y_ > ar.min_y_) {
    min_y_ = ar.min_y_;
  }
  if (max_x_ < ar.max_x_) {
    max_x_ = ar.max_x_;
  }
  if (max_y_ < ar.max_y_) {
    max_y_ = ar.max_y_;
  }
}

DlRect AccumulationRect::GetBounds() const {
  return (max_x_ >= min_x_ && max_y_ >= min_y_)
             ? DlRect::MakeLTRB(min_x_, min_y_, max_x_, max_y_)
             : DlRect();
}

void AccumulationRect::reset() {
  min_x_ = std::numeric_limits<DlScalar>::infinity();
  min_y_ = std::numeric_limits<DlScalar>::infinity();
  max_x_ = -std::numeric_limits<DlScalar>::infinity();
  max_y_ = -std::numeric_limits<DlScalar>::infinity();
  overlap_detected_ = false;
}

}  // namespace flutter
