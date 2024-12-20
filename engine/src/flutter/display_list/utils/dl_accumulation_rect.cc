// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/utils/dl_accumulation_rect.h"

namespace flutter {

void AccumulationRect::accumulate(SkScalar x, SkScalar y) {
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

void AccumulationRect::accumulate(SkRect r) {
  if (r.isEmpty()) {
    return;
  }
  if (r.fLeft < max_x_ && r.fRight > min_x_ &&  //
      r.fTop < max_y_ && r.fBottom > min_y_) {
    record_overlapping_bounds();
  }
  if (min_x_ > r.fLeft) {
    min_x_ = r.fLeft;
  }
  if (min_y_ > r.fTop) {
    min_y_ = r.fTop;
  }
  if (max_x_ < r.fRight) {
    max_x_ = r.fRight;
  }
  if (max_y_ < r.fBottom) {
    max_y_ = r.fBottom;
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

SkRect AccumulationRect::bounds() const {
  return (max_x_ >= min_x_ && max_y_ >= min_y_)
             ? SkRect::MakeLTRB(min_x_, min_y_, max_x_, max_y_)
             : SkRect::MakeEmpty();
}

void AccumulationRect::reset() {
  min_x_ = std::numeric_limits<SkScalar>::infinity();
  min_y_ = std::numeric_limits<SkScalar>::infinity();
  max_x_ = -std::numeric_limits<SkScalar>::infinity();
  max_y_ = -std::numeric_limits<SkScalar>::infinity();
  overlap_detected_ = false;
}

}  // namespace flutter
