// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/dl_paint.h"

#include "flutter/display_list/utils/dl_comparable.h"

namespace flutter {

DlPaint::DlPaint(DlColor color)
    : blend_mode_(static_cast<unsigned>(DlBlendMode::kDefaultMode)),
      draw_style_(static_cast<unsigned>(DlDrawStyle::kDefaultStyle)),
      stroke_cap_(static_cast<unsigned>(DlStrokeCap::kDefaultCap)),
      stroke_join_(static_cast<unsigned>(DlStrokeJoin::kDefaultJoin)),
      is_anti_alias_(false),
      is_invert_colors_(false),
      color_(color),
      stroke_width_(kDefaultWidth),
      stroke_miter_(kDefaultMiter) {}

bool DlPaint::operator==(DlPaint const& other) const {
  return blend_mode_ == other.blend_mode_ &&              //
         draw_style_ == other.draw_style_ &&              //
         stroke_cap_ == other.stroke_cap_ &&              //
         stroke_join_ == other.stroke_join_ &&            //
         is_anti_alias_ == other.is_anti_alias_ &&        //
         is_invert_colors_ == other.is_invert_colors_ &&  //
         color_ == other.color_ &&                        //
         stroke_width_ == other.stroke_width_ &&          //
         stroke_miter_ == other.stroke_miter_ &&          //
         Equals(color_source_, other.color_source_) &&    //
         Equals(color_filter_, other.color_filter_) &&    //
         Equals(image_filter_, other.image_filter_) &&    //
         Equals(mask_filter_, other.mask_filter_);
}

const DlPaint DlPaint::kDefault;

}  // namespace flutter
