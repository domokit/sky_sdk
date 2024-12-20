// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/effects/color_filters/dl_blend_color_filter.h"

namespace flutter {

std::shared_ptr<const DlColorFilter> DlBlendColorFilter::Make(
    DlColor color,
    DlBlendMode mode) {
  switch (mode) {
    case DlBlendMode::kDst: {
      return nullptr;
    }
    case DlBlendMode::kSrcOver: {
      if (color.isTransparent()) {
        return nullptr;
      }
      if (color.isOpaque()) {
        mode = DlBlendMode::kSrc;
      }
      break;
    }
    case DlBlendMode::kDstOver:
    case DlBlendMode::kDstOut:
    case DlBlendMode::kSrcATop:
    case DlBlendMode::kXor:
    case DlBlendMode::kDarken: {
      if (color.isTransparent()) {
        return nullptr;
      }
      break;
    }
    case DlBlendMode::kDstIn: {
      if (color.isOpaque()) {
        return nullptr;
      }
      break;
    }
    default:
      break;
  }
  return std::make_shared<DlBlendColorFilter>(color, mode);
}

bool DlBlendColorFilter::modifies_transparent_black() const {
  switch (mode_) {
    // These modes all act like kSrc when the dest is all 0s.
    // So they modify transparent black when the src color is
    // not transparent.
    case DlBlendMode::kSrc:
    case DlBlendMode::kSrcOver:
    case DlBlendMode::kDstOver:
    case DlBlendMode::kSrcOut:
    case DlBlendMode::kDstATop:
    case DlBlendMode::kXor:
    case DlBlendMode::kPlus:
    case DlBlendMode::kScreen:
    case DlBlendMode::kOverlay:
    case DlBlendMode::kDarken:
    case DlBlendMode::kLighten:
    case DlBlendMode::kColorDodge:
    case DlBlendMode::kColorBurn:
    case DlBlendMode::kHardLight:
    case DlBlendMode::kSoftLight:
    case DlBlendMode::kDifference:
    case DlBlendMode::kExclusion:
    case DlBlendMode::kMultiply:
    case DlBlendMode::kHue:
    case DlBlendMode::kSaturation:
    case DlBlendMode::kColor:
    case DlBlendMode::kLuminosity:
      return !color_.isTransparent();

    // These modes are all like kDst when the dest is all 0s.
    // So they never modify transparent black.
    case DlBlendMode::kClear:
    case DlBlendMode::kDst:
    case DlBlendMode::kSrcIn:
    case DlBlendMode::kDstIn:
    case DlBlendMode::kDstOut:
    case DlBlendMode::kSrcATop:
    case DlBlendMode::kModulate:
      return false;
  }
}

bool DlBlendColorFilter::can_commute_with_opacity() const {
  switch (mode_) {
    case DlBlendMode::kClear:
    case DlBlendMode::kDst:
    case DlBlendMode::kSrcIn:
    case DlBlendMode::kDstIn:
    case DlBlendMode::kDstOut:
    case DlBlendMode::kSrcATop:
    case DlBlendMode::kModulate:
      return true;

    case DlBlendMode::kSrc:
    case DlBlendMode::kSrcOver:
    case DlBlendMode::kDstOver:
    case DlBlendMode::kSrcOut:
    case DlBlendMode::kDstATop:
    case DlBlendMode::kXor:
    case DlBlendMode::kPlus:
    case DlBlendMode::kScreen:
    case DlBlendMode::kOverlay:
    case DlBlendMode::kDarken:
    case DlBlendMode::kLighten:
    case DlBlendMode::kColorDodge:
    case DlBlendMode::kColorBurn:
    case DlBlendMode::kHardLight:
    case DlBlendMode::kSoftLight:
    case DlBlendMode::kDifference:
    case DlBlendMode::kExclusion:
    case DlBlendMode::kMultiply:
    case DlBlendMode::kHue:
    case DlBlendMode::kSaturation:
    case DlBlendMode::kColor:
    case DlBlendMode::kLuminosity:
      return color_.isTransparent();
  }
}

}  // namespace flutter
