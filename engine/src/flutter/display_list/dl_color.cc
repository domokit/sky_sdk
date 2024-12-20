// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/dl_color.h"

namespace flutter {

namespace {
const std::array<DlScalar, 12> kP3ToSrgb = {
    1.306671048092539,  -0.298061942172353,
    0.213228303487995,  -0.213580156254466,  //
    -0.117390025596251, 1.127722006101976,
    0.109727644608938,  -0.109450321455370,  //
    0.214813187718391,  0.054268702864647,
    1.406898424029350,  -0.364892765879631};

DlColor transform(const DlColor& color,
                  const std::array<DlScalar, 12>& matrix,
                  DlColorSpace color_space) {
  return DlColor(color.getAlphaF(),
                 matrix[0] * color.getRedF() +        //
                     matrix[1] * color.getGreenF() +  //
                     matrix[2] * color.getBlueF() +   //
                     matrix[3],                       //
                 matrix[4] * color.getRedF() +        //
                     matrix[5] * color.getGreenF() +  //
                     matrix[6] * color.getBlueF() +   //
                     matrix[7],                       //
                 matrix[8] * color.getRedF() +        //
                     matrix[9] * color.getGreenF() +  //
                     matrix[10] * color.getBlueF() +  //
                     matrix[11],                      //
                 color_space);
}
}  // namespace

DlColor DlColor::withColorSpace(DlColorSpace color_space) const {
  switch (color_space_) {
    case DlColorSpace::kSRGB:
      switch (color_space) {
        case DlColorSpace::kSRGB:
          return *this;
        case DlColorSpace::kExtendedSRGB:
          return DlColor(alpha_, red_, green_, blue_,
                         DlColorSpace::kExtendedSRGB);
        case DlColorSpace::kDisplayP3:
          FML_CHECK(false) << "not implemented";
          return *this;
      }
    case DlColorSpace::kExtendedSRGB:
      switch (color_space) {
        case DlColorSpace::kSRGB:
          FML_CHECK(false) << "not implemented";
          return *this;
        case DlColorSpace::kExtendedSRGB:
          return *this;
        case DlColorSpace::kDisplayP3:
          FML_CHECK(false) << "not implemented";
          return *this;
      }
    case DlColorSpace::kDisplayP3:
      switch (color_space) {
        case DlColorSpace::kSRGB:
          FML_CHECK(false) << "not implemented";
          return *this;
        case DlColorSpace::kExtendedSRGB:
          return transform(*this, kP3ToSrgb, DlColorSpace::kExtendedSRGB);
        case DlColorSpace::kDisplayP3:
          return *this;
      }
  }
}

}  // namespace flutter
