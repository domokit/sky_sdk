// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TOOLKIT_INTEROP_COLOR_SOURCE_H_
#define FLUTTER_IMPELLER_TOOLKIT_INTEROP_COLOR_SOURCE_H_

#include <vector>

#include "flutter/display_list/effects/dl_color_source.h"
#include "impeller/geometry/color.h"
#include "impeller/geometry/matrix.h"
#include "impeller/geometry/point.h"
#include "impeller/toolkit/interop/formats.h"
#include "impeller/toolkit/interop/impeller.h"
#include "impeller/toolkit/interop/object.h"
#include "impeller/toolkit/interop/texture.h"

namespace impeller::interop {

class ColorSource final
    : public Object<ColorSource,
                    IMPELLER_INTERNAL_HANDLE_NAME(ImpellerColorSource)> {
 public:
  static ScopedObject<ColorSource> MakeLinearGradient(
      const Point& start_point,
      const Point& end_point,
      const std::vector<flutter::DlColor>& colors,
      const std::vector<Scalar>& stops,
      flutter::DlTileMode tile_mode,
      const Matrix& transformation);

  static ScopedObject<ColorSource> MakeRadialGradient(
      const Point& center,
      Scalar radius,
      const std::vector<flutter::DlColor>& colors,
      const std::vector<Scalar>& stops,
      flutter::DlTileMode tile_mode,
      const Matrix& transformation);

  static ScopedObject<ColorSource> MakeConicalGradient(
      const Point& start_center,
      Scalar start_radius,
      const Point& end_center,
      Scalar end_radius,
      const std::vector<flutter::DlColor>& colors,
      const std::vector<Scalar>& stops,
      flutter::DlTileMode tile_mode,
      const Matrix& transformation);

  static ScopedObject<ColorSource> MakeSweepGradient(
      const Point& center,
      Scalar start,
      Scalar end,
      const std::vector<flutter::DlColor>& colors,
      const std::vector<Scalar>& stops,
      flutter::DlTileMode tile_mode,
      const Matrix& transformation);

  static ScopedObject<ColorSource> MakeImage(
      const Texture& image,
      flutter::DlTileMode horizontal_tile_mode,
      flutter::DlTileMode vertical_tile_mode,
      flutter::DlImageSampling sampling,
      const Matrix& transformation);

  explicit ColorSource(std::shared_ptr<flutter::DlColorSource> source);

  ~ColorSource() override;

  ColorSource(const ColorSource&) = delete;

  ColorSource& operator=(const ColorSource&) = delete;

  bool IsValid() const;

  const std::shared_ptr<flutter::DlColorSource>& GetColorSource() const;

 private:
  std::shared_ptr<flutter::DlColorSource> color_source_;
};

}  // namespace impeller::interop

#endif  // FLUTTER_IMPELLER_TOOLKIT_INTEROP_COLOR_SOURCE_H_
