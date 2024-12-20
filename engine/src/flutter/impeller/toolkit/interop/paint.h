// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TOOLKIT_INTEROP_PAINT_H_
#define FLUTTER_IMPELLER_TOOLKIT_INTEROP_PAINT_H_

#include "flutter/display_list/dl_color.h"
#include "flutter/display_list/dl_paint.h"
#include "impeller/geometry/color.h"
#include "impeller/toolkit/interop/color_filter.h"
#include "impeller/toolkit/interop/color_source.h"
#include "impeller/toolkit/interop/formats.h"
#include "impeller/toolkit/interop/image_filter.h"
#include "impeller/toolkit/interop/impeller.h"
#include "impeller/toolkit/interop/mask_filter.h"
#include "impeller/toolkit/interop/object.h"

namespace impeller::interop {

class Paint final
    : public Object<Paint, IMPELLER_INTERNAL_HANDLE_NAME(ImpellerPaint)> {
 public:
  Paint();

  ~Paint() override;

  Paint(const Paint&) = delete;

  Paint& operator=(const Paint&) = delete;

  const flutter::DlPaint& GetPaint() const;

  void SetColor(flutter::DlColor color);

  void SetBlendMode(BlendMode mode);

  void SetDrawStyle(flutter::DlDrawStyle style);

  void SetStrokeCap(flutter::DlStrokeCap stroke_cap);

  void SetStrokeJoin(flutter::DlStrokeJoin stroke_join);

  void SetStrokeWidth(Scalar width);

  void SetStrokeMiter(Scalar miter);

  void SetColorFilter(const ColorFilter& filter);

  void SetColorSource(const ColorSource& source);

  void SetImageFilter(const ImageFilter& filter);

  void SetMaskFilter(const MaskFilter& filter);

 private:
  flutter::DlPaint paint_;
};

}  // namespace impeller::interop

#endif  // FLUTTER_IMPELLER_TOOLKIT_INTEROP_PAINT_H_
