// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TOOLKIT_INTEROP_DL_BUILDER_H_
#define FLUTTER_IMPELLER_TOOLKIT_INTEROP_DL_BUILDER_H_

#include "flutter/display_list/dl_builder.h"
#include "flutter/display_list/dl_canvas.h"
#include "impeller/geometry/scalar.h"
#include "impeller/geometry/size.h"
#include "impeller/toolkit/interop/dl.h"
#include "impeller/toolkit/interop/formats.h"
#include "impeller/toolkit/interop/image_filter.h"
#include "impeller/toolkit/interop/impeller.h"
#include "impeller/toolkit/interop/object.h"
#include "impeller/toolkit/interop/paint.h"
#include "impeller/toolkit/interop/paragraph.h"
#include "impeller/toolkit/interop/path.h"
#include "impeller/toolkit/interop/texture.h"

namespace impeller::interop {

class DisplayListBuilder final
    : public Object<DisplayListBuilder,
                    IMPELLER_INTERNAL_HANDLE_NAME(ImpellerDisplayListBuilder)> {
 public:
  explicit DisplayListBuilder(const ImpellerRect* rect);

  ~DisplayListBuilder() override;

  DisplayListBuilder(const DisplayListBuilder&) = delete;

  DisplayListBuilder& operator=(const DisplayListBuilder&) = delete;

  void Save();

  void SaveLayer(const Rect& bounds,
                 const Paint* paint,
                 const ImageFilter* backdrop);

  void Restore();

  void Scale(Size scale);

  void Rotate(Degrees angle);

  void Translate(Point translation);

  Matrix GetTransform() const;

  void SetTransform(const Matrix& matrix);

  void Transform(const Matrix& matrix);

  void ResetTransform();

  uint32_t GetSaveCount() const;

  void RestoreToCount(uint32_t count);

  void ClipRect(const Rect& rect, flutter::DlCanvas::ClipOp op);

  void ClipOval(const Rect& rect, flutter::DlCanvas::ClipOp op);

  void ClipRoundedRect(const Rect& rect,
                       const RoundingRadii& radii,
                       flutter::DlCanvas::ClipOp op);

  void ClipPath(const Path& path, flutter::DlCanvas::ClipOp op);

  void DrawPaint(const Paint& paint);

  void DrawLine(const Point& from, const Point& to, const Paint& paint);

  void DrawDashedLine(const Point& from,
                      const Point& to,
                      Scalar on_length,
                      Scalar off_length,
                      const Paint& paint);

  void DrawRect(const Rect& rect, const Paint& paint);

  void DrawOval(const Rect& oval_bounds, const Paint& paint);

  void DrawRoundedRect(const Rect& rect,
                       const RoundingRadii& radii,
                       const Paint& paint);

  void DrawRoundedRectDifference(const Rect& outer_rect,
                                 const RoundingRadii& outer_radii,
                                 const Rect& inner_rect,
                                 const RoundingRadii& inner_radii,
                                 const Paint& paint);

  void DrawPath(const Path& path, const Paint& paint);

  void DrawTexture(const Texture& texture,
                   const Point& point,
                   flutter::DlImageSampling sampling,
                   const Paint* paint);

  void DrawTextureRect(const Texture& texture,
                       const Rect& src_rect,
                       const Rect& dst_rect,
                       flutter::DlImageSampling sampling,
                       const Paint* paint);

  void DrawDisplayList(const DisplayList& dl, Scalar opacity);

  void DrawParagraph(const Paragraph& paragraph, Point point);

  ScopedObject<DisplayList> Build();

 private:
  flutter::DisplayListBuilder builder_;
};

}  // namespace impeller::interop

#endif  // FLUTTER_IMPELLER_TOOLKIT_INTEROP_DL_BUILDER_H_
