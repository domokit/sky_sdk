// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_DISPLAY_LIST_SKIA_DL_SK_DISPATCHER_H_
#define FLUTTER_DISPLAY_LIST_SKIA_DL_SK_DISPATCHER_H_

#include "flutter/display_list/display_list.h"
#include "flutter/display_list/dl_op_receiver.h"
#include "flutter/display_list/skia/dl_sk_paint_dispatcher.h"
#include "flutter/display_list/skia/dl_sk_types.h"
#include "flutter/fml/macros.h"

namespace flutter {

//------------------------------------------------------------------------------
/// @brief      Backend implementation of |DlOpReceiver| for |SkCanvas|.
///
/// @see       DlOpReceiver
class DlSkCanvasDispatcher : public virtual DlOpReceiver,
                             public DlSkPaintDispatchHelper {
 public:
  explicit DlSkCanvasDispatcher(SkCanvas* canvas, DlScalar opacity = SK_Scalar1)
      : DlSkPaintDispatchHelper(opacity),
        canvas_(canvas),
        original_transform_(canvas->getLocalToDevice()) {}

  const SkPaint* safe_paint(bool use_attributes);

  void save() override;
  void restore() override;
  void saveLayer(const DlRect& bounds,
                 const SaveLayerOptions options,
                 const DlImageFilter* backdrop,
                 std::optional<int64_t> backdrop_id) override;

  void translate(DlScalar tx, DlScalar ty) override;
  void scale(DlScalar sx, DlScalar sy) override;
  void rotate(DlScalar degrees) override;
  void skew(DlScalar sx, DlScalar sy) override;
  // clang-format off
  // 2x3 2D affine subset of a 4x4 transform in row major order
  void transform2DAffine(DlScalar mxx, DlScalar mxy, DlScalar mxt,
                         DlScalar myx, DlScalar myy, DlScalar myt) override;
  // full 4x4 transform in row major order
  void transformFullPerspective(
      DlScalar mxx, DlScalar mxy, DlScalar mxz, DlScalar mxt,
      DlScalar myx, DlScalar myy, DlScalar myz, DlScalar myt,
      DlScalar mzx, DlScalar mzy, DlScalar mzz, DlScalar mzt,
      DlScalar mwx, DlScalar mwy, DlScalar mwz, DlScalar mwt) override;
  // clang-format on
  void transformReset() override;

  void clipRect(const DlRect& rect, ClipOp clip_op, bool is_aa) override;
  void clipOval(const DlRect& bounds, ClipOp clip_op, bool is_aa) override;
  void clipRoundRect(const DlRoundRect& rrect,
                     ClipOp clip_op,
                     bool is_aa) override;
  void clipPath(const DlPath& path, ClipOp clip_op, bool is_aa) override;

  void drawPaint() override;
  void drawColor(DlColor color, DlBlendMode mode) override;
  void drawLine(const DlPoint& p0, const DlPoint& p1) override;
  void drawDashedLine(const DlPoint& p0,
                      const DlPoint& p1,
                      DlScalar on_length,
                      DlScalar off_length) override;
  void drawRect(const DlRect& rect) override;
  void drawOval(const DlRect& bounds) override;
  void drawCircle(const DlPoint& center, DlScalar radius) override;
  void drawRoundRect(const DlRoundRect& rrect) override;
  void drawDiffRoundRect(const DlRoundRect& outer,
                         const DlRoundRect& inner) override;
  void drawPath(const DlPath& path) override;
  void drawArc(const DlRect& bounds,
               DlScalar start,
               DlScalar sweep,
               bool useCenter) override;
  void drawPoints(PointMode mode, uint32_t count, const DlPoint pts[]) override;
  void drawVertices(const std::shared_ptr<DlVertices>& vertices,
                    DlBlendMode mode) override;
  void drawImage(const sk_sp<DlImage> image,
                 const DlPoint& point,
                 DlImageSampling sampling,
                 bool render_with_attributes) override;
  void drawImageRect(const sk_sp<DlImage> image,
                     const DlRect& src,
                     const DlRect& dst,
                     DlImageSampling sampling,
                     bool render_with_attributes,
                     SrcRectConstraint constraint) override;
  void drawImageNine(const sk_sp<DlImage> image,
                     const DlIRect& center,
                     const DlRect& dst,
                     DlFilterMode filter,
                     bool render_with_attributes) override;
  void drawAtlas(const sk_sp<DlImage> atlas,
                 const SkRSXform xform[],
                 const DlRect tex[],
                 const DlColor colors[],
                 int count,
                 DlBlendMode mode,
                 DlImageSampling sampling,
                 const DlRect* cullRect,
                 bool render_with_attributes) override;
  void drawDisplayList(const sk_sp<DisplayList> display_list,
                       DlScalar opacity) override;
  void drawTextBlob(const sk_sp<SkTextBlob> blob,
                    DlScalar x,
                    DlScalar y) override;
  void drawTextFrame(const std::shared_ptr<impeller::TextFrame>& text_frame,
                     DlScalar x,
                     DlScalar y) override;
  void drawShadow(const DlPath& path,
                  const DlColor color,
                  const DlScalar elevation,
                  bool transparent_occluder,
                  DlScalar dpr) override;

  static void DrawShadow(SkCanvas* canvas,
                         const SkPath& path,
                         DlColor color,
                         float elevation,
                         bool transparentOccluder,
                         DlScalar dpr);

 private:
  SkCanvas* canvas_;
  const SkM44 original_transform_;
  SkPaint temp_paint_;
};

}  // namespace flutter

#endif  // FLUTTER_DISPLAY_LIST_SKIA_DL_SK_DISPATCHER_H_
