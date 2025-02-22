// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_DISPLAY_LIST_UTILS_DL_RECEIVER_UTILS_H_
#define FLUTTER_DISPLAY_LIST_UTILS_DL_RECEIVER_UTILS_H_

#include "flutter/display_list/dl_op_receiver.h"
#include "flutter/fml/logging.h"

// This file contains various utility classes to ease implementing
// a Flutter DisplayList DlOpReceiver, including:
//
// IgnoreAttributeDispatchHelper:
// IgnoreClipDispatchHelper:
// IgnoreTransformDispatchHelper
//     Empty overrides of all of the associated methods of DlOpReceiver
//     for receivers that only track some of the rendering operations

namespace flutter {

// A utility class that will ignore all DlOpReceiver methods relating
// to the setting of attributes.
class IgnoreAttributeDispatchHelper : public virtual DlOpReceiver {
 public:
  void setAntiAlias(bool aa) override {}
  void setInvertColors(bool invert) override {}
  void setStrokeCap(DlStrokeCap cap) override {}
  void setStrokeJoin(DlStrokeJoin join) override {}
  void setDrawStyle(DlDrawStyle style) override {}
  void setStrokeWidth(float width) override {}
  void setStrokeMiter(float limit) override {}
  void setColor(DlColor color) override {}
  void setBlendMode(DlBlendMode mode) override {}
  void setColorSource(const DlColorSource* source) override {}
  void setImageFilter(const DlImageFilter* filter) override {}
  void setColorFilter(const DlColorFilter* filter) override {}
  void setMaskFilter(const DlMaskFilter* filter) override {}
};

// A utility class that will ignore all DlOpReceiver methods relating
// to setting a clip.
class IgnoreClipDispatchHelper : public virtual DlOpReceiver {
  void clipRect(const DlRect& rect, DlClipOp clip_op, bool is_aa) override {}
  void clipOval(const DlRect& bounds, DlClipOp clip_op, bool is_aa) override {}
  void clipRoundRect(const DlRoundRect& rrect,
                     DlClipOp clip_op,
                     bool is_aa) override {}
  void clipPath(const DlPath& path, DlClipOp clip_op, bool is_aa) override {}
  void clipRoundSuperellipse(const DlRoundSuperellipse& rse,
                             DlClipOp clip_op,
                             bool is_aa) override {}
};

// A utility class that will ignore all DlOpReceiver methods relating
// to modifying the transform.
class IgnoreTransformDispatchHelper : public virtual DlOpReceiver {
 public:
  void translate(DlScalar tx, DlScalar ty) override {}
  void scale(DlScalar sx, DlScalar sy) override {}
  void rotate(DlScalar degrees) override {}
  void skew(DlScalar sx, DlScalar sy) override {}
  // clang-format off
  // 2x3 2D affine subset of a 4x4 transform in row major order
  void transform2DAffine(DlScalar mxx, DlScalar mxy, DlScalar mxt,
                         DlScalar myx, DlScalar myy, DlScalar myt) override {}
  // full 4x4 transform in row major order
  void transformFullPerspective(
      DlScalar mxx, DlScalar mxy, DlScalar mxz, DlScalar mxt,
      DlScalar myx, DlScalar myy, DlScalar myz, DlScalar myt,
      DlScalar mzx, DlScalar mzy, DlScalar mzz, DlScalar mzt,
      DlScalar mwx, DlScalar mwy, DlScalar mwz, DlScalar mwt) override {}
  // clang-format on
  void transformReset() override {}
};

class IgnoreDrawDispatchHelper : public virtual DlOpReceiver {
 public:
  void save() override {}
  void saveLayer(const DlRect& bounds,
                 const SaveLayerOptions options,
                 const DlImageFilter* backdrop,
                 std::optional<int64_t> backdrop_id) override {}
  void restore() override {}
  void drawColor(DlColor color, DlBlendMode mode) override {}
  void drawPaint() override {}
  void drawLine(const DlPoint& p0, const DlPoint& p1) override {}
  void drawDashedLine(const DlPoint& p0,
                      const DlPoint& p1,
                      DlScalar on_length,
                      DlScalar off_length) override {}
  void drawRect(const DlRect& rect) override {}
  void drawOval(const DlRect& bounds) override {}
  void drawCircle(const DlPoint& center, DlScalar radius) override {}
  void drawRoundRect(const DlRoundRect& rrect) override {}
  void drawDiffRoundRect(const DlRoundRect& outer,
                         const DlRoundRect& inner) override {}
  void drawRoundSuperellipse(const DlRoundSuperellipse& rse) override {}
  void drawPath(const DlPath& path) override {}
  void drawArc(const DlRect& oval_bounds,
               DlScalar start_degrees,
               DlScalar sweep_degrees,
               bool use_center) override {}
  void drawPoints(DlPointMode mode,
                  uint32_t count,
                  const DlPoint points[]) override {}
  void drawVertices(const std::shared_ptr<DlVertices>& vertices,
                    DlBlendMode mode) override {}
  void drawImage(const sk_sp<DlImage> image,
                 const DlPoint& point,
                 DlImageSampling sampling,
                 bool render_with_attributes) override {}
  void drawImageRect(const sk_sp<DlImage> image,
                     const DlRect& src,
                     const DlRect& dst,
                     DlImageSampling sampling,
                     bool render_with_attributes,
                     DlSrcRectConstraint constraint) override {}
  void drawImageNine(const sk_sp<DlImage> image,
                     const DlIRect& center,
                     const DlRect& dst,
                     DlFilterMode filter,
                     bool render_with_attributes) override {}
  void drawAtlas(const sk_sp<DlImage> atlas,
                 const DlRSTransform xform[],
                 const DlRect tex[],
                 const DlColor colors[],
                 int count,
                 DlBlendMode mode,
                 DlImageSampling sampling,
                 const DlRect* cull_rect,
                 bool render_with_attributes) override {}
  void drawDisplayList(const sk_sp<DisplayList> display_list,
                       DlScalar opacity) override {}
  void drawTextBlob(const sk_sp<SkTextBlob> blob,
                    DlScalar x,
                    DlScalar y) override {}
  void drawTextFrame(const std::shared_ptr<impeller::TextFrame>& text_frame,
                     DlScalar x,
                     DlScalar y) override {}
  void drawShadow(const DlPath& path,
                  const DlColor color,
                  const DlScalar elevation,
                  bool transparent_occluder,
                  DlScalar dpr) override {}
};

}  // namespace flutter

#endif  // FLUTTER_DISPLAY_LIST_UTILS_DL_RECEIVER_UTILS_H_
