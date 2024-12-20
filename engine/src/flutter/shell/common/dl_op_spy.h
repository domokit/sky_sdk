// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_COMMON_DL_OP_SPY_H_
#define FLUTTER_SHELL_COMMON_DL_OP_SPY_H_

#include "flutter/display_list/dl_op_receiver.h"
#include "flutter/display_list/utils/dl_receiver_utils.h"

namespace flutter {

//------------------------------------------------------------------------------
/// Receives to drawing commands of a DisplayListBuilder.
///
/// This is used to determine whether any non-transparent pixels will be drawn
/// on the canvas.
/// All the drawImage operations are considered drawing non-transparent pixels.
///
/// To use this class, dispatch the operations from DisplayList to a concrete
/// DlOpSpy object, and check the result of `did_draw` method.
///
/// ```
///    DlOpSpy dl_op_spy;
///    display_list.Dispatch(dl_op_spy);
///    bool did_draw = dl_op_spy.did_draw()
/// ```
///
class DlOpSpy final : public virtual DlOpReceiver,
                      private IgnoreAttributeDispatchHelper,
                      private IgnoreClipDispatchHelper,
                      private IgnoreTransformDispatchHelper {
 public:
  //----------------------------------------------------------------------------
  /// @brief      Returns true if any non transparent content has been drawn.
  bool did_draw();

 private:
  void setColor(DlColor color) override;
  void setColorSource(const DlColorSource* source) override;
  void save() override;
  void saveLayer(const DlRect& bounds,
                 const SaveLayerOptions options,
                 const DlImageFilter* backdrop,
                 std::optional<int64_t> backdrop_id) override;
  void restore() override;
  void drawColor(DlColor color, DlBlendMode mode) override;
  void drawPaint() override;
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
  void drawArc(const DlRect& oval_bounds,
               DlScalar start_degrees,
               DlScalar sweep_degrees,
               bool use_center) override;
  void drawPoints(PointMode mode,
                  uint32_t count,
                  const DlPoint points[]) override;
  void drawVertices(const std::shared_ptr<DlVertices>& vertices,
                    DlBlendMode mode) override;
  void drawImage(const sk_sp<DlImage> image,
                 const DlPoint& point,
                 DlImageSampling sampling,
                 bool render_with_attributes) override;
  void drawImageRect(
      const sk_sp<DlImage> image,
      const DlRect& src,
      const DlRect& dst,
      DlImageSampling sampling,
      bool render_with_attributes,
      SrcRectConstraint constraint = SrcRectConstraint::kFast) override;
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
                 const DlRect* cull_rect,
                 bool render_with_attributes) override;
  void drawDisplayList(const sk_sp<DisplayList> display_list,
                       DlScalar opacity = SK_Scalar1) override;
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

  // Most recently set color, used when color_source goes to null
  DlColor color_;

  // Indicates if the attributes are set to values that will modify the
  // destination. For now, the test only checks if there is a non-transparent
  // color set.
  bool will_draw_ = true;

  bool did_draw_ = false;
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_COMMON_DL_OP_SPY_H_
