// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/common/dl_op_spy.h"

namespace flutter {

bool DlOpSpy::did_draw() {
  return did_draw_;
}

void DlOpSpy::setColor(DlColor color) {
  color_ = color;
  if (color.isTransparent()) {
    will_draw_ = false;
  } else {
    will_draw_ = true;
  }
}
void DlOpSpy::setColorSource(const DlColorSource* source) {
  if (!source) {
    // Restore settings based on previously set color
    setColor(color_);
    return;
  }
  will_draw_ = true;
}
void DlOpSpy::save() {}
void DlOpSpy::saveLayer(const DlRect& bounds,
                        const SaveLayerOptions options,
                        const DlImageFilter* backdrop,
                        std::optional<int64_t> backdrop_id) {}
void DlOpSpy::restore() {}
void DlOpSpy::drawColor(DlColor color, DlBlendMode mode) {
  did_draw_ |= !color.isTransparent();
}
void DlOpSpy::drawPaint() {
  did_draw_ |= will_draw_;
}
// TODO(cyanglaz): check whether the shape (line, rect, oval, etc) needs to be
// evaluated. https://github.com/flutter/flutter/issues/123803
void DlOpSpy::drawLine(const DlPoint& p0, const DlPoint& p1) {
  did_draw_ |= will_draw_;
}
void DlOpSpy::drawDashedLine(const DlPoint& p0,
                             const DlPoint& p1,
                             DlScalar on_length,
                             DlScalar off_length) {
  did_draw_ |= will_draw_;
}
void DlOpSpy::drawRect(const DlRect& rect) {
  did_draw_ |= will_draw_;
}
void DlOpSpy::drawOval(const DlRect& bounds) {
  did_draw_ |= will_draw_;
}
void DlOpSpy::drawCircle(const DlPoint& center, DlScalar radius) {
  did_draw_ |= will_draw_;
}
void DlOpSpy::drawRoundRect(const DlRoundRect& rrect) {
  did_draw_ |= will_draw_;
}
void DlOpSpy::drawDiffRoundRect(const DlRoundRect& outer,
                                const DlRoundRect& inner) {
  did_draw_ |= will_draw_;
}
void DlOpSpy::drawRoundSuperellipse(const DlRoundSuperellipse& rse) {
  did_draw_ |= will_draw_;
}
void DlOpSpy::drawPath(const DlPath& path) {
  did_draw_ |= will_draw_;
}
void DlOpSpy::drawArc(const DlRect& oval_bounds,
                      DlScalar start_degrees,
                      DlScalar sweep_degrees,
                      bool use_center) {
  did_draw_ |= will_draw_;
}
void DlOpSpy::drawPoints(DlPointMode mode,
                         uint32_t count,
                         const DlPoint points[]) {
  did_draw_ |= will_draw_;
}
void DlOpSpy::drawVertices(const std::shared_ptr<DlVertices>& vertices,
                           DlBlendMode mode) {
  did_draw_ |= will_draw_;
}
// In theory, below drawImage methods can produce a transparent screen when a
// transparent image is provided. The operation of determine whether an image is
// transparent needs examine all the pixels in the image object, which is slow.
// Drawing a completely transparent image is not a valid use case, thus, such
// case is ignored.
void DlOpSpy::drawImage(const sk_sp<DlImage> image,
                        const DlPoint& point,
                        DlImageSampling sampling,
                        bool render_with_attributes) {
  did_draw_ = true;
}
void DlOpSpy::drawImageRect(const sk_sp<DlImage> image,
                            const DlRect& src,
                            const DlRect& dst,
                            DlImageSampling sampling,
                            bool render_with_attributes,
                            DlSrcRectConstraint constraint) {
  did_draw_ = true;
}
void DlOpSpy::drawImageNine(const sk_sp<DlImage> image,
                            const DlIRect& center,
                            const DlRect& dst,
                            DlFilterMode filter,
                            bool render_with_attributes) {
  did_draw_ = true;
}
void DlOpSpy::drawAtlas(const sk_sp<DlImage> atlas,
                        const DlRSTransform xform[],
                        const DlRect tex[],
                        const DlColor colors[],
                        int count,
                        DlBlendMode mode,
                        DlImageSampling sampling,
                        const DlRect* cull_rect,
                        bool render_with_attributes) {
  did_draw_ = true;
}
void DlOpSpy::drawDisplayList(const sk_sp<DisplayList> display_list,
                              DlScalar opacity) {
  if (did_draw_ || opacity == 0) {
    return;
  }
  DlOpSpy receiver;
  display_list->Dispatch(receiver);
  did_draw_ |= receiver.did_draw();
}
void DlOpSpy::drawTextBlob(const sk_sp<SkTextBlob> blob,
                           DlScalar x,
                           DlScalar y) {
  did_draw_ |= will_draw_;
}

void DlOpSpy::drawTextFrame(
    const std::shared_ptr<impeller::TextFrame>& text_frame,
    DlScalar x,
    DlScalar y) {
  did_draw_ |= will_draw_;
}

void DlOpSpy::drawShadow(const DlPath& path,
                         const DlColor color,
                         const DlScalar elevation,
                         bool transparent_occluder,
                         DlScalar dpr) {
  did_draw_ |= !color.isTransparent();
}

}  // namespace flutter
