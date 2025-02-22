// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#if !SLIMPELLER

#include "flutter/display_list/skia/dl_sk_canvas.h"

#include "flutter/display_list/effects/image_filters/dl_blur_image_filter.h"
#include "flutter/display_list/skia/dl_sk_conversions.h"
#include "flutter/display_list/skia/dl_sk_dispatcher.h"
#include "flutter/fml/trace_event.h"

#include "third_party/skia/include/core/SkColorFilter.h"
#include "third_party/skia/include/effects/SkDashPathEffect.h"
#include "third_party/skia/include/gpu/ganesh/GrDirectContext.h"
#include "third_party/skia/include/gpu/ganesh/GrRecordingContext.h"

namespace flutter {

class SkOptionalPaint {
 public:
  // SkOptionalPaint is only valid for ops that do not use the ColorSource
  explicit SkOptionalPaint(const DlPaint* dl_paint) {
    if (dl_paint && !dl_paint->isDefault()) {
      sk_paint_ = ToNonShaderSk(*dl_paint);
      ptr_ = &sk_paint_;
    } else {
      ptr_ = nullptr;
    }
  }

  SkPaint* operator()() { return ptr_; }

 private:
  SkPaint sk_paint_;
  SkPaint* ptr_;
};

void DlSkCanvasAdapter::set_canvas(SkCanvas* canvas) {
  delegate_ = canvas;
}

DlISize DlSkCanvasAdapter::GetBaseLayerDimensions() const {
  return ToDlISize(delegate_->getBaseLayerSize());
}

SkImageInfo DlSkCanvasAdapter::GetImageInfo() const {
  return delegate_->imageInfo();
}

void DlSkCanvasAdapter::Save() {
  delegate_->save();
}

void DlSkCanvasAdapter::SaveLayer(const std::optional<DlRect>& bounds,
                                  const DlPaint* paint,
                                  const DlImageFilter* backdrop,
                                  std::optional<int64_t> backdrop_id) {
  sk_sp<SkImageFilter> sk_backdrop = ToSk(backdrop);
  SkOptionalPaint sk_paint(paint);
  TRACE_EVENT0("flutter", "Canvas::saveLayer");
  SkCanvas::SaveLayerRec params(ToSkRect(bounds), sk_paint(), sk_backdrop.get(),
                                0);
  if (sk_backdrop && backdrop->asBlur()) {
    params.fBackdropTileMode = ToSk(backdrop->asBlur()->tile_mode());
  }
  delegate_->saveLayer(params);
}

void DlSkCanvasAdapter::Restore() {
  delegate_->restore();
}

int DlSkCanvasAdapter::GetSaveCount() const {
  return delegate_->getSaveCount();
}

void DlSkCanvasAdapter::RestoreToCount(int restore_count) {
  delegate_->restoreToCount(restore_count);
}

void DlSkCanvasAdapter::Translate(SkScalar tx, SkScalar ty) {
  delegate_->translate(tx, ty);
}

void DlSkCanvasAdapter::Scale(SkScalar sx, SkScalar sy) {
  delegate_->scale(sx, sy);
}

void DlSkCanvasAdapter::Rotate(SkScalar degrees) {
  delegate_->rotate(degrees);
}

void DlSkCanvasAdapter::Skew(SkScalar sx, SkScalar sy) {
  delegate_->skew(sx, sy);
}

// clang-format off

// 2x3 2D affine subset of a 4x4 transform in row major order
void DlSkCanvasAdapter::Transform2DAffine(
    SkScalar mxx, SkScalar mxy, SkScalar mxt,
    SkScalar myx, SkScalar myy, SkScalar myt) {
  delegate_->concat(SkMatrix::MakeAll(mxx, mxy, mxt, myx, myy, myt, 0, 0, 1));
}

// full 4x4 transform in row major order
void DlSkCanvasAdapter::TransformFullPerspective(
    SkScalar mxx, SkScalar mxy, SkScalar mxz, SkScalar mxt,
    SkScalar myx, SkScalar myy, SkScalar myz, SkScalar myt,
    SkScalar mzx, SkScalar mzy, SkScalar mzz, SkScalar mzt,
    SkScalar mwx, SkScalar mwy, SkScalar mwz, SkScalar mwt) {
  delegate_->concat(SkM44(mxx, mxy, mxz, mxt,
                          myx, myy, myz, myt,
                          mzx, mzy, mzz, mzt,
                          mwx, mwy, mwz, mwt));
}

// clang-format on

void DlSkCanvasAdapter::TransformReset() {
  delegate_->resetMatrix();
}

void DlSkCanvasAdapter::Transform(const DlMatrix& matrix) {
  delegate_->concat(ToSkM44(matrix));
}

void DlSkCanvasAdapter::SetTransform(const DlMatrix& matrix) {
  delegate_->setMatrix(ToSkM44(matrix));
}

/// Returns the 4x4 full perspective transform representing all transform
/// operations executed so far in this DisplayList within the enclosing
/// save stack.
DlMatrix DlSkCanvasAdapter::GetMatrix() const {
  return ToDlMatrix(delegate_->getLocalToDevice());
}

void DlSkCanvasAdapter::ClipRect(const DlRect& rect,
                                 DlClipOp clip_op,
                                 bool is_aa) {
  delegate_->clipRect(ToSkRect(rect), ToSk(clip_op), is_aa);
}

void DlSkCanvasAdapter::ClipOval(const DlRect& bounds,
                                 DlClipOp clip_op,
                                 bool is_aa) {
  delegate_->clipRRect(SkRRect::MakeOval(ToSkRect(bounds)), ToSk(clip_op),
                       is_aa);
}

void DlSkCanvasAdapter::ClipRoundRect(const DlRoundRect& rrect,
                                      DlClipOp clip_op,
                                      bool is_aa) {
  delegate_->clipRRect(ToSkRRect(rrect), ToSk(clip_op), is_aa);
}

void DlSkCanvasAdapter::ClipRoundSuperellipse(const DlRoundSuperellipse& rse,
                                              DlClipOp clip_op,
                                              bool is_aa) {
  // Skia doesn't support round superellipse, thus fall back to round rectangle.
  delegate_->clipRRect(ToApproximateSkRRect(rse), ToSk(clip_op), is_aa);
}

void DlSkCanvasAdapter::ClipPath(const DlPath& path,
                                 DlClipOp clip_op,
                                 bool is_aa) {
  path.WillRenderSkPath();
  delegate_->clipPath(path.GetSkPath(), ToSk(clip_op), is_aa);
}

/// Conservative estimate of the bounds of all outstanding clip operations
/// measured in the coordinate space within which this DisplayList will
/// be rendered.
DlRect DlSkCanvasAdapter::GetDestinationClipCoverage() const {
  return ToDlRect(delegate_->getDeviceClipBounds());
}

/// Conservative estimate of the bounds of all outstanding clip operations
/// transformed into the local coordinate space in which currently
/// recorded rendering operations are interpreted.
DlRect DlSkCanvasAdapter::GetLocalClipCoverage() const {
  return ToDlRect(delegate_->getLocalClipBounds());
}

/// Return true iff the supplied bounds are easily shown to be outside
/// of the current clip bounds. This method may conservatively return
/// false if it cannot make the determination.
bool DlSkCanvasAdapter::QuickReject(const DlRect& bounds) const {
  return delegate_->quickReject(ToSkRect(bounds));
}

void DlSkCanvasAdapter::DrawPaint(const DlPaint& paint) {
  delegate_->drawPaint(ToSk(paint));
}

void DlSkCanvasAdapter::DrawColor(DlColor color, DlBlendMode mode) {
  delegate_->drawColor(ToSk(color), ToSk(mode));
}

void DlSkCanvasAdapter::DrawLine(const DlPoint& p0,
                                 const DlPoint& p1,
                                 const DlPaint& paint) {
  delegate_->drawLine(ToSkPoint(p0), ToSkPoint(p1), ToStrokedSk(paint));
}

void DlSkCanvasAdapter::DrawDashedLine(const DlPoint& p0,
                                       const DlPoint& p1,
                                       DlScalar on_length,
                                       DlScalar off_length,
                                       const DlPaint& paint) {
  SkPaint dashed_paint = ToStrokedSk(paint);
  SkScalar intervals[2] = {on_length, off_length};
  dashed_paint.setPathEffect(SkDashPathEffect::Make(intervals, 2, 0.0f));
  delegate_->drawLine(ToSkPoint(p0), ToSkPoint(p1), dashed_paint);
}

void DlSkCanvasAdapter::DrawRect(const DlRect& rect, const DlPaint& paint) {
  delegate_->drawRect(ToSkRect(rect), ToSk(paint));
}

void DlSkCanvasAdapter::DrawOval(const DlRect& bounds, const DlPaint& paint) {
  delegate_->drawOval(ToSkRect(bounds), ToSk(paint));
}

void DlSkCanvasAdapter::DrawCircle(const DlPoint& center,
                                   SkScalar radius,
                                   const DlPaint& paint) {
  delegate_->drawCircle(ToSkPoint(center), radius, ToSk(paint));
}

void DlSkCanvasAdapter::DrawRoundRect(const DlRoundRect& rrect,
                                      const DlPaint& paint) {
  delegate_->drawRRect(ToSkRRect(rrect), ToSk(paint));
}

void DlSkCanvasAdapter::DrawDiffRoundRect(const DlRoundRect& outer,
                                          const DlRoundRect& inner,
                                          const DlPaint& paint) {
  delegate_->drawDRRect(ToSkRRect(outer), ToSkRRect(inner), ToSk(paint));
}

void DlSkCanvasAdapter::DrawRoundSuperellipse(const DlRoundSuperellipse& rse,
                                              const DlPaint& paint) {
  // Skia doesn't support round superellipse, thus fall back to round rectangle.
  delegate_->drawRRect(ToApproximateSkRRect(rse), ToSk(paint));
}

void DlSkCanvasAdapter::DrawPath(const DlPath& path, const DlPaint& paint) {
  path.WillRenderSkPath();
  delegate_->drawPath(path.GetSkPath(), ToSk(paint));
}

void DlSkCanvasAdapter::DrawArc(const DlRect& bounds,
                                DlScalar start,
                                DlScalar sweep,
                                bool useCenter,
                                const DlPaint& paint) {
  delegate_->drawArc(ToSkRect(bounds), start, sweep, useCenter, ToSk(paint));
}

void DlSkCanvasAdapter::DrawPoints(DlPointMode mode,
                                   uint32_t count,
                                   const DlPoint pts[],
                                   const DlPaint& paint) {
  delegate_->drawPoints(ToSk(mode), count, ToSkPoints(pts), ToStrokedSk(paint));
}

void DlSkCanvasAdapter::DrawVertices(
    const std::shared_ptr<DlVertices>& vertices,
    DlBlendMode mode,
    const DlPaint& paint) {
  delegate_->drawVertices(ToSk(vertices), ToSk(mode), ToSk(paint));
}

void DlSkCanvasAdapter::DrawImage(const sk_sp<DlImage>& image,
                                  const DlPoint& point,
                                  DlImageSampling sampling,
                                  const DlPaint* paint) {
  SkOptionalPaint sk_paint(paint);
  sk_sp<SkImage> sk_image = image->skia_image();
  delegate_->drawImage(sk_image.get(), point.x, point.y, ToSk(sampling),
                       sk_paint());
}

void DlSkCanvasAdapter::DrawImageRect(const sk_sp<DlImage>& image,
                                      const DlRect& src,
                                      const DlRect& dst,
                                      DlImageSampling sampling,
                                      const DlPaint* paint,
                                      DlSrcRectConstraint constraint) {
  SkOptionalPaint sk_paint(paint);
  sk_sp<SkImage> sk_image = image->skia_image();
  delegate_->drawImageRect(sk_image.get(), ToSkRect(src), ToSkRect(dst),
                           ToSk(sampling), sk_paint(), ToSk(constraint));
}

void DlSkCanvasAdapter::DrawImageNine(const sk_sp<DlImage>& image,
                                      const DlIRect& center,
                                      const DlRect& dst,
                                      DlFilterMode filter,
                                      const DlPaint* paint) {
  SkOptionalPaint sk_paint(paint);
  sk_sp<SkImage> sk_image = image->skia_image();
  delegate_->drawImageNine(sk_image.get(), ToSkIRect(center), ToSkRect(dst),
                           ToSk(filter), sk_paint());
}

void DlSkCanvasAdapter::DrawAtlas(const sk_sp<DlImage>& atlas,
                                  const DlRSTransform xform[],
                                  const DlRect tex[],
                                  const DlColor colors[],
                                  int count,
                                  DlBlendMode mode,
                                  DlImageSampling sampling,
                                  const DlRect* cullRect,
                                  const DlPaint* paint) {
  SkOptionalPaint sk_paint(paint);
  sk_sp<SkImage> sk_image = atlas->skia_image();
  std::vector<SkColor> sk_colors;
  sk_colors.reserve(count);
  for (int i = 0; i < count; ++i) {
    sk_colors.push_back(colors[i].argb());
  }
  delegate_->drawAtlas(sk_image.get(), ToSk(xform), ToSkRects(tex),
                       sk_colors.data(), count, ToSk(mode), ToSk(sampling),
                       ToSkRect(cullRect), sk_paint());
}

void DlSkCanvasAdapter::DrawDisplayList(const sk_sp<DisplayList> display_list,
                                        SkScalar opacity) {
  const int restore_count = delegate_->getSaveCount();

  // Figure out whether we can apply the opacity during dispatch or
  // if we need a saveLayer.
  if (opacity < SK_Scalar1 && !display_list->can_apply_group_opacity()) {
    TRACE_EVENT0("flutter", "Canvas::saveLayer");
    delegate_->saveLayerAlphaf(&display_list->bounds(), opacity);
    opacity = SK_Scalar1;
  } else {
    delegate_->save();
  }

  DlSkCanvasDispatcher dispatcher(delegate_, opacity);
  if (display_list->has_rtree()) {
    display_list->Dispatch(dispatcher, delegate_->getLocalClipBounds());
  } else {
    display_list->Dispatch(dispatcher);
  }

  delegate_->restoreToCount(restore_count);
}

void DlSkCanvasAdapter::DrawTextBlob(const sk_sp<SkTextBlob>& blob,
                                     SkScalar x,
                                     SkScalar y,
                                     const DlPaint& paint) {
  delegate_->drawTextBlob(blob, x, y, ToSk(paint));
}

void DlSkCanvasAdapter::DrawTextFrame(
    const std::shared_ptr<impeller::TextFrame>& text_frame,
    SkScalar x,
    SkScalar y,
    const DlPaint& paint) {
  FML_CHECK(false);
}

void DlSkCanvasAdapter::DrawShadow(const DlPath& path,
                                   const DlColor color,
                                   const SkScalar elevation,
                                   bool transparent_occluder,
                                   SkScalar dpr) {
  path.WillRenderSkPath();
  DlSkCanvasDispatcher::DrawShadow(delegate_, path.GetSkPath(), color,
                                   elevation, transparent_occluder, dpr);
}

void DlSkCanvasAdapter::Flush() {
#if defined(SK_GANESH)
  auto dContext = GrAsDirectContext(delegate_->recordingContext());

  if (dContext) {
    dContext->flushAndSubmit();
  }
#endif  // defined(SK_GANESH)
}

}  // namespace flutter

#endif  //  !SLIMPELLER
