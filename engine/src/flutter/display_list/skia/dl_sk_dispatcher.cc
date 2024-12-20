// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <cstdint>

#include "flutter/display_list/skia/dl_sk_dispatcher.h"

#include "flutter/display_list/dl_blend_mode.h"
#include "flutter/display_list/effects/image_filters/dl_blur_image_filter.h"
#include "flutter/display_list/skia/dl_sk_conversions.h"
#include "flutter/display_list/skia/dl_sk_types.h"
#include "flutter/fml/trace_event.h"

#include "third_party/skia/include/effects/SkDashPathEffect.h"
#include "third_party/skia/include/utils/SkShadowUtils.h"

namespace flutter {

const SkPaint* DlSkCanvasDispatcher::safe_paint(bool use_attributes) {
  if (use_attributes) {
    // The accumulated SkPaint object will already have incorporated
    // any attribute overrides.
    // Any rendering operation that uses an optional paint will ignore
    // the shader in the paint so we inform that |paint()| method so
    // that it can set the dither flag appropriately.
    return &paint(false);
  } else if (has_opacity()) {
    temp_paint_.setAlphaf(opacity());
    return &temp_paint_;
  } else {
    return nullptr;
  }
}

void DlSkCanvasDispatcher::save() {
  canvas_->save();
  // save has no impact on attributes, but it needs to register a record
  // on the restore stack so that the eventual call to restore() will
  // know what to do at that time. We could annotate the restore record
  // with a flag that the record came from a save call, but it is simpler
  // to just pass in the current opacity value as the value to be used by
  // the children and let the utility calls notice that it didn't change.
  save_opacity(opacity());
}
void DlSkCanvasDispatcher::restore() {
  canvas_->restore();
  restore_opacity();
}
void DlSkCanvasDispatcher::saveLayer(const DlRect& bounds,
                                     const SaveLayerOptions options,
                                     const DlImageFilter* backdrop,
                                     std::optional<int64_t> backdrop_id) {
  if (!options.content_is_clipped() && options.can_distribute_opacity() &&
      backdrop == nullptr) {
    // We know that:
    // - no bounds is needed for clipping here
    // - no backdrop filter is used to initialize the layer
    // - the current attributes only have an alpha
    // - the children are compatible with individually rendering with
    //   an inherited opacity
    // Therefore we can just use a save instead of a saveLayer and pass the
    // intended opacity to the children.
    canvas_->save();
    // If the saveLayer does not use attributes, the children should continue
    // to render with the inherited opacity unmodified. If attributes are to
    // be applied, the children should render with the combination of the
    // inherited opacity combined with the alpha from the current color.
    save_opacity(options.renders_with_attributes() ? combined_opacity()
                                                   : opacity());
  } else {
    TRACE_EVENT0("flutter", "Canvas::saveLayer");
    const SkPaint* paint = safe_paint(options.renders_with_attributes());
    const sk_sp<SkImageFilter> sk_backdrop = ToSk(backdrop);
    const SkRect* sl_bounds =
        options.bounds_from_caller() ? &ToSkRect(bounds) : nullptr;
    SkCanvas::SaveLayerRec params(sl_bounds, paint, sk_backdrop.get(), 0);
    if (sk_backdrop && backdrop->asBlur()) {
      params.fBackdropTileMode = ToSk(backdrop->asBlur()->tile_mode());
    }
    canvas_->saveLayer(params);
    // saveLayer will apply the current opacity on behalf of the children
    // so they will inherit an opaque opacity.
    save_opacity(SK_Scalar1);
  }
}

void DlSkCanvasDispatcher::translate(DlScalar tx, DlScalar ty) {
  canvas_->translate(tx, ty);
}
void DlSkCanvasDispatcher::scale(DlScalar sx, DlScalar sy) {
  canvas_->scale(sx, sy);
}
void DlSkCanvasDispatcher::rotate(DlScalar degrees) {
  canvas_->rotate(degrees);
}
void DlSkCanvasDispatcher::skew(DlScalar sx, DlScalar sy) {
  canvas_->skew(sx, sy);
}
// clang-format off
// 2x3 2D affine subset of a 4x4 transform in row major order
void DlSkCanvasDispatcher::transform2DAffine(
    DlScalar mxx, DlScalar mxy, DlScalar mxt,
    DlScalar myx, DlScalar myy, DlScalar myt) {
  // Internally concat(SkMatrix) gets redirected to concat(SkM44)
  // so we just jump directly to the SkM44 version
  canvas_->concat(SkM44(mxx, mxy, 0, mxt,
                        myx, myy, 0, myt,
                         0,   0,  1,  0,
                         0,   0,  0,  1));
}
// full 4x4 transform in row major order
void DlSkCanvasDispatcher::transformFullPerspective(
    DlScalar mxx, DlScalar mxy, DlScalar mxz, DlScalar mxt,
    DlScalar myx, DlScalar myy, DlScalar myz, DlScalar myt,
    DlScalar mzx, DlScalar mzy, DlScalar mzz, DlScalar mzt,
    DlScalar mwx, DlScalar mwy, DlScalar mwz, DlScalar mwt) {
  canvas_->concat(SkM44(mxx, mxy, mxz, mxt,
                        myx, myy, myz, myt,
                        mzx, mzy, mzz, mzt,
                        mwx, mwy, mwz, mwt));
}
// clang-format on
void DlSkCanvasDispatcher::transformReset() {
  canvas_->setMatrix(original_transform_);
}

void DlSkCanvasDispatcher::clipRect(const DlRect& rect,
                                    ClipOp clip_op,
                                    bool is_aa) {
  canvas_->clipRect(ToSkRect(rect), ToSk(clip_op), is_aa);
}
void DlSkCanvasDispatcher::clipOval(const DlRect& bounds,
                                    ClipOp clip_op,
                                    bool is_aa) {
  canvas_->clipRRect(SkRRect::MakeOval(ToSkRect(bounds)), ToSk(clip_op), is_aa);
}
void DlSkCanvasDispatcher::clipRoundRect(const DlRoundRect& rrect,
                                         ClipOp clip_op,
                                         bool is_aa) {
  canvas_->clipRRect(ToSkRRect(rrect), ToSk(clip_op), is_aa);
}
void DlSkCanvasDispatcher::clipPath(const DlPath& path,
                                    ClipOp clip_op,
                                    bool is_aa) {
  path.WillRenderSkPath();
  canvas_->clipPath(path.GetSkPath(), ToSk(clip_op), is_aa);
}

void DlSkCanvasDispatcher::drawPaint() {
  const SkPaint& sk_paint = paint();
  SkImageFilter* filter = sk_paint.getImageFilter();
  if (filter && !filter->asColorFilter(nullptr)) {
    // drawPaint does an implicit saveLayer if an SkImageFilter is
    // present that cannot be replaced by an SkColorFilter.
    TRACE_EVENT0("flutter", "Canvas::saveLayer");
  }
  canvas_->drawPaint(sk_paint);
}
void DlSkCanvasDispatcher::drawColor(DlColor color, DlBlendMode mode) {
  // SkCanvas::drawColor(SkColor) does the following conversion anyway
  // We do it here manually to increase precision on applying opacity
  SkColor4f color4f = SkColor4f::FromColor(ToSk(color));
  color4f.fA *= opacity();
  canvas_->drawColor(color4f, ToSk(mode));
}
void DlSkCanvasDispatcher::drawLine(const DlPoint& p0, const DlPoint& p1) {
  canvas_->drawLine(ToSkPoint(p0), ToSkPoint(p1), paint());
}
void DlSkCanvasDispatcher::drawDashedLine(const DlPoint& p0,
                                          const DlPoint& p1,
                                          DlScalar on_length,
                                          DlScalar off_length) {
  SkPaint dash_paint = paint();
  SkScalar intervals[] = {on_length, off_length};
  dash_paint.setPathEffect(SkDashPathEffect::Make(intervals, 2, 0.0f));
  canvas_->drawLine(ToSkPoint(p0), ToSkPoint(p1), dash_paint);
}
void DlSkCanvasDispatcher::drawRect(const DlRect& rect) {
  canvas_->drawRect(ToSkRect(rect), paint());
}
void DlSkCanvasDispatcher::drawOval(const DlRect& bounds) {
  canvas_->drawOval(ToSkRect(bounds), paint());
}
void DlSkCanvasDispatcher::drawCircle(const DlPoint& center, DlScalar radius) {
  canvas_->drawCircle(ToSkPoint(center), radius, paint());
}
void DlSkCanvasDispatcher::drawRoundRect(const DlRoundRect& rrect) {
  canvas_->drawRRect(ToSkRRect(rrect), paint());
}
void DlSkCanvasDispatcher::drawDiffRoundRect(const DlRoundRect& outer,
                                             const DlRoundRect& inner) {
  canvas_->drawDRRect(ToSkRRect(outer), ToSkRRect(inner), paint());
}
void DlSkCanvasDispatcher::drawPath(const DlPath& path) {
  path.WillRenderSkPath();
  canvas_->drawPath(path.GetSkPath(), paint());
}
void DlSkCanvasDispatcher::drawArc(const DlRect& bounds,
                                   DlScalar start,
                                   DlScalar sweep,
                                   bool useCenter) {
  canvas_->drawArc(ToSkRect(bounds), start, sweep, useCenter, paint());
}
void DlSkCanvasDispatcher::drawPoints(PointMode mode,
                                      uint32_t count,
                                      const DlPoint pts[]) {
  canvas_->drawPoints(ToSk(mode), count, ToSkPoints(pts), paint());
}
void DlSkCanvasDispatcher::drawVertices(
    const std::shared_ptr<DlVertices>& vertices,
    DlBlendMode mode) {
  canvas_->drawVertices(ToSk(vertices), ToSk(mode), paint());
}
void DlSkCanvasDispatcher::drawImage(const sk_sp<DlImage> image,
                                     const DlPoint& point,
                                     DlImageSampling sampling,
                                     bool render_with_attributes) {
  canvas_->drawImage(image ? image->skia_image() : nullptr, point.x, point.y,
                     ToSk(sampling), safe_paint(render_with_attributes));
}
void DlSkCanvasDispatcher::drawImageRect(const sk_sp<DlImage> image,
                                         const DlRect& src,
                                         const DlRect& dst,
                                         DlImageSampling sampling,
                                         bool render_with_attributes,
                                         SrcRectConstraint constraint) {
  canvas_->drawImageRect(image ? image->skia_image() : nullptr, ToSkRect(src),
                         ToSkRect(dst), ToSk(sampling),
                         safe_paint(render_with_attributes), ToSk(constraint));
}
void DlSkCanvasDispatcher::drawImageNine(const sk_sp<DlImage> image,
                                         const DlIRect& center,
                                         const DlRect& dst,
                                         DlFilterMode filter,
                                         bool render_with_attributes) {
  if (!image) {
    return;
  }
  auto skia_image = image->skia_image();
  if (!skia_image) {
    return;
  }
  canvas_->drawImageNine(skia_image.get(), ToSkIRect(center), ToSkRect(dst),
                         ToSk(filter), safe_paint(render_with_attributes));
}
void DlSkCanvasDispatcher::drawAtlas(const sk_sp<DlImage> atlas,
                                     const SkRSXform xform[],
                                     const DlRect tex[],
                                     const DlColor colors[],
                                     int count,
                                     DlBlendMode mode,
                                     DlImageSampling sampling,
                                     const DlRect* cullRect,
                                     bool render_with_attributes) {
  if (!atlas) {
    return;
  }
  auto skia_atlas = atlas->skia_image();
  if (!skia_atlas) {
    return;
  }
  std::vector<SkColor> sk_colors;
  if (colors != nullptr) {
    sk_colors.reserve(count);
    for (int i = 0; i < count; ++i) {
      sk_colors.push_back(colors[i].argb());
    }
  }
  canvas_->drawAtlas(skia_atlas.get(), xform, ToSkRects(tex),
                     sk_colors.empty() ? nullptr : sk_colors.data(), count,
                     ToSk(mode), ToSk(sampling), ToSkRect(cullRect),
                     safe_paint(render_with_attributes));
}
void DlSkCanvasDispatcher::drawDisplayList(
    const sk_sp<DisplayList> display_list,
    DlScalar opacity) {
  const int restore_count = canvas_->getSaveCount();

  // Compute combined opacity and figure out whether we can apply it
  // during dispatch or if we need a saveLayer.
  SkScalar combined_opacity = opacity * this->opacity();
  if (combined_opacity < SK_Scalar1 &&
      !display_list->can_apply_group_opacity()) {
    TRACE_EVENT0("flutter", "Canvas::saveLayer");
    canvas_->saveLayerAlphaf(&display_list->bounds(), combined_opacity);
    combined_opacity = SK_Scalar1;
  } else {
    canvas_->save();
  }

  // Create a new CanvasDispatcher to isolate the actions of the
  // display_list from the current environment.
  DlSkCanvasDispatcher dispatcher(canvas_, combined_opacity);
  if (display_list->rtree()) {
    display_list->Dispatch(dispatcher, canvas_->getLocalClipBounds());
  } else {
    display_list->Dispatch(dispatcher);
  }

  // Restore canvas state to what it was before dispatching.
  canvas_->restoreToCount(restore_count);
}
void DlSkCanvasDispatcher::drawTextBlob(const sk_sp<SkTextBlob> blob,
                                        DlScalar x,
                                        DlScalar y) {
  canvas_->drawTextBlob(blob, x, y, paint());
}

void DlSkCanvasDispatcher::drawTextFrame(
    const std::shared_ptr<impeller::TextFrame>& text_frame,
    DlScalar x,
    DlScalar y) {
  FML_CHECK(false);
}

void DlSkCanvasDispatcher::DrawShadow(SkCanvas* canvas,
                                      const SkPath& path,
                                      DlColor color,
                                      float elevation,
                                      bool transparentOccluder,
                                      DlScalar dpr) {
  const SkScalar kAmbientAlpha = 0.039f;
  const SkScalar kSpotAlpha = 0.25f;

  uint32_t flags = transparentOccluder
                       ? SkShadowFlags::kTransparentOccluder_ShadowFlag
                       : SkShadowFlags::kNone_ShadowFlag;
  flags |= SkShadowFlags::kDirectionalLight_ShadowFlag;
  SkColor in_ambient =
      SkColorSetA(ToSk(color), kAmbientAlpha * color.getAlpha());
  SkColor in_spot = SkColorSetA(ToSk(color), kSpotAlpha * color.getAlpha());
  SkColor ambient_color, spot_color;
  SkShadowUtils::ComputeTonalColors(in_ambient, in_spot, &ambient_color,
                                    &spot_color);
  SkShadowUtils::DrawShadow(
      canvas, path, SkPoint3::Make(0, 0, dpr * elevation),
      SkPoint3::Make(0, -1, 1),
      DlCanvas::kShadowLightRadius / DlCanvas::kShadowLightHeight,
      ambient_color, spot_color, flags);
}

void DlSkCanvasDispatcher::drawShadow(const DlPath& path,
                                      const DlColor color,
                                      const DlScalar elevation,
                                      bool transparent_occluder,
                                      DlScalar dpr) {
  path.WillRenderSkPath();
  DrawShadow(canvas_, path.GetSkPath(), color, elevation, transparent_occluder,
             dpr);
}

}  // namespace flutter
