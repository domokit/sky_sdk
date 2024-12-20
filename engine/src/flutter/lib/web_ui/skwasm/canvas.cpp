// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "export.h"
#include "helpers.h"
#include "wrappers.h"

#include "third_party/skia/include/core/SkPoint3.h"
#include "third_party/skia/include/core/SkVertices.h"
#include "third_party/skia/include/utils/SkShadowUtils.h"
#include "third_party/skia/modules/skparagraph/include/Paragraph.h"

using namespace skia::textlayout;

using namespace Skwasm;

namespace {
// These numbers have been chosen empirically to give a result closest to the
// material spec.
// These values are also used by the CanvasKit renderer and the native engine.
// See:
//   flutter/display_list/skia/dl_sk_dispatcher.cc
//   flutter/lib/web_ui/lib/src/engine/canvaskit/util.dart
constexpr SkScalar kShadowAmbientAlpha = 0.039;
constexpr SkScalar kShadowSpotAlpha = 0.25;
constexpr SkScalar kShadowLightRadius = 1.1;
constexpr SkScalar kShadowLightHeight = 600.0;
constexpr SkScalar kShadowLightXOffset = 0;
constexpr SkScalar kShadowLightYOffset = -450;
}  // namespace

SKWASM_EXPORT void canvas_saveLayer(SkCanvas* canvas,
                                    SkRect* rect,
                                    SkPaint* paint,
                                    SkImageFilter* backdrop,
                                    SkTileMode backdropTileMode) {
  canvas->saveLayer(SkCanvas::SaveLayerRec(rect, paint, backdrop,
                                           backdropTileMode, nullptr, 0));
}

SKWASM_EXPORT void canvas_save(SkCanvas* canvas) {
  canvas->save();
}

SKWASM_EXPORT void canvas_restore(SkCanvas* canvas) {
  canvas->restore();
}

SKWASM_EXPORT void canvas_restoreToCount(SkCanvas* canvas, int count) {
  canvas->restoreToCount(count);
}

SKWASM_EXPORT int canvas_getSaveCount(SkCanvas* canvas) {
  return canvas->getSaveCount();
}

SKWASM_EXPORT void canvas_translate(SkCanvas* canvas,
                                    SkScalar dx,
                                    SkScalar dy) {
  canvas->translate(dx, dy);
}

SKWASM_EXPORT void canvas_scale(SkCanvas* canvas, SkScalar sx, SkScalar sy) {
  canvas->scale(sx, sy);
}

SKWASM_EXPORT void canvas_rotate(SkCanvas* canvas, SkScalar degrees) {
  canvas->rotate(degrees);
}

SKWASM_EXPORT void canvas_skew(SkCanvas* canvas, SkScalar sx, SkScalar sy) {
  canvas->skew(sx, sy);
}

SKWASM_EXPORT void canvas_transform(SkCanvas* canvas, const SkM44* matrix44) {
  canvas->concat(*matrix44);
}

SKWASM_EXPORT void canvas_clipRect(SkCanvas* canvas,
                                   const SkRect* rect,
                                   SkClipOp op,
                                   bool antialias) {
  canvas->clipRect(*rect, op, antialias);
}

SKWASM_EXPORT void canvas_clipRRect(SkCanvas* canvas,
                                    const SkScalar* rrectValues,
                                    bool antialias) {
  canvas->clipRRect(createRRect(rrectValues), antialias);
}

SKWASM_EXPORT void canvas_clipPath(SkCanvas* canvas,
                                   SkPath* path,
                                   bool antialias) {
  canvas->clipPath(*path, antialias);
}

SKWASM_EXPORT void canvas_drawColor(SkCanvas* canvas,
                                    SkColor color,
                                    SkBlendMode blendMode) {
  canvas->drawColor(color, blendMode);
}

SKWASM_EXPORT void canvas_drawLine(SkCanvas* canvas,
                                   SkScalar x1,
                                   SkScalar y1,
                                   SkScalar x2,
                                   SkScalar y2,
                                   SkPaint* paint) {
  canvas->drawLine(x1, y1, x2, y2, *paint);
}

SKWASM_EXPORT void canvas_drawPaint(SkCanvas* canvas, SkPaint* paint) {
  canvas->drawPaint(*paint);
}

SKWASM_EXPORT void canvas_drawRect(SkCanvas* canvas,
                                   SkRect* rect,
                                   SkPaint* paint) {
  canvas->drawRect(*rect, *paint);
}

SKWASM_EXPORT void canvas_drawRRect(SkCanvas* canvas,
                                    const SkScalar* rrectValues,
                                    SkPaint* paint) {
  canvas->drawRRect(createRRect(rrectValues), *paint);
}

SKWASM_EXPORT void canvas_drawDRRect(SkCanvas* canvas,
                                     const SkScalar* outerRrectValues,
                                     const SkScalar* innerRrectValues,
                                     SkPaint* paint) {
  canvas->drawDRRect(createRRect(outerRrectValues),
                     createRRect(innerRrectValues), *paint);
}

SKWASM_EXPORT void canvas_drawOval(SkCanvas* canvas,
                                   const SkRect* rect,
                                   SkPaint* paint) {
  canvas->drawOval(*rect, *paint);
}

SKWASM_EXPORT void canvas_drawCircle(SkCanvas* canvas,
                                     SkScalar x,
                                     SkScalar y,
                                     SkScalar radius,
                                     SkPaint* paint) {
  canvas->drawCircle(x, y, radius, *paint);
}

SKWASM_EXPORT void canvas_drawArc(SkCanvas* canvas,
                                  const SkRect* rect,
                                  SkScalar startAngleDegrees,
                                  SkScalar sweepAngleDegrees,
                                  bool useCenter,
                                  SkPaint* paint) {
  canvas->drawArc(*rect, startAngleDegrees, sweepAngleDegrees, useCenter,
                  *paint);
}

SKWASM_EXPORT void canvas_drawPath(SkCanvas* canvas,
                                   SkPath* path,
                                   SkPaint* paint) {
  canvas->drawPath(*path, *paint);
}

SKWASM_EXPORT void canvas_drawShadow(SkCanvas* canvas,
                                     SkPath* path,
                                     SkScalar elevation,
                                     SkScalar devicePixelRatio,
                                     SkColor color,
                                     bool transparentOccluder) {
  SkColor inAmbient =
      SkColorSetA(color, kShadowAmbientAlpha * SkColorGetA(color));
  SkColor inSpot = SkColorSetA(color, kShadowSpotAlpha * SkColorGetA(color));
  SkColor outAmbient;
  SkColor outSpot;
  SkShadowUtils::ComputeTonalColors(inAmbient, inSpot, &outAmbient, &outSpot);
  uint32_t flags = transparentOccluder
                       ? SkShadowFlags::kTransparentOccluder_ShadowFlag
                       : SkShadowFlags::kNone_ShadowFlag;
  flags |= SkShadowFlags::kDirectionalLight_ShadowFlag;
  SkShadowUtils::DrawShadow(
      canvas, *path, SkPoint3::Make(0.0f, 0.0f, elevation * devicePixelRatio),
      SkPoint3::Make(kShadowLightXOffset, kShadowLightYOffset,
                     kShadowLightHeight * devicePixelRatio),
      devicePixelRatio * kShadowLightRadius, outAmbient, outSpot, flags);
}

SKWASM_EXPORT void canvas_drawParagraph(SkCanvas* canvas,
                                        Paragraph* paragraph,
                                        SkScalar x,
                                        SkScalar y) {
  paragraph->paint(canvas, x, y);
}

SKWASM_EXPORT void canvas_drawPicture(SkCanvas* canvas, SkPicture* picture) {
  canvas->drawPicture(picture);
}

SKWASM_EXPORT void canvas_drawImage(SkCanvas* canvas,
                                    SkImage* image,
                                    SkScalar offsetX,
                                    SkScalar offsetY,
                                    SkPaint* paint,
                                    FilterQuality quality) {
  canvas->drawImage(image, offsetX, offsetY, samplingOptionsForQuality(quality),
                    paint);
}

SKWASM_EXPORT void canvas_drawImageRect(SkCanvas* canvas,
                                        SkImage* image,
                                        SkRect* sourceRect,
                                        SkRect* destRect,
                                        SkPaint* paint,
                                        FilterQuality quality) {
  canvas->drawImageRect(image, *sourceRect, *destRect,
                        samplingOptionsForQuality(quality), paint,
                        SkCanvas::kStrict_SrcRectConstraint);
}

SKWASM_EXPORT void canvas_drawImageNine(SkCanvas* canvas,
                                        SkImage* image,
                                        SkIRect* centerRect,
                                        SkRect* destinationRect,
                                        SkPaint* paint,
                                        FilterQuality quality) {
  canvas->drawImageNine(image, *centerRect, *destinationRect,
                        filterModeForQuality(quality), paint);
}

SKWASM_EXPORT void canvas_drawVertices(SkCanvas* canvas,
                                       SkVertices* vertices,
                                       SkBlendMode mode,
                                       SkPaint* paint) {
  canvas->drawVertices(sk_ref_sp<SkVertices>(vertices), mode, *paint);
}

SKWASM_EXPORT void canvas_drawPoints(SkCanvas* canvas,
                                     SkCanvas::PointMode mode,
                                     SkPoint* points,
                                     int pointCount,
                                     SkPaint* paint) {
  canvas->drawPoints(mode, pointCount, points, *paint);
}

SKWASM_EXPORT void canvas_drawAtlas(SkCanvas* canvas,
                                    SkImage* atlas,
                                    SkRSXform* transforms,
                                    SkRect* rects,
                                    SkColor* colors,
                                    int spriteCount,
                                    SkBlendMode mode,
                                    SkRect* cullRect,
                                    SkPaint* paint) {
  canvas->drawAtlas(
      atlas, transforms, rects, colors, spriteCount, mode,
      SkSamplingOptions{SkFilterMode::kLinear, SkMipmapMode::kNone}, cullRect,
      paint);
}

SKWASM_EXPORT void canvas_getTransform(SkCanvas* canvas, SkM44* outTransform) {
  *outTransform = canvas->getLocalToDevice();
}

SKWASM_EXPORT void canvas_getLocalClipBounds(SkCanvas* canvas,
                                             SkRect* outRect) {
  *outRect = canvas->getLocalClipBounds();
}

SKWASM_EXPORT void canvas_getDeviceClipBounds(SkCanvas* canvas,
                                              SkIRect* outRect) {
  *outRect = canvas->getDeviceClipBounds();
}
