// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/lib/ui/painting/canvas.h"

#include <cmath>

#include "flutter/display_list/dl_builder.h"
#include "flutter/lib/ui/floating_point.h"
#include "flutter/lib/ui/painting/image.h"
#include "flutter/lib/ui/painting/image_filter.h"
#include "flutter/lib/ui/painting/paint.h"
#include "flutter/lib/ui/ui_dart_state.h"
#include "flutter/lib/ui/window/platform_configuration.h"

using tonic::ToDart;

namespace flutter {

IMPLEMENT_WRAPPERTYPEINFO(ui, Canvas);

void Canvas::Create(Dart_Handle wrapper,
                    PictureRecorder* recorder,
                    double left,
                    double top,
                    double right,
                    double bottom) {
  UIDartState::ThrowIfUIOperationsProhibited();

  if (!recorder) {
    Dart_ThrowException(
        ToDart("Canvas constructor called with non-genuine PictureRecorder."));
    return;
  }

  fml::RefPtr<Canvas> canvas =
      fml::MakeRefCounted<Canvas>(recorder->BeginRecording(
          DlRect::MakeLTRB(SafeNarrow(left), SafeNarrow(top), SafeNarrow(right),
                           SafeNarrow(bottom))));
  recorder->set_canvas(canvas);
  canvas->AssociateWithDartWrapper(wrapper);
}

Canvas::Canvas(sk_sp<DisplayListBuilder> builder)
    : display_list_builder_(std::move(builder)) {}

Canvas::~Canvas() {}

void Canvas::save() {
  if (display_list_builder_) {
    builder()->Save();
  }
}

void Canvas::saveLayerWithoutBounds(Dart_Handle paint_objects,
                                    Dart_Handle paint_data) {
  Paint paint(paint_objects, paint_data);

  FML_DCHECK(paint.isNotNull());
  if (display_list_builder_) {
    DlPaint dl_paint;
    const DlPaint* save_paint =
        paint.paint(dl_paint, kSaveLayerWithPaintFlags, DlTileMode::kDecal);
    FML_DCHECK(save_paint);
    TRACE_EVENT0("flutter", "ui.Canvas::saveLayer (Recorded)");
    builder()->SaveLayer(nullptr, save_paint);
  }
}

void Canvas::saveLayer(double left,
                       double top,
                       double right,
                       double bottom,
                       Dart_Handle paint_objects,
                       Dart_Handle paint_data) {
  Paint paint(paint_objects, paint_data);

  FML_DCHECK(paint.isNotNull());
  DlRect bounds = DlRect::MakeLTRB(SafeNarrow(left), SafeNarrow(top),
                                   SafeNarrow(right), SafeNarrow(bottom));
  if (display_list_builder_) {
    DlPaint dl_paint;
    const DlPaint* save_paint =
        paint.paint(dl_paint, kSaveLayerWithPaintFlags, DlTileMode::kDecal);
    FML_DCHECK(save_paint);
    TRACE_EVENT0("flutter", "ui.Canvas::saveLayer (Recorded)");
    builder()->SaveLayer(bounds, save_paint);
  }
}

void Canvas::restore() {
  if (display_list_builder_) {
    builder()->Restore();
  }
}

int Canvas::getSaveCount() {
  if (display_list_builder_) {
    return builder()->GetSaveCount();
  } else {
    return 0;
  }
}

void Canvas::restoreToCount(int count) {
  if (display_list_builder_ && count < getSaveCount()) {
    builder()->RestoreToCount(count);
  }
}

void Canvas::translate(double dx, double dy) {
  if (display_list_builder_) {
    builder()->Translate(SafeNarrow(dx), SafeNarrow(dy));
  }
}

void Canvas::scale(double sx, double sy) {
  if (display_list_builder_) {
    builder()->Scale(SafeNarrow(sx), SafeNarrow(sy));
  }
}

void Canvas::rotate(double radians) {
  if (display_list_builder_) {
    builder()->Rotate(SafeNarrow(radians) * 180.0f / static_cast<float>(M_PI));
  }
}

void Canvas::skew(double sx, double sy) {
  if (display_list_builder_) {
    builder()->Skew(SafeNarrow(sx), SafeNarrow(sy));
  }
}

void Canvas::transform(const tonic::Float64List& matrix4) {
  // The Float array stored by Dart Matrix4 is in column-major order
  // DisplayList TransformFullPerspective takes row-major matrix order
  if (display_list_builder_) {
    // clang-format off
    builder()->TransformFullPerspective(
        SafeNarrow(matrix4[ 0]), SafeNarrow(matrix4[ 4]), SafeNarrow(matrix4[ 8]), SafeNarrow(matrix4[12]),
        SafeNarrow(matrix4[ 1]), SafeNarrow(matrix4[ 5]), SafeNarrow(matrix4[ 9]), SafeNarrow(matrix4[13]),
        SafeNarrow(matrix4[ 2]), SafeNarrow(matrix4[ 6]), SafeNarrow(matrix4[10]), SafeNarrow(matrix4[14]),
        SafeNarrow(matrix4[ 3]), SafeNarrow(matrix4[ 7]), SafeNarrow(matrix4[11]), SafeNarrow(matrix4[15]));
    // clang-format on
  }
}

void Canvas::getTransform(Dart_Handle matrix4_handle) {
  if (display_list_builder_) {
    // The Float array stored by DlMatrix is in column-major order
    DlMatrix matrix = builder()->GetMatrix();
    auto matrix4 = tonic::Float64List(matrix4_handle);
    for (int i = 0; i < 16; i++) {
      matrix4[i] = matrix.m[i];
    }
  }
}

void Canvas::clipRect(double left,
                      double top,
                      double right,
                      double bottom,
                      DlCanvas::ClipOp clipOp,
                      bool doAntiAlias) {
  if (display_list_builder_) {
    builder()->ClipRect(DlRect::MakeLTRB(SafeNarrow(left), SafeNarrow(top),
                                         SafeNarrow(right), SafeNarrow(bottom)),
                        clipOp, doAntiAlias);
  }
}

void Canvas::clipRRect(const RRect& rrect, bool doAntiAlias) {
  if (display_list_builder_) {
    builder()->ClipRoundRect(rrect.rrect, DlCanvas::ClipOp::kIntersect,
                             doAntiAlias);
  }
}

void Canvas::clipPath(const CanvasPath* path, bool doAntiAlias) {
  if (!path) {
    Dart_ThrowException(
        ToDart("Canvas.clipPath called with non-genuine Path."));
    return;
  }
  if (display_list_builder_) {
    builder()->ClipPath(path->path(), DlCanvas::ClipOp::kIntersect,
                        doAntiAlias);
  }
}

void Canvas::getDestinationClipBounds(Dart_Handle rect_handle) {
  if (display_list_builder_) {
    auto rect = tonic::Float64List(rect_handle);
    DlRect bounds = builder()->GetDestinationClipCoverage();
    rect[0] = bounds.GetLeft();
    rect[1] = bounds.GetTop();
    rect[2] = bounds.GetRight();
    rect[3] = bounds.GetBottom();
  }
}

void Canvas::getLocalClipBounds(Dart_Handle rect_handle) {
  if (display_list_builder_) {
    auto rect = tonic::Float64List(rect_handle);
    DlRect bounds = builder()->GetLocalClipCoverage();
    rect[0] = bounds.GetLeft();
    rect[1] = bounds.GetTop();
    rect[2] = bounds.GetRight();
    rect[3] = bounds.GetBottom();
  }
}

void Canvas::drawColor(uint32_t color, DlBlendMode blend_mode) {
  if (display_list_builder_) {
    builder()->DrawColor(DlColor(color), blend_mode);
  }
}

void Canvas::drawLine(double x1,
                      double y1,
                      double x2,
                      double y2,
                      Dart_Handle paint_objects,
                      Dart_Handle paint_data) {
  Paint paint(paint_objects, paint_data);

  FML_DCHECK(paint.isNotNull());
  if (display_list_builder_) {
    DlPaint dl_paint;
    paint.paint(dl_paint, kDrawLineFlags, DlTileMode::kDecal);
    builder()->DrawLine(DlPoint(SafeNarrow(x1), SafeNarrow(y1)),
                        DlPoint(SafeNarrow(x2), SafeNarrow(y2)), dl_paint);
  }
}

void Canvas::drawPaint(Dart_Handle paint_objects, Dart_Handle paint_data) {
  Paint paint(paint_objects, paint_data);

  FML_DCHECK(paint.isNotNull());
  if (display_list_builder_) {
    DlPaint dl_paint;
    paint.paint(dl_paint, kDrawPaintFlags, DlTileMode::kClamp);
    std::shared_ptr<DlImageFilter> filter = dl_paint.getImageFilter();
    if (filter && !filter->asColorFilter()) {
      // drawPaint does an implicit saveLayer if an DlImageFilter is
      // present that cannot be replaced by an DlColorFilter.
      TRACE_EVENT0("flutter", "ui.Canvas::saveLayer (Recorded)");
    }
    builder()->DrawPaint(dl_paint);
  }
}

void Canvas::drawRect(double left,
                      double top,
                      double right,
                      double bottom,
                      Dart_Handle paint_objects,
                      Dart_Handle paint_data) {
  Paint paint(paint_objects, paint_data);

  FML_DCHECK(paint.isNotNull());
  if (display_list_builder_) {
    DlPaint dl_paint;
    paint.paint(dl_paint, kDrawRectFlags, DlTileMode::kDecal);
    builder()->DrawRect(DlRect::MakeLTRB(SafeNarrow(left), SafeNarrow(top),
                                         SafeNarrow(right), SafeNarrow(bottom)),
                        dl_paint);
  }
}

void Canvas::drawRRect(const RRect& rrect,
                       Dart_Handle paint_objects,
                       Dart_Handle paint_data) {
  Paint paint(paint_objects, paint_data);

  FML_DCHECK(paint.isNotNull());
  if (display_list_builder_) {
    DlPaint dl_paint;
    paint.paint(dl_paint, kDrawRRectFlags, DlTileMode::kDecal);
    builder()->DrawRoundRect(rrect.rrect, dl_paint);
  }
}

void Canvas::drawDRRect(const RRect& outer,
                        const RRect& inner,
                        Dart_Handle paint_objects,
                        Dart_Handle paint_data) {
  Paint paint(paint_objects, paint_data);

  FML_DCHECK(paint.isNotNull());
  if (display_list_builder_) {
    DlPaint dl_paint;
    paint.paint(dl_paint, kDrawDRRectFlags, DlTileMode::kDecal);
    builder()->DrawDiffRoundRect(outer.rrect, inner.rrect, dl_paint);
  }
}

void Canvas::drawOval(double left,
                      double top,
                      double right,
                      double bottom,
                      Dart_Handle paint_objects,
                      Dart_Handle paint_data) {
  Paint paint(paint_objects, paint_data);

  FML_DCHECK(paint.isNotNull());
  if (display_list_builder_) {
    DlPaint dl_paint;
    paint.paint(dl_paint, kDrawOvalFlags, DlTileMode::kDecal);
    builder()->DrawOval(DlRect::MakeLTRB(SafeNarrow(left), SafeNarrow(top),
                                         SafeNarrow(right), SafeNarrow(bottom)),
                        dl_paint);
  }
}

void Canvas::drawCircle(double x,
                        double y,
                        double radius,
                        Dart_Handle paint_objects,
                        Dart_Handle paint_data) {
  Paint paint(paint_objects, paint_data);

  FML_DCHECK(paint.isNotNull());
  if (display_list_builder_) {
    DlPaint dl_paint;
    paint.paint(dl_paint, kDrawCircleFlags, DlTileMode::kDecal);
    builder()->DrawCircle(DlPoint(SafeNarrow(x), SafeNarrow(y)),
                          SafeNarrow(radius), dl_paint);
  }
}

void Canvas::drawArc(double left,
                     double top,
                     double right,
                     double bottom,
                     double startAngle,
                     double sweepAngle,
                     bool useCenter,
                     Dart_Handle paint_objects,
                     Dart_Handle paint_data) {
  Paint paint(paint_objects, paint_data);

  FML_DCHECK(paint.isNotNull());
  if (display_list_builder_) {
    DlPaint dl_paint;
    paint.paint(dl_paint,
                useCenter ? kDrawArcWithCenterFlags : kDrawArcNoCenterFlags,
                DlTileMode::kDecal);
    builder()->DrawArc(
        DlRect::MakeLTRB(SafeNarrow(left), SafeNarrow(top), SafeNarrow(right),
                         SafeNarrow(bottom)),
        SafeNarrow(startAngle) * 180.0f / static_cast<float>(M_PI),
        SafeNarrow(sweepAngle) * 180.0f / static_cast<float>(M_PI), useCenter,
        dl_paint);
  }
}

void Canvas::drawPath(const CanvasPath* path,
                      Dart_Handle paint_objects,
                      Dart_Handle paint_data) {
  Paint paint(paint_objects, paint_data);

  FML_DCHECK(paint.isNotNull());
  if (!path) {
    Dart_ThrowException(
        ToDart("Canvas.drawPath called with non-genuine Path."));
    return;
  }
  if (display_list_builder_) {
    DlPaint dl_paint;
    paint.paint(dl_paint, kDrawPathFlags, DlTileMode::kDecal);
    builder()->DrawPath(path->path(), dl_paint);
  }
}

Dart_Handle Canvas::drawImage(const CanvasImage* image,
                              double x,
                              double y,
                              Dart_Handle paint_objects,
                              Dart_Handle paint_data,
                              int filterQualityIndex) {
  Paint paint(paint_objects, paint_data);

  FML_DCHECK(paint.isNotNull());
  if (!image) {
    return ToDart("Canvas.drawImage called with non-genuine Image.");
  }

  auto dl_image = image->image();
  if (!dl_image) {
    return Dart_Null();
  }
  auto error = dl_image->get_error();
  if (error) {
    return ToDart(error.value());
  }

  auto sampling = ImageFilter::SamplingFromIndex(filterQualityIndex);
  if (display_list_builder_) {
    DlPaint dl_paint;
    const DlPaint* opt_paint =
        paint.paint(dl_paint, kDrawImageWithPaintFlags, DlTileMode::kClamp);
    builder()->DrawImage(dl_image, DlPoint(SafeNarrow(x), SafeNarrow(y)),
                         sampling, opt_paint);
  }
  return Dart_Null();
}

Dart_Handle Canvas::drawImageRect(const CanvasImage* image,
                                  double src_left,
                                  double src_top,
                                  double src_right,
                                  double src_bottom,
                                  double dst_left,
                                  double dst_top,
                                  double dst_right,
                                  double dst_bottom,
                                  Dart_Handle paint_objects,
                                  Dart_Handle paint_data,
                                  int filterQualityIndex) {
  Paint paint(paint_objects, paint_data);

  FML_DCHECK(paint.isNotNull());
  if (!image) {
    return ToDart("Canvas.drawImageRect called with non-genuine Image.");
  }

  auto dl_image = image->image();
  if (!dl_image) {
    return Dart_Null();
  }
  auto error = dl_image->get_error();
  if (error) {
    return ToDart(error.value());
  }

  DlRect src = DlRect::MakeLTRB(SafeNarrow(src_left), SafeNarrow(src_top),
                                SafeNarrow(src_right), SafeNarrow(src_bottom));
  DlRect dst = DlRect::MakeLTRB(SafeNarrow(dst_left), SafeNarrow(dst_top),
                                SafeNarrow(dst_right), SafeNarrow(dst_bottom));
  auto sampling = ImageFilter::SamplingFromIndex(filterQualityIndex);
  if (display_list_builder_) {
    DlPaint dl_paint;
    const DlPaint* opt_paint =
        paint.paint(dl_paint, kDrawImageRectWithPaintFlags, DlTileMode::kClamp);
    builder()->DrawImageRect(dl_image, src, dst, sampling, opt_paint,
                             DlCanvas::SrcRectConstraint::kFast);
  }
  return Dart_Null();
}

Dart_Handle Canvas::drawImageNine(const CanvasImage* image,
                                  double center_left,
                                  double center_top,
                                  double center_right,
                                  double center_bottom,
                                  double dst_left,
                                  double dst_top,
                                  double dst_right,
                                  double dst_bottom,
                                  Dart_Handle paint_objects,
                                  Dart_Handle paint_data,
                                  int bitmapSamplingIndex) {
  Paint paint(paint_objects, paint_data);

  FML_DCHECK(paint.isNotNull());
  if (!image) {
    return ToDart("Canvas.drawImageNine called with non-genuine Image.");
  }
  auto dl_image = image->image();
  if (!dl_image) {
    return Dart_Null();
  }
  auto error = dl_image->get_error();
  if (error) {
    return ToDart(error.value());
  }

  DlRect center =
      DlRect::MakeLTRB(SafeNarrow(center_left), SafeNarrow(center_top),
                       SafeNarrow(center_right), SafeNarrow(center_bottom));
  DlIRect icenter = DlIRect::Round(center);
  DlRect dst = DlRect::MakeLTRB(SafeNarrow(dst_left), SafeNarrow(dst_top),
                                SafeNarrow(dst_right), SafeNarrow(dst_bottom));
  auto filter = ImageFilter::FilterModeFromIndex(bitmapSamplingIndex);
  if (display_list_builder_) {
    DlPaint dl_paint;
    const DlPaint* opt_paint =
        paint.paint(dl_paint, kDrawImageNineWithPaintFlags, DlTileMode::kClamp);
    builder()->DrawImageNine(dl_image, icenter, dst, filter, opt_paint);
  }
  return Dart_Null();
}

void Canvas::drawPicture(Picture* picture) {
  if (!picture) {
    Dart_ThrowException(
        ToDart("Canvas.drawPicture called with non-genuine Picture."));
    return;
  }
  if (picture->display_list()) {
    if (display_list_builder_) {
      builder()->DrawDisplayList(picture->display_list());
    }
  } else {
    FML_DCHECK(false);
  }
}

void Canvas::drawPoints(Dart_Handle paint_objects,
                        Dart_Handle paint_data,
                        DlCanvas::PointMode point_mode,
                        const tonic::Float32List& points) {
  Paint paint(paint_objects, paint_data);

  static_assert(sizeof(DlPoint) == sizeof(float) * 2,
                "DlPoint doesn't use floats.");

  FML_DCHECK(paint.isNotNull());
  if (display_list_builder_) {
    DlPaint dl_paint;
    switch (point_mode) {
      case DlCanvas::PointMode::kPoints:
        paint.paint(dl_paint, kDrawPointsAsPointsFlags, DlTileMode::kDecal);
        break;
      case DlCanvas::PointMode::kLines:
        paint.paint(dl_paint, kDrawPointsAsLinesFlags, DlTileMode::kDecal);
        break;
      case DlCanvas::PointMode::kPolygon:
        paint.paint(dl_paint, kDrawPointsAsPolygonFlags, DlTileMode::kDecal);
        break;
    }
    builder()->DrawPoints(point_mode,
                          points.num_elements() / 2,  // DlPoints have 2 floats
                          reinterpret_cast<const DlPoint*>(points.data()),
                          dl_paint);
  }
}

void Canvas::drawVertices(const Vertices* vertices,
                          DlBlendMode blend_mode,
                          Dart_Handle paint_objects,
                          Dart_Handle paint_data) {
  Paint paint(paint_objects, paint_data);

  if (!vertices) {
    Dart_ThrowException(
        ToDart("Canvas.drawVertices called with non-genuine Vertices."));
    return;
  }
  FML_DCHECK(paint.isNotNull());
  if (display_list_builder_) {
    DlPaint dl_paint;
    paint.paint(dl_paint, kDrawVerticesFlags, DlTileMode::kDecal);
    builder()->DrawVertices(vertices->vertices(), blend_mode, dl_paint);
  }
}

Dart_Handle Canvas::drawAtlas(Dart_Handle paint_objects,
                              Dart_Handle paint_data,
                              int filterQualityIndex,
                              CanvasImage* atlas,
                              Dart_Handle transforms_handle,
                              Dart_Handle rects_handle,
                              Dart_Handle colors_handle,
                              DlBlendMode blend_mode,
                              Dart_Handle cull_rect_handle) {
  Paint paint(paint_objects, paint_data);

  if (!atlas) {
    return ToDart(
        "Canvas.drawAtlas or Canvas.drawRawAtlas called with "
        "non-genuine Image.");
  }

  auto dl_image = atlas->image();
  auto error = dl_image->get_error();
  if (error) {
    return ToDart(error.value());
  }

  static_assert(sizeof(SkRSXform) == sizeof(float) * 4,
                "SkRSXform doesn't use floats.");
  static_assert(sizeof(DlRect) == sizeof(float) * 4,
                "DlRect doesn't use floats.");

  auto sampling = ImageFilter::SamplingFromIndex(filterQualityIndex);

  FML_DCHECK(paint.isNotNull());
  if (display_list_builder_) {
    tonic::Float32List transforms(transforms_handle);
    tonic::Float32List rects(rects_handle);
    tonic::Int32List colors(colors_handle);
    tonic::Float32List cull_rect(cull_rect_handle);

    std::vector<DlColor> dl_color(colors.num_elements());
    size_t count = colors.num_elements();
    for (size_t i = 0; i < count; i++) {
      dl_color[i] = DlColor(colors[i]);
    }

    DlPaint dl_paint;
    const DlPaint* opt_paint =
        paint.paint(dl_paint, kDrawAtlasWithPaintFlags, DlTileMode::kClamp);
    builder()->DrawAtlas(
        dl_image, reinterpret_cast<const SkRSXform*>(transforms.data()),
        reinterpret_cast<const DlRect*>(rects.data()), dl_color.data(),
        rects.num_elements() / 4,  // DlRect have four floats.
        blend_mode, sampling, reinterpret_cast<const DlRect*>(cull_rect.data()),
        opt_paint);
  }
  return Dart_Null();
}

void Canvas::drawShadow(const CanvasPath* path,
                        uint32_t color,
                        double elevation,
                        bool transparentOccluder) {
  if (!path) {
    Dart_ThrowException(
        ToDart("Canvas.drawShader called with non-genuine Path."));
    return;
  }

  // Not using SafeNarrow because DPR will always be a relatively small number.
  const ViewportMetrics* metrics =
      UIDartState::Current()->platform_configuration()->GetMetrics(0);
  DlScalar dpr;
  // TODO(dkwingsmt): We should support rendering shadows on non-implicit views.
  // However, currently this method has no way to get the target view ID.
  if (metrics == nullptr) {
    dpr = 1.0f;
  } else {
    dpr = static_cast<float>(metrics->device_pixel_ratio);
  }
  if (display_list_builder_) {
    builder()->DrawShadow(path->path(), DlColor(color), SafeNarrow(elevation),
                          transparentOccluder, dpr);
  }
}

void Canvas::Invalidate() {
  display_list_builder_ = nullptr;
  if (dart_wrapper()) {
    ClearDartWrapper();
  }
}

}  // namespace flutter
