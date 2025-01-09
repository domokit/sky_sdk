// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;
import 'dart:typed_data';

import 'package:ui/ui.dart' as ui;

import '../color_filter.dart';
import '../display.dart';
import 'canvaskit_api.dart';
import 'color_filter.dart';
import 'image.dart';
import 'image_filter.dart';
import 'painting.dart';
import 'path.dart';
import 'picture.dart';
import 'text.dart';
import 'util.dart';
import 'vertices.dart';

/// Memoized value for ClipOp.Intersect, so we don't have to hit JS-interop
/// every time we need it.
final SkClipOp _clipOpIntersect = canvasKit.ClipOp.Intersect;

/// A Dart wrapper around Skia's [SkCanvas].
///
/// This is intentionally not memory-managing the underlying [SkCanvas]. See
/// the docs on [SkCanvas], which explain the reason.
class CkCanvas {
  CkCanvas(this.skCanvas);

  // Cubic equation coefficients recommended by Mitchell & Netravali
  // in their paper on cubic interpolation.
  static const double _kMitchellNetravali_B = 1.0 / 3.0;
  static const double _kMitchellNetravali_C = 1.0 / 3.0;

  final SkCanvas skCanvas;

  int? get saveCount => skCanvas.getSaveCount().toInt();

  void clear(ui.Color color) {
    skCanvas.clear(toSharedSkColor1(color));
  }

  void clipPath(CkPath path, bool doAntiAlias) {
    skCanvas.clipPath(path.skiaObject, _clipOpIntersect, doAntiAlias);
  }

  void clipRRect(ui.RRect rrect, bool doAntiAlias) {
    skCanvas.clipRRect(toSkRRect(rrect), _clipOpIntersect, doAntiAlias);
  }

  void clipRect(ui.Rect rect, ui.ClipOp clipOp, bool doAntiAlias) {
    skCanvas.clipRect(toSkRect(rect), toSkClipOp(clipOp), doAntiAlias);
  }

  ui.Rect getDeviceClipBounds() {
    return rectFromSkIRect(skCanvas.getDeviceClipBounds());
  }

  void drawArc(ui.Rect oval, double startAngle, double sweepAngle, bool useCenter, CkPaint paint) {
    const double toDegrees = 180 / math.pi;

    final skPaint = paint.toSkPaint();
    skCanvas.drawArc(
      toSkRect(oval),
      startAngle * toDegrees,
      sweepAngle * toDegrees,
      useCenter,
      skPaint,
    );
    skPaint.delete();
  }

  // TODO(flar): CanvasKit does not expose sampling options available on SkCanvas.drawAtlas
  void drawAtlasRaw(
    CkPaint paint,
    CkImage atlas,
    Float32List rstTransforms,
    Float32List rects,
    Uint32List? colors,
    ui.BlendMode blendMode,
  ) {
    final skPaint = paint.toSkPaint(defaultBlurTileMode: ui.TileMode.clamp);
    skCanvas.drawAtlas(
      atlas.skImage,
      rects,
      rstTransforms,
      skPaint,
      toSkBlendMode(blendMode),
      colors,
    );
    skPaint.delete();
  }

  void drawCircle(ui.Offset c, double radius, CkPaint paint) {
    final skPaint = paint.toSkPaint();
    skCanvas.drawCircle(c.dx, c.dy, radius, skPaint);
    skPaint.delete();
  }

  void drawColor(ui.Color color, ui.BlendMode blendMode) {
    skCanvas.drawColorInt(color.value.toDouble(), toSkBlendMode(blendMode));
  }

  void drawDRRect(ui.RRect outer, ui.RRect inner, CkPaint paint) {
    final skPaint = paint.toSkPaint();
    skCanvas.drawDRRect(toSkRRect(outer), toSkRRect(inner), skPaint);
    skPaint.delete();
  }

  void drawImage(CkImage image, ui.Offset offset, CkPaint paint) {
    final ui.FilterQuality filterQuality = paint.filterQuality;
    final skPaint = paint.toSkPaint(defaultBlurTileMode: ui.TileMode.clamp);
    if (filterQuality == ui.FilterQuality.high) {
      skCanvas.drawImageCubic(
        image.skImage,
        offset.dx,
        offset.dy,
        _kMitchellNetravali_B,
        _kMitchellNetravali_C,
        skPaint,
      );
    } else {
      skCanvas.drawImageOptions(
        image.skImage,
        offset.dx,
        offset.dy,
        toSkFilterMode(filterQuality),
        toSkMipmapMode(filterQuality),
        skPaint,
      );
    }
    skPaint.delete();
  }

  void drawImageRect(CkImage image, ui.Rect src, ui.Rect dst, CkPaint paint) {
    final ui.FilterQuality filterQuality = paint.filterQuality;
    final skPaint = paint.toSkPaint(defaultBlurTileMode: ui.TileMode.clamp);
    if (filterQuality == ui.FilterQuality.high) {
      skCanvas.drawImageRectCubic(
        image.skImage,
        toSkRect(src),
        toSkRect(dst),
        _kMitchellNetravali_B,
        _kMitchellNetravali_C,
        skPaint,
      );
    } else {
      skCanvas.drawImageRectOptions(
        image.skImage,
        toSkRect(src),
        toSkRect(dst),
        toSkFilterMode(filterQuality),
        toSkMipmapMode(filterQuality),
        skPaint,
      );
    }
    skPaint.delete();
  }

  void drawImageNine(CkImage image, ui.Rect center, ui.Rect dst, CkPaint paint) {
    final skPaint = paint.toSkPaint(defaultBlurTileMode: ui.TileMode.clamp);
    skCanvas.drawImageNine(
      image.skImage,
      toSkRect(center),
      toSkRect(dst),
      toSkFilterMode(paint.filterQuality),
      skPaint,
    );
    skPaint.delete();
  }

  void drawLine(ui.Offset p1, ui.Offset p2, CkPaint paint) {
    final skPaint = paint.toSkPaint();
    skCanvas.drawLine(p1.dx, p1.dy, p2.dx, p2.dy, skPaint);
    skPaint.delete();
  }

  void drawOval(ui.Rect rect, CkPaint paint) {
    final skPaint = paint.toSkPaint();
    skCanvas.drawOval(toSkRect(rect), skPaint);
    skPaint.delete();
  }

  void drawPaint(CkPaint paint) {
    final skPaint = paint.toSkPaint();
    skCanvas.drawPaint(skPaint);
    skPaint.delete();
  }

  void drawParagraph(CkParagraph paragraph, ui.Offset offset) {
    skCanvas.drawParagraph(paragraph.skiaObject, offset.dx, offset.dy);
  }

  void drawPath(CkPath path, CkPaint paint) {
    final skPaint = paint.toSkPaint();
    skCanvas.drawPath(path.skiaObject, skPaint);
    skPaint.delete();
  }

  void drawPicture(CkPicture picture) {
    assert(picture.debugCheckNotDisposed('Failed to draw picture.'));
    skCanvas.drawPicture(picture.skiaObject);
  }

  void drawPoints(CkPaint paint, ui.PointMode pointMode, Float32List points) {
    final skPaint = paint.toSkPaint();
    skCanvas.drawPoints(toSkPointMode(pointMode), points, skPaint);
    skPaint.delete();
  }

  void drawRRect(ui.RRect rrect, CkPaint paint) {
    final skPaint = paint.toSkPaint();
    skCanvas.drawRRect(toSkRRect(rrect), skPaint);
    skPaint.delete();
  }

  void drawRect(ui.Rect rect, CkPaint paint) {
    final skPaint = paint.toSkPaint();
    skCanvas.drawRect(toSkRect(rect), skPaint);
    skPaint.delete();
  }

  void drawShadow(CkPath path, ui.Color color, double elevation, bool transparentOccluder) {
    drawSkShadow(
      skCanvas,
      path,
      color,
      elevation,
      transparentOccluder,
      EngineFlutterDisplay.instance.devicePixelRatio,
    );
  }

  void drawVertices(CkVertices vertices, ui.BlendMode blendMode, CkPaint paint) {
    final skPaint = paint.toSkPaint();
    skCanvas.drawVertices(vertices.skiaObject, toSkBlendMode(blendMode), skPaint);
    skPaint.delete();
  }

  void restore() {
    skCanvas.restore();
  }

  void restoreToCount(int count) {
    skCanvas.restoreToCount(count.toDouble());
  }

  void rotate(double radians) {
    skCanvas.rotate(radians * 180.0 / math.pi, 0.0, 0.0);
  }

  int save() {
    return skCanvas.save().toInt();
  }

  void saveLayer(ui.Rect bounds, CkPaint? paint) {
    final skPaint = paint?.toSkPaint();
    skCanvas.saveLayer(skPaint, toSkRect(bounds), null, null, canvasKit.TileMode.Clamp);
    skPaint?.delete();
  }

  void saveLayerWithoutBounds(CkPaint? paint) {
    final skPaint = paint?.toSkPaint();
    skCanvas.saveLayer(skPaint, null, null, null, canvasKit.TileMode.Clamp);
    skPaint?.delete();
  }

  void saveLayerWithFilter(ui.Rect bounds, ui.ImageFilter filter, [CkPaint? paint]) {
    final CkManagedSkImageFilterConvertible convertible;
    if (filter is ui.ColorFilter) {
      convertible = createCkColorFilter(filter as EngineColorFilter)!;
    } else {
      convertible = filter as CkManagedSkImageFilterConvertible;
    }
    // There are 2 ImageFilter objects applied here. The filter in the paint
    // object is applied to the contents and its default tile mode is decal
    // (automatically applied by toSkPaint).
    // The filter supplied as an argument to this function [convertible] will
    // be applied to the backdrop and its default tile mode will be mirror.
    // We also pass in the blur tile mode as an argument to saveLayer because
    // that operation will not adopt the tile mode from the backdrop filter
    // and instead needs it supplied to the saveLayer call itself as a
    // separate argument.
    convertible.withSkImageFilter((SkImageFilter filter) {
      final skPaint = paint?.toSkPaint(/*ui.TileMode.decal*/);
      skCanvas.saveLayer(
        skPaint,
        toSkRect(bounds),
        filter,
        0,
        toSkTileMode(convertible.backdropTileMode ?? ui.TileMode.mirror),
      );
      skPaint?.delete();
    }, defaultBlurTileMode: ui.TileMode.mirror);
  }

  void scale(double sx, double sy) {
    skCanvas.scale(sx, sy);
  }

  void skew(double sx, double sy) {
    skCanvas.skew(sx, sy);
  }

  void transform(Float32List matrix4) {
    skCanvas.concat(toSkM44FromFloat32(matrix4));
  }

  void translate(double dx, double dy) {
    skCanvas.translate(dx, dy);
  }

  bool quickReject(ui.Rect rect) {
    return skCanvas.quickReject(toSkRect(rect));
  }

  Float32List getLocalToDevice() {
    final List<dynamic> list = skCanvas.getLocalToDevice();
    final Float32List matrix4 = Float32List(16);
    for (int r = 0; r < 4; r++) {
      for (int c = 0; c < 4; c++) {
        matrix4[c * 4 + r] = (list[r * 4 + c] as num).toDouble();
      }
    }
    return matrix4;
  }
}
