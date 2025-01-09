// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';

import 'package:ui/ui.dart' as ui;

import '../validators.dart';
import '../vector_math.dart';
import 'canvas.dart';
import 'canvaskit_api.dart';
import 'image.dart';
import 'painting.dart';
import 'path.dart';
import 'picture.dart';
import 'picture_recorder.dart';
import 'text.dart';
import 'vertices.dart';

/// An implementation of [ui.Canvas] that is backed by a CanvasKit canvas.
class CanvasKitCanvas implements ui.Canvas {
  factory CanvasKitCanvas(ui.PictureRecorder recorder, [ui.Rect? cullRect]) {
    if (recorder.isRecording) {
      throw ArgumentError('"recorder" must not already be associated with another Canvas.');
    }
    cullRect ??= ui.Rect.largest;
    final CkPictureRecorder ckRecorder = recorder as CkPictureRecorder;
    return CanvasKitCanvas._(ckRecorder.beginRecording(cullRect));
  }

  CanvasKitCanvas._(this._canvas);

  final CkCanvas _canvas;

  @override
  void save() {
    _canvas.save();
  }

  @override
  void saveLayer(ui.Rect? bounds, ui.Paint paint) {
    if (bounds == null) {
      _saveLayerWithoutBounds(paint);
    } else {
      assert(rectIsValid(bounds));
      _saveLayer(bounds, paint);
    }
  }

  void _saveLayerWithoutBounds(ui.Paint paint) {
    _canvas.saveLayerWithoutBounds(paint as CkPaint);
  }

  void _saveLayer(ui.Rect bounds, ui.Paint paint) {
    _canvas.saveLayer(bounds, paint as CkPaint);
  }

  @override
  void restore() {
    _canvas.restore();
  }

  @override
  void restoreToCount(int count) {
    _canvas.restoreToCount(count);
  }

  @override
  int getSaveCount() {
    return _canvas.saveCount!;
  }

  @override
  void translate(double dx, double dy) {
    _canvas.translate(dx, dy);
  }

  @override
  void scale(double sx, [double? sy]) => _scale(sx, sy ?? sx);

  void _scale(double sx, double sy) {
    _canvas.scale(sx, sy);
  }

  @override
  void rotate(double radians) {
    _canvas.rotate(radians);
  }

  @override
  void skew(double sx, double sy) {
    _canvas.skew(sx, sy);
  }

  @override
  void transform(Float64List matrix4) {
    if (matrix4.length != 16) {
      throw ArgumentError('"matrix4" must have 16 entries.');
    }
    _transform(toMatrix32(matrix4));
  }

  void _transform(Float32List matrix4) {
    _canvas.transform(matrix4);
  }

  @override
  Float64List getTransform() {
    return toMatrix64(_canvas.getLocalToDevice());
  }

  @override
  void clipRect(ui.Rect rect, {ui.ClipOp clipOp = ui.ClipOp.intersect, bool doAntiAlias = true}) {
    assert(rectIsValid(rect));
    _clipRect(rect, clipOp, doAntiAlias);
  }

  void _clipRect(ui.Rect rect, ui.ClipOp clipOp, bool doAntiAlias) {
    _canvas.clipRect(rect, clipOp, doAntiAlias);
  }

  @override
  void clipRRect(ui.RRect rrect, {bool doAntiAlias = true}) {
    assert(rrectIsValid(rrect));
    _clipRRect(rrect, doAntiAlias);
  }

  void _clipRRect(ui.RRect rrect, bool doAntiAlias) {
    _canvas.clipRRect(rrect, doAntiAlias);
  }

  @override
  void clipPath(ui.Path path, {bool doAntiAlias = true}) {
    _canvas.clipPath(path as CkPath, doAntiAlias);
  }

  @override
  ui.Rect getLocalClipBounds() {
    final Matrix4 transform = Matrix4.fromFloat32List(_canvas.getLocalToDevice());
    if (transform.invert() == 0) {
      // non-invertible transforms collapse space to a line or point
      return ui.Rect.zero;
    }
    return transform.transformRect(_canvas.getDeviceClipBounds());
  }

  @override
  ui.Rect getDestinationClipBounds() {
    return _canvas.getDeviceClipBounds();
  }

  @override
  void drawColor(ui.Color color, ui.BlendMode blendMode) {
    _drawColor(color, blendMode);
  }

  void _drawColor(ui.Color color, ui.BlendMode blendMode) {
    _canvas.drawColor(color, blendMode);
  }

  @override
  void drawLine(ui.Offset p1, ui.Offset p2, ui.Paint paint) {
    assert(offsetIsValid(p1));
    assert(offsetIsValid(p2));
    _drawLine(p1, p2, paint);
  }

  void _drawLine(ui.Offset p1, ui.Offset p2, ui.Paint paint) {
    _canvas.drawLine(p1, p2, paint as CkPaint);
  }

  @override
  void drawPaint(ui.Paint paint) {
    _drawPaint(paint);
  }

  void _drawPaint(ui.Paint paint) {
    _canvas.drawPaint(paint as CkPaint);
  }

  @override
  void drawRect(ui.Rect rect, ui.Paint paint) {
    assert(rectIsValid(rect));
    _drawRect(rect, paint);
  }

  void _drawRect(ui.Rect rect, ui.Paint paint) {
    _canvas.drawRect(rect, paint as CkPaint);
  }

  @override
  void drawRRect(ui.RRect rrect, ui.Paint paint) {
    assert(rrectIsValid(rrect));
    _drawRRect(rrect, paint);
  }

  void _drawRRect(ui.RRect rrect, ui.Paint paint) {
    _canvas.drawRRect(rrect, paint as CkPaint);
  }

  @override
  void drawDRRect(ui.RRect outer, ui.RRect inner, ui.Paint paint) {
    assert(rrectIsValid(outer));
    assert(rrectIsValid(inner));
    _drawDRRect(outer, inner, paint);
  }

  void _drawDRRect(ui.RRect outer, ui.RRect inner, ui.Paint paint) {
    _canvas.drawDRRect(outer, inner, paint as CkPaint);
  }

  @override
  void drawOval(ui.Rect rect, ui.Paint paint) {
    assert(rectIsValid(rect));
    _drawOval(rect, paint);
  }

  void _drawOval(ui.Rect rect, ui.Paint paint) {
    _canvas.drawOval(rect, paint as CkPaint);
  }

  @override
  void drawCircle(ui.Offset c, double radius, ui.Paint paint) {
    assert(offsetIsValid(c));
    _drawCircle(c, radius, paint);
  }

  void _drawCircle(ui.Offset c, double radius, ui.Paint paint) {
    _canvas.drawCircle(c, radius, paint as CkPaint);
  }

  @override
  void drawArc(ui.Rect rect, double startAngle, double sweepAngle, bool useCenter, ui.Paint paint) {
    assert(rectIsValid(rect));
    _drawArc(rect, startAngle, sweepAngle, useCenter, paint);
  }

  void _drawArc(
    ui.Rect rect,
    double startAngle,
    double sweepAngle,
    bool useCenter,
    ui.Paint paint,
  ) {
    _canvas.drawArc(rect, startAngle, sweepAngle, useCenter, paint as CkPaint);
  }

  @override
  void drawPath(ui.Path path, ui.Paint paint) {
    _canvas.drawPath(path as CkPath, paint as CkPaint);
  }

  @override
  void drawImage(ui.Image image, ui.Offset p, ui.Paint paint) {
    assert(offsetIsValid(p));
    _canvas.drawImage(image as CkImage, p, paint as CkPaint);
  }

  @override
  void drawImageRect(ui.Image image, ui.Rect src, ui.Rect dst, ui.Paint paint) {
    assert(rectIsValid(src));
    assert(rectIsValid(dst));
    _canvas.drawImageRect(image as CkImage, src, dst, paint as CkPaint);
  }

  @override
  void drawImageNine(ui.Image image, ui.Rect center, ui.Rect dst, ui.Paint paint) {
    assert(rectIsValid(center));
    assert(rectIsValid(dst));
    _canvas.drawImageNine(image as CkImage, center, dst, paint as CkPaint);
  }

  @override
  void drawPicture(ui.Picture picture) {
    _canvas.drawPicture(picture as CkPicture);
  }

  @override
  void drawParagraph(ui.Paragraph paragraph, ui.Offset offset) {
    assert(offsetIsValid(offset));
    _drawParagraph(paragraph, offset);
  }

  void _drawParagraph(ui.Paragraph paragraph, ui.Offset offset) {
    _canvas.drawParagraph(paragraph as CkParagraph, offset);
  }

  @override
  void drawPoints(ui.PointMode pointMode, List<ui.Offset> points, ui.Paint paint) {
    final SkFloat32List skPoints = toMallocedSkPoints(points);
    _canvas.drawPoints(paint as CkPaint, pointMode, skPoints.toTypedArray());
    free(skPoints);
  }

  @override
  void drawRawPoints(ui.PointMode pointMode, Float32List points, ui.Paint paint) {
    if (points.length % 2 != 0) {
      throw ArgumentError('"points" must have an even number of values.');
    }
    _canvas.drawPoints(paint as CkPaint, pointMode, points);
  }

  @override
  void drawVertices(ui.Vertices vertices, ui.BlendMode blendMode, ui.Paint paint) {
    _canvas.drawVertices(vertices as CkVertices, blendMode, paint as CkPaint);
  }

  @override
  void drawAtlas(
    ui.Image atlas,
    List<ui.RSTransform> transforms,
    List<ui.Rect> rects,
    List<ui.Color>? colors,
    ui.BlendMode? blendMode,
    ui.Rect? cullRect,
    ui.Paint paint,
  ) {
    assert(colors == null || colors.isEmpty || blendMode != null);

    final int rectCount = rects.length;
    if (transforms.length != rectCount) {
      throw ArgumentError('"transforms" and "rects" lengths must match.');
    }
    if (colors != null && colors.isNotEmpty && colors.length != rectCount) {
      throw ArgumentError(
        'If non-null, "colors" length must match that of "transforms" and "rects".',
      );
    }

    final Float32List rstTransformBuffer = Float32List(rectCount * 4);
    final Float32List rectBuffer = Float32List(rectCount * 4);

    for (int i = 0; i < rectCount; ++i) {
      final int index0 = i * 4;
      final int index1 = index0 + 1;
      final int index2 = index0 + 2;
      final int index3 = index0 + 3;
      final ui.RSTransform rstTransform = transforms[i];
      final ui.Rect rect = rects[i];
      assert(rectIsValid(rect));
      rstTransformBuffer[index0] = rstTransform.scos;
      rstTransformBuffer[index1] = rstTransform.ssin;
      rstTransformBuffer[index2] = rstTransform.tx;
      rstTransformBuffer[index3] = rstTransform.ty;
      rectBuffer[index0] = rect.left;
      rectBuffer[index1] = rect.top;
      rectBuffer[index2] = rect.right;
      rectBuffer[index3] = rect.bottom;
    }

    final Uint32List? colorBuffer =
        (colors == null || colors.isEmpty) ? null : toFlatColors(colors);

    _drawAtlas(
      paint,
      atlas,
      rstTransformBuffer,
      rectBuffer,
      colorBuffer,
      blendMode ?? ui.BlendMode.src,
    );
  }

  @override
  void drawRawAtlas(
    ui.Image atlas,
    Float32List rstTransforms,
    Float32List rects,
    Int32List? colors,
    ui.BlendMode? blendMode,
    ui.Rect? cullRect,
    ui.Paint paint,
  ) {
    assert(colors == null || blendMode != null);

    final int rectCount = rects.length;
    if (rstTransforms.length != rectCount) {
      throw ArgumentError('"rstTransforms" and "rects" lengths must match.');
    }
    if (rectCount % 4 != 0) {
      throw ArgumentError('"rstTransforms" and "rects" lengths must be a multiple of four.');
    }
    if (colors != null && colors.length * 4 != rectCount) {
      throw ArgumentError(
        'If non-null, "colors" length must be one fourth the length of "rstTransforms" and "rects".',
      );
    }

    Uint32List? unsignedColors;
    if (colors != null) {
      unsignedColors = colors.buffer.asUint32List(colors.offsetInBytes, colors.length);
    }

    _drawAtlas(paint, atlas, rstTransforms, rects, unsignedColors, blendMode ?? ui.BlendMode.src);
  }

  // TODO(hterkelsen): Pass a cull_rect once CanvasKit supports that.
  void _drawAtlas(
    ui.Paint paint,
    ui.Image atlas,
    Float32List rstTransforms,
    Float32List rects,
    Uint32List? colors,
    ui.BlendMode blendMode,
  ) {
    _canvas.drawAtlasRaw(
      paint as CkPaint,
      atlas as CkImage,
      rstTransforms,
      rects,
      colors,
      blendMode,
    );
  }

  @override
  void drawShadow(ui.Path path, ui.Color color, double elevation, bool transparentOccluder) {
    _drawShadow(path, color, elevation, transparentOccluder);
  }

  void _drawShadow(ui.Path path, ui.Color color, double elevation, bool transparentOccluder) {
    _canvas.drawShadow(path as CkPath, color, elevation, transparentOccluder);
  }
}
