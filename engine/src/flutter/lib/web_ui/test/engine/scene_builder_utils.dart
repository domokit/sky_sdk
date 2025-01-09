// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';

import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;

class StubPicture implements ScenePicture {
  StubPicture(this.cullRect);

  @override
  final ui.Rect cullRect;

  @override
  int get approximateBytesUsed => throw UnimplementedError();

  @override
  bool get debugDisposed => throw UnimplementedError();

  @override
  void dispose() {}

  @override
  Future<ui.Image> toImage(int width, int height) {
    throw UnimplementedError();
  }

  @override
  ui.Image toImageSync(int width, int height) {
    throw UnimplementedError();
  }
}

class StubCompositePicture extends StubPicture {
  StubCompositePicture(this.children)
    : super(
        children.fold(null, (ui.Rect? previousValue, StubPicture child) {
              final ui.Rect childRect = child.cullRect;
              if (childRect.isEmpty) {
                return previousValue;
              }
              return previousValue?.expandToInclude(child.cullRect) ?? child.cullRect;
            }) ??
            ui.Rect.zero,
      );

  final List<StubPicture> children;
}

class StubPictureRecorder implements ui.PictureRecorder {
  StubPictureRecorder(this.canvas);

  final StubSceneCanvas canvas;

  @override
  ui.Picture endRecording() {
    return StubCompositePicture(canvas.pictures);
  }

  @override
  bool get isRecording => throw UnimplementedError();
}

class StubSceneCanvas implements SceneCanvas {
  List<StubPicture> pictures = <StubPicture>[];

  // We actually use offsets in some of the tests, so we need to track the
  // translate calls as they are made.
  List<ui.Offset> offsetStack = <ui.Offset>[ui.Offset.zero];

  ui.Offset get currentOffset {
    return offsetStack.last;
  }

  set currentOffset(ui.Offset offset) {
    offsetStack[offsetStack.length - 1] = offset;
  }

  @override
  void drawPicture(ui.Picture picture) {
    pictures.add(StubPicture((picture as StubPicture).cullRect.shift(currentOffset)));
  }

  @override
  void clipPath(ui.Path path, {bool doAntiAlias = true}) {}

  @override
  void clipRRect(ui.RRect rrect, {bool doAntiAlias = true}) {}

  @override
  void clipRect(ui.Rect rect, {ui.ClipOp clipOp = ui.ClipOp.intersect, bool doAntiAlias = true}) {}

  @override
  void drawArc(
    ui.Rect rect,
    double startAngle,
    double sweepAngle,
    bool useCenter,
    ui.Paint paint,
  ) {}

  @override
  void drawAtlas(
    ui.Image atlas,
    List<ui.RSTransform> transforms,
    List<ui.Rect> rects,
    List<ui.Color>? colors,
    ui.BlendMode? blendMode,
    ui.Rect? cullRect,
    ui.Paint paint,
  ) {}

  @override
  void drawCircle(ui.Offset c, double radius, ui.Paint paint) {}

  @override
  void drawColor(ui.Color color, ui.BlendMode blendMode) {}

  @override
  void drawDRRect(ui.RRect outer, ui.RRect inner, ui.Paint paint) {}

  @override
  void drawImage(ui.Image image, ui.Offset offset, ui.Paint paint) {}

  @override
  void drawImageNine(ui.Image image, ui.Rect center, ui.Rect dst, ui.Paint paint) {}

  @override
  void drawImageRect(ui.Image image, ui.Rect src, ui.Rect dst, ui.Paint paint) {}

  @override
  void drawLine(ui.Offset p1, ui.Offset p2, ui.Paint paint) {}

  @override
  void drawOval(ui.Rect rect, ui.Paint paint) {}

  @override
  void drawPaint(ui.Paint paint) {}

  @override
  void drawParagraph(ui.Paragraph paragraph, ui.Offset offset) {}

  @override
  void drawPath(ui.Path path, ui.Paint paint) {}

  @override
  void drawPoints(ui.PointMode pointMode, List<ui.Offset> points, ui.Paint paint) {}

  @override
  void drawRRect(ui.RRect rrect, ui.Paint paint) {}

  @override
  void drawRawAtlas(
    ui.Image atlas,
    Float32List rstTransforms,
    Float32List rects,
    Int32List? colors,
    ui.BlendMode? blendMode,
    ui.Rect? cullRect,
    ui.Paint paint,
  ) {}

  @override
  void drawRawPoints(ui.PointMode pointMode, Float32List points, ui.Paint paint) {}

  @override
  void drawRect(ui.Rect rect, ui.Paint paint) {}

  @override
  void drawShadow(ui.Path path, ui.Color color, double elevation, bool transparentOccluder) {}

  @override
  void drawVertices(ui.Vertices vertices, ui.BlendMode blendMode, ui.Paint paint) {}

  @override
  ui.Rect getDestinationClipBounds() {
    throw UnimplementedError();
  }

  @override
  ui.Rect getLocalClipBounds() {
    throw UnimplementedError();
  }

  @override
  int getSaveCount() {
    throw UnimplementedError();
  }

  @override
  Float64List getTransform() {
    throw UnimplementedError();
  }

  @override
  void restore() {
    offsetStack.removeLast();
  }

  @override
  void restoreToCount(int count) {}

  @override
  void rotate(double radians) {}

  @override
  void save() {
    offsetStack.add(currentOffset);
  }

  @override
  void saveLayer(ui.Rect? bounds, ui.Paint paint) {}

  @override
  void saveLayerWithFilter(ui.Rect? bounds, ui.Paint paint, ui.ImageFilter backdropFilter) {}

  @override
  void scale(double sx, [double? sy]) {}

  @override
  void skew(double sx, double sy) {}

  @override
  void transform(Float64List matrix4) {}

  @override
  void translate(double dx, double dy) {
    currentOffset += ui.Offset(dx, dy);
  }
}
