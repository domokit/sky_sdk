// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

part of ui;

enum PointMode { points, lines, polygon }

enum ClipOp { difference, intersect }

enum VertexMode { triangles, triangleStrip, triangleFan }

abstract class Vertices {
  factory Vertices(
    VertexMode mode,
    List<Offset> positions, {
    List<Color>? colors,
    List<Offset>? textureCoordinates,
    List<int>? indices,
  }) {
    return engine.renderer.createVertices(
      mode,
      positions,
      textureCoordinates: textureCoordinates,
      colors: colors,
      indices: indices,
    );
  }
  factory Vertices.raw(
    VertexMode mode,
    Float32List positions, {
    Int32List? colors,
    Float32List? textureCoordinates,
    Uint16List? indices,
  }) {
    return engine.renderer.createVerticesRaw(
      mode,
      positions,
      textureCoordinates: textureCoordinates,
      colors: colors,
      indices: indices,
    );
  }

  void dispose();
  bool get debugDisposed;
}

abstract class PictureRecorder {
  factory PictureRecorder() => engine.renderer.createPictureRecorder();
  bool get isRecording;
  Picture endRecording();
}

abstract class Canvas {
  factory Canvas(PictureRecorder recorder, [Rect? cullRect]) =>
      engine.renderer.createCanvas(recorder, cullRect);
  void save();
  void saveLayer(Rect? bounds, Paint paint);
  void restore();
  int getSaveCount();
  void restoreToCount(int count);
  void translate(double dx, double dy);
  void scale(double sx, [double? sy]);
  void rotate(double radians);
  void skew(double sx, double sy);
  void transform(Float64List matrix4);
  Float64List getTransform();
  void clipRect(Rect rect, {ClipOp clipOp = ClipOp.intersect, bool doAntiAlias = true});
  void clipRRect(RRect rrect, {bool doAntiAlias = true});
  void clipPath(Path path, {bool doAntiAlias = true});
  Rect getLocalClipBounds();
  Rect getDestinationClipBounds();
  void drawColor(Color color, BlendMode blendMode);
  void drawLine(Offset p1, Offset p2, Paint paint);
  void drawPaint(Paint paint);
  void drawRect(Rect rect, Paint paint);
  void drawRRect(RRect rrect, Paint paint);
  void drawDRRect(RRect outer, RRect inner, Paint paint);
  void drawOval(Rect rect, Paint paint);
  void drawCircle(Offset c, double radius, Paint paint);
  void drawArc(Rect rect, double startAngle, double sweepAngle, bool useCenter, Paint paint);
  void drawPath(Path path, Paint paint);
  void drawImage(Image image, Offset offset, Paint paint);
  void drawImageRect(Image image, Rect src, Rect dst, Paint paint);
  void drawImageNine(Image image, Rect center, Rect dst, Paint paint);
  void drawPicture(Picture picture);
  void drawParagraph(Paragraph paragraph, Offset offset);
  void drawPoints(PointMode pointMode, List<Offset> points, Paint paint);
  void drawRawPoints(PointMode pointMode, Float32List points, Paint paint);

  void drawVertices(Vertices vertices, BlendMode blendMode, Paint paint);
  void drawAtlas(
    Image atlas,
    List<RSTransform> transforms,
    List<Rect> rects,
    List<Color>? colors,
    BlendMode? blendMode,
    Rect? cullRect,
    Paint paint,
  );
  void drawRawAtlas(
    Image atlas,
    Float32List rstTransforms,
    Float32List rects,
    Int32List? colors,
    BlendMode? blendMode,
    Rect? cullRect,
    Paint paint,
  );
  void drawShadow(Path path, Color color, double elevation, bool transparentOccluder);
}

typedef PictureEventCallback = void Function(Picture picture);

abstract class Picture {
  static PictureEventCallback? onCreate;
  static PictureEventCallback? onDispose;
  Future<Image> toImage(int width, int height);
  Image toImageSync(int width, int height);
  void dispose();
  bool get debugDisposed;
  int get approximateBytesUsed;
}

enum PathFillType { nonZero, evenOdd }
// Must be kept in sync with SkPathOp

enum PathOperation { difference, intersect, union, xor, reverseDifference }

abstract class PictureRasterizationException implements Exception {
  String get message;
  StackTrace? stack;
}
