// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:ffi';
import 'dart:typed_data';

import 'package:ui/src/engine.dart';
import 'package:ui/src/engine/skwasm/skwasm_impl.dart';
import 'package:ui/ui.dart' as ui;

enum PathDirection { clockwise, counterClockwise }

enum PathArcSize { small, large }

class SkwasmPath extends SkwasmObjectWrapper<RawPath> implements ScenePath {
  factory SkwasmPath() {
    return SkwasmPath.fromHandle(pathCreate());
  }

  factory SkwasmPath.from(SkwasmPath source) {
    return SkwasmPath.fromHandle(pathCopy(source.handle));
  }

  SkwasmPath.fromHandle(PathHandle handle) : super(handle, _registry);

  static final SkwasmFinalizationRegistry<RawPath> _registry = SkwasmFinalizationRegistry<RawPath>(
    pathDispose,
  );

  @override
  ui.PathFillType get fillType => ui.PathFillType.values[pathGetFillType(handle)];

  @override
  set fillType(ui.PathFillType fillType) => pathSetFillType(handle, fillType.index);

  @override
  void moveTo(double x, double y) => pathMoveTo(handle, x, y);

  @override
  void relativeMoveTo(double x, double y) => pathRelativeMoveTo(handle, x, y);

  @override
  void lineTo(double x, double y) => pathLineTo(handle, x, y);

  @override
  void relativeLineTo(double x, double y) => pathRelativeLineTo(handle, x, y);

  @override
  void quadraticBezierTo(double x1, double y1, double x2, double y2) =>
      pathQuadraticBezierTo(handle, x1, y1, x2, y2);

  @override
  void relativeQuadraticBezierTo(double x1, double y1, double x2, double y2) =>
      pathRelativeQuadraticBezierTo(handle, x1, y1, x2, y2);

  @override
  void cubicTo(double x1, double y1, double x2, double y2, double x3, double y3) =>
      pathCubicTo(handle, x1, y1, x2, y2, x3, y3);

  @override
  void relativeCubicTo(double x1, double y1, double x2, double y2, double x3, double y3) =>
      pathRelativeCubicTo(handle, x1, y1, x2, y2, x3, y3);

  @override
  void conicTo(double x1, double y1, double x2, double y2, double w) =>
      pathConicTo(handle, x1, y1, x2, y2, w);

  @override
  void relativeConicTo(double x1, double y1, double x2, double y2, double w) =>
      pathRelativeConicTo(handle, x1, y1, x2, y2, w);

  @override
  void arcTo(ui.Rect rect, double startAngle, double sweepAngle, bool forceMoveTo) {
    withStackScope((StackScope s) {
      pathArcToOval(
        handle,
        s.convertRectToNative(rect),
        ui.toDegrees(startAngle),
        ui.toDegrees(sweepAngle),
        forceMoveTo,
      );
    });
  }

  @override
  void arcToPoint(
    ui.Offset arcEnd, {
    ui.Radius radius = ui.Radius.zero,
    double rotation = 0.0,
    bool largeArc = false,
    bool clockwise = true,
  }) {
    final PathArcSize arcSize = largeArc ? PathArcSize.large : PathArcSize.small;
    final PathDirection pathDirection =
        clockwise ? PathDirection.clockwise : PathDirection.counterClockwise;
    pathArcToRotated(
      handle,
      radius.x,
      radius.y,
      ui.toDegrees(rotation),
      arcSize.index,
      pathDirection.index,
      arcEnd.dx,
      arcEnd.dy,
    );
  }

  @override
  void relativeArcToPoint(
    ui.Offset arcEndDelta, {
    ui.Radius radius = ui.Radius.zero,
    double rotation = 0.0,
    bool largeArc = false,
    bool clockwise = true,
  }) {
    final PathArcSize arcSize = largeArc ? PathArcSize.large : PathArcSize.small;
    final PathDirection pathDirection =
        clockwise ? PathDirection.clockwise : PathDirection.counterClockwise;
    pathRelativeArcToRotated(
      handle,
      radius.x,
      radius.y,
      ui.toDegrees(rotation),
      arcSize.index,
      pathDirection.index,
      arcEndDelta.dx,
      arcEndDelta.dy,
    );
  }

  @override
  void addRect(ui.Rect rect) {
    withStackScope((StackScope s) {
      pathAddRect(handle, s.convertRectToNative(rect));
    });
  }

  @override
  void addOval(ui.Rect rect) {
    withStackScope((StackScope s) {
      pathAddOval(handle, s.convertRectToNative(rect));
    });
  }

  @override
  void addArc(ui.Rect rect, double startAngle, double sweepAngle) {
    withStackScope((StackScope s) {
      pathAddArc(
        handle,
        s.convertRectToNative(rect),
        ui.toDegrees(startAngle),
        ui.toDegrees(sweepAngle),
      );
    });
  }

  @override
  void addPolygon(List<ui.Offset> points, bool close) {
    withStackScope((StackScope s) {
      pathAddPolygon(handle, s.convertPointArrayToNative(points), points.length, close);
    });
  }

  @override
  void addRRect(ui.RRect rrect) {
    withStackScope((StackScope s) {
      pathAddRRect(handle, s.convertRRectToNative(rrect));
    });
  }

  @override
  void addPath(ui.Path path, ui.Offset offset, {Float64List? matrix4}) {
    _addPath(path, offset, false, matrix4: matrix4);
  }

  @override
  void extendWithPath(ui.Path path, ui.Offset offset, {Float64List? matrix4}) {
    _addPath(path, offset, true, matrix4: matrix4);
  }

  void _addPath(ui.Path path, ui.Offset offset, bool extend, {Float64List? matrix4}) {
    assert(path is SkwasmPath);
    withStackScope((StackScope s) {
      final Pointer<Float> convertedMatrix = s.convertMatrix4toSkMatrix(
        matrix4 ?? Matrix4.identity().toFloat64(),
      );
      convertedMatrix[2] += offset.dx;
      convertedMatrix[5] += offset.dy;
      pathAddPath(handle, (path as SkwasmPath).handle, convertedMatrix, extend);
    });
  }

  @override
  void close() => pathClose(handle);

  @override
  void reset() => pathReset(handle);

  @override
  bool contains(ui.Offset point) => pathContains(handle, point.dx, point.dy);

  @override
  ui.Path shift(ui.Offset offset) =>
      transform(Matrix4.translationValues(offset.dx, offset.dy, 0.0).toFloat64());

  @override
  ui.Path transform(Float64List matrix4) {
    return withStackScope((StackScope s) {
      final PathHandle newPathHandle = pathCopy(handle);
      pathTransform(newPathHandle, s.convertMatrix4toSkMatrix(matrix4));
      return SkwasmPath.fromHandle(newPathHandle);
    });
  }

  @override
  ui.Rect getBounds() {
    return withStackScope((StackScope s) {
      final Pointer<Float> rectBuffer = s.allocFloatArray(4);
      pathGetBounds(handle, rectBuffer);
      return s.convertRectFromNative(rectBuffer);
    });
  }

  static SkwasmPath combine(ui.PathOperation operation, SkwasmPath path1, SkwasmPath path2) =>
      SkwasmPath.fromHandle(pathCombine(operation.index, path1.handle, path2.handle));

  @override
  ui.PathMetrics computeMetrics({bool forceClosed = false}) {
    return SkwasmPathMetrics(path: this, forceClosed: forceClosed);
  }

  @override
  String toSvgString() {
    final SkStringHandle skString = pathGetSvgString(handle);
    final Pointer<Int8> buffer = skStringGetData(skString);
    final int length = skStringGetLength(skString);
    final List<int> characters = List<int>.generate(length, (int i) => buffer[i]);
    final String svgString = utf8.decode(characters);
    skStringFree(skString);
    return svgString;
  }
}
