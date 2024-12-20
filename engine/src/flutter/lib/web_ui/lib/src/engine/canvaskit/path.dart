// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;
import 'dart:typed_data';

import 'package:ui/ui.dart' as ui;

import '../scene_painting.dart';
import '../vector_math.dart';
import 'canvaskit_api.dart';
import 'native_memory.dart';
import 'path_metrics.dart';

/// An implementation of [ui.Path] which is backed by an `SkPath`.
///
/// The `SkPath` is required for `CkCanvas` methods which take a path.
class CkPath implements ScenePath {
  factory CkPath() {
    final SkPath skPath = SkPath();
    skPath.setFillType(toSkFillType(ui.PathFillType.nonZero));
    return CkPath._(skPath, ui.PathFillType.nonZero);
  }

  factory CkPath.from(CkPath other) {
    final SkPath skPath = other.skiaObject.copy();
    skPath.setFillType(toSkFillType(other._fillType));
    return CkPath._(skPath, other._fillType);
  }

  factory CkPath.fromSkPath(SkPath skPath, ui.PathFillType fillType) {
    skPath.setFillType(toSkFillType(fillType));
    return CkPath._(skPath, fillType);
  }

  CkPath._(SkPath nativeObject, this._fillType) {
    _ref = UniqueRef<SkPath>(this, nativeObject, 'Path');
  }

  late final UniqueRef<SkPath> _ref;

  SkPath get skiaObject => _ref.nativeObject;

  ui.PathFillType _fillType;

  @override
  ui.PathFillType get fillType => _fillType;

  @override
  set fillType(ui.PathFillType newFillType) {
    if (_fillType == newFillType) {
      return;
    }
    _fillType = newFillType;
    skiaObject.setFillType(toSkFillType(newFillType));
  }

  @override
  void addArc(ui.Rect oval, double startAngle, double sweepAngle) {
    const double toDegrees = 180.0 / math.pi;
    skiaObject.addArc(toSkRect(oval), startAngle * toDegrees, sweepAngle * toDegrees);
  }

  @override
  void addOval(ui.Rect oval) {
    skiaObject.addOval(toSkRect(oval), false, 1);
  }

  @override
  void addPath(ui.Path path, ui.Offset offset, {Float64List? matrix4}) {
    List<double> skMatrix;
    if (matrix4 == null) {
      skMatrix = toSkMatrixFromFloat32(
        Matrix4.translationValues(offset.dx, offset.dy, 0.0).storage,
      );
    } else {
      skMatrix = toSkMatrixFromFloat64(matrix4);
      skMatrix[2] += offset.dx;
      skMatrix[5] += offset.dy;
    }
    final CkPath otherPath = path as CkPath;
    skiaObject.addPath(
      otherPath.skiaObject,
      skMatrix[0],
      skMatrix[1],
      skMatrix[2],
      skMatrix[3],
      skMatrix[4],
      skMatrix[5],
      skMatrix[6],
      skMatrix[7],
      skMatrix[8],
      false,
    );
  }

  @override
  void addPolygon(List<ui.Offset> points, bool close) {
    final SkFloat32List encodedPoints = toMallocedSkPoints(points);
    skiaObject.addPoly(encodedPoints.toTypedArray(), close);
    free(encodedPoints);
  }

  @override
  void addRRect(ui.RRect rrect) {
    skiaObject.addRRect(toSkRRect(rrect), false);
  }

  @override
  void addRect(ui.Rect rect) {
    skiaObject.addRect(toSkRect(rect));
  }

  @override
  void arcTo(ui.Rect rect, double startAngle, double sweepAngle, bool forceMoveTo) {
    const double toDegrees = 180.0 / math.pi;
    skiaObject.arcToOval(
      toSkRect(rect),
      startAngle * toDegrees,
      sweepAngle * toDegrees,
      forceMoveTo,
    );
  }

  @override
  void arcToPoint(
    ui.Offset arcEnd, {
    ui.Radius radius = ui.Radius.zero,
    double rotation = 0.0,
    bool largeArc = false,
    bool clockwise = true,
  }) {
    skiaObject.arcToRotated(
      radius.x,
      radius.y,
      rotation,
      !largeArc,
      !clockwise,
      arcEnd.dx,
      arcEnd.dy,
    );
  }

  @override
  void close() {
    skiaObject.close();
  }

  @override
  ui.PathMetrics computeMetrics({bool forceClosed = false}) {
    return CkPathMetrics(this, forceClosed);
  }

  @override
  void conicTo(double x1, double y1, double x2, double y2, double w) {
    skiaObject.conicTo(x1, y1, x2, y2, w);
  }

  @override
  bool contains(ui.Offset point) {
    return skiaObject.contains(point.dx, point.dy);
  }

  @override
  void cubicTo(double x1, double y1, double x2, double y2, double x3, double y3) {
    skiaObject.cubicTo(x1, y1, x2, y2, x3, y3);
  }

  @override
  void extendWithPath(ui.Path path, ui.Offset offset, {Float64List? matrix4}) {
    List<double> skMatrix;
    if (matrix4 == null) {
      skMatrix = toSkMatrixFromFloat32(
        Matrix4.translationValues(offset.dx, offset.dy, 0.0).storage,
      );
    } else {
      skMatrix = toSkMatrixFromFloat64(matrix4);
      skMatrix[2] += offset.dx;
      skMatrix[5] += offset.dy;
    }
    final CkPath otherPath = path as CkPath;
    skiaObject.addPath(
      otherPath.skiaObject,
      skMatrix[0],
      skMatrix[1],
      skMatrix[2],
      skMatrix[3],
      skMatrix[4],
      skMatrix[5],
      skMatrix[6],
      skMatrix[7],
      skMatrix[8],
      true,
    );
  }

  @override
  ui.Rect getBounds() => fromSkRect(skiaObject.getBounds());

  @override
  void lineTo(double x, double y) {
    skiaObject.lineTo(x, y);
  }

  @override
  void moveTo(double x, double y) {
    skiaObject.moveTo(x, y);
  }

  @override
  void quadraticBezierTo(double x1, double y1, double x2, double y2) {
    skiaObject.quadTo(x1, y1, x2, y2);
  }

  @override
  void relativeArcToPoint(
    ui.Offset arcEndDelta, {
    ui.Radius radius = ui.Radius.zero,
    double rotation = 0.0,
    bool largeArc = false,
    bool clockwise = true,
  }) {
    skiaObject.rArcTo(
      radius.x,
      radius.y,
      rotation,
      !largeArc,
      !clockwise,
      arcEndDelta.dx,
      arcEndDelta.dy,
    );
  }

  @override
  void relativeConicTo(double x1, double y1, double x2, double y2, double w) {
    skiaObject.rConicTo(x1, y1, x2, y2, w);
  }

  @override
  void relativeCubicTo(double x1, double y1, double x2, double y2, double x3, double y3) {
    skiaObject.rCubicTo(x1, y1, x2, y2, x3, y3);
  }

  @override
  void relativeLineTo(double dx, double dy) {
    skiaObject.rLineTo(dx, dy);
  }

  @override
  void relativeMoveTo(double dx, double dy) {
    skiaObject.rMoveTo(dx, dy);
  }

  @override
  void relativeQuadraticBezierTo(double x1, double y1, double x2, double y2) {
    skiaObject.rQuadTo(x1, y1, x2, y2);
  }

  @override
  void reset() {
    // Only reset the local field. Skia will reset its internal state via
    // SkPath.reset() below.
    _fillType = ui.PathFillType.nonZero;
    skiaObject.reset();
  }

  @override
  CkPath shift(ui.Offset offset) {
    // `SkPath.transform` mutates the existing path, so create a copy and call
    // `transform` on the copy.
    final SkPath shiftedPath = skiaObject.copy();
    shiftedPath.transform(1.0, 0.0, offset.dx, 0.0, 1.0, offset.dy, 0.0, 0.0, 1.0);
    return CkPath.fromSkPath(shiftedPath, _fillType);
  }

  static CkPath combine(ui.PathOperation operation, ui.Path uiPath1, ui.Path uiPath2) {
    final CkPath path1 = uiPath1 as CkPath;
    final CkPath path2 = uiPath2 as CkPath;
    final SkPath newPath = canvasKit.Path.MakeFromOp(
      path1.skiaObject,
      path2.skiaObject,
      toSkPathOp(operation),
    );
    return CkPath.fromSkPath(newPath, path1._fillType);
  }

  @override
  ui.Path transform(Float64List matrix4) {
    final SkPath newPath = skiaObject.copy();
    final Float32List m = toSkMatrixFromFloat64(matrix4);
    newPath.transform(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
    return CkPath.fromSkPath(newPath, _fillType);
  }

  @override
  String toSvgString() {
    return skiaObject.toSVGString();
  }

  /// Return `true` if this path contains no segments.
  bool get isEmpty {
    return skiaObject.isEmpty();
  }
}
