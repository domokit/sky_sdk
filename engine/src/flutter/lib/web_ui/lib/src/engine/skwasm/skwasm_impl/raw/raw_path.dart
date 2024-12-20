// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@DefaultAsset('skwasm')
library skwasm_impl;

import 'dart:ffi';

import 'package:ui/src/engine/skwasm/skwasm_impl.dart';

final class RawPath extends Opaque {}

typedef PathHandle = Pointer<RawPath>;

@Native<PathHandle Function()>(symbol: 'path_create', isLeaf: true)
external PathHandle pathCreate();

@Native<Void Function(PathHandle)>(symbol: 'path_dispose', isLeaf: true)
external void pathDispose(PathHandle path);

@Native<PathHandle Function(PathHandle)>(symbol: 'path_copy', isLeaf: true)
external PathHandle pathCopy(PathHandle path);

@Native<Void Function(PathHandle, Int)>(symbol: 'path_setFillType', isLeaf: true)
external void pathSetFillType(PathHandle path, int fillType);

@Native<Int Function(PathHandle)>(symbol: 'path_getFillType', isLeaf: true)
external int pathGetFillType(PathHandle path);

@Native<Void Function(PathHandle, Float, Float)>(symbol: 'path_moveTo', isLeaf: true)
external void pathMoveTo(PathHandle path, double x, double y);

@Native<Void Function(PathHandle, Float, Float)>(symbol: 'path_relativeMoveTo', isLeaf: true)
external void pathRelativeMoveTo(PathHandle path, double x, double y);

@Native<Void Function(PathHandle, Float, Float)>(symbol: 'path_lineTo', isLeaf: true)
external void pathLineTo(PathHandle path, double x, double y);

@Native<Void Function(PathHandle, Float, Float)>(symbol: 'path_relativeLineTo', isLeaf: true)
external void pathRelativeLineTo(PathHandle path, double x, double y);

@Native<Void Function(PathHandle, Float, Float, Float, Float)>(
  symbol: 'path_quadraticBezierTo',
  isLeaf: true,
)
external void pathQuadraticBezierTo(PathHandle path, double x1, double y1, double x2, double y2);

@Native<Void Function(PathHandle, Float, Float, Float, Float)>(
  symbol: 'path_relativeQuadraticBezierTo',
  isLeaf: true,
)
external void pathRelativeQuadraticBezierTo(
  PathHandle path,
  double x1,
  double y1,
  double x2,
  double y2,
);

@Native<Void Function(PathHandle, Float, Float, Float, Float, Float, Float)>(
  symbol: 'path_cubicTo',
  isLeaf: true,
)
external void pathCubicTo(
  PathHandle path,
  double x1,
  double y1,
  double x2,
  double y2,
  double x3,
  double y3,
);

@Native<Void Function(PathHandle, Float, Float, Float, Float, Float, Float)>(
  symbol: 'path_relativeCubicTo',
  isLeaf: true,
)
external void pathRelativeCubicTo(
  PathHandle path,
  double x1,
  double y1,
  double x2,
  double y2,
  double x3,
  double y3,
);

@Native<Void Function(PathHandle, Float, Float, Float, Float, Float)>(
  symbol: 'path_conicTo',
  isLeaf: true,
)
external void pathConicTo(PathHandle path, double x1, double y1, double x2, double y2, double w);

@Native<Void Function(PathHandle, Float, Float, Float, Float, Float)>(
  symbol: 'path_relativeConicTo',
  isLeaf: true,
)
external void pathRelativeConicTo(
  PathHandle path,
  double x1,
  double y1,
  double x2,
  double y2,
  double w,
);

@Native<Void Function(PathHandle, RawRect, Float, Float, Bool)>(
  symbol: 'path_arcToOval',
  isLeaf: true,
)
external void pathArcToOval(
  PathHandle path,
  RawRect rect,
  double startAngle,
  double sweepAngle,
  bool forceMoveto,
);

@Native<Void Function(PathHandle, Float, Float, Float, Int, Int, Float, Float)>(
  symbol: 'path_arcToRotated',
  isLeaf: true,
)
external void pathArcToRotated(
  PathHandle path,
  double rx,
  double ry,
  double xAxisRotate,
  int arcSize,
  int pathDirection,
  double x,
  double y,
);

@Native<Void Function(PathHandle, Float, Float, Float, Int, Int, Float, Float)>(
  symbol: 'path_relativeArcToRotated',
  isLeaf: true,
)
external void pathRelativeArcToRotated(
  PathHandle path,
  double rx,
  double ry,
  double xAxisRotate,
  int arcSize,
  int pathDirection,
  double x,
  double y,
);

@Native<Void Function(PathHandle, RawRect)>(symbol: 'path_addRect', isLeaf: true)
external void pathAddRect(PathHandle path, RawRect oval);

@Native<Void Function(PathHandle, RawRect)>(symbol: 'path_addOval', isLeaf: true)
external void pathAddOval(PathHandle path, RawRect oval);

@Native<Void Function(PathHandle, RawRect, Float, Float)>(symbol: 'path_addArc', isLeaf: true)
external void pathAddArc(
  PathHandle path,
  RawRect ovalRect,
  double startAngleDegrees,
  double sweepAngleDegrees,
);

@Native<Void Function(PathHandle, RawPointArray, Int, Bool)>(
  symbol: 'path_addPolygon',
  isLeaf: true,
)
external void pathAddPolygon(PathHandle path, RawPointArray points, int pointCount, bool close);

@Native<Void Function(PathHandle, RawRRect)>(symbol: 'path_addRRect', isLeaf: true)
external void pathAddRRect(PathHandle path, RawRRect rrectValues);

@Native<Void Function(PathHandle, PathHandle, RawMatrix33, Bool)>(
  symbol: 'path_addPath',
  isLeaf: true,
)
external void pathAddPath(PathHandle path, PathHandle other, RawMatrix33 matrix33, bool extendPath);

@Native<Void Function(PathHandle)>(symbol: 'path_close', isLeaf: true)
external void pathClose(PathHandle path);

@Native<Void Function(PathHandle)>(symbol: 'path_reset', isLeaf: true)
external void pathReset(PathHandle path);

@Native<Bool Function(PathHandle, Float, Float)>(symbol: 'path_contains', isLeaf: true)
external bool pathContains(PathHandle path, double x, double y);

@Native<Void Function(PathHandle, RawMatrix33)>(symbol: 'path_transform', isLeaf: true)
external void pathTransform(PathHandle path, RawMatrix33 matrix33);

@Native<Void Function(PathHandle, RawRect)>(symbol: 'path_getBounds', isLeaf: true)
external void pathGetBounds(PathHandle path, RawRect outRect);

@Native<PathHandle Function(Int, PathHandle, PathHandle)>(symbol: 'path_combine', isLeaf: true)
external PathHandle pathCombine(int operation, PathHandle path1, PathHandle path2);

@Native<SkStringHandle Function(PathHandle)>(symbol: 'path_getSvgString', isLeaf: true)
external SkStringHandle pathGetSvgString(PathHandle path);
