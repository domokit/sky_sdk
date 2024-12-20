// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';

import 'package:ui/ui.dart' as ui;

import 'conic.dart';
import 'cubic.dart';
import 'path_utils.dart';

/// Computes tangent at point x,y on a line.
void tangentLine(Float32List pts, double x, double y, List<ui.Offset> tangents) {
  final double y0 = pts[1];
  final double y1 = pts[3];
  if (!SPath.between(y0, y, y1)) {
    return;
  }
  final double x0 = pts[0];
  final double x1 = pts[2];
  if (!SPath.between(x0, x, x1)) {
    return;
  }
  final double dx = x1 - x0;
  final double dy = y1 - y0;
  if (!SPath.nearlyEqual((x - x0) * dy, dx * (y - y0))) {
    return;
  }
  tangents.add(ui.Offset(dx, dy));
}

/// Computes tangent at point x,y on a quadratic curve.
void tangentQuad(Float32List pts, double x, double y, List<ui.Offset> tangents) {
  final double y0 = pts[1];
  final double y1 = pts[3];
  final double y2 = pts[5];
  if (!SPath.between(y0, y, y1) && !SPath.between(y1, y, y2)) {
    return;
  }
  final double x0 = pts[0];
  final double x1 = pts[2];
  final double x2 = pts[4];
  if (!SPath.between(x0, x, x1) && !SPath.between(x1, x, x2)) {
    return;
  }
  final QuadRoots roots = QuadRoots();
  final int n = roots.findRoots(y0 - 2 * y1 + y2, 2 * (y1 - y0), y0 - y);
  for (int index = 0; index < n; ++index) {
    final double t = index == 0 ? roots.root0! : roots.root1!;
    final double C = x0;
    final double A = x2 - 2 * x1 + C;
    final double B = 2 * (x1 - C);
    final double xt = polyEval(A, B, C, t);
    if (!SPath.nearlyEqual(x, xt)) {
      continue;
    }
    tangents.add(_evalQuadTangentAt(x0, y0, x1, y1, x2, y2, t));
  }
}

ui.Offset _evalQuadTangentAt(
  double x0,
  double y0,
  double x1,
  double y1,
  double x2,
  double y2,
  double t,
) {
  // The derivative of a quad equation is 2(b - a +(a - 2b +c)t).
  // This returns a zero tangent vector when t is 0 or 1, and the control
  // point is equal to the end point. In this case, use the quad end points to
  // compute the tangent.

  if ((t == 0 && x0 == x1 && y0 == y1) || (t == 1 && x1 == x2 && y1 == y2)) {
    return ui.Offset(x2 - x0, y2 - y0);
  }
  assert(t >= 0 && t <= 1.0);

  final double bx = x1 - x0;
  final double by = y1 - y0;
  final double ax = x2 - x1 - bx;
  final double ay = y2 - y1 - by;
  final double tx = ax * t + bx;
  final double ty = ay * t + by;
  return ui.Offset(tx * 2, ty * 2);
}

/// Computes tangent at point x,y on a conic curve.
void tangentConic(Float32List pts, double x, double y, double weight, List<ui.Offset> tangents) {
  final double y0 = pts[1];
  final double y1 = pts[3];
  final double y2 = pts[5];
  if (!SPath.between(y0, y, y1) && !SPath.between(y1, y, y2)) {
    return;
  }
  final double x0 = pts[0];
  final double x1 = pts[2];
  final double x2 = pts[4];
  if (!SPath.between(x0, x, x1) && !SPath.between(x1, x, x2)) {
    return;
  }
  // Check extrema.
  double A = y2;
  double B = y1 * weight - y * weight + y;
  double C = y0;
  // A = a + c - 2*(b*w - yCept*w + yCept)
  A += C - 2 * B;
  // B = b*w - w * yCept + yCept - a
  B -= C;
  C -= y;
  final QuadRoots quadRoots = QuadRoots();
  final int n = quadRoots.findRoots(A, 2 * B, C);
  for (int index = 0; index < n; ++index) {
    final double t = index == 0 ? quadRoots.root0! : quadRoots.root1!;
    final double xt = Conic.evalNumerator(x0, x1, x2, weight, t) / Conic.evalDenominator(weight, t);
    if (!SPath.nearlyEqual(x, xt)) {
      continue;
    }
    final Conic conic = Conic(x0, y0, x1, y1, x2, y2, weight);
    tangents.add(conic.evalTangentAt(t));
  }
}

/// Computes tangent at point x,y on a cubic curve.
void tangentCubic(Float32List pts, double x, double y, List<ui.Offset> tangents) {
  final double y3 = pts[7];
  final double y0 = pts[1];
  final double y1 = pts[3];
  final double y2 = pts[5];
  if (!SPath.between(y0, y, y1) && !SPath.between(y1, y, y2) && !SPath.between(y2, y, y3)) {
    return;
  }
  final double x0 = pts[0];
  final double x1 = pts[2];
  final double x2 = pts[4];
  final double x3 = pts[6];
  if (!SPath.between(x0, x, x1) && !SPath.between(x1, x, x2) && !SPath.between(x2, x, x3)) {
    return;
  }
  final Float32List dst = Float32List(20);
  final int n = chopCubicAtYExtrema(pts, dst);
  for (int i = 0; i <= n; ++i) {
    final int bufferPos = i * 6;
    final double? t = chopMonoAtY(dst, i * 6, y);
    if (t == null) {
      continue;
    }
    final double xt = evalCubicPts(
      dst[bufferPos],
      dst[bufferPos + 2],
      dst[bufferPos + 4],
      dst[bufferPos + 6],
      t,
    );
    if (!SPath.nearlyEqual(x, xt)) {
      continue;
    }
    tangents.add(_evalCubicTangentAt(dst, bufferPos, t));
  }
}

ui.Offset _evalCubicTangentAt(Float32List points, int bufferPos, double t) {
  assert(t >= 0 && t <= 1.0);
  final double y3 = points[7 + bufferPos];
  final double y0 = points[1 + bufferPos];
  final double y1 = points[3 + bufferPos];
  final double y2 = points[5 + bufferPos];
  final double x0 = points[0 + bufferPos];
  final double x1 = points[2 + bufferPos];
  final double x2 = points[4 + bufferPos];
  final double x3 = points[6 + bufferPos];
  // The derivative equation returns a zero tangent vector when t is 0 or 1,
  // and the adjacent control point is equal to the end point. In this case,
  // use the next control point or the end points to compute the tangent.
  if ((t == 0 && x0 == x1 && y0 == y1) || (t == 1 && x2 == x3 && y2 == y3)) {
    double dx, dy;
    if (t == 0) {
      dx = x2 - x0;
      dy = y2 - y0;
    } else {
      dx = x3 - x1;
      dy = y3 - y1;
    }
    if (dx == 0 && dy == 0) {
      dx = x3 - x0;
      dy = y3 - y0;
    }
    return ui.Offset(dx, dy);
  } else {
    return _evalCubicDerivative(x0, y0, x1, y1, x2, y2, x3, y3, t);
  }
}

ui.Offset _evalCubicDerivative(
  double x0,
  double y0,
  double x1,
  double y1,
  double x2,
  double y2,
  double x3,
  double y3,
  double t,
) {
  final SkQuadCoefficients coeff = SkQuadCoefficients(
    x3 + 3 * (x1 - x2) - x0,
    y3 + 3 * (y1 - y2) - y0,
    2 * (x2 - (2 * x1) + x0),
    2 * (y2 - (2 * y1) + y0),
    x1 - x0,
    y1 - y0,
  );
  return ui.Offset(coeff.evalX(t), coeff.evalY(t));
}
