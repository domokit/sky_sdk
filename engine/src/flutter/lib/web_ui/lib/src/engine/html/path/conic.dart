// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;
import 'dart:typed_data';

import 'package:ui/ui.dart' as ui;

import 'path_utils.dart';

/// Converts conic curve to a list of quadratic curves for rendering on
/// canvas or conversion to svg.
///
/// See "High order approximation of conic sections by quadratic splines"
/// by Michael Floater, 1993.
/// Skia implementation reference:
/// https://github.com/google/skia/blob/main/src/core/SkGeometry.cpp
class Conic {
  Conic(this.p0x, this.p0y, this.p1x, this.p1y, this.p2x, this.p2y, this.fW);

  double p0x, p0y, p1x, p1y, p2x, p2y;
  final double fW;
  static const int _maxSubdivisionCount = 5;

  /// Returns array of points for the approximation of the conic as quad(s).
  ///
  /// First offset is start Point. Each pair of offsets after are quadratic
  /// control and end points.
  List<ui.Offset> toQuads() {
    final List<ui.Offset> pointList = <ui.Offset>[];
    // This value specifies error bound.
    const double conicTolerance = 1.0 / 4.0;

    // Based on error bound, compute how many times we should subdivide
    final int subdivideCount = _computeSubdivisionCount(conicTolerance);

    // Split conic into quads, writes quad coordinates into [_pointList] and
    // returns number of quads.
    assert(subdivideCount >= 0 && subdivideCount <= _maxSubdivisionCount);
    int quadCount = 1 << subdivideCount;
    bool skipSubdivide = false;
    pointList.add(ui.Offset(p0x, p0y));
    if (subdivideCount == _maxSubdivisionCount) {
      // We have an extreme number of quads, chop this conic and check if
      // it generates a pair of lines, in which case we should not subdivide.
      final _ConicPair dst = _ConicPair();
      _chop(dst);
      final Conic conic0 = dst.first!;
      final Conic conic1 = dst.second!;
      // If this chop generates pair of lines no need to subdivide.
      if (conic0.p1x == conic0.p2x &&
          conic0.p1y == conic0.p2y &&
          conic1.p0x == conic1.p1x &&
          conic1.p0y == conic1.p1y) {
        final ui.Offset controlPointOffset = ui.Offset(conic0.p1x, conic0.p1y);
        pointList.add(controlPointOffset);
        pointList.add(controlPointOffset);
        pointList.add(controlPointOffset);
        pointList.add(ui.Offset(conic1.p2x, conic1.p2y));
        quadCount = 2;
        skipSubdivide = true;
      }
    }
    if (!skipSubdivide) {
      _subdivide(this, subdivideCount, pointList);
    }

    // If there are any non-finite generated points, pin to middle of hull.
    final int pointCount = 2 * quadCount + 1;
    bool hasNonFinitePoints = false;
    for (int p = 0; p < pointCount; ++p) {
      if (pointList[p].dx.isNaN || pointList[p].dy.isNaN) {
        hasNonFinitePoints = true;
        break;
      }
    }
    if (hasNonFinitePoints) {
      for (int p = 1; p < pointCount - 1; ++p) {
        pointList[p] = ui.Offset(p1x, p1y);
      }
    }
    return pointList;
  }

  // Subdivides a conic and writes to points list.
  static void _subdivide(Conic src, int level, List<ui.Offset> pointList) {
    assert(level >= 0);
    if (0 == level) {
      // At lowest subdivision point, copy control point and end point to
      // target.
      pointList.add(ui.Offset(src.p1x, src.p1y));
      pointList.add(ui.Offset(src.p2x, src.p2y));
      return;
    }
    final _ConicPair dst = _ConicPair();
    src._chop(dst);
    final Conic conic0 = dst.first!;
    final Conic conic1 = dst.second!;
    final double startY = src.p0y;
    final double endY = src.p2y;
    final double cpY = src.p1y;
    if (SPath.between(startY, cpY, endY)) {
      // Ensure that chopped conics maintain their y-order.
      final double midY = conic0.p2y;
      if (!SPath.between(startY, midY, endY)) {
        // The computed midpoint is outside end points, move it to
        // closer one.
        final double closerY = (midY - startY).abs() < (midY - endY).abs() ? startY : endY;
        conic0.p2y = conic1.p0y = closerY;
      }
      if (!SPath.between(startY, conic0.p1y, conic0.p2y)) {
        // First control point not between start and end points, move it
        // to start.
        conic0.p1y = startY;
      }
      if (!SPath.between(conic1.p0y, conic1.p1y, endY)) {
        // Second control point not between start and end points, move it
        // to end.
        conic1.p1y = endY;
      }
      // Verify that conics points are ordered.
      assert(SPath.between(startY, conic0.p1y, conic0.p2y));
      assert(SPath.between(conic0.p1y, conic0.p2y, conic1.p1y));
      assert(SPath.between(conic0.p2y, conic1.p1y, endY));
    }
    --level;
    _subdivide(conic0, level, pointList);
    _subdivide(conic1, level, pointList);
  }

  static double _subdivideWeightValue(double w) {
    return math.sqrt(0.5 + w * 0.5);
  }

  // Splits conic into 2 parts based on weight.
  void _chop(_ConicPair pair) {
    final double scale = 1.0 / (1.0 + fW);
    final double newW = _subdivideWeightValue(fW);
    final ui.Offset wp1 = ui.Offset(fW * p1x, fW * p1y);
    ui.Offset m = ui.Offset(
      (p0x + (2 * wp1.dx) + p2x) * scale * 0.5,
      (p0y + 2 * wp1.dy + p2y) * scale * 0.5,
    );
    if (m.dx.isNaN || m.dy.isNaN) {
      final double w2 = fW * 2;
      final double scaleHalf = 1.0 / (1 + fW) * 0.5;
      m = ui.Offset((p0x + (w2 * p1x) + p2x) * scaleHalf, (p0y + (w2 * p1y) + p2y) * scaleHalf);
    }
    pair.first = Conic(p0x, p0y, (p0x + wp1.dx) * scale, (p0y + wp1.dy) * scale, m.dx, m.dy, newW);
    pair.second = Conic(m.dx, m.dy, (p2x + wp1.dx) * scale, (p2y + wp1.dy) * scale, p2x, p2y, newW);
  }

  void chopAtYExtrema(List<Conic> dst) {
    final double? t = _findYExtrema();
    if (t == null) {
      dst.add(this);
      return;
    }
    if (!_chopAt(t, dst, cleanupMiddle: true)) {
      // If chop can't return finite values, don't chop.
      dst.add(this);
      return;
    }
  }

  ///////////////////////////////////////////////////////////////////////////////
  //
  // NURB representation for conics.  Helpful explanations at:
  //
  // http://citeseerx.ist.psu.edu/viewdoc/
  //   download?doi=10.1.1.44.5740&rep=rep1&type=ps
  // and
  // http://www.cs.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/NURBS/RB-conics.html
  //
  // F = (A (1 - t)^2 + C t^2 + 2 B (1 - t) t w)
  //     ------------------------------------------
  //         ((1 - t)^2 + t^2 + 2 (1 - t) t w)
  //
  //   = {t^2 (P0 + P2 - 2 P1 w), t (-2 P0 + 2 P1 w), P0}
  //     ------------------------------------------------
  //             {t^2 (2 - 2 w), t (-2 + 2 w), 1}
  //
  // F' = 2 (C t (1 + t (-1 + w)) - A (-1 + t) (t (-1 + w) - w) + B (1 - 2 t) w)
  //
  //  t^2 : (2 P0 - 2 P2 - 2 P0 w + 2 P2 w)
  //  t^1 : (-2 P0 + 2 P2 + 4 P0 w - 4 P1 w)
  //  t^0 : -2 P0 w + 2 P1 w
  //
  //  We disregard magnitude, so we can freely ignore the denominator of F', and
  //  divide the numerator by 2
  //
  //    coeff[0] for t^2
  //    coeff[1] for t^1
  //    coeff[2] for t^0
  //
  double? _findYExtrema() {
    final double p20 = p2y - p0y;
    final double p10 = p1y - p0y;
    final double wP10 = fW * p10;
    final double coeff0 = fW * p20 - p20;
    final double coeff1 = p20 - 2 * wP10;
    final double coeff2 = wP10;
    final QuadRoots quadRoots = QuadRoots();
    final int rootCount = quadRoots.findRoots(coeff0, coeff1, coeff2);
    assert(rootCount == 0 || rootCount == 1);
    if (rootCount == 1) {
      return quadRoots.root0;
    }
    return null;
  }

  bool _chopAt(double t, List<Conic> dst, {bool cleanupMiddle = false}) {
    // Map conic to 3D.
    final double tx0 = p0x;
    final double ty0 = p0y;
    const double tz0 = 1;
    final double tx1 = p1x * fW;
    final double ty1 = p1y * fW;
    final double tz1 = fW;
    final double tx2 = p2x;
    final double ty2 = p2y;
    const double tz2 = 1;
    // Now interpolate each dimension.
    final double dx0 = tx0 + (tx1 - tx0) * t;
    final double dx2 = tx1 + (tx2 - tx1) * t;
    final double dx1 = dx0 + (dx2 - dx0) * t;
    final double dy0 = ty0 + (ty1 - ty0) * t;
    final double dy2 = ty1 + (ty2 - ty1) * t;
    final double dy1 = dy0 + (dy2 - dy0) * t;
    final double dz0 = tz0 + (tz1 - tz0) * t;
    final double dz2 = tz1 + (tz2 - tz1) * t;
    final double dz1 = dz0 + (dz2 - dz0) * t;
    // Compute new weights.
    final double root = math.sqrt(dz1);
    if (SPath.nearlyEqual(root, 0)) {
      return false;
    }
    final double w0 = dz0 / root;
    final double w2 = dz2 / root;
    if (SPath.nearlyEqual(dz0, 0) || SPath.nearlyEqual(dz1, 0) || SPath.nearlyEqual(dz2, 0)) {
      return false;
    }
    // Now we can construct the 2 conics by projecting 3D down to 2D.
    final double chopPointX = dx1 / dz1;
    final double chopPointY = dy1 / dz1;

    double cp0y = dy0 / dz0;
    double cp1y = dy2 / dz2;
    if (cleanupMiddle) {
      // Clean-up the middle, since we know t was meant to be at
      // an Y-extrema.
      cp0y = chopPointY;
      cp1y = chopPointY;
    }

    final Conic conic0 = Conic(p0x, p0y, dx0 / dz0, cp0y, chopPointX, chopPointY, w0);
    final Conic conic1 = Conic(chopPointX, chopPointY, dx2 / dz2, cp1y, p2x, p2y, w2);
    dst.add(conic0);
    dst.add(conic1);
    return true;
  }

  /// Computes number of binary subdivisions of the curve given
  /// the tolerance.
  ///
  /// The number of subdivisions never exceed [_maxSubdivisionCount].
  int _computeSubdivisionCount(double tolerance) {
    assert(tolerance.isFinite);
    // Expecting finite coordinates.
    assert(
      p0x.isFinite && p1x.isFinite && p2x.isFinite && p0y.isFinite && p1y.isFinite && p2y.isFinite,
    );
    if (tolerance < 0) {
      return 0;
    }
    // See "High order approximation of conic sections by quadratic splines"
    // by Michael Floater, 1993.
    // Error bound e0 = |a| |p0 - 2p1 + p2| / 4(2 + a).
    final double a = fW - 1;
    final double k = a / (4.0 * (2.0 + a));
    final double x = k * (p0x - 2 * p1x + p2x);
    final double y = k * (p0y - 2 * p1y + p2y);

    double error = math.sqrt(x * x + y * y);
    int pow2 = 0;
    for (; pow2 < _maxSubdivisionCount; ++pow2) {
      if (error <= tolerance) {
        break;
      }
      error *= 0.25;
    }
    return pow2;
  }

  ui.Offset evalTangentAt(double t) {
    // The derivative equation returns a zero tangent vector when t is 0 or 1,
    // and the control point is equal to the end point.
    // In this case, use the conic endpoints to compute the tangent.
    if ((t == 0 && p0x == p1x && p0y == p1y) || (t == 1 && p1x == p2x && p1y == p2y)) {
      return ui.Offset(p2x - p0x, p2y - p0y);
    }
    final double p20x = p2x - p0x;
    final double p20y = p2y - p0y;
    final double p10x = p1x - p0x;
    final double p10y = p1y - p0y;

    final double cx = fW * p10x;
    final double cy = fW * p10y;
    final double ax = fW * p20x - p20x;
    final double ay = fW * p20y - p20y;
    final double bx = p20x - cx - cx;
    final double by = p20y - cy - cy;
    final SkQuadCoefficients quadC = SkQuadCoefficients(ax, ay, bx, by, cx, cy);
    return ui.Offset(quadC.evalX(t), quadC.evalY(t));
  }

  static double evalNumerator(double p0, double p1, double p2, double w, double t) {
    assert(t >= 0 && t <= 1);
    final double src2w = p1 * w;
    final double C = p0;
    final double A = p2 - 2 * src2w + C;
    final double B = 2 * (src2w - C);
    return polyEval(A, B, C, t);
  }

  static double evalDenominator(double w, double t) {
    final double B = 2 * (w - 1);
    const double C = 1;
    final double A = -B;
    return polyEval(A, B, C, t);
  }
}

class QuadBounds {
  double minX = 0;
  double minY = 0;
  double maxX = 0;
  double maxY = 0;

  void calculateBounds(Float32List points, int pointIndex) {
    final double x1 = points[pointIndex++];
    final double y1 = points[pointIndex++];
    final double cpX = points[pointIndex++];
    final double cpY = points[pointIndex++];
    final double x2 = points[pointIndex++];
    final double y2 = points[pointIndex++];

    minX = math.min(x1, x2);
    minY = math.min(y1, y2);
    maxX = math.max(x1, x2);
    maxY = math.max(y1, y2);

    // At extrema's derivative = 0.
    // Solve for
    // -2x1+2tx1 + 2cpX + 4tcpX + 2tx2 = 0
    // -2x1 + 2cpX +2t(x1 + 2cpX + x2) = 0
    // t = (x1 - cpX) / (x1 - 2cpX + x2)
    double denom = x1 - (2 * cpX) + x2;
    if (denom.abs() > SPath.scalarNearlyZero) {
      final double t1 = (x1 - cpX) / denom;
      if ((t1 >= 0) && (t1 <= 1.0)) {
        // Solve (x,y) for curve at t = tx to find extrema
        final double tprime = 1.0 - t1;
        final double extremaX = (tprime * tprime * x1) + (2 * t1 * tprime * cpX) + (t1 * t1 * x2);
        final double extremaY = (tprime * tprime * y1) + (2 * t1 * tprime * cpY) + (t1 * t1 * y2);
        // Expand bounds.
        minX = math.min(minX, extremaX);
        maxX = math.max(maxX, extremaX);
        minY = math.min(minY, extremaY);
        maxY = math.max(maxY, extremaY);
      }
    }
    // Now calculate dy/dt = 0
    denom = y1 - (2 * cpY) + y2;
    if (denom.abs() > SPath.scalarNearlyZero) {
      final double t2 = (y1 - cpY) / denom;
      if ((t2 >= 0) && (t2 <= 1.0)) {
        final double tprime2 = 1.0 - t2;
        final double extrema2X =
            (tprime2 * tprime2 * x1) + (2 * t2 * tprime2 * cpX) + (t2 * t2 * x2);
        final double extrema2Y =
            (tprime2 * tprime2 * y1) + (2 * t2 * tprime2 * cpY) + (t2 * t2 * y2);
        // Expand bounds.
        minX = math.min(minX, extrema2X);
        maxX = math.max(maxX, extrema2X);
        minY = math.min(minY, extrema2Y);
        maxY = math.max(maxY, extrema2Y);
      }
    }
  }
}

class ConicBounds {
  double minX = 0;
  double minY = 0;
  double maxX = 0;
  double maxY = 0;
  void calculateBounds(Float32List points, double w, int pointIndex) {
    final double x1 = points[pointIndex++];
    final double y1 = points[pointIndex++];
    final double cpX = points[pointIndex++];
    final double cpY = points[pointIndex++];
    final double x2 = points[pointIndex++];
    final double y2 = points[pointIndex++];

    minX = math.min(x1, x2);
    minY = math.min(y1, y2);
    maxX = math.max(x1, x2);
    maxY = math.max(y1, y2);

    // {t^2 (P0 + P2 - 2 P1 w), t (-2 P0 + 2 P1 w), P0}
    // ------------------------------------------------
    //       {t^2 (2 - 2 w), t (-2 + 2 w), 1}
    // Calculate coefficients and solve root.
    final QuadRoots roots = QuadRoots();
    final double p20x = x2 - x1;
    final double p10x = cpX - x1;
    final double wP10x = w * p10x;
    final double ax = w * p20x - p20x;
    final double bx = p20x - 2 * wP10x;
    final double cx = wP10x;
    int n = roots.findRoots(ax, bx, cx);
    if (n != 0) {
      final double t1 = roots.root0!;
      if ((t1 >= 0) && (t1 <= 1.0)) {
        final double denom = Conic.evalDenominator(w, t1);
        double numerator = Conic.evalNumerator(x1, cpX, x2, w, t1);
        final double extremaX = numerator / denom;
        numerator = Conic.evalNumerator(y1, cpY, y2, w, t1);
        final double extremaY = numerator / denom;
        // Expand bounds.
        minX = math.min(minX, extremaX);
        maxX = math.max(maxX, extremaX);
        minY = math.min(minY, extremaY);
        maxY = math.max(maxY, extremaY);
      }
    }
    final double p20y = y2 - y1;
    final double p10y = cpY - y1;
    final double wP10y = w * p10y;
    final double a = w * p20y - p20y;
    final double b = p20y - 2 * wP10y;
    final double c = wP10y;
    n = roots.findRoots(a, b, c);

    if (n != 0) {
      final double t2 = roots.root0!;
      if ((t2 >= 0) && (t2 <= 1.0)) {
        final double denom = Conic.evalDenominator(w, t2);
        double numerator = Conic.evalNumerator(x1, cpX, x2, w, t2);
        final double extrema2X = numerator / denom;
        numerator = Conic.evalNumerator(y1, cpY, y2, w, t2);
        final double extrema2Y = numerator / denom;
        // Expand bounds.
        minX = math.min(minX, extrema2X);
        maxX = math.max(maxX, extrema2X);
        minY = math.min(minY, extrema2Y);
        maxY = math.max(maxY, extrema2Y);
      }
    }
  }
}

class _ConicPair {
  Conic? first;
  Conic? second;
}
