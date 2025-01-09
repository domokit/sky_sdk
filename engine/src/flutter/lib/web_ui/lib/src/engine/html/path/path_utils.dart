// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;

import 'package:ui/ui.dart' as ui;

import 'path_ref.dart';

/// Mask used to keep track of types of verbs used in a path segment.
class SPathSegmentMask {
  static const int kLine_SkPathSegmentMask = 1 << 0;
  static const int kQuad_SkPathSegmentMask = 1 << 1;
  static const int kConic_SkPathSegmentMask = 1 << 2;
  static const int kCubic_SkPathSegmentMask = 1 << 3;
}

/// Types of path operations.
class SPathVerb {
  static const int kMove = 0; // 1 point
  static const int kLine = 1; // 2 points
  static const int kQuad = 2; // 3 points
  static const int kConic = 3; // 3 points + 1 weight
  static const int kCubic = 4; // 4 points
  static const int kClose = 5; // 0 points
}

abstract final class SPath {
  static const int kMoveVerb = SPathVerb.kMove;
  static const int kLineVerb = SPathVerb.kLine;
  static const int kQuadVerb = SPathVerb.kQuad;
  static const int kConicVerb = SPathVerb.kConic;
  static const int kCubicVerb = SPathVerb.kCubic;
  static const int kCloseVerb = SPathVerb.kClose;
  static const int kDoneVerb = SPathVerb.kClose + 1;

  static const int kLineSegmentMask = SPathSegmentMask.kLine_SkPathSegmentMask;
  static const int kQuadSegmentMask = SPathSegmentMask.kQuad_SkPathSegmentMask;
  static const int kConicSegmentMask = SPathSegmentMask.kConic_SkPathSegmentMask;
  static const int kCubicSegmentMask = SPathSegmentMask.kCubic_SkPathSegmentMask;

  static const double scalarNearlyZero = 1.0 / (1 << 12);

  /// Square root of 2 divided by 2. Useful for sin45 = cos45 = 1/sqrt(2).
  static const double scalarRoot2Over2 = 0.707106781;

  /// True if (a <= b <= c) || (a >= b >= c)
  static bool between(double a, double b, double c) {
    return (a - b) * (c - b) <= 0;
  }

  /// Returns -1 || 0 || 1 depending on the sign of value:
  /// -1 if x < 0
  ///  0 if x == 0
  ///  1 if x > 0
  static int scalarSignedAsInt(double x) {
    return x < 0 ? -1 : ((x > 0) ? 1 : 0);
  }

  static bool nearlyEqual(double value1, double value2) =>
      (value1 - value2).abs() < SPath.scalarNearlyZero;

  // Snaps a value to zero if almost zero (within tolerance).
  static double snapToZero(double value) => SPath.nearlyEqual(value, 0.0) ? 0.0 : value;

  static bool isInteger(double value) => value.floor() == value;
}

class SPathAddPathMode {
  // Append to destination unaltered.
  static const int kAppend = 0;
  // Add line if prior contour is not closed.
  static const int kExtend = 1;
}

class SPathDirection {
  /// Uninitialized value for empty paths.
  static const int kUnknown = -1;

  /// clockwise direction for adding closed contours.
  static const int kCW = 0;

  /// counter-clockwise direction for adding closed contours.
  static const int kCCW = 1;
}

class SPathConvexityType {
  static const int kUnknown = -1;
  static const int kConvex = 0;
  static const int kConcave = 1;
}

class SPathSegmentState {
  /// The current contour is empty. Starting processing or have just closed
  /// a contour.
  static const int kEmptyContour = 0;

  /// Have seen a move, but nothing else.
  static const int kAfterMove = 1;

  /// Have seen a primitive but not yet closed the path. Also the initial state.
  static const int kAfterPrimitive = 2;
}

/// Quadratic roots. See Numerical Recipes in C.
///
///    Q = -1/2 (B + sign(B) sqrt[B*B - 4*A*C])
///    x1 = Q / A
///    x2 = C / Q
class QuadRoots {
  QuadRoots();

  double? root0;
  double? root1;

  /// Returns roots as list.
  List<double> get roots =>
      (root0 == null) ? <double>[] : (root1 == null ? <double>[root0!] : <double>[root0!, root1!]);

  int findRoots(double a, double b, double c) {
    int rootCount = 0;
    if (a == 0) {
      root0 = validUnitDivide(-c, b);
      return root0 == null ? 0 : 1;
    }

    double dr = b * b - 4 * a * c;
    if (dr < 0) {
      return 0;
    }
    dr = math.sqrt(dr);
    if (!dr.isFinite) {
      return 0;
    }

    final double q = (b < 0) ? -(b - dr) / 2 : -(b + dr) / 2;
    double? res = validUnitDivide(q, a);
    if (res != null) {
      root0 = res;
      ++rootCount;
    }
    res = validUnitDivide(c, q);
    if (res != null) {
      if (rootCount == 0) {
        root0 = res;
        ++rootCount;
      } else {
        root1 = res;
        ++rootCount;
      }
    }
    if (rootCount == 2) {
      if (root0! > root1!) {
        final double swap = root0!;
        root0 = root1;
        root1 = swap;
      } else if (root0 == root1) {
        return 1; // skip the double root
      }
    }
    return rootCount;
  }
}

double? validUnitDivide(double numer, double denom) {
  if (numer < 0) {
    numer = -numer;
    denom = -denom;
  }
  if (denom == 0 || numer == 0 || numer >= denom) {
    return null;
  }
  final double r = numer / denom;
  if (r.isNaN) {
    return null;
  }
  if (r == 0) {
    // catch underflow if numer <<<< denom
    return null;
  }
  return r;
}

bool isRRectOval(ui.RRect rrect) {
  if ((rrect.tlRadiusX + rrect.trRadiusX) != rrect.width) {
    return false;
  }
  if ((rrect.tlRadiusY + rrect.trRadiusY) != rrect.height) {
    return false;
  }
  if (rrect.tlRadiusX != rrect.blRadiusX ||
      rrect.trRadiusX != rrect.brRadiusX ||
      rrect.tlRadiusY != rrect.blRadiusY ||
      rrect.trRadiusY != rrect.brRadiusY) {
    return false;
  }
  return true;
}

/// Evaluates degree 2 polynomial (quadratic).
double polyEval(double A, double B, double C, double t) => (A * t + B) * t + C;

/// Evaluates degree 3 polynomial (cubic).
double polyEval4(double A, double B, double C, double D, double t) => ((A * t + B) * t + C) * t + D;

// Interpolate between two doubles (Not using lerpDouble here since it null
// checks and treats values as 0).
double interpolate(double startValue, double endValue, double t) =>
    (startValue * (1 - t)) + endValue * t;

double dotProduct(double x0, double y0, double x1, double y1) {
  return x0 * x1 + y0 * y1;
}

// Helper class for computing convexity for a single contour.
//
// Iteratively looks at angle (using cross product) between consecutive vectors
// formed by path.
class Convexicator {
  static const int kValueNeverReturnedBySign = 2;

  // Second point of contour start that forms a vector.
  // Used to handle close operator to compute angle between last vector and
  // first.
  double? firstVectorEndPointX;
  double? firstVectorEndPointY;

  double? priorX;
  double? priorY;

  double? lastX;
  double? lastY;

  double? currX;
  double? currY;

  // Last vector to use to compute angle.
  double? lastVecX;
  double? lastVecY;

  bool _isFinite = true;
  int _firstDirection = SPathDirection.kUnknown;
  int _reversals = 0;

  /// SPathDirection of contour.
  int get firstDirection => _firstDirection;

  DirChange _expectedDirection = DirChange.kInvalid;

  void setMovePt(double x, double y) {
    currX = priorX = lastX = x;
    currY = priorY = lastY = y;
  }

  bool addPoint(double x, double y) {
    if (x == currX && y == currY) {
      // Skip zero length vector.
      return true;
    }
    currX = x;
    currY = y;
    final double vecX = currX! - lastX!;
    final double vecY = currY! - lastY!;
    if (priorX == lastX && priorY == lastY) {
      // First non-zero vector.
      lastVecX = vecX;
      lastVecY = vecY;
      firstVectorEndPointX = x;
      firstVectorEndPointY = y;
    } else if (!_addVector(vecX, vecY)) {
      return false;
    }
    priorX = lastX;
    priorY = lastY;
    lastX = x;
    lastY = y;
    return true;
  }

  bool close() {
    // Add another point from path closing point to end of first vector.
    return addPoint(firstVectorEndPointX!, firstVectorEndPointY!);
  }

  bool get isFinite => _isFinite;

  int get reversals => _reversals;

  DirChange _directionChange(double curVecX, double curVecY) {
    // Cross product = ||lastVec|| * ||curVec|| * sin(theta) * N
    // sin(theta) angle between two vectors is positive for angles 0..180 and
    // negative for greater, providing left or right direction.
    final double lastX = lastVecX!;
    final double lastY = lastVecY!;
    final double cross = lastX * curVecY - lastY * curVecX;
    if (!cross.isFinite) {
      return DirChange.kUnknown;
    }
    // Detect straight and backwards direction change.
    // Instead of comparing absolute crossproduct size, compare
    // largest component double+crossproduct.
    final double smallest = math.min(curVecX, math.min(curVecY, math.min(lastX, lastY)));
    final double largest = math.max(
      math.max(curVecX, math.max(curVecY, math.max(lastX, lastY))),
      -smallest,
    );
    if (SPath.nearlyEqual(largest, largest + cross)) {
      const double nearlyZeroSquared = SPath.scalarNearlyZero * SPath.scalarNearlyZero;
      if (SPath.nearlyEqual(lengthSquared(lastX, lastY), nearlyZeroSquared) ||
          SPath.nearlyEqual(lengthSquared(curVecX, curVecY), nearlyZeroSquared)) {
        // Length of either vector is smaller than tolerance to be able
        // to compute direction.
        return DirChange.kUnknown;
      }
      // The vectors are parallel, sign of dot product gives us direction.
      // cosine is positive for straight -90 < Theta < 90
      return dotProduct(lastX, lastY, curVecX, curVecY) < 0
          ? DirChange.kBackwards
          : DirChange.kStraight;
    }
    return cross > 0 ? DirChange.kRight : DirChange.kLeft;
  }

  bool _addVector(double curVecX, double curVecY) {
    final DirChange dir = _directionChange(curVecX, curVecY);
    final bool isDirectionRight = dir == DirChange.kRight;
    if (dir == DirChange.kLeft || isDirectionRight) {
      if (_expectedDirection == DirChange.kInvalid) {
        // First valid direction. From this point on expect always left.
        _expectedDirection = dir;
        _firstDirection = isDirectionRight ? SPathDirection.kCW : SPathDirection.kCCW;
      } else if (dir != _expectedDirection) {
        _firstDirection = SPathDirection.kUnknown;
        return false;
      }
      lastVecX = curVecX;
      lastVecY = curVecY;
    } else {
      switch (dir) {
        case DirChange.kBackwards:
          // Allow path to reverse direction twice.
          // Given path.moveTo(0,0) lineTo(1,1)
          //   - First reversal: direction change formed by line (0,0 1,1),
          //     line (1,1 0,0)
          //   - Second reversal: direction change formed by line (1,1 0,0),
          //     line (0,0 1,1)
          lastVecX = curVecX;
          lastVecY = curVecY;
          return ++_reversals < 3;
        case DirChange.kUnknown:
          return _isFinite = false;
        default:
          break;
      }
    }
    return true;
  }

  // Quick test to detect concave by looking at number of changes in direction
  // of vectors formed by path points (excluding control points).
  static int bySign(PathRef pathRef, int pointIndex, int numPoints) {
    final int lastPointIndex = pointIndex + numPoints;
    int currentPoint = pointIndex++;
    final int firstPointIndex = currentPoint;
    int signChangeCountX = 0;
    int signChangeCountY = 0;
    int lastSx = kValueNeverReturnedBySign;
    int lastSy = kValueNeverReturnedBySign;
    for (int outerLoop = 0; outerLoop < 2; ++outerLoop) {
      while (pointIndex != lastPointIndex) {
        final double vecX = pathRef.pointXAt(pointIndex) - pathRef.pointXAt(currentPoint);
        final double vecY = pathRef.pointYAt(pointIndex) - pathRef.pointYAt(currentPoint);
        if (!(vecX == 0 && vecY == 0)) {
          // Give up if vector construction failed.
          // give up if vector construction failed
          if (!(vecX.isFinite && vecY.isFinite)) {
            return SPathConvexityType.kUnknown;
          }
          final int sx = vecX < 0 ? 1 : 0;
          final int sy = vecY < 0 ? 1 : 0;
          signChangeCountX += (sx != lastSx) ? 1 : 0;
          signChangeCountY += (sy != lastSy) ? 1 : 0;
          if (signChangeCountX > 3 || signChangeCountY > 3) {
            return SPathConvexityType.kConcave;
          }
          lastSx = sx;
          lastSy = sy;
        }
        currentPoint = pointIndex++;
        if (outerLoop != 0) {
          break;
        }
      }
      pointIndex = firstPointIndex;
    }
    return SPathConvexityType.kConvex;
  }
}

enum DirChange {
  kUnknown,
  kLeft,
  kRight,
  kStraight,
  kBackwards, // if double back, allow simple lines to be convex
  kInvalid,
}

double lengthSquaredOffset(ui.Offset offset) {
  final double dx = offset.dx;
  final double dy = offset.dy;
  return dx * dx + dy * dy;
}

double lengthSquared(double dx, double dy) => dx * dx + dy * dy;

/// Evaluates A * t^2 + B * t + C = 0 for quadratic curve.
class SkQuadCoefficients {
  SkQuadCoefficients(double x0, double y0, double x1, double y1, double x2, double y2)
    : cx = x0,
      cy = y0,
      bx = 2 * (x1 - x0),
      by = 2 * (y1 - y0),
      ax = x2 - (2 * x1) + x0,
      ay = y2 - (2 * y1) + y0;
  final double ax, ay, bx, by, cx, cy;

  double evalX(double t) => (ax * t + bx) * t + cx;

  double evalY(double t) => (ay * t + by) * t + cy;
}
