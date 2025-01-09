// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;
import 'dart:typed_data';

import 'package:ui/ui.dart' as ui;

import '../../validators.dart';
import '../../vector_math.dart';
import 'conic.dart';
import 'cubic.dart';
import 'path_iterator.dart';
import 'path_metrics.dart';
import 'path_ref.dart';
import 'path_utils.dart';
import 'path_windings.dart';
import 'tangent.dart';

/// A complex, one-dimensional subset of a plane.
///
/// Path consist of segments of various types, such as lines,
/// arcs, or beziers. Subpaths can be open or closed, and can
/// self-intersect.
///
/// Stores the verbs and points as they are given to us, with exceptions:
///   - we only record "Close" if it was immediately preceded by Move | Line | Quad | Cubic
///   - we insert a Move(0,0) if Line | Quad | Cubic is our first command
///
///   The iterator does more cleanup, especially if forceClose == true
///   1. If we encounter degenerate segments, remove them
///   2. if we encounter Close, return a cons'd up Line() first (if the curr-pt != start-pt)
///   3. if we encounter Move without a preceding Close, and forceClose is true, goto #2
///   4. if we encounter Line | Quad | Cubic after Close, cons up a Move
class SurfacePath implements ui.Path {
  SurfacePath() : pathRef = PathRef() {
    _resetFields();
  }

  /// Creates a copy of another [Path].
  SurfacePath.from(SurfacePath source) : pathRef = PathRef()..copy(source.pathRef, 0, 0) {
    _copyFields(source);
  }

  /// Creates a shifted copy of another [Path].
  SurfacePath.shiftedFrom(SurfacePath source, double offsetX, double offsetY)
    : pathRef = PathRef.shiftedFrom(source.pathRef, offsetX, offsetY) {
    _copyFields(source);
  }

  SurfacePath.shallowCopy(SurfacePath source) : pathRef = PathRef.shallowCopy(source.pathRef) {
    _copyFields(source);
  }

  // Initial valid of last move to index so we can detect if a move to
  // needs to be inserted after contour closure. See [close].
  static const int kInitialLastMoveToIndexValue = 0;

  PathRef pathRef;
  ui.PathFillType _fillType = ui.PathFillType.nonZero;
  // Store point index + 1 of last moveTo instruction.
  // If contour has been closed or path is in initial state, the value is
  // negated.
  int fLastMoveToIndex = kInitialLastMoveToIndexValue;
  int _convexityType = SPathConvexityType.kUnknown;
  int _firstDirection = SPathDirection.kUnknown;

  void _resetFields() {
    fLastMoveToIndex = kInitialLastMoveToIndexValue;
    _fillType = ui.PathFillType.nonZero;
    _resetAfterEdit();
  }

  void _resetAfterEdit() {
    _convexityType = SPathConvexityType.kUnknown;
    _firstDirection = SPathDirection.kUnknown;
  }

  void _copyFields(SurfacePath source) {
    _fillType = source._fillType;
    fLastMoveToIndex = source.fLastMoveToIndex;
    _convexityType = source._convexityType;
    _firstDirection = source._firstDirection;
  }

  /// Determines how the interior of this path is calculated.
  ///
  /// Defaults to the non-zero winding rule, [PathFillType.nonZero].
  @override
  ui.PathFillType get fillType => _fillType;

  @override
  set fillType(ui.PathFillType value) {
    _fillType = value;
  }

  /// Returns true if [SurfacePath] contain equal verbs and equal weights.
  bool isInterpolatable(SurfacePath compare) =>
      compare.pathRef.countVerbs() == pathRef.countVerbs() &&
      compare.pathRef.countPoints() == pathRef.countPoints() &&
      compare.pathRef.countWeights() == pathRef.countWeights();

  bool interpolate(SurfacePath ending, double weight, SurfacePath out) {
    final int pointCount = pathRef.countPoints();
    if (pointCount != ending.pathRef.countPoints()) {
      return false;
    }
    if (pointCount == 0) {
      return true;
    }
    out.reset();
    out.addPathWithMode(this, 0, 0, null, SPathAddPathMode.kAppend);
    PathRef.interpolate(ending.pathRef, weight, out.pathRef);
    return true;
  }

  /// Clears the [Path] object, returning it to the
  /// same state it had when it was created. The _current point_ is
  /// reset to the origin.
  @override
  void reset() {
    if (pathRef.countVerbs() != 0) {
      pathRef = PathRef();
      _resetFields();
    }
  }

  ///  Sets [SurfacePath] to its initial state, preserving internal storage.
  ///  Removes verb array, SkPoint array, and weights, and sets FillType to
  ///  kWinding. Internal storage associated with SkPath is retained.
  ///
  ///  Use rewind() instead of reset() if SkPath storage will be reused and
  ///  performance is critical.
  void rewind() {
    pathRef.rewind();
    _resetFields();
  }

  /// Returns if contour is closed.
  ///
  /// Contour is closed if [SurfacePath] verb array was last modified by
  /// close(). When stroked, closed contour draws join instead of cap at first
  /// and last point.
  bool get isLastContourClosed {
    final int verbCount = pathRef.countVerbs();
    return verbCount > 0 && (pathRef.atVerb(verbCount - 1) == SPathVerb.kClose);
  }

  /// Returns true for finite SkPoint array values between negative SK_ScalarMax
  /// and positive SK_ScalarMax. Returns false for any SkPoint array value of
  /// SK_ScalarInfinity, SK_ScalarNegativeInfinity, or SK_ScalarNaN.
  bool get isFinite {
    _debugValidate();
    return pathRef.isFinite;
  }

  void _debugValidate() {
    assert(pathRef.isValid);
  }

  /// Return true if path is a single line and returns points in out.
  bool isLine(Float32List out) {
    assert(out.length >= 4);
    final int verbCount = pathRef.countPoints();
    if (2 == verbCount &&
        pathRef.atVerb(0) == SPathVerb.kMove &&
        pathRef.atVerb(1) != SPathVerb.kLine) {
      out[0] = pathRef.points[0];
      out[1] = pathRef.points[1];
      out[2] = pathRef.points[2];
      out[3] = pathRef.points[3];
      return true;
    }
    return false;
  }

  /// Starts a new subpath at the given coordinate.
  @override
  void moveTo(double x, double y) {
    // remember our index
    final int pointIndex = pathRef.growForVerb(SPathVerb.kMove, 0);
    fLastMoveToIndex = pointIndex + 1;
    pathRef.setPoint(pointIndex, x, y);
    _resetAfterEdit();
  }

  /// Starts a new subpath at the given offset from the current point.
  @override
  void relativeMoveTo(double dx, double dy) {
    final int pointCount = pathRef.countPoints();
    if (pointCount == 0) {
      moveTo(dx, dy);
    } else {
      int pointIndex = (pointCount - 1) * 2;
      final double lastPointX = pathRef.points[pointIndex++];
      final double lastPointY = pathRef.points[pointIndex];
      moveTo(lastPointX + dx, lastPointY + dy);
    }
  }

  void _injectMoveToIfNeeded() {
    if (fLastMoveToIndex <= 0) {
      double x, y;
      if (pathRef.countPoints() == 0) {
        x = y = 0.0;
      } else {
        int pointIndex = 2 * (-fLastMoveToIndex - 1);
        x = pathRef.points[pointIndex++];
        y = pathRef.points[pointIndex];
      }
      moveTo(x, y);
    }
  }

  /// Adds a straight line segment from the current point to the given
  /// point.
  @override
  void lineTo(double x, double y) {
    if (fLastMoveToIndex <= 0) {
      _injectMoveToIfNeeded();
    }
    final int pointIndex = pathRef.growForVerb(SPathVerb.kLine, 0);
    pathRef.setPoint(pointIndex, x, y);
    _resetAfterEdit();
  }

  /// Adds a straight line segment from the current point to the point
  /// at the given offset from the current point.
  @override
  void relativeLineTo(double dx, double dy) {
    final int pointCount = pathRef.countPoints();
    if (pointCount == 0) {
      lineTo(dx, dy);
    } else {
      int pointIndex = (pointCount - 1) * 2;
      final double lastPointX = pathRef.points[pointIndex++];
      final double lastPointY = pathRef.points[pointIndex];
      lineTo(lastPointX + dx, lastPointY + dy);
    }
  }

  /// Adds a quadratic bezier segment that curves from the current
  /// point to the given point (x2,y2), using the control point
  /// (x1,y1).
  @override
  void quadraticBezierTo(double x1, double y1, double x2, double y2) {
    _injectMoveToIfNeeded();
    _quadTo(x1, y1, x2, y2);
  }

  /// Adds a quadratic bezier segment that curves from the current
  /// point to the point at the offset (x2,y2) from the current point,
  /// using the control point at the offset (x1,y1) from the current
  /// point.
  @override
  void relativeQuadraticBezierTo(double x1, double y1, double x2, double y2) {
    final int pointCount = pathRef.countPoints();
    if (pointCount == 0) {
      quadraticBezierTo(x1, y1, x2, y2);
    } else {
      int pointIndex = (pointCount - 1) * 2;
      final double lastPointX = pathRef.points[pointIndex++];
      final double lastPointY = pathRef.points[pointIndex];
      quadraticBezierTo(x1 + lastPointX, y1 + lastPointY, x2 + lastPointX, y2 + lastPointY);
    }
  }

  void _quadTo(double x1, double y1, double x2, double y2) {
    final int pointIndex = pathRef.growForVerb(SPathVerb.kQuad, 0);
    pathRef.setPoint(pointIndex, x1, y1);
    pathRef.setPoint(pointIndex + 1, x2, y2);
    _resetAfterEdit();
  }

  /// Adds a bezier segment that curves from the current point to the
  /// given point (x2,y2), using the control points (x1,y1) and the
  /// weight w. If the weight is greater than 1, then the curve is a
  /// hyperbola; if the weight equals 1, it's a parabola; and if it is
  /// less than 1, it is an ellipse.
  @override
  void conicTo(double x1, double y1, double x2, double y2, double w) {
    _injectMoveToIfNeeded();
    final int pointIndex = pathRef.growForVerb(SPathVerb.kConic, w);
    pathRef.setPoint(pointIndex, x1, y1);
    pathRef.setPoint(pointIndex + 1, x2, y2);
    _resetAfterEdit();
  }

  /// Adds a bezier segment that curves from the current point to the
  /// point at the offset (x2,y2) from the current point, using the
  /// control point at the offset (x1,y1) from the current point and
  /// the weight w. If the weight is greater than 1, then the curve is
  /// a hyperbola; if the weight equals 1, it's a parabola; and if it
  /// is less than 1, it is an ellipse.
  @override
  void relativeConicTo(double x1, double y1, double x2, double y2, double w) {
    final int pointCount = pathRef.countPoints();
    if (pointCount == 0) {
      conicTo(x1, y1, x2, y2, w);
    } else {
      int pointIndex = (pointCount - 1) * 2;
      final double lastPointX = pathRef.points[pointIndex++];
      final double lastPointY = pathRef.points[pointIndex];
      conicTo(lastPointX + x1, lastPointY + y1, lastPointX + x2, lastPointY + y2, w);
    }
  }

  /// Adds a cubic bezier segment that curves from the current point
  /// to the given point (x3,y3), using the control points (x1,y1) and
  /// (x2,y2).
  @override
  void cubicTo(double x1, double y1, double x2, double y2, double x3, double y3) {
    _injectMoveToIfNeeded();
    final int pointIndex = pathRef.growForVerb(SPathVerb.kCubic, 0);
    pathRef.setPoint(pointIndex, x1, y1);
    pathRef.setPoint(pointIndex + 1, x2, y2);
    pathRef.setPoint(pointIndex + 2, x3, y3);
    _resetAfterEdit();
  }

  /// Adds a cubic bezier segment that curves from the current point
  /// to the point at the offset (x3,y3) from the current point, using
  /// the control points at the offsets (x1,y1) and (x2,y2) from the
  /// current point.
  @override
  void relativeCubicTo(double x1, double y1, double x2, double y2, double x3, double y3) {
    final int pointCount = pathRef.countPoints();
    if (pointCount == 0) {
      cubicTo(x1, y1, x2, y2, x3, y3);
    } else {
      int pointIndex = (pointCount - 1) * 2;
      final double lastPointX = pathRef.points[pointIndex++];
      final double lastPointY = pathRef.points[pointIndex];
      cubicTo(
        x1 + lastPointX,
        y1 + lastPointY,
        x2 + lastPointX,
        y2 + lastPointY,
        x3 + lastPointX,
        y3 + lastPointY,
      );
    }
  }

  /// Closes the last subpath, as if a straight line had been drawn
  /// from the current point to the first point of the subpath.
  @override
  void close() {
    _debugValidate();
    // Don't add verb if it is the first instruction or close as already
    // been added.
    final int verbCount = pathRef.countVerbs();
    if (verbCount != 0 && pathRef.atVerb(verbCount - 1) != SPathVerb.kClose) {
      pathRef.growForVerb(SPathVerb.kClose, 0);
    }
    if (fLastMoveToIndex >= 0) {
      // Signal that we need a moveTo to follow next if not specified.
      fLastMoveToIndex = -fLastMoveToIndex;
    }
    _resetAfterEdit();
  }

  /// Adds a new subpath that consists of four lines that outline the
  /// given rectangle.
  @override
  void addRect(ui.Rect rect) {
    addRectWithDirection(rect, SPathDirection.kCW, 0);
  }

  bool _hasOnlyMoveTos() {
    final int verbCount = pathRef.countVerbs();
    for (int i = 0; i < verbCount; i++) {
      switch (pathRef.atVerb(i)) {
        case SPathVerb.kLine:
        case SPathVerb.kQuad:
        case SPathVerb.kConic:
        case SPathVerb.kCubic:
          return false;
      }
    }
    return true;
  }

  void addRectWithDirection(ui.Rect rect, int direction, int startIndex) {
    assert(direction != SPathDirection.kUnknown);
    final bool isRect = _hasOnlyMoveTos();
    // SkAutoDisableDirectionCheck.
    final int finalDirection = _hasOnlyMoveTos() ? direction : SPathDirection.kUnknown;
    final int pointIndex0 = pathRef.growForVerb(SPathVerb.kMove, 0);
    fLastMoveToIndex = pointIndex0 + 1;
    final int pointIndex1 = pathRef.growForVerb(SPathVerb.kLine, 0);
    final int pointIndex2 = pathRef.growForVerb(SPathVerb.kLine, 0);
    final int pointIndex3 = pathRef.growForVerb(SPathVerb.kLine, 0);
    pathRef.growForVerb(SPathVerb.kClose, 0);
    if (direction == SPathDirection.kCW) {
      pathRef.setPoint(pointIndex0, rect.left, rect.top);
      pathRef.setPoint(pointIndex1, rect.right, rect.top);
      pathRef.setPoint(pointIndex2, rect.right, rect.bottom);
      pathRef.setPoint(pointIndex3, rect.left, rect.bottom);
    } else {
      pathRef.setPoint(pointIndex3, rect.left, rect.bottom);
      pathRef.setPoint(pointIndex2, rect.right, rect.bottom);
      pathRef.setPoint(pointIndex1, rect.right, rect.top);
      pathRef.setPoint(pointIndex0, rect.left, rect.top);
    }
    pathRef.setIsRect(isRect, direction == SPathDirection.kCCW, 0);
    _resetAfterEdit();
    // SkAutoDisableDirectionCheck.
    _firstDirection = finalDirection;
    // TODO(ferhat): optimize by setting pathRef bounds if bounds are already computed.
  }

  /// If the `forceMoveTo` argument is false, adds a straight line
  /// segment and an arc segment.
  ///
  /// If the `forceMoveTo` argument is true, starts a new subpath
  /// consisting of an arc segment.
  ///
  /// In either case, the arc segment consists of the arc that follows
  /// the edge of the oval bounded by the given rectangle, from
  /// startAngle radians around the oval up to startAngle + sweepAngle
  /// radians around the oval, with zero radians being the point on
  /// the right hand side of the oval that crosses the horizontal line
  /// that intersects the center of the rectangle and with positive
  /// angles going clockwise around the oval.
  ///
  /// The line segment added if `forceMoveTo` is false starts at the
  /// current point and ends at the start of the arc.
  @override
  void arcTo(ui.Rect rect, double startAngle, double sweepAngle, bool forceMoveTo) {
    assert(rectIsValid(rect));
    // If width or height is 0, we still stroke a line, only abort if both
    // are empty.
    if (rect.width == 0 && rect.height == 0) {
      return;
    }
    if (pathRef.countPoints() == 0) {
      forceMoveTo = true;
    }
    final ui.Offset? lonePoint = _arcIsLonePoint(rect, startAngle, sweepAngle);
    if (lonePoint != null) {
      if (forceMoveTo) {
        moveTo(lonePoint.dx, lonePoint.dy);
      } else {
        lineTo(lonePoint.dx, lonePoint.dy);
      }
    }
    // Convert angles to unit vectors.
    double stopAngle = startAngle + sweepAngle;
    final double cosStart = math.cos(startAngle);
    final double sinStart = math.sin(startAngle);
    double cosStop = math.cos(stopAngle);
    double sinStop = math.sin(stopAngle);

    // If the sweep angle is nearly (but less than) 360, then due to precision
    // loss in radians-conversion and/or sin/cos, we may end up with coincident
    // vectors, which will fool quad arc build into doing nothing (bad) instead
    // of drawing a nearly complete circle (good).
    // e.g. canvas.drawArc(0, 359.99, ...)
    // -vs- canvas.drawArc(0, 359.9, ...)
    // Detect this edge case, and tweak the stop vector.
    if (SPath.nearlyEqual(cosStart, cosStop) && SPath.nearlyEqual(sinStart, sinStop)) {
      final double sweep = sweepAngle.abs() * 180.0 / math.pi;
      if (sweep <= 360 && sweep > 359) {
        // Use tiny angle (in radians) to tweak.
        final double deltaRad = sweepAngle < 0 ? -1.0 / 512.0 : 1.0 / 512.0;
        do {
          stopAngle -= deltaRad;
          cosStop = math.cos(stopAngle);
          sinStop = math.sin(stopAngle);
        } while (cosStart == cosStop && sinStart == sinStop);
      }
    }
    final int dir = sweepAngle > 0 ? SPathDirection.kCW : SPathDirection.kCCW;
    final double endAngle = startAngle + sweepAngle;
    final double radiusX = rect.width / 2.0;
    final double radiusY = rect.height / 2.0;
    final double px = rect.center.dx + (radiusX * math.cos(endAngle));
    final double py = rect.center.dy + (radiusY * math.sin(endAngle));
    // At this point, we know that the arc is not a lone point, but
    // startV == stopV indicates that the sweepAngle is too small such that
    // angles_to_unit_vectors cannot handle it
    if (cosStart == cosStop && sinStart == sinStop) {
      // Add moveTo to start point if forceMoveTo is true. Otherwise a lineTo
      // unless we're sufficiently close to start point currently. This prevents
      // spurious lineTos when adding a series of contiguous arcs from the same
      // oval.
      if (forceMoveTo) {
        moveTo(px, py);
      } else {
        _lineToIfNotTooCloseToLastPoint(px, py);
      }
      // We are done with tiny sweep approximated by line.
      return;
    }

    // Convert arc defined by start/end unit vectors to conics (max 5).

    // Dot product
    final double x = (cosStart * cosStop) + (sinStart * sinStop);
    // Cross product
    double y = (cosStart * sinStop) - (sinStart * cosStop);
    final double absY = y.abs();
    // Check for coincident vectors (angle is nearly 0 or 180).
    // The cross product for angles 0 and 180 will be zero, we use the
    // dot product sign to distinguish between the two.
    if (absY <= SPath.scalarNearlyZero &&
        x > 0 &&
        ((y >= 0 && dir == SPathDirection.kCW) || (y <= 0 && dir == SPathDirection.kCCW))) {
      // No conics, just use single line to connect point.
      if (forceMoveTo) {
        moveTo(px, py);
      } else {
        _lineToIfNotTooCloseToLastPoint(px, py);
      }
      return;
    }

    // Normalize to clockwise
    if (dir == SPathDirection.kCCW) {
      y = -y;
    }

    // Use 1 conic per quadrant of a circle.
    // 0..90 -> quadrant 0
    // 90..180 -> quadrant 1
    // 180..270 -> quadrant 2
    // 270..360 -> quadrant 3

    const List<ui.Offset> quadPoints = <ui.Offset>[
      ui.Offset(1, 0),
      ui.Offset(1, 1),
      ui.Offset(0, 1),
      ui.Offset(-1, 1),
      ui.Offset(-1, 0),
      ui.Offset(-1, -1),
      ui.Offset(0, -1),
      ui.Offset(1, -1),
    ];

    int quadrant = 0;
    if (0 == y) {
      // 180 degrees between vectors.
      quadrant = 2;
      assert((x + 1).abs() <= SPath.scalarNearlyZero);
    } else if (0 == x) {
      // Dot product 0 means 90 degrees between vectors.
      assert((absY - 1) <= SPath.scalarNearlyZero);
      quadrant = y > 0 ? 1 : 3; // 90 or 270
    } else {
      if (y < 0) {
        quadrant += 2;
      }
      if ((x < 0) != (y < 0)) {
        quadrant += 1;
      }
    }

    final List<Conic> conics = <Conic>[];

    const double quadrantWeight = SPath.scalarRoot2Over2;
    int conicCount = quadrant;
    for (int i = 0; i < conicCount; i++) {
      final int quadPointIndex = i * 2;
      final ui.Offset p0 = quadPoints[quadPointIndex];
      final ui.Offset p1 = quadPoints[quadPointIndex + 1];
      final ui.Offset p2 = quadPoints[quadPointIndex + 2];
      conics.add(Conic(p0.dx, p0.dy, p1.dx, p1.dy, p2.dx, p2.dy, quadrantWeight));
    }

    // Now compute any remaining ( < 90degree ) arc for last conic.
    final double finalPx = x;
    final double finalPy = y;
    final ui.Offset lastQuadrantPoint = quadPoints[quadrant * 2];
    // Dot product between last quadrant vector and last point on arc.
    final double dot = (x * lastQuadrantPoint.dx) + (y * lastQuadrantPoint.dy);
    if (dot < 1) {
      // Compute the bisector vector and then rescale to be the off curve point.
      // Length is cos(theta/2) using half angle identity we get
      // length = sqrt(2 / (1 + cos(theta)). We already have cos from computing
      // dot. Computed weight is cos(theta/2).
      double offCurveX = lastQuadrantPoint.dx + x;
      double offCurveY = lastQuadrantPoint.dy + y;
      final double cosThetaOver2 = math.sqrt((1.0 + dot) / 2.0);
      final double unscaledLength = math.sqrt((offCurveX * offCurveX) + (offCurveY * offCurveY));
      assert(unscaledLength > SPath.scalarNearlyZero);
      offCurveX /= cosThetaOver2 * unscaledLength;
      offCurveY /= cosThetaOver2 * unscaledLength;
      if (!SPath.nearlyEqual(offCurveX, lastQuadrantPoint.dx) ||
          !SPath.nearlyEqual(offCurveY, lastQuadrantPoint.dy)) {
        conics.add(
          Conic(
            lastQuadrantPoint.dx,
            lastQuadrantPoint.dy,
            offCurveX,
            offCurveY,
            finalPx,
            finalPy,
            cosThetaOver2,
          ),
        );
        ++conicCount;
      }
    }

    // Any points we generate based on unit vectors cos/sinStart , cos/sinStop
    // we rotate to start vector, scale by rect.width/2 rect.height/2 and
    // then translate to center point.
    final double scaleX = rect.width / 2;
    final bool ccw = dir == SPathDirection.kCCW;
    final double scaleY = rect.height / 2;
    final double centerX = rect.center.dx;
    final double centerY = rect.center.dy;
    for (final Conic conic in conics) {
      double x = conic.p0x;
      double y = ccw ? -conic.p0y : conic.p0y;
      conic.p0x = (cosStart * x - sinStart * y) * scaleX + centerX;
      conic.p0y = (cosStart * y + sinStart * x) * scaleY + centerY;
      x = conic.p1x;
      y = ccw ? -conic.p1y : conic.p1y;
      conic.p1x = (cosStart * x - sinStart * y) * scaleX + centerX;
      conic.p1y = (cosStart * y + sinStart * x) * scaleY + centerY;
      x = conic.p2x;
      y = ccw ? -conic.p2y : conic.p2y;
      conic.p2x = (cosStart * x - sinStart * y) * scaleX + centerX;
      conic.p2y = (cosStart * y + sinStart * x) * scaleY + centerY;
    }
    // Now output points.
    final double firstConicPx = conics[0].p0x;
    final double firstConicPy = conics[0].p0y;
    if (forceMoveTo) {
      moveTo(firstConicPx, firstConicPy);
    } else {
      _lineToIfNotTooCloseToLastPoint(firstConicPx, firstConicPy);
    }
    for (int i = 0; i < conicCount; i++) {
      final Conic conic = conics[i];
      conicTo(conic.p1x, conic.p1y, conic.p2x, conic.p2y, conic.fW);
    }
    _resetAfterEdit();
  }

  void _lineToIfNotTooCloseToLastPoint(double px, double py) {
    final int pointCount = pathRef.countPoints();
    if (pointCount != 0) {
      final ui.Offset lastPoint = pathRef.atPoint(pointCount - 1);
      final double lastPointX = lastPoint.dx;
      final double lastPointY = lastPoint.dy;
      if (!SPath.nearlyEqual(px, lastPointX) || !SPath.nearlyEqual(py, lastPointY)) {
        lineTo(px, py);
      }
    }
  }

  /// Appends up to four conic curves weighted to describe an oval of `radius`
  /// and rotated by `rotation` (measured in degrees and clockwise).
  ///
  /// The first curve begins from the last point in the path and the last ends
  /// at `arcEnd`. The curves follow a path in a direction determined by
  /// `clockwise` and `largeArc` in such a way that the sweep angle
  /// is always less than 360 degrees.
  ///
  /// A simple line is appended if either radii are zero or the last
  /// point in the path is `arcEnd`. The radii are scaled to fit the last path
  /// point if both are greater than zero but too small to describe an arc.
  ///
  /// See Conversion from endpoint to center parametrization described in
  /// https://www.w3.org/TR/SVG/implnote.html#ArcConversionEndpointToCenter
  /// as reference for implementation.
  @override
  void arcToPoint(
    ui.Offset arcEnd, {
    ui.Radius radius = ui.Radius.zero,
    double rotation = 0.0,
    bool largeArc = false,
    bool clockwise = true,
  }) {
    assert(offsetIsValid(arcEnd));
    assert(radiusIsValid(radius));

    _injectMoveToIfNeeded();
    final int pointCount = pathRef.countPoints();
    double lastPointX, lastPointY;
    if (pointCount == 0) {
      lastPointX = lastPointY = 0;
    } else {
      int pointIndex = (pointCount - 1) * 2;
      lastPointX = pathRef.points[pointIndex++];
      lastPointY = pathRef.points[pointIndex];
    }
    // lastPointX, lastPointY are the coordinates of start point on path,
    // x,y is final point of arc.
    final double x = arcEnd.dx;
    final double y = arcEnd.dy;

    // rx,ry are the radii of the eclipse (semi-major/semi-minor axis)
    double rx = radius.x.abs();
    double ry = radius.y.abs();

    // If the current point and target point for the arc are identical, it
    // should be treated as a zero length path. This ensures continuity in
    // animations.
    final bool isSamePoint = lastPointX == x && lastPointY == y;

    // If rx = 0 or ry = 0 then this arc is treated as a straight line segment
    // (a "lineto") joining the endpoints.
    // http://www.w3.org/TR/SVG/implnote.html#ArcOutOfRangeParameters
    if (isSamePoint || rx.toInt() == 0 || ry.toInt() == 0) {
      if (rx == 0 || ry == 0) {
        lineTo(x, y);
        return;
      }
    }

    // As an intermediate point to finding center parametrization, place the
    // origin on the midpoint between start/end points and rotate to align
    // coordinate axis with axes of the ellipse.
    final double midPointX = (lastPointX - x) / 2.0;
    final double midPointY = (lastPointY - y) / 2.0;

    // Convert rotation or radians.
    final double xAxisRotation = math.pi * rotation / 180.0;

    // Cache cos/sin value.
    final double cosXAxisRotation = math.cos(xAxisRotation);
    final double sinXAxisRotation = math.sin(xAxisRotation);

    // Calculate rotated midpoint.
    final double xPrime = (cosXAxisRotation * midPointX) + (sinXAxisRotation * midPointY);
    final double yPrime = (-sinXAxisRotation * midPointX) + (cosXAxisRotation * midPointY);

    // Check if the radii are big enough to draw the arc, scale radii if not.
    // http://www.w3.org/TR/SVG/implnote.html#ArcCorrectionOutOfRangeRadii
    double rxSquare = rx * rx;
    double rySquare = ry * ry;
    final double xPrimeSquare = xPrime * xPrime;
    final double yPrimeSquare = yPrime * yPrime;

    double radiiScale = (xPrimeSquare / rxSquare) + (yPrimeSquare / rySquare);
    if (radiiScale > 1) {
      radiiScale = math.sqrt(radiiScale);
      rx *= radiiScale;
      ry *= radiiScale;
      rxSquare = rx * rx;
      rySquare = ry * ry;
    }

    // Switch to unit vectors
    double unitPts0x = (lastPointX * cosXAxisRotation + lastPointY * sinXAxisRotation) / rx;
    double unitPts0y = (lastPointY * cosXAxisRotation - lastPointX * sinXAxisRotation) / ry;
    double unitPts1x = (x * cosXAxisRotation + y * sinXAxisRotation) / rx;
    double unitPts1y = (y * cosXAxisRotation - x * sinXAxisRotation) / ry;
    double deltaX = unitPts1x - unitPts0x;
    double deltaY = unitPts1y - unitPts0y;

    final double d = deltaX * deltaX + deltaY * deltaY;
    double scaleFactor = math.sqrt(math.max(1 / d - 0.25, 0.0));
    // largeArc is false if arc is spanning less than or equal to 180 degrees.
    // clockwise is false if arc sweeps through decreasing angles or true
    // if sweeping through increasing angles.
    // rotation is the angle from the x-axis of the current coordinate
    // system to the x-axis of the eclipse.
    if (largeArc == clockwise) {
      scaleFactor = -scaleFactor;
    }
    deltaX *= scaleFactor;
    deltaY *= scaleFactor;
    // Compute transformed center. eq. 5.2
    final double centerPointX = (unitPts0x + unitPts1x) / 2 - deltaY;
    final double centerPointY = (unitPts0y + unitPts1y) / 2 + deltaX;
    unitPts0x -= centerPointX;
    unitPts0y -= centerPointY;
    unitPts1x -= centerPointX;
    unitPts1y -= centerPointY;

    // Calculate start angle and sweep.
    final double theta1 = math.atan2(unitPts0y, unitPts0x);
    final double theta2 = math.atan2(unitPts1y, unitPts1x);
    double thetaArc = theta2 - theta1;
    if (clockwise && thetaArc < 0) {
      thetaArc += math.pi * 2.0;
    } else if (!clockwise && thetaArc > 0) {
      thetaArc -= math.pi * 2.0;
    }
    // Guard against tiny angles. See skbug.com/9272.
    if (thetaArc.abs() < (math.pi / (1000.0 * 1000.0))) {
      lineTo(x, y);
      return;
    }

    // The arc may be slightly bigger than 1/4 circle, so allow up to 1/3rd.
    final int segments = (thetaArc / (2.0 * math.pi / 3.0)).abs().ceil();
    final double thetaWidth = thetaArc / segments;
    final double t = math.tan(thetaWidth / 2.0);
    if (!t.isFinite) {
      return;
    }

    final double w = math.sqrt(0.5 + math.cos(thetaWidth) * 0.5);
    double startTheta = theta1;

    // Computing the arc width introduces rounding errors that cause arcs
    // to start outside their marks. A round rect may lose convexity as a
    // result. If the input values are on integers, place the conic on
    // integers as well.
    final bool expectIntegers =
        SPath.nearlyEqual(math.pi / 2 - thetaWidth.abs(), 0) &&
        SPath.isInteger(rx) &&
        SPath.isInteger(ry) &&
        SPath.isInteger(x) &&
        SPath.isInteger(y);

    for (int i = 0; i < segments; i++) {
      final double endTheta = startTheta + thetaWidth;
      final double sinEndTheta = SPath.snapToZero(math.sin(endTheta));
      final double cosEndTheta = SPath.snapToZero(math.cos(endTheta));
      double unitPts1x = cosEndTheta + centerPointX;
      double unitPts1y = sinEndTheta + centerPointY;
      double unitPts0x = unitPts1x + t * sinEndTheta;
      double unitPts0y = unitPts1y - t * cosEndTheta;
      unitPts0x = unitPts0x * rx;
      unitPts0y = unitPts0y * ry;
      unitPts1x = unitPts1x * rx;
      unitPts1y = unitPts1y * ry;
      double xStart = unitPts0x * cosXAxisRotation - unitPts0y * sinXAxisRotation;
      double yStart = unitPts0y * cosXAxisRotation + unitPts0x * sinXAxisRotation;
      double xEnd = unitPts1x * cosXAxisRotation - unitPts1y * sinXAxisRotation;
      double yEnd = unitPts1y * cosXAxisRotation + unitPts1x * sinXAxisRotation;
      if (expectIntegers) {
        xStart = (xStart + 0.5).floorToDouble();
        yStart = (yStart + 0.5).floorToDouble();
        xEnd = (xEnd + 0.5).floorToDouble();
        yEnd = (yEnd + 0.5).floorToDouble();
      }
      conicTo(xStart, yStart, xEnd, yEnd, w);
      startTheta = endTheta;
    }
  }

  /// Appends up to four conic curves weighted to describe an oval of `radius`
  /// and rotated by `rotation` (measured in degrees and clockwise).
  ///
  /// The last path point is described by (px, py).
  ///
  /// The first curve begins from the last point in the path and the last ends
  /// at `arcEndDelta.dx + px` and `arcEndDelta.dy + py`. The curves follow a
  /// path in a direction determined by `clockwise` and `largeArc`
  /// in such a way that the sweep angle is always less than 360 degrees.
  ///
  /// A simple line is appended if either radii are zero, or, both
  /// `arcEndDelta.dx` and `arcEndDelta.dy` are zero. The radii are scaled to
  /// fit the last path point if both are greater than zero but too small to
  /// describe an arc.
  @override
  void relativeArcToPoint(
    ui.Offset arcEndDelta, {
    ui.Radius radius = ui.Radius.zero,
    double rotation = 0.0,
    bool largeArc = false,
    bool clockwise = true,
  }) {
    assert(offsetIsValid(arcEndDelta));
    assert(radiusIsValid(radius));

    final int pointCount = pathRef.countPoints();
    double lastPointX, lastPointY;
    if (pointCount == 0) {
      lastPointX = lastPointY = 0;
    } else {
      int pointIndex = (pointCount - 1) * 2;
      lastPointX = pathRef.points[pointIndex++];
      lastPointY = pathRef.points[pointIndex];
    }
    arcToPoint(
      ui.Offset(lastPointX + arcEndDelta.dx, lastPointY + arcEndDelta.dy),
      radius: radius,
      rotation: rotation,
      largeArc: largeArc,
      clockwise: clockwise,
    );
  }

  /// Adds a new subpath that consists of a curve that forms the
  /// ellipse that fills the given rectangle.
  ///
  /// To add a circle, pass an appropriate rectangle as `oval`.
  /// [Rect.fromCircle] can be used to easily describe the circle's center
  /// [Offset] and radius.
  @override
  void addOval(ui.Rect oval) {
    _addOval(oval, SPathDirection.kCW, 0);
  }

  void _addOval(ui.Rect oval, int direction, int startIndex) {
    assert(rectIsValid(oval));
    assert(direction != SPathDirection.kUnknown);
    final bool isOval = _hasOnlyMoveTos();

    const double weight = SPath.scalarRoot2Over2;
    final double left = oval.left;
    final double right = oval.right;
    final double centerX = (left + right) / 2.0;
    final double top = oval.top;
    final double bottom = oval.bottom;
    final double centerY = (top + bottom) / 2.0;
    if (direction == SPathDirection.kCW) {
      moveTo(right, centerY);
      conicTo(right, bottom, centerX, bottom, weight);
      conicTo(left, bottom, left, centerY, weight);
      conicTo(left, top, centerX, top, weight);
      conicTo(right, top, right, centerY, weight);
    } else {
      moveTo(right, centerY);
      conicTo(right, top, centerX, top, weight);
      conicTo(left, top, left, centerY, weight);
      conicTo(left, bottom, centerX, bottom, weight);
      conicTo(right, bottom, right, centerY, weight);
    }
    close();
    pathRef.setIsOval(isOval, direction == SPathDirection.kCCW, 0);
    _resetAfterEdit();
    // AutoDisableDirectionCheck
    if (isOval) {
      _firstDirection = direction;
    } else {
      _firstDirection = SPathDirection.kUnknown;
    }
  }

  /// Appends arc to path, as the start of new contour. Arc added is part of
  /// ellipse bounded by oval, from startAngle through sweepAngle. Both
  /// startAngle and sweepAngle are measured in degrees,
  /// where zero degrees is aligned with the positive x-axis,
  /// and positive sweeps extends arc clockwise.
  ///
  /// If sweepAngle <= -360, or sweepAngle >= 360; and startAngle modulo 90
  /// is nearly zero, append oval instead of arc. Otherwise, sweepAngle values
  /// are treated modulo 360, and arc may or may not draw depending on numeric
  /// rounding.
  @override
  void addArc(ui.Rect oval, double startAngle, double sweepAngle) {
    assert(rectIsValid(oval));
    if (0 == sweepAngle) {
      return;
    }
    const double kFullCircleAngle = math.pi * 2;

    if (sweepAngle >= kFullCircleAngle || sweepAngle <= -kFullCircleAngle) {
      // We can treat the arc as an oval if it begins at one of our legal starting positions.
      final double startOver90 = startAngle / (math.pi / 2.0);
      final double startOver90I = (startOver90 + 0.5).floorToDouble();
      final double error = startOver90 - startOver90I;
      if (SPath.nearlyEqual(error, 0)) {
        // Index 1 is at startAngle == 0.
        double startIndex = startOver90I + 1.0 % 4.0;
        startIndex = startIndex < 0 ? startIndex + 4.0 : startIndex;
        _addOval(
          oval,
          sweepAngle > 0 ? SPathDirection.kCW : SPathDirection.kCCW,
          startIndex.toInt(),
        );
        return;
      }
    }
    arcTo(oval, startAngle, sweepAngle, true);
  }

  /// Adds a new subpath with a sequence of line segments that connect the given
  /// points.
  ///
  /// If `close` is true, a final line segment will be added that connects the
  /// last point to the first point.
  ///
  /// The `points` argument is interpreted as offsets from the origin.
  @override
  void addPolygon(List<ui.Offset> points, bool close) {
    final int pointCount = points.length;
    if (pointCount <= 0) {
      return;
    }
    final int pointIndex = pathRef.growForVerb(SPathVerb.kMove, 0);
    fLastMoveToIndex = pointIndex + 1;
    pathRef.setPoint(pointIndex, points[0].dx, points[0].dy);
    pathRef.growForRepeatedVerb(SPathVerb.kLine, pointCount - 1);
    for (int i = 1; i < pointCount; i++) {
      pathRef.setPoint(pointIndex + i, points[i].dx, points[i].dy);
    }
    if (close) {
      this.close();
    }
    _resetAfterEdit();
    _debugValidate();
  }

  /// Adds a new subpath that consists of the straight lines and
  /// curves needed to form the rounded rectangle described by the
  /// argument.
  @override
  void addRRect(ui.RRect rrect) {
    _addRRect(rrect, SPathDirection.kCW, 6);
  }

  void _addRRect(ui.RRect rrect, int direction, int startIndex) {
    assert(rrectIsValid(rrect));
    assert(direction != SPathDirection.kUnknown);

    final bool isRRect = _hasOnlyMoveTos();
    final ui.Rect bounds = rrect.outerRect;
    if (rrect.isRect || rrect.isEmpty) {
      // degenerate(rect) => radii points are collapsing.
      addRectWithDirection(bounds, direction, (startIndex + 1) ~/ 2);
    } else if (isRRectOval(rrect)) {
      // degenerate(oval) => line points are collapsing.
      _addOval(bounds, direction, startIndex ~/ 2);
    } else {
      const double weight = SPath.scalarRoot2Over2;
      final double left = bounds.left;
      final double right = bounds.right;
      final double top = bounds.top;
      final double bottom = bounds.bottom;
      final double width = right - left;
      final double height = bottom - top;
      // Proportionally scale down all radii to fit. Find the minimum ratio
      // of a side and the radii on that side (for all four sides) and use
      // that to scale down _all_ the radii. This algorithm is from the
      // W3 spec (http://www.w3.org/TR/css3-background/) section 5.5
      final double tlRadiusX = math.max(0, rrect.tlRadiusX);
      final double trRadiusX = math.max(0, rrect.trRadiusX);
      final double blRadiusX = math.max(0, rrect.blRadiusX);
      final double brRadiusX = math.max(0, rrect.brRadiusX);
      final double tlRadiusY = math.max(0, rrect.tlRadiusY);
      final double trRadiusY = math.max(0, rrect.trRadiusY);
      final double blRadiusY = math.max(0, rrect.blRadiusY);
      final double brRadiusY = math.max(0, rrect.brRadiusY);
      double scale = _computeMinScale(tlRadiusX, trRadiusX, width, 1.0);
      scale = _computeMinScale(blRadiusX, brRadiusX, width, scale);
      scale = _computeMinScale(tlRadiusY, trRadiusY, height, scale);
      scale = _computeMinScale(blRadiusY, brRadiusY, height, scale);

      // Inlined version of:
      moveTo(left, bottom - scale * blRadiusY);
      lineTo(left, top + scale * tlRadiusY);
      conicTo(left, top, left + scale * tlRadiusX, top, weight);
      lineTo(right - scale * trRadiusX, top);
      conicTo(right, top, right, top + scale * trRadiusY, weight);
      lineTo(right, bottom - scale * brRadiusY);
      conicTo(right, bottom, right - scale * brRadiusX, bottom, weight);
      lineTo(left + scale * blRadiusX, bottom);
      conicTo(left, bottom, left, bottom - scale * blRadiusY, weight);
      close();
      // SkAutoDisableDirectionCheck.
      _firstDirection = isRRect ? direction : SPathDirection.kUnknown;
      pathRef.setIsRRect(isRRect, direction == SPathDirection.kCCW, startIndex % 8, rrect);
    }
    _debugValidate();
  }

  /// Adds a new subpath that consists of the given `path` offset by the given
  /// `offset`.
  ///
  /// If `matrix4` is specified, the path will be transformed by this matrix
  /// after the matrix is translated by the given offset. The matrix is a 4x4
  /// matrix stored in column major order.
  @override
  void addPath(ui.Path path, ui.Offset offset, {Float64List? matrix4}) {
    addPathWithMode(
      path,
      offset.dx,
      offset.dy,
      matrix4 == null ? null : toMatrix32(matrix4),
      SPathAddPathMode.kAppend,
    );
  }

  /// Adds a new subpath that consists of the given `path` offset by the given
  /// `offset`, and using the given [mode].
  ///
  /// If `matrix4` is not null, the path will be transformed by this matrix
  /// after the matrix is translated by the given offset.
  void addPathWithMode(
    ui.Path path,
    double offsetX,
    double offsetY,
    Float32List? matrix4,
    int mode,
  ) {
    SurfacePath source = path as SurfacePath;
    if (source.pathRef.isEmpty) {
      return;
    }
    // Detect if we're trying to add ourself, set source to a copy.
    if (source.pathRef == pathRef) {
      source = SurfacePath.from(this);
    }

    final int previousPointCount = pathRef.countPoints();

    // Fast path add points,verbs if matrix doesn't have perspective and
    // we are not extending.
    if (mode == SPathAddPathMode.kAppend && (matrix4 == null || _isSimple2dTransform(matrix4))) {
      pathRef.append(source.pathRef);
    } else {
      bool firstVerb = true;
      final PathRefIterator iter = PathRefIterator(source.pathRef);
      final Float32List outPts = Float32List(PathRefIterator.kMaxBufferSize);
      int verb;
      while ((verb = iter.next(outPts)) != SPath.kDoneVerb) {
        switch (verb) {
          case SPath.kMoveVerb:
            final double point0X =
                matrix4 == null
                    ? outPts[0] + offsetX
                    : (matrix4[0] * (outPts[0] + offsetX)) +
                        (matrix4[4] * (outPts[1] + offsetY)) +
                        matrix4[12];
            final double point0Y =
                matrix4 == null
                    ? outPts[1] + offsetY
                    : (matrix4[1] * (outPts[0] + offsetX)) +
                        (matrix4[5] * (outPts[1] + offsetY)) +
                        matrix4[13] +
                        offsetY;
            if (firstVerb && !pathRef.isEmpty) {
              assert(mode == SPathAddPathMode.kExtend);
              // In case last contour is closed inject move to.
              _injectMoveToIfNeeded();
              double lastPointX;
              double lastPointY;
              if (previousPointCount == 0) {
                lastPointX = lastPointY = 0;
              } else {
                int listIndex = 2 * (previousPointCount - 1);
                lastPointX = pathRef.points[listIndex++];
                lastPointY = pathRef.points[listIndex];
              }
              // don't add lineTo if it is degenerate.
              if (fLastMoveToIndex <= 0 ||
                  (previousPointCount != 0) ||
                  lastPointX != point0X ||
                  lastPointY != point0Y) {
                lineTo(outPts[0], outPts[1]);
              }
            } else {
              moveTo(outPts[0], outPts[1]);
            }
          case SPath.kLineVerb:
            lineTo(outPts[2], outPts[3]);
          case SPath.kQuadVerb:
            _quadTo(outPts[2], outPts[3], outPts[4], outPts[5]);
          case SPath.kConicVerb:
            conicTo(outPts[2], outPts[3], outPts[4], outPts[5], iter.conicWeight);
          case SPath.kCubicVerb:
            cubicTo(outPts[2], outPts[3], outPts[4], outPts[5], outPts[6], outPts[7]);
          case SPath.kCloseVerb:
            close();
        }
        firstVerb = false;
      }
    }

    // Shift [fLastMoveToIndex] by existing point count.
    if (source.fLastMoveToIndex >= 0) {
      fLastMoveToIndex = previousPointCount + source.fLastMoveToIndex;
    }
    // Translate/transform all points.
    final int newPointCount = pathRef.countPoints();
    final Float32List points = pathRef.points;
    for (int p = previousPointCount * 2; p < (newPointCount * 2); p += 2) {
      if (matrix4 == null) {
        points[p] += offsetX;
        points[p + 1] += offsetY;
      } else {
        final double x = points[p];
        final double y = points[p + 1];
        points[p] = (matrix4[0] * x) + (matrix4[4] * y) + (matrix4[12] + offsetX);
        points[p + 1] = (matrix4[1] * x) + (matrix4[5] * y) + (matrix4[13] + offsetY);
      }
    }
    _resetAfterEdit();
  }

  /// Adds the given path to this path by extending the current segment of this
  /// path with the first segment of the given path.
  ///
  /// If `matrix4` is specified, the path will be transformed by this matrix
  /// after the matrix is translated by the given `offset`.  The matrix is a 4x4
  /// matrix stored in column major order.
  @override
  void extendWithPath(ui.Path path, ui.Offset offset, {Float64List? matrix4}) {
    assert(offsetIsValid(offset));
    addPathWithMode(
      path,
      offset.dx,
      offset.dy,
      matrix4 == null ? null : toMatrix32(matrix4),
      SPathAddPathMode.kExtend,
    );
  }

  /// Tests to see if the given point is within the path. (That is, whether the
  /// point would be in the visible portion of the path if the path was used
  /// with [Canvas.clipPath].)
  ///
  /// The `point` argument is interpreted as an offset from the origin.
  ///
  /// Returns true if the point is in the path, and false otherwise.
  ///
  /// Note: Not very efficient, it creates a canvas, plays path and calls
  /// Context2D isPointInPath. If performance becomes issue, retaining
  /// RawRecordingCanvas can remove create/remove rootElement cost.
  @override
  bool contains(ui.Offset point) {
    assert(offsetIsValid(point));
    if (pathRef.isEmpty) {
      return false;
    }
    // Check bounds including right/bottom.
    final ui.Rect bounds = getBounds();
    final double x = point.dx;
    final double y = point.dy;
    if (x < bounds.left || y < bounds.top || x > bounds.right || y > bounds.bottom) {
      return false;
    }
    final PathWinding windings = PathWinding(pathRef, point.dx, point.dy);
    final bool evenOddFill = ui.PathFillType.evenOdd == _fillType;
    int w = windings.w;
    if (evenOddFill) {
      w &= 1;
    }
    if (w != 0) {
      return true;
    }
    final int onCurveCount = windings.onCurveCount;
    if (onCurveCount <= 1) {
      return onCurveCount != 0;
    }
    if ((onCurveCount & 1) != 0 || evenOddFill) {
      return (onCurveCount & 1) != 0;
    }
    // If the point touches an even number of curves, and the fill is winding,
    // check for coincidence. Count coincidence as places where the on curve
    // points have identical tangents.
    final PathIterator iter = PathIterator(pathRef, true);
    final Float32List buffer = Float32List(8 + 10);
    final List<ui.Offset> tangents = <ui.Offset>[];
    bool done = false;
    do {
      final int oldCount = tangents.length;
      switch (iter.next(buffer)) {
        case SPath.kMoveVerb:
        case SPath.kCloseVerb:
          break;
        case SPath.kLineVerb:
          tangentLine(buffer, x, y, tangents);
        case SPath.kQuadVerb:
          tangentQuad(buffer, x, y, tangents);
        case SPath.kConicVerb:
          tangentConic(buffer, x, y, iter.conicWeight, tangents);
        case SPath.kCubicVerb:
          tangentCubic(buffer, x, y, tangents);
        case SPath.kDoneVerb:
          done = true;
      }
      if (tangents.length > oldCount) {
        final int last = tangents.length - 1;
        final ui.Offset tangent = tangents[last];
        if (SPath.nearlyEqual(lengthSquaredOffset(tangent), 0)) {
          tangents.removeAt(last);
        } else {
          for (int index = 0; index < last; ++index) {
            final ui.Offset test = tangents[index];
            final double crossProduct = test.dx * tangent.dy - test.dy * tangent.dx;
            if (SPath.nearlyEqual(crossProduct, 0) &&
                SPath.scalarSignedAsInt(tangent.dx * test.dx) <= 0 &&
                SPath.scalarSignedAsInt(tangent.dy * test.dy) <= 0) {
              final ui.Offset offset = tangents.removeAt(last);
              if (index != tangents.length) {
                tangents[index] = offset;
              }
              break;
            }
          }
        }
      }
    } while (!done);
    return tangents.isNotEmpty;
  }

  /// Returns a copy of the path with all the segments of every
  /// subpath translated by the given offset.
  @override
  SurfacePath shift(ui.Offset offset) => SurfacePath.shiftedFrom(this, offset.dx, offset.dy);

  /// Returns a copy of the path with all the segments of every
  /// sub path transformed by the given matrix.
  @override
  SurfacePath transform(Float64List matrix4) {
    final SurfacePath newPath = SurfacePath.from(this);
    newPath._transform(matrix4);
    return newPath;
  }

  void _transform(Float64List m) {
    pathRef.startEdit();
    final int pointCount = pathRef.countPoints();
    final Float32List points = pathRef.points;
    final int len = pointCount * 2;
    for (int i = 0; i < len; i += 2) {
      final double x = points[i];
      final double y = points[i + 1];
      final double w = 1.0 / ((m[3] * x) + (m[7] * y) + m[15]);
      final double transformedX = ((m[0] * x) + (m[4] * y) + m[12]) * w;
      final double transformedY = ((m[1] * x) + (m[5] * y) + m[13]) * w;
      points[i] = transformedX;
      points[i + 1] = transformedY;
    }
    // TODO(ferhat): optimize for axis aligned or scale/translate type transforms.
    _convexityType = SPathConvexityType.kUnknown;
  }

  void setConvexityType(int value) {
    _convexityType = value;
  }

  int _setComputedConvexity(int value) {
    assert(value != SPathConvexityType.kUnknown);
    setConvexityType(value);
    return value;
  }

  /// Returns the convexity type, computing if needed. Never returns kUnknown.
  int get convexityType {
    if (_convexityType != SPathConvexityType.kUnknown) {
      return _convexityType;
    }
    return _internalGetConvexity();
  }

  /// Returns the current convexity type, skips computing if unknown.
  ///
  /// Provides a signal to path users if convexity has been calculated in
  /// which case _firstDirection is a valid result.
  int getConvexityTypeOrUnknown() => _convexityType;

  /// Returns true if the path is convex. If necessary, it will first compute
  /// the convexity.
  bool get isConvex => SPathConvexityType.kConvex == convexityType;

  // Computes convexity and first direction.
  int _internalGetConvexity() {
    final Float32List pts = Float32List(20);
    PathIterator iter = PathIterator(pathRef, true);
    // Check to see if path changes direction more than three times as quick
    // concave test.
    int pointCount = pathRef.countPoints();
    // Last moveTo index may exceed point count if data comes from fuzzer.
    if (0 < fLastMoveToIndex && fLastMoveToIndex < pointCount) {
      pointCount = fLastMoveToIndex;
    }
    if (pointCount > 3) {
      int pointIndex = 0;
      // only consider the last of the initial move tos
      while (SPath.kMoveVerb == iter.next(pts)) {
        pointIndex++;
      }
      --pointIndex;
      final int convexity = Convexicator.bySign(pathRef, pointIndex, pointCount - pointIndex);
      if (SPathConvexityType.kConcave == convexity) {
        setConvexityType(SPathConvexityType.kConcave);
        return SPathConvexityType.kConcave;
      } else if (SPathConvexityType.kUnknown == convexity) {
        return SPathConvexityType.kUnknown;
      }
      iter = PathIterator(pathRef, true);
    } else if (!pathRef.isFinite) {
      return SPathConvexityType.kUnknown;
    }
    // Path passed quick concave check, now compute actual convexity.
    int contourCount = 0;
    int count;
    final Convexicator state = Convexicator();
    int verb;
    while ((verb = iter.next(pts)) != SPath.kDoneVerb) {
      switch (verb) {
        case SPath.kMoveVerb:
          // If we have more than  1 contour bail out.
          if (++contourCount > 1) {
            return _setComputedConvexity(SPathConvexityType.kConcave);
          }
          state.setMovePt(pts[0], pts[1]);
          count = 0;
        case SPath.kLineVerb:
          count = 1;
        case SPath.kQuadVerb:
          count = 2;
        case SPath.kConicVerb:
          count = 2;
        case SPath.kCubicVerb:
          count = 3;
        case SPath.kCloseVerb:
          if (!state.close()) {
            if (!state.isFinite) {
              return SPathConvexityType.kUnknown;
            }
            return _setComputedConvexity(SPathConvexityType.kConcave);
          }
          count = 0;
        default:
          return _setComputedConvexity(SPathConvexityType.kConcave);
      }
      final int len = count * 2;
      for (int i = 2; i <= len; i += 2) {
        if (!state.addPoint(pts[i], pts[i + 1])) {
          if (!state.isFinite) {
            return SPathConvexityType.kUnknown;
          }
          return _setComputedConvexity(SPathConvexityType.kConcave);
        }
      }
    }

    if (_firstDirection == SPathDirection.kUnknown) {
      if (state.firstDirection == SPathDirection.kUnknown && !pathRef.getBounds().isEmpty) {
        return _setComputedConvexity(
          state.reversals < 3 ? SPathConvexityType.kConvex : SPathConvexityType.kConcave,
        );
      }
      _firstDirection = state.firstDirection;
    }
    _setComputedConvexity(SPathConvexityType.kConvex);
    return _convexityType;
  }

  /// Computes the bounding rectangle for this path.
  ///
  /// A path containing only axis-aligned points on the same straight line will
  /// have no area, and therefore `Rect.isEmpty` will return true for such a
  /// path. Consider checking `rect.width + rect.height > 0.0` instead, or
  /// using the [computeMetrics] API to check the path length.
  ///
  /// For many more elaborate paths, the bounds may be inaccurate.  For example,
  /// when a path contains a circle, the points used to compute the bounds are
  /// the circle's implied control points, which form a square around the
  /// circle; if the circle has a transformation applied using [transform] then
  /// that square is rotated, and the (axis-aligned, non-rotated) bounding box
  /// therefore ends up grossly overestimating the actual area covered by the
  /// circle.
  // see https://skia.org/user/api/SkPath_Reference#SkPath_getBounds
  @override
  ui.Rect getBounds() {
    if (pathRef.isRRect != -1 || pathRef.isOval != -1) {
      return pathRef.getBounds();
    }
    if (!pathRef.fBoundsIsDirty && pathRef.cachedBounds != null) {
      return pathRef.cachedBounds!;
    }
    bool ltrbInitialized = false;
    double left = 0.0, top = 0.0, right = 0.0, bottom = 0.0;
    double minX = 0.0, maxX = 0.0, minY = 0.0, maxY = 0.0;
    final PathRefIterator iter = PathRefIterator(pathRef);
    final Float32List points = pathRef.points;
    int verb;
    CubicBounds? cubicBounds;
    QuadBounds? quadBounds;
    ConicBounds? conicBounds;
    while ((verb = iter.nextIndex()) != SPath.kDoneVerb) {
      final int pIndex = iter.iterIndex;
      switch (verb) {
        case SPath.kMoveVerb:
          minX = maxX = points[pIndex];
          minY = maxY = points[pIndex + 1];
        case SPath.kLineVerb:
          minX = maxX = points[pIndex + 2];
          minY = maxY = points[pIndex + 3];
        case SPath.kQuadVerb:
          quadBounds ??= QuadBounds();
          quadBounds.calculateBounds(points, pIndex);
          minX = quadBounds.minX;
          minY = quadBounds.minY;
          maxX = quadBounds.maxX;
          maxY = quadBounds.maxY;
        case SPath.kConicVerb:
          conicBounds ??= ConicBounds();
          conicBounds.calculateBounds(points, iter.conicWeight, pIndex);
          minX = conicBounds.minX;
          minY = conicBounds.minY;
          maxX = conicBounds.maxX;
          maxY = conicBounds.maxY;
        case SPath.kCubicVerb:
          cubicBounds ??= CubicBounds();
          cubicBounds.calculateBounds(points, pIndex);
          minX = cubicBounds.minX;
          minY = cubicBounds.minY;
          maxX = cubicBounds.maxX;
          maxY = cubicBounds.maxY;
      }
      if (!ltrbInitialized) {
        left = minX;
        right = maxX;
        top = minY;
        bottom = maxY;
        ltrbInitialized = true;
      } else {
        left = math.min(left, minX);
        right = math.max(right, maxX);
        top = math.min(top, minY);
        bottom = math.max(bottom, maxY);
      }
    }
    final ui.Rect newBounds =
        ltrbInitialized ? ui.Rect.fromLTRB(left, top, right, bottom) : ui.Rect.zero;
    pathRef.getBounds();
    pathRef.cachedBounds = newBounds;
    return newBounds;
  }

  /// Creates a [PathMetrics] object for this path.
  ///
  /// If `forceClosed` is set to true, the contours of the path will be measured
  /// as if they had been closed, even if they were not explicitly closed.
  @override
  SurfacePathMetrics computeMetrics({bool forceClosed = false}) {
    return SurfacePathMetrics(pathRef, forceClosed);
  }

  /// Detects if path is rounded rectangle.
  ///
  /// Returns rounded rectangle or null.
  ///
  /// Used for web optimization of physical shape represented as
  /// a persistent div.
  ui.RRect? toRoundedRect() => pathRef.getRRect();

  /// Detects if path is simple rectangle.
  ///
  /// Returns rectangle or null.
  ///
  /// Used for web optimization of physical shape represented as
  /// a persistent div. !Warning it does not detect if closed, don't use this
  /// for optimizing strokes.
  ui.Rect? toRect() => pathRef.getRect();

  /// Detects if path is a vertical or horizontal line.
  ///
  /// Returns LTRB or null.
  ///
  /// Used for web optimization of physical shape represented as
  /// a persistent div.
  ui.Rect? toStraightLine() => pathRef.getStraightLine();

  /// Detects if path is simple oval.
  ///
  /// Returns bounding rectangle or null.
  ///
  /// Used for web optimization of physical shape represented as
  /// a persistent div.
  ui.Rect? toCircle() => pathRef.isOval == -1 ? null : pathRef.getBounds();

  /// Returns if Path is empty.
  /// Empty Path may have FillType but has no points, verbs or weights.
  /// Constructor, reset and rewind makes SkPath empty.
  bool get isEmpty => 0 == pathRef.countVerbs();

  @override
  String toString() {
    String result = super.toString();
    assert(() {
      final StringBuffer sb = StringBuffer();
      sb.write('Path(');
      final PathRefIterator iter = PathRefIterator(pathRef);
      final Float32List points = pathRef.points;
      int verb;
      while ((verb = iter.nextIndex()) != SPath.kDoneVerb) {
        final int pIndex = iter.iterIndex;
        switch (verb) {
          case SPath.kMoveVerb:
            sb.write('MoveTo(${points[pIndex]}, ${points[pIndex + 1]})');
          case SPath.kLineVerb:
            sb.write('LineTo(${points[pIndex + 2]}, ${points[pIndex + 3]})');
          case SPath.kQuadVerb:
            sb.write(
              'Quad(${points[pIndex + 2]}, ${points[pIndex + 3]},'
              ' ${points[pIndex + 3]}, ${points[pIndex + 4]})',
            );
          case SPath.kConicVerb:
            sb.write(
              'Conic(${points[pIndex + 2]}, ${points[pIndex + 3]},'
              ' ${points[pIndex + 3]}, ${points[pIndex + 4]}, w = ${iter.conicWeight})',
            );
          case SPath.kCubicVerb:
            sb.write(
              'Cubic(${points[pIndex + 2]}, ${points[pIndex + 3]},'
              ' ${points[pIndex + 3]}, ${points[pIndex + 4]}, '
              ' ${points[pIndex + 5]}, ${points[pIndex + 6]})',
            );
          case SPath.kCloseVerb:
            sb.write('Close()');
        }
        if (iter.peek() != SPath.kDoneVerb) {
          sb.write(' ');
        }
      }
      sb.write(')');
      result = sb.toString();
      return true;
    }());
    return result;
  }
}

// Returns Offset if arc is lone point and should be approximated with
// moveTo/lineTo.
ui.Offset? _arcIsLonePoint(ui.Rect oval, double startAngle, double sweepAngle) {
  if (0 == sweepAngle && (0 == startAngle || 360.0 == startAngle)) {
    // This path can be used to move into and out of ovals. If not
    // treated as a special case the moves can distort the oval's
    // bounding box (and break the circle special case).
    return ui.Offset(oval.right, oval.center.dy);
  }
  return null;
}

// Computed scaling factor for opposing sides with corner radius given
// a [limit] max width or height.
double _computeMinScale(double radius1, double radius2, double limit, double scale) {
  final double totalRadius = radius1 + radius2;
  if (totalRadius <= limit) {
    // Radii fit within the limit so return existing scale factor.
    return scale;
  }
  return math.min(limit / totalRadius, scale);
}

bool _isSimple2dTransform(Float32List m) =>
    m[15] ==
        1.0 && // start reading from the last element to eliminate range checks in subsequent reads.
    m[14] == 0.0 && // z translation is NOT simple
    // m[13] - y translation is simple
    // m[12] - x translation is simple
    m[11] == 0.0 &&
    m[10] == 1.0 &&
    m[9] == 0.0 &&
    m[8] == 0.0 &&
    m[7] == 0.0 &&
    m[6] == 0.0 &&
    // m[5] - scale y is simple
    // m[4] - 2D rotation is simple
    m[3] == 0.0 &&
    m[2] == 0.0;
// m[1] - 2D rotation is simple
// m[0] - scale x is simple
