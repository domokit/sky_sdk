// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;
import 'dart:typed_data';

import 'package:ui/ui.dart' as ui;

import 'path_utils.dart';

/// Stores the path verbs, points and conic weights.
///
/// This is a Dart port of Skia SkPathRef class.
/// For reference Flutter Gallery average points array size is 5.9, max 25
/// we start with [_pointsCapacity] 10 to reduce allocations during growth.
///
/// Unlike native skia GenID is not supported since we don't have requirement
/// to update caches due to content changes.
class PathRef {
  PathRef()
    : fPoints = Float32List(kInitialPointsCapacity * 2),
      _fVerbs = Uint8List(kInitialVerbsCapacity) {
    _fPointsCapacity = kInitialPointsCapacity;
    _fVerbsCapacity = kInitialVerbsCapacity;
    _resetFields();
  }

  /// Creates a copy of the path by pointing new path to a current
  /// points,verbs and weights arrays. If original path is mutated by adding
  /// more verbs, this copy only returns path at the time of copy and shares
  /// typed arrays of original path.
  PathRef.shallowCopy(PathRef ref) : fPoints = ref.fPoints, _fVerbs = ref._fVerbs {
    _fVerbsCapacity = ref._fVerbsCapacity;
    _fVerbsLength = ref._fVerbsLength;

    _fPointsCapacity = ref._fPointsCapacity;
    _fPointsLength = ref._fPointsLength;

    _conicWeightsCapacity = ref._conicWeightsCapacity;
    _conicWeightsLength = ref._conicWeightsLength;
    _conicWeights = ref._conicWeights;
    fBoundsIsDirty = ref.fBoundsIsDirty;
    if (!fBoundsIsDirty) {
      fBounds = ref.fBounds;
      cachedBounds = ref.cachedBounds;
      fIsFinite = ref.fIsFinite;
    }
    fSegmentMask = ref.fSegmentMask;
    fIsOval = ref.fIsOval;
    fIsRRect = ref.fIsRRect;
    fIsRect = ref.fIsRect;
    fRRectOrOvalIsCCW = ref.fRRectOrOvalIsCCW;
    fRRectOrOvalStartIdx = ref.fRRectOrOvalStartIdx;
    debugValidate();
  }

  /// Returns a new path by translating [source] by [offsetX], [offsetY].
  PathRef.shiftedFrom(PathRef source, double offsetX, double offsetY)
    : fPoints = _fPointsFromSource(source, offsetX, offsetY),
      _fVerbs = _fVerbsFromSource(source) {
    _conicWeightsCapacity = source._conicWeightsCapacity;
    _conicWeightsLength = source._conicWeightsLength;
    if (source._conicWeights != null) {
      _conicWeights = Float32List(_conicWeightsCapacity);
      _conicWeights!.setAll(0, source._conicWeights!);
    }
    _fVerbsCapacity = source._fVerbsCapacity;
    _fVerbsLength = source._fVerbsLength;

    _fPointsCapacity = source._fPointsCapacity;
    _fPointsLength = source._fPointsLength;
    fBoundsIsDirty = source.fBoundsIsDirty;
    if (!fBoundsIsDirty) {
      fBounds = source.fBounds!.translate(offsetX, offsetY);
      cachedBounds = source.cachedBounds?.translate(offsetX, offsetY);
      fIsFinite = source.fIsFinite;
    }
    fSegmentMask = source.fSegmentMask;
    fIsOval = source.fIsOval;
    fIsRRect = source.fIsRRect;
    fIsRect = source.fIsRect;
    fRRectOrOvalIsCCW = source.fRRectOrOvalIsCCW;
    fRRectOrOvalStartIdx = source.fRRectOrOvalStartIdx;
    debugValidate();
  }

  // Value to use to check against to insert move(0,0) when a command
  // is added without moveTo.
  static const int kInitialLastMoveToIndex = -1;

  // SerializationOffsets
  static const int kLegacyRRectOrOvalStartIdx_SerializationShift = 28; // requires 3 bits, ignored.
  static const int kLegacyRRectOrOvalIsCCW_SerializationShift = 27; // requires 1 bit, ignored.
  static const int kLegacyIsRRect_SerializationShift = 26; // requires 1 bit, ignored.
  static const int kIsFinite_SerializationShift = 25; // requires 1 bit
  static const int kLegacyIsOval_SerializationShift = 24; // requires 1 bit, ignored.
  static const int kSegmentMask_SerializationShift = 0; // requires 4 bits (deprecated)

  static const int kInitialPointsCapacity = 8;
  static const int kInitialVerbsCapacity = 8;

  /// Bounds of points that define path.
  ui.Rect? fBounds;

  /// Computed tight bounds of path (may exclude curve control points).
  ui.Rect? cachedBounds;
  int _fPointsCapacity = 0;
  int _fPointsLength = 0;
  int _fVerbsCapacity = 0;
  Float32List fPoints;
  Uint8List _fVerbs;
  int _fVerbsLength = 0;
  int _conicWeightsCapacity = 0;
  Float32List? _conicWeights;
  int _conicWeightsLength = 0;

  // Resets state to initial except points and verbs storage.
  void _resetFields() {
    fBoundsIsDirty = true; // this also invalidates fIsFinite
    fSegmentMask = 0;
    fIsOval = false;
    fIsRRect = false;
    fIsRect = false;
    // The next two values don't matter unless fIsOval or fIsRRect are true.
    fRRectOrOvalIsCCW = false;
    fRRectOrOvalStartIdx = 0xAC;
    assert(() {
      debugValidate();
      return true;
    }());
  }

  /// Given a point index stores [x],[y].
  void setPoint(int pointIndex, double x, double y) {
    assert(pointIndex < _fPointsLength);
    final int index = pointIndex * 2;
    fPoints[index] = x;
    fPoints[index + 1] = y;
  }

  Float32List get points => fPoints;
  Float32List? get conicWeights => _conicWeights;

  int countPoints() => _fPointsLength;
  int countVerbs() => _fVerbsLength;
  int countWeights() => _conicWeightsLength;

  /// Convenience method for reading verb at index.
  int atVerb(int index) {
    return _fVerbs[index];
  }

  ui.Offset atPoint(int index) {
    return ui.Offset(fPoints[index * 2], fPoints[index * 2 + 1]);
  }

  double pointXAt(int index) => fPoints[index * 2];

  double pointYAt(int index) => fPoints[index * 2 + 1];

  double atWeight(int index) {
    return _conicWeights![index];
  }

  ///  Returns true if all of the points in this path are finite, meaning
  ///  there are no infinities and no NaNs.
  bool get isFinite {
    if (fBoundsIsDirty) {
      _computeBounds();
    }
    return fIsFinite;
  }

  ///  Returns a mask, where each bit corresponding to a SegmentMask is
  ///  set if the path contains 1 or more segments of that type.
  ///  Returns 0 for an empty path (no segments).
  int get segmentMasks => fSegmentMask;

  /// Returns start index if the path is an oval or -1 if not.
  ///
  /// Tracking whether a path is an oval is considered an
  /// optimization for performance and so some paths that are in
  /// fact ovals can report false.
  int get isOval => fIsOval ? fRRectOrOvalStartIdx : -1;
  bool get isOvalCCW => fRRectOrOvalIsCCW;

  int get isRRect => fIsRRect ? fRRectOrOvalStartIdx : -1;
  int get isRect => fIsRect ? fRRectOrOvalStartIdx : -1;
  ui.RRect? getRRect() => fIsRRect ? _getRRect() : null;
  ui.Rect? getRect() {
    /// Use _detectRect() for detection if explicitly addRect was used (fIsRect) or
    /// it is a potential due to moveTo + 3 lineTo verbs.
    if (fIsRect) {
      return ui.Rect.fromLTRB(atPoint(0).dx, atPoint(0).dy, atPoint(1).dx, atPoint(2).dy);
    } else {
      return _fVerbsLength == 4 ? _detectRect() : null;
    }
  }

  bool get isRectCCW => fRRectOrOvalIsCCW;

  bool get hasComputedBounds => !fBoundsIsDirty;

  /// Returns the bounds of the path's points. If the path contains 0 or 1
  /// points, the bounds is set to (0,0,0,0), and isEmpty() will return true.
  /// Note: this bounds may be larger than the actual shape, since curves
  /// do not extend as far as their control points.
  ui.Rect getBounds() {
    if (fBoundsIsDirty) {
      _computeBounds();
    }
    return fBounds!;
  }

  /// Reconstructs Rect from path commands.
  ///
  /// Detects clockwise starting with horizontal line.
  ui.Rect? _detectRect() {
    assert(_fVerbs[0] == SPath.kMoveVerb);
    final double x0 = atPoint(0).dx;
    final double y0 = atPoint(0).dy;
    final double x1 = atPoint(1).dx;
    final double y1 = atPoint(1).dy;
    if (_fVerbs[1] != SPath.kLineVerb || y1 != y0) {
      return null;
    }
    final double width = x1 - x0;
    final double x2 = atPoint(2).dx;
    final double y2 = atPoint(2).dy;
    if (_fVerbs[2] != SPath.kLineVerb || x2 != x1) {
      return null;
    }
    final double height = y2 - y1;
    final double x3 = atPoint(3).dx;
    final double y3 = atPoint(3).dy;
    if (_fVerbs[3] != SPath.kLineVerb || y3 != y2) {
      return null;
    }
    if ((x2 - x3) != width || (y3 - y0) != height) {
      return null;
    }
    final double x = math.min(x0, x1);
    final double y = math.min(y0, y2);
    return ui.Rect.fromLTWH(x, y, width.abs(), height.abs());
  }

  /// Returns horizontal/vertical line bounds or null if not a line.
  ui.Rect? getStraightLine() {
    if (_fVerbsLength != 2 || _fVerbs[0] != SPath.kMoveVerb || _fVerbs[1] != SPath.kLineVerb) {
      return null;
    }
    final double x0 = fPoints[0];
    final double y0 = fPoints[1];
    final double x1 = fPoints[2];
    final double y1 = fPoints[3];
    if (y0 == y1 || x0 == x1) {
      return ui.Rect.fromLTRB(x0, y0, x1, y1);
    }
    return null;
  }

  /// Reconstructs RRect from path commands.
  ///
  /// Expect 4 Conics and lines between.
  /// Use conic points to calculate corner radius.
  ui.RRect _getRRect() {
    final ui.Rect bounds = getBounds();
    // Radii x,y of 4 corners
    final List<ui.Radius> radii = <ui.Radius>[];
    final PathRefIterator iter = PathRefIterator(this);
    final Float32List pts = Float32List(PathRefIterator.kMaxBufferSize);
    int verb = iter.next(pts);
    assert(SPath.kMoveVerb == verb);
    int cornerIndex = 0;
    while ((verb = iter.next(pts)) != SPath.kDoneVerb) {
      if (SPath.kConicVerb == verb) {
        final double controlPx = pts[2];
        final double controlPy = pts[3];
        final double vector1_0x = controlPx - pts[0];
        final double vector1_0y = controlPy - pts[1];
        final double vector2_1x = pts[4] - pts[2];
        final double vector2_1y = pts[5] - pts[3];
        double dx, dy;
        // Depending on the corner we have control point at same
        // horizontal position as startpoint or same vertical position.
        // The location delta of control point specifies corner radius.
        if (vector1_0x != 0.0) {
          // For CW : Top right or bottom left corners.
          dx = vector1_0x.abs();
          dy = vector2_1y.abs();
        } else if (vector1_0y != 0.0) {
          dx = vector2_1x.abs();
          dy = vector1_0y.abs();
        } else {
          dx = vector1_0x.abs();
          dy = vector1_0y.abs();
        }
        assert(() {
          final int checkCornerIndex =
              SPath.nearlyEqual(controlPx, bounds.left)
                  ? (SPath.nearlyEqual(controlPy, bounds.top)
                      ? _Corner.kUpperLeft
                      : _Corner.kLowerLeft)
                  : (SPath.nearlyEqual(controlPy, bounds.top)
                      ? _Corner.kUpperRight
                      : _Corner.kLowerRight);
          return checkCornerIndex == cornerIndex;
        }());
        radii.add(ui.Radius.elliptical(dx, dy));
        ++cornerIndex;
      } else {
        assert(() {
          if (verb == SPath.kLineVerb) {
            final bool isVerticalOrHorizontal =
                SPath.nearlyEqual(pts[2], pts[0]) || SPath.nearlyEqual(pts[3], pts[1]);
            assert(
              isVerticalOrHorizontal,
              'An RRect path must only contain vertical and horizontal lines.',
            );
          } else {
            assert(verb == SPath.kCloseVerb);
          }
          return true;
        }());
      }
    }
    return ui.RRect.fromRectAndCorners(
      bounds,
      topLeft: radii[_Corner.kUpperLeft],
      topRight: radii[_Corner.kUpperRight],
      bottomRight: radii[_Corner.kLowerRight],
      bottomLeft: radii[_Corner.kLowerLeft],
    );
  }

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is PathRef && equals(other);
  }

  @override
  int get hashCode => Object.hash(fSegmentMask, fPoints, _conicWeights, _fVerbs);

  bool equals(PathRef ref) {
    // We explicitly check fSegmentMask as a quick-reject. We could skip it,
    // since it is only a cache of info in the fVerbs, but its a fast way to
    // notice a difference
    if (fSegmentMask != ref.fSegmentMask) {
      return false;
    }

    final int pointCount = countPoints();
    if (pointCount != ref.countPoints()) {
      return false;
    }

    final int len = pointCount * 2;
    for (int i = 0; i < len; i++) {
      if (fPoints[i] != ref.fPoints[i]) {
        return false;
      }
    }

    if (_conicWeights == null) {
      if (ref._conicWeights != null) {
        return false;
      }
    } else {
      if (ref._conicWeights == null) {
        return false;
      }
      final int weightCount = _conicWeights!.length;
      if (ref._conicWeights!.length != weightCount) {
        return false;
      }
      for (int i = 0; i < weightCount; i++) {
        if (_conicWeights![i] != ref._conicWeights![i]) {
          return false;
        }
      }
    }
    final int verbCount = countVerbs();
    if (verbCount != ref.countVerbs()) {
      return false;
    }
    for (int i = 0; i < verbCount; i++) {
      if (_fVerbs[i] != ref._fVerbs[i]) {
        return false;
      }
    }
    if (ref.countVerbs() == 0) {
      assert(ref.countPoints() == 0);
    }
    return true;
  }

  static Float32List _fPointsFromSource(PathRef source, double offsetX, double offsetY) {
    final int sourceLength = source._fPointsLength;
    final int sourceCapacity = source._fPointsCapacity;
    final Float32List dest = Float32List(sourceCapacity * 2);
    final Float32List sourcePoints = source.points;
    final int len = sourceLength * 2;
    for (int i = 0; i < len; i += 2) {
      dest[i] = sourcePoints[i] + offsetX;
      dest[i + 1] = sourcePoints[i + 1] + offsetY;
    }
    return dest;
  }

  static Uint8List _fVerbsFromSource(PathRef source) {
    final Uint8List verbs = Uint8List(source._fVerbsCapacity);
    verbs.setAll(0, source._fVerbs);
    return verbs;
  }

  /// Copies contents from a source path [ref].
  void copy(PathRef ref, int additionalReserveVerbs, int additionalReservePoints) {
    ref.debugValidate();
    final int verbCount = ref.countVerbs();
    final int pointCount = ref.countPoints();
    final int weightCount = ref.countWeights();
    resetToSize(
      verbCount,
      pointCount,
      weightCount,
      additionalReserveVerbs,
      additionalReservePoints,
    );

    _fVerbs.setAll(0, ref._fVerbs);
    fPoints.setAll(0, ref.fPoints);
    if (ref._conicWeights == null) {
      _conicWeights = null;
    } else {
      _conicWeights!.setAll(0, ref._conicWeights!);
    }
    assert(verbCount == 0 || _fVerbs[0] == ref._fVerbs[0]);
    fBoundsIsDirty = ref.fBoundsIsDirty;
    if (!fBoundsIsDirty) {
      fBounds = ref.fBounds;
      cachedBounds = ref.cachedBounds;
      fIsFinite = ref.fIsFinite;
    }
    fSegmentMask = ref.fSegmentMask;
    fIsOval = ref.fIsOval;
    fIsRRect = ref.fIsRRect;
    fIsRect = ref.fIsRect;
    fRRectOrOvalIsCCW = ref.fRRectOrOvalIsCCW;
    fRRectOrOvalStartIdx = ref.fRRectOrOvalStartIdx;
    debugValidate();
  }

  void _resizePoints(int newLength) {
    if (newLength > _fPointsCapacity) {
      _fPointsCapacity = newLength + 10;
      final Float32List newPoints = Float32List(_fPointsCapacity * 2);
      newPoints.setAll(0, fPoints);
      fPoints = newPoints;
    }
    _fPointsLength = newLength;
  }

  void _resizeVerbs(int newLength) {
    if (newLength > _fVerbsCapacity) {
      _fVerbsCapacity = newLength + 8;
      final Uint8List newVerbs = Uint8List(_fVerbsCapacity);
      newVerbs.setAll(0, _fVerbs);
      _fVerbs = newVerbs;
    }
    _fVerbsLength = newLength;
  }

  void _resizeConicWeights(int newLength) {
    if (newLength > _conicWeightsCapacity) {
      _conicWeightsCapacity = newLength + 4;
      final Float32List newWeights = Float32List(_conicWeightsCapacity);
      if (_conicWeights != null) {
        newWeights.setAll(0, _conicWeights!);
      }
      _conicWeights = newWeights;
    }
    _conicWeightsLength = newLength;
  }

  void append(PathRef source) {
    final int pointCount = source.countPoints();
    final int curLength = _fPointsLength;
    final int newPointCount = curLength + pointCount;
    startEdit();
    _resizePoints(newPointCount);
    final Float32List sourcePoints = source.points;
    for (
      int source = pointCount * 2 - 1, dst = newPointCount * 2 - 1;
      source >= 0;
      source--, dst--
    ) {
      fPoints[dst] = sourcePoints[source];
    }
    final int verbCount = countVerbs();
    final int newVerbCount = source.countVerbs();
    _resizeVerbs(verbCount + newVerbCount);
    for (int i = 0; i < newVerbCount; i++) {
      _fVerbs[verbCount + i] = source._fVerbs[i];
    }
    if (source._conicWeights != null) {
      final int weightCount = countWeights();
      final int newWeightCount = source.countWeights();
      _resizeConicWeights(weightCount + newWeightCount);
      final Float32List sourceWeights = source._conicWeights!;
      final Float32List dest = _conicWeights!;
      for (int i = 0; i < newWeightCount; i++) {
        dest[weightCount + i] = sourceWeights[i];
      }
    }
    fBoundsIsDirty = true;
  }

  /// Doesn't read fSegmentMask, but (re)computes it from the verbs array
  int computeSegmentMask() {
    final Uint8List verbs = _fVerbs;
    int mask = 0;
    final int verbCount = countVerbs();
    for (int i = 0; i < verbCount; ++i) {
      switch (verbs[i]) {
        case SPath.kLineVerb:
          mask |= SPath.kLineSegmentMask;
        case SPath.kQuadVerb:
          mask |= SPath.kQuadSegmentMask;
        case SPath.kConicVerb:
          mask |= SPath.kConicSegmentMask;
        case SPath.kCubicVerb:
          mask |= SPath.kCubicSegmentMask;
        default:
          break;
      }
    }
    return mask;
  }

  /// This is incorrectly defined as instance method on SkPathRef although
  /// SkPath instance method first makes a copy of itself into out and
  /// then interpolates based on weight.
  static void interpolate(PathRef ending, double weight, PathRef out) {
    assert(out.countPoints() == ending.countPoints());
    final int count = out.countPoints() * 2;
    final Float32List outValues = out.points;
    final Float32List inValues = ending.points;
    for (int index = 0; index < count; ++index) {
      outValues[index] = outValues[index] * weight + inValues[index] * (1.0 - weight);
    }
    out.fBoundsIsDirty = true;
    out.startEdit();
  }

  /// Computes bounds and fIsFinite based on points.
  ///
  /// Used by getBounds() and cached.
  void _computeBounds() {
    debugValidate();
    assert(fBoundsIsDirty);
    final int pointCount = countPoints();
    fBoundsIsDirty = false;
    cachedBounds = null;
    double accum = 0;
    if (pointCount == 0) {
      fBounds = ui.Rect.zero;
      fIsFinite = true;
    } else {
      double minX, maxX, minY, maxY;
      minX = maxX = fPoints[0];
      accum *= minX;
      minY = maxY = fPoints[1];
      accum *= minY;
      final int len = 2 * pointCount;
      for (int i = 2; i < len; i += 2) {
        final double x = fPoints[i];
        accum *= x;
        final double y = fPoints[i + 1];
        accum *= y;
        minX = math.min(minX, x);
        minY = math.min(minY, y);
        maxX = math.max(maxX, x);
        maxY = math.max(maxY, y);
      }
      final bool allFinite = accum * 0 == 0;
      if (allFinite) {
        fBounds = ui.Rect.fromLTRB(minX, minY, maxX, maxY);
        fIsFinite = true;
      } else {
        fBounds = ui.Rect.zero;
        fIsFinite = false;
      }
    }
  }

  /// Sets to initial state preserving internal storage.
  void rewind() {
    _fPointsLength = 0;
    _fVerbsLength = 0;
    _conicWeightsLength = 0;
    _resetFields();
  }

  /// Resets the path ref with verbCount verbs and pointCount points, all
  /// uninitialized. Also allocates space for reserveVerb additional verbs
  /// and reservePoints additional points.
  void resetToSize(
    int verbCount,
    int pointCount,
    int conicCount, [
    int reserveVerbs = 0,
    int reservePoints = 0,
  ]) {
    debugValidate();
    fBoundsIsDirty = true; // this also invalidates fIsFinite

    fSegmentMask = 0;
    startEdit();

    _resizePoints(pointCount + reservePoints);
    _resizeVerbs(verbCount + reserveVerbs);
    _resizeConicWeights(conicCount);
    debugValidate();
  }

  /// Increases the verb count 1, records the new verb, and creates room for
  /// the requisite number of additional points. A pointer to the first point
  /// is returned. Any new points are uninitialized.
  int growForVerb(int verb, double weight) {
    debugValidate();
    int pCnt;
    int mask = 0;
    String? throwError;
    switch (verb) {
      case SPath.kMoveVerb:
        pCnt = 1;
      case SPath.kLineVerb:
        mask = SPath.kLineSegmentMask;
        pCnt = 1;
      case SPath.kQuadVerb:
        mask = SPath.kQuadSegmentMask;
        pCnt = 2;
      case SPath.kConicVerb:
        mask = SPath.kConicSegmentMask;
        pCnt = 2;
      case SPath.kCubicVerb:
        mask = SPath.kCubicSegmentMask;
        pCnt = 3;
      case SPath.kCloseVerb:
        pCnt = 0;
      case SPath.kDoneVerb:
        assert(() {
          throwError = 'growForVerb called for kDone';
          return true;
        }());
        pCnt = 0;
      default:
        assert(() {
          throwError = 'growForVerb called for unknown verb';
          return true;
        }());
        pCnt = 0;
    }

    if (throwError != null) {
      throw Exception(throwError);
    }

    fSegmentMask |= mask;
    fBoundsIsDirty = true; // this also invalidates fIsFinite
    startEdit();

    final int verbCount = countVerbs();
    _resizeVerbs(verbCount + 1);
    _fVerbs[verbCount] = verb;

    if (SPath.kConicVerb == verb) {
      final int weightCount = countWeights();
      _resizeConicWeights(weightCount + 1);
      _conicWeights![weightCount] = weight;
    }
    final int ptsIndex = _fPointsLength;
    _resizePoints(ptsIndex + pCnt);
    debugValidate();
    return ptsIndex;
  }

  /// Increases the verb count by numVbs and point count by the required amount.
  /// The new points are uninitialized. All the new verbs are set to the
  /// specified verb. If 'verb' is kConic_Verb, 'weights' will return a
  /// pointer to the uninitialized conic weights.
  ///
  /// This is an optimized version for [SPath.addPolygon].
  int growForRepeatedVerb(int /*SkPath::Verb*/ verb, int numVbs) {
    debugValidate();
    startEdit();
    int pCnt;
    int mask = 0;
    String? throwError;
    switch (verb) {
      case SPath.kMoveVerb:
        pCnt = numVbs;
      case SPath.kLineVerb:
        mask = SPath.kLineSegmentMask;
        pCnt = numVbs;
      case SPath.kQuadVerb:
        mask = SPath.kQuadSegmentMask;
        pCnt = 2 * numVbs;
      case SPath.kConicVerb:
        mask = SPath.kConicSegmentMask;
        pCnt = 2 * numVbs;
      case SPath.kCubicVerb:
        mask = SPath.kCubicSegmentMask;
        pCnt = 3 * numVbs;
      case SPath.kCloseVerb:
        pCnt = 0;
      case SPath.kDoneVerb:
        assert(() {
          throwError = 'growForVerb called for kDone';
          return true;
        }());
        pCnt = 0;
      default:
        assert(() {
          throwError = 'growForVerb called for unknown verb';
          return true;
        }());
        pCnt = 0;
    }

    if (throwError != null) {
      throw Exception(throwError);
    }

    fSegmentMask |= mask;
    fBoundsIsDirty = true; // this also invalidates fIsFinite
    startEdit();

    if (SPath.kConicVerb == verb) {
      _resizeConicWeights(countWeights() + numVbs);
    }
    final int verbCount = countVerbs();
    _resizeVerbs(verbCount + numVbs);
    for (int i = 0; i < numVbs; i++) {
      _fVerbs[verbCount + i] = verb;
    }

    final int ptsIndex = _fPointsLength;
    _resizePoints(ptsIndex + pCnt);
    debugValidate();
    return ptsIndex;
  }

  /// Concatenates all verbs from 'path' onto our own verbs array. Increases the point count by the
  /// number of points in 'path', and the conic weight count by the number of conics in 'path'.
  ///
  /// Returns pointers to the uninitialized points and conic weights data.
  void growForVerbsInPath(PathRef path) {
    debugValidate();
    startEdit();
    fSegmentMask |= path.fSegmentMask;
    fBoundsIsDirty = true; // this also invalidates fIsFinite

    final int numVerbs = path.countVerbs();
    if (numVerbs != 0) {
      final int curLength = countVerbs();
      _resizePoints(curLength + numVerbs);
      _fVerbs.setAll(curLength, path._fVerbs);
    }

    final int numPts = path.countPoints();
    if (numPts != 0) {
      final int curLength = countPoints();
      _resizePoints(curLength + numPts);
      fPoints.setAll(curLength * 2, path.fPoints);
    }

    final int numConics = path.countWeights();
    if (numConics != 0) {
      final int curLength = countWeights();
      _resizeConicWeights(curLength + numConics);
      final Float32List sourceWeights = path._conicWeights!;
      final Float32List destWeights = _conicWeights!;
      for (int i = 0; i < numConics; i++) {
        destWeights[curLength + i] = sourceWeights[i];
      }
    }

    debugValidate();
  }

  /// Resets higher level curve detection before a new edit is started.
  ///
  /// SurfacePath.addOval, addRRect will set these flags after the verbs and
  /// points are added.
  void startEdit() {
    fIsOval = false;
    fIsRRect = false;
    fIsRect = false;
    cachedBounds = null;
    fBoundsIsDirty = true;
  }

  void setIsOval(bool isOval, bool isCCW, int start) {
    fIsOval = isOval;
    fRRectOrOvalIsCCW = isCCW;
    fRRectOrOvalStartIdx = start;
  }

  void setIsRRect(bool isRRect, bool isCCW, int start, ui.RRect rrect) {
    fIsRRect = isRRect;
    fRRectOrOvalIsCCW = isCCW;
    fRRectOrOvalStartIdx = start;
  }

  void setIsRect(bool isRect, bool isCCW, int start) {
    fIsRect = isRect;
    fRRectOrOvalIsCCW = isCCW;
    fRRectOrOvalStartIdx = start;
  }

  Float32List getPoints() {
    debugValidate();
    return fPoints;
  }

  static const int kMinSize = 256;

  bool fBoundsIsDirty = true;
  bool fIsFinite = true; // only meaningful if bounds are valid

  bool fIsOval = false;
  bool fIsRRect = false;
  bool fIsRect = false;
  // Both the circle and rrect special cases have a notion of direction and starting point
  // The next two variables store that information for either.
  bool fRRectOrOvalIsCCW = false;
  int fRRectOrOvalStartIdx = -1;
  int fSegmentMask = 0;

  bool get isValid {
    if (fIsOval || fIsRRect) {
      // Currently we don't allow both of these to be set.
      if (fIsOval == fIsRRect) {
        return false;
      }
      if (fIsOval) {
        if (fRRectOrOvalStartIdx >= 4) {
          return false;
        }
      } else {
        if (fRRectOrOvalStartIdx >= 8) {
          return false;
        }
      }
    }
    if (fIsRect) {
      if (fIsOval || fIsRRect) {
        return false;
      }
      if (fRRectOrOvalStartIdx >= 4) {
        return false;
      }
    }

    if (!fBoundsIsDirty && !fBounds!.isEmpty) {
      bool isFinite = true;
      final ui.Rect bounds = fBounds!;
      final double boundsLeft = bounds.left;
      final double boundsTop = bounds.top;
      final double boundsRight = bounds.right;
      final double boundsBottom = bounds.bottom;
      final int len = _fPointsLength * 2;
      for (int i = 0; i < len; i += 2) {
        final double pointX = fPoints[i];
        final double pointY = fPoints[i + 1];
        const double tolerance = 0.0001;
        final bool pointIsFinite = pointX.isFinite && pointY.isFinite;
        if (pointIsFinite &&
            (pointX + tolerance < boundsLeft ||
                pointY + tolerance < boundsTop ||
                pointX - tolerance > boundsRight ||
                pointY - tolerance > boundsBottom)) {
          return false;
        }
        if (!pointIsFinite) {
          isFinite = false;
        }
      }
      if (fIsFinite != isFinite) {
        // Inconsistent state. Cached [fIsFinite] doesn't match what we found.
        return false;
      }
    }
    return true;
  }

  bool get isEmpty => countVerbs() == 0;

  void debugValidate() {
    assert(isValid);
  }

  /// Returns point index of maximum y in path points.
  int findMaxY(int pointIndex, int count) {
    assert(count > 0);
    // move to y component.
    double max = fPoints[pointIndex * 2 + 1];
    int firstIndex = pointIndex;
    for (int i = 1; i < count; i++) {
      final double y = fPoints[(pointIndex + i) * 2];
      if (y > max) {
        max = y;
        firstIndex = pointIndex + i;
      }
    }
    return firstIndex;
  }

  /// Returns index of point that is different from point at [index].
  ///
  /// Used to get previous/next points that dont coincide for calculating
  /// cross product at a point.
  int findDiffPoint(int index, int n, int inc) {
    int i = index;
    for (;;) {
      i = (i + inc) % n;
      if (i == index) {
        // we wrapped around, so abort
        break;
      }
      if (fPoints[index * 2] != fPoints[i * 2] || fPoints[index * 2 + 1] != fPoints[i * 2 + 1]) {
        // found a different point, success!
        break;
      }
    }
    return i;
  }
}

class PathRefIterator {
  PathRefIterator(this.pathRef) {
    _pointIndex = 0;
    if (!pathRef.isFinite) {
      // Don't allow iteration through non-finite points, prepare to return
      // done verb.
      _verbIndex = pathRef.countVerbs();
    }
  }

  final PathRef pathRef;
  int _conicWeightIndex = -1;
  int _verbIndex = 0;
  int _pointIndex = 0;

  /// Maximum buffer size required for points in [next] calls.
  static const int kMaxBufferSize = 8;

  int iterIndex = 0;

  /// Returns current point index.
  int get pointIndex => _pointIndex ~/ 2;

  /// Advances to start of next contour (move verb).
  ///
  /// Usage:
  ///   int startPointIndex = PathRefIterator._pointIndex;
  ///   int nextContourPointIndex = iter.skipToNextContour();
  ///   int pointCountInContour = nextContourPointIndex - startPointIndex;
  int skipToNextContour() {
    int verb = -1;
    int curPointIndex = _pointIndex;
    do {
      curPointIndex = _pointIndex;
      verb = nextIndex();
    } while (verb != SPath.kDoneVerb && (iterIndex == 0 || verb != SPath.kMoveVerb));
    return (verb == SPath.kDoneVerb ? _pointIndex : curPointIndex) ~/ 2;
  }

  /// Returns next verb and [iterIndex] with location of first point.
  int nextIndex() {
    if (_verbIndex == pathRef.countVerbs()) {
      return SPath.kDoneVerb;
    }
    final int verb = pathRef._fVerbs[_verbIndex++];
    switch (verb) {
      case SPath.kMoveVerb:
        iterIndex = _pointIndex;
        _pointIndex += 2;
      case SPath.kLineVerb:
        iterIndex = _pointIndex - 2;
        _pointIndex += 2;
      case SPath.kConicVerb:
        _conicWeightIndex++;
        iterIndex = _pointIndex - 2;
        _pointIndex += 4;
      case SPath.kQuadVerb:
        iterIndex = _pointIndex - 2;
        _pointIndex += 4;
      case SPath.kCubicVerb:
        iterIndex = _pointIndex - 2;
        _pointIndex += 6;
      case SPath.kCloseVerb:
        break;
      case SPath.kDoneVerb:
        assert(_verbIndex == pathRef.countVerbs());
      default:
        throw FormatException('Unsupport Path verb $verb');
    }
    return verb;
  }

  // Returns next verb and reads associated points into [outPts].
  int next(Float32List outPts) {
    if (_verbIndex == pathRef.countVerbs()) {
      return SPath.kDoneVerb;
    }
    final int verb = pathRef._fVerbs[_verbIndex++];
    final Float32List points = pathRef.points;
    int pointIndex = _pointIndex;
    switch (verb) {
      case SPath.kMoveVerb:
        outPts[0] = points[pointIndex++];
        outPts[1] = points[pointIndex++];
      case SPath.kLineVerb:
        outPts[0] = points[pointIndex - 2];
        outPts[1] = points[pointIndex - 1];
        outPts[2] = points[pointIndex++];
        outPts[3] = points[pointIndex++];
      case SPath.kConicVerb:
        _conicWeightIndex++;
        outPts[0] = points[pointIndex - 2];
        outPts[1] = points[pointIndex - 1];
        outPts[2] = points[pointIndex++];
        outPts[3] = points[pointIndex++];
        outPts[4] = points[pointIndex++];
        outPts[5] = points[pointIndex++];
      case SPath.kQuadVerb:
        outPts[0] = points[pointIndex - 2];
        outPts[1] = points[pointIndex - 1];
        outPts[2] = points[pointIndex++];
        outPts[3] = points[pointIndex++];
        outPts[4] = points[pointIndex++];
        outPts[5] = points[pointIndex++];
      case SPath.kCubicVerb:
        outPts[0] = points[pointIndex - 2];
        outPts[1] = points[pointIndex - 1];
        outPts[2] = points[pointIndex++];
        outPts[3] = points[pointIndex++];
        outPts[4] = points[pointIndex++];
        outPts[5] = points[pointIndex++];
        outPts[6] = points[pointIndex++];
        outPts[7] = points[pointIndex++];
      case SPath.kCloseVerb:
        break;
      case SPath.kDoneVerb:
        assert(_verbIndex == pathRef.countVerbs());
      default:
        throw FormatException('Unsupport Path verb $verb');
    }
    _pointIndex = pointIndex;
    return verb;
  }

  double get conicWeight => pathRef._conicWeights![_conicWeightIndex];

  int peek() => _verbIndex < pathRef.countVerbs() ? pathRef._fVerbs[_verbIndex] : SPath.kDoneVerb;
}

class _Corner {
  static const int kUpperLeft = 0;
  static const int kUpperRight = 1;
  static const int kLowerRight = 2;
  static const int kLowerLeft = 3;
}
