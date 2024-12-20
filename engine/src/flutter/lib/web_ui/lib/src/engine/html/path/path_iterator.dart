// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';

import 'package:ui/ui.dart' as ui;

import 'path_ref.dart';
import 'path_utils.dart';

// Iterates through path including generating closing segments.
class PathIterator {
  PathIterator(this.pathRef, bool forceClose)
    : _forceClose = forceClose,
      _verbCount = pathRef.countVerbs() {
    _pointIndex = 0;
    if (!pathRef.isFinite) {
      // Don't allow iteration through non-finite points, prepare to return
      // done verb.
      _verbIndex = pathRef.countVerbs();
    }
  }

  final PathRef pathRef;
  final bool _forceClose;
  final int _verbCount;

  bool _needClose = false;
  int _segmentState = SPathSegmentState.kEmptyContour;
  int _conicWeightIndex = -1;
  double _lastPointX = 0;
  double _lastPointY = 0;
  double _moveToX = 0;
  double _moveToY = 0;
  int _verbIndex = 0;
  int _pointIndex = 0;

  int get pathVerbIndex => _verbIndex;
  int get conicWeightIndex => _conicWeightIndex;

  /// Maximum buffer size required for points in [next] calls.
  static const int kMaxBufferSize = 8;

  /// Returns true if first contour on path is closed.
  bool isClosedContour() {
    if (_verbCount == 0 || _verbIndex == _verbCount) {
      return false;
    }
    if (_forceClose) {
      return true;
    }
    int verbIndex = 0;
    // Skip starting moveTo.
    if (pathRef.atVerb(verbIndex) == SPath.kMoveVerb) {
      ++verbIndex;
    }
    while (verbIndex < _verbCount) {
      final int verb = pathRef.atVerb(verbIndex++);
      if (SPath.kMoveVerb == verb) {
        break;
      }
      if (SPath.kCloseVerb == verb) {
        return true;
      }
    }
    return false;
  }

  int _autoClose(Float32List outPts) {
    if (_lastPointX != _moveToX || _lastPointY != _moveToY) {
      // Handle special case where comparison above will return true for
      // NaN != NaN although it should be false.
      if (_lastPointX.isNaN || _lastPointY.isNaN || _moveToX.isNaN || _moveToY.isNaN) {
        return SPath.kCloseVerb;
      }
      outPts[0] = _lastPointX;
      outPts[1] = _lastPointY;
      outPts[2] = _moveToX;
      outPts[3] = _moveToY;
      _lastPointX = _moveToX;
      _lastPointY = _moveToY;
      return SPath.kLineVerb;
    } else {
      outPts[0] = _moveToX;
      outPts[1] = _moveToY;
      return SPath.kCloseVerb;
    }
  }

  // Returns true if caller should use moveTo, false if last point of
  // previous primitive.
  ui.Offset _constructMoveTo() {
    if (_segmentState == SPathSegmentState.kAfterMove) {
      // Set the first return point to move point.
      _segmentState = SPathSegmentState.kAfterPrimitive;
      return ui.Offset(_moveToX, _moveToY);
    }
    return ui.Offset(pathRef.points[_pointIndex - 2], pathRef.points[_pointIndex - 1]);
  }

  int peek() {
    if (_verbIndex < pathRef.countVerbs()) {
      return pathRef.atVerb(_verbIndex);
    }
    if (_needClose && _segmentState == SPathSegmentState.kAfterPrimitive) {
      return (_lastPointX != _moveToX || _lastPointY != _moveToY)
          ? SPath.kLineVerb
          : SPath.kCloseVerb;
    }
    return SPath.kDoneVerb;
  }

  // Returns next verb and reads associated points into [outPts].
  int next(Float32List outPts) {
    if (_verbIndex == pathRef.countVerbs()) {
      // Close the curve if requested and if there is some curve to close
      if (_needClose && _segmentState == SPathSegmentState.kAfterPrimitive) {
        if (SPath.kLineVerb == _autoClose(outPts)) {
          return SPath.kLineVerb;
        }
        _needClose = false;
        return SPath.kCloseVerb;
      }
      return SPath.kDoneVerb;
    }
    int verb = pathRef.atVerb(_verbIndex++);
    switch (verb) {
      case SPath.kMoveVerb:
        if (_needClose) {
          // Move back one verb.
          _verbIndex--;
          final int autoVerb = _autoClose(outPts);
          if (autoVerb == SPath.kCloseVerb) {
            _needClose = false;
          }
          return autoVerb;
        }
        if (_verbIndex == _verbCount) {
          return SPath.kDoneVerb;
        }
        final double offsetX = pathRef.points[_pointIndex++];
        final double offsetY = pathRef.points[_pointIndex++];
        _moveToX = offsetX;
        _moveToY = offsetY;
        outPts[0] = offsetX;
        outPts[1] = offsetY;
        _segmentState = SPathSegmentState.kAfterMove;
        _lastPointX = _moveToX;
        _lastPointY = _moveToY;
        _needClose = _forceClose;
      case SPath.kLineVerb:
        final ui.Offset start = _constructMoveTo();
        final double offsetX = pathRef.points[_pointIndex++];
        final double offsetY = pathRef.points[_pointIndex++];
        outPts[0] = start.dx;
        outPts[1] = start.dy;
        outPts[2] = offsetX;
        outPts[3] = offsetY;
        _lastPointX = offsetX;
        _lastPointY = offsetY;
      case SPath.kConicVerb:
        _conicWeightIndex++;
        final ui.Offset start = _constructMoveTo();
        outPts[0] = start.dx;
        outPts[1] = start.dy;
        outPts[2] = pathRef.points[_pointIndex++];
        outPts[3] = pathRef.points[_pointIndex++];
        _lastPointX = outPts[4] = pathRef.points[_pointIndex++];
        _lastPointY = outPts[5] = pathRef.points[_pointIndex++];
      case SPath.kQuadVerb:
        final ui.Offset start = _constructMoveTo();
        outPts[0] = start.dx;
        outPts[1] = start.dy;
        outPts[2] = pathRef.points[_pointIndex++];
        outPts[3] = pathRef.points[_pointIndex++];
        _lastPointX = outPts[4] = pathRef.points[_pointIndex++];
        _lastPointY = outPts[5] = pathRef.points[_pointIndex++];
      case SPath.kCubicVerb:
        final ui.Offset start = _constructMoveTo();
        outPts[0] = start.dx;
        outPts[1] = start.dy;
        outPts[2] = pathRef.points[_pointIndex++];
        outPts[3] = pathRef.points[_pointIndex++];
        outPts[4] = pathRef.points[_pointIndex++];
        outPts[5] = pathRef.points[_pointIndex++];
        _lastPointX = outPts[6] = pathRef.points[_pointIndex++];
        _lastPointY = outPts[7] = pathRef.points[_pointIndex++];
      case SPath.kCloseVerb:
        verb = _autoClose(outPts);
        if (verb == SPath.kLineVerb) {
          // Move back one verb since we constructed line for this close verb.
          _verbIndex--;
        } else {
          _needClose = false;
          _segmentState = SPathSegmentState.kEmptyContour;
        }
        _lastPointX = _moveToX;
        _lastPointY = _moveToY;
      case SPath.kDoneVerb:
        assert(_verbIndex == pathRef.countVerbs());
      default:
        throw FormatException('Unsupport Path verb $verb');
    }
    return verb;
  }

  double get conicWeight => pathRef.atWeight(_conicWeightIndex);
}
