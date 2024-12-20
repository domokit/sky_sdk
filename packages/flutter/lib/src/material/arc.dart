// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/widgets.dart';
library;

import 'dart:math' as math;
import 'dart:ui' show lerpDouble;

import 'package:flutter/animation.dart';
import 'package:flutter/foundation.dart';

// How close the begin and end points must be to an axis to be considered
// vertical or horizontal.
const double _kOnAxisDelta = 2.0;

/// A [Tween] that interpolates an [Offset] along a circular arc.
///
/// This class specializes the interpolation of [Tween<Offset>] so that instead
/// of a straight line, the intermediate points follow the arc of a circle in a
/// manner consistent with Material Design principles.
///
/// The arc's radius is related to the bounding box that contains the [begin]
/// and [end] points. If the bounding box is taller than it is wide, then the
/// center of the circle will be horizontally aligned with the end point.
/// Otherwise the center of the circle will be aligned with the begin point.
/// The arc's sweep is always less than or equal to 90 degrees.
///
/// See also:
///
///  * [Tween], for a discussion on how to use interpolation objects.
///  * [MaterialRectArcTween], which extends this concept to interpolating [Rect]s.
class MaterialPointArcTween extends Tween<Offset> {
  /// Creates a [Tween] for animating [Offset]s along a circular arc.
  ///
  /// The [begin] and [end] properties must be non-null before the tween is
  /// first used, but the arguments can be null if the values are going to be
  /// filled in later.
  MaterialPointArcTween({super.begin, super.end});

  bool _dirty = true;

  void _initialize() {
    assert(this.begin != null);
    assert(this.end != null);

    final Offset begin = this.begin!;
    final Offset end = this.end!;

    // An explanation with a diagram can be found at https://goo.gl/vMSdRg
    final Offset delta = end - begin;
    final double deltaX = delta.dx.abs();
    final double deltaY = delta.dy.abs();
    final double distanceFromAtoB = delta.distance;
    final Offset c = Offset(end.dx, begin.dy);

    double sweepAngle() => 2.0 * math.asin(distanceFromAtoB / (2.0 * _radius!));

    if (deltaX > _kOnAxisDelta && deltaY > _kOnAxisDelta) {
      if (deltaX < deltaY) {
        _radius = distanceFromAtoB * distanceFromAtoB / (c - begin).distance / 2.0;
        _center = Offset(end.dx + _radius! * (begin.dx - end.dx).sign, end.dy);
        if (begin.dx < end.dx) {
          _beginAngle = sweepAngle() * (begin.dy - end.dy).sign;
          _endAngle = 0.0;
        } else {
          _beginAngle = math.pi + sweepAngle() * (end.dy - begin.dy).sign;
          _endAngle = math.pi;
        }
      } else {
        _radius = distanceFromAtoB * distanceFromAtoB / (c - end).distance / 2.0;
        _center = Offset(begin.dx, begin.dy + (end.dy - begin.dy).sign * _radius!);
        if (begin.dy < end.dy) {
          _beginAngle = -math.pi / 2.0;
          _endAngle = _beginAngle! + sweepAngle() * (end.dx - begin.dx).sign;
        } else {
          _beginAngle = math.pi / 2.0;
          _endAngle = _beginAngle! + sweepAngle() * (begin.dx - end.dx).sign;
        }
      }
      assert(_beginAngle != null);
      assert(_endAngle != null);
    } else {
      _beginAngle = null;
      _endAngle = null;
    }
    _dirty = false;
  }

  /// The center of the circular arc, null if [begin] and [end] are horizontally or
  /// vertically aligned, or if either is null.
  Offset? get center {
    if (begin == null || end == null) {
      return null;
    }
    if (_dirty) {
      _initialize();
    }
    return _center;
  }

  Offset? _center;

  /// The radius of the circular arc, null if [begin] and [end] are horizontally or
  /// vertically aligned, or if either is null.
  double? get radius {
    if (begin == null || end == null) {
      return null;
    }
    if (_dirty) {
      _initialize();
    }
    return _radius;
  }

  double? _radius;

  /// The beginning of the arc's sweep in radians, measured from the positive x
  /// axis. Positive angles turn clockwise.
  ///
  /// This will be null if [begin] and [end] are horizontally or vertically
  /// aligned, or if either is null.
  double? get beginAngle {
    if (begin == null || end == null) {
      return null;
    }
    if (_dirty) {
      _initialize();
    }
    return _beginAngle;
  }

  double? _beginAngle;

  /// The end of the arc's sweep in radians, measured from the positive x axis.
  /// Positive angles turn clockwise.
  ///
  /// This will be null if [begin] and [end] are horizontally or vertically
  /// aligned, or if either is null.
  double? get endAngle {
    if (begin == null || end == null) {
      return null;
    }
    if (_dirty) {
      _initialize();
    }
    return _beginAngle;
  }

  double? _endAngle;

  @override
  set begin(Offset? value) {
    if (value != begin) {
      super.begin = value;
      _dirty = true;
    }
  }

  @override
  set end(Offset? value) {
    if (value != end) {
      super.end = value;
      _dirty = true;
    }
  }

  @override
  Offset lerp(double t) {
    if (_dirty) {
      _initialize();
    }
    if (t == 0.0) {
      return begin!;
    }
    if (t == 1.0) {
      return end!;
    }
    if (_beginAngle == null || _endAngle == null) {
      return Offset.lerp(begin, end, t)!;
    }
    final double angle = lerpDouble(_beginAngle, _endAngle, t)!;
    final double x = math.cos(angle) * _radius!;
    final double y = math.sin(angle) * _radius!;
    return _center! + Offset(x, y);
  }

  @override
  String toString() {
    return '${objectRuntimeType(this, 'MaterialPointArcTween')}($begin \u2192 $end; center=$center, radius=$radius, beginAngle=$beginAngle, endAngle=$endAngle)';
  }
}

enum _CornerId { topLeft, topRight, bottomLeft, bottomRight }

class _Diagonal {
  const _Diagonal(this.beginId, this.endId);
  final _CornerId beginId;
  final _CornerId endId;
}

const List<_Diagonal> _allDiagonals = <_Diagonal>[
  _Diagonal(_CornerId.topLeft, _CornerId.bottomRight),
  _Diagonal(_CornerId.bottomRight, _CornerId.topLeft),
  _Diagonal(_CornerId.topRight, _CornerId.bottomLeft),
  _Diagonal(_CornerId.bottomLeft, _CornerId.topRight),
];

typedef _KeyFunc<T> = double Function(T input);

// Select the element for which the key function returns the maximum value.
T _maxBy<T>(Iterable<T> input, _KeyFunc<T> keyFunc) {
  late T maxValue;
  double? maxKey;
  for (final T value in input) {
    final double key = keyFunc(value);
    if (maxKey == null || key > maxKey) {
      maxValue = value;
      maxKey = key;
    }
  }
  return maxValue;
}

/// A [Tween] that interpolates a [Rect] by having its opposite corners follow
/// circular arcs.
///
/// This class specializes the interpolation of [Tween<Rect>] so that instead of
/// growing or shrinking linearly, opposite corners of the rectangle follow arcs
/// in a manner consistent with Material Design principles.
///
/// Specifically, the rectangle corners whose diagonals are closest to the overall
/// direction of the animation follow arcs defined with [MaterialPointArcTween].
///
/// See also:
///
///  * [MaterialRectCenterArcTween], which interpolates a rect along a circular
///    arc between the begin and end [Rect]'s centers.
///  * [Tween], for a discussion on how to use interpolation objects.
///  * [MaterialPointArcTween], the analog for [Offset] interpolation.
///  * [RectTween], which does a linear rectangle interpolation.
///  * [Hero.createRectTween], which can be used to specify the tween that defines
///    a hero's path.
class MaterialRectArcTween extends RectTween {
  /// Creates a [Tween] for animating [Rect]s along a circular arc.
  ///
  /// The [begin] and [end] properties must be non-null before the tween is
  /// first used, but the arguments can be null if the values are going to be
  /// filled in later.
  MaterialRectArcTween({super.begin, super.end});

  bool _dirty = true;

  void _initialize() {
    assert(begin != null);
    assert(end != null);
    final Offset centersVector = end!.center - begin!.center;
    final _Diagonal diagonal = _maxBy<_Diagonal>(
      _allDiagonals,
      (_Diagonal d) => _diagonalSupport(centersVector, d),
    );
    _beginArc = MaterialPointArcTween(
      begin: _cornerFor(begin!, diagonal.beginId),
      end: _cornerFor(end!, diagonal.beginId),
    );
    _endArc = MaterialPointArcTween(
      begin: _cornerFor(begin!, diagonal.endId),
      end: _cornerFor(end!, diagonal.endId),
    );
    _dirty = false;
  }

  double _diagonalSupport(Offset centersVector, _Diagonal diagonal) {
    final Offset delta = _cornerFor(begin!, diagonal.endId) - _cornerFor(begin!, diagonal.beginId);
    final double length = delta.distance;
    return centersVector.dx * delta.dx / length + centersVector.dy * delta.dy / length;
  }

  Offset _cornerFor(Rect rect, _CornerId id) {
    return switch (id) {
      _CornerId.topLeft => rect.topLeft,
      _CornerId.topRight => rect.topRight,
      _CornerId.bottomLeft => rect.bottomLeft,
      _CornerId.bottomRight => rect.bottomRight,
    };
  }

  /// The path of the corresponding [begin], [end] rectangle corners that lead
  /// the animation.
  MaterialPointArcTween? get beginArc {
    if (begin == null) {
      return null;
    }
    if (_dirty) {
      _initialize();
    }
    return _beginArc;
  }

  late MaterialPointArcTween _beginArc;

  /// The path of the corresponding [begin], [end] rectangle corners that trail
  /// the animation.
  MaterialPointArcTween? get endArc {
    if (end == null) {
      return null;
    }
    if (_dirty) {
      _initialize();
    }
    return _endArc;
  }

  late MaterialPointArcTween _endArc;

  @override
  set begin(Rect? value) {
    if (value != begin) {
      super.begin = value;
      _dirty = true;
    }
  }

  @override
  set end(Rect? value) {
    if (value != end) {
      super.end = value;
      _dirty = true;
    }
  }

  @override
  Rect lerp(double t) {
    if (_dirty) {
      _initialize();
    }
    if (t == 0.0) {
      return begin!;
    }
    if (t == 1.0) {
      return end!;
    }
    return Rect.fromPoints(_beginArc.lerp(t), _endArc.lerp(t));
  }

  @override
  String toString() {
    return '${objectRuntimeType(this, 'MaterialRectArcTween')}($begin \u2192 $end; beginArc=$beginArc, endArc=$endArc)';
  }
}

/// A [Tween] that interpolates a [Rect] by moving it along a circular arc from
/// [begin]'s [Rect.center] to [end]'s [Rect.center] while interpolating the
/// rectangle's width and height.
///
/// The arc that defines that center of the interpolated rectangle as it morphs
/// from [begin] to [end] is a [MaterialPointArcTween].
///
/// See also:
///
///  * [MaterialRectArcTween], A [Tween] that interpolates a [Rect] by having
///    its opposite corners follow circular arcs.
///  * [Tween], for a discussion on how to use interpolation objects.
///  * [MaterialPointArcTween], the analog for [Offset] interpolation.
///  * [RectTween], which does a linear rectangle interpolation.
///  * [Hero.createRectTween], which can be used to specify the tween that defines
///    a hero's path.
class MaterialRectCenterArcTween extends RectTween {
  /// Creates a [Tween] for animating [Rect]s along a circular arc.
  ///
  /// The [begin] and [end] properties must be non-null before the tween is
  /// first used, but the arguments can be null if the values are going to be
  /// filled in later.
  MaterialRectCenterArcTween({super.begin, super.end});

  bool _dirty = true;

  void _initialize() {
    assert(begin != null);
    assert(end != null);
    _centerArc = MaterialPointArcTween(begin: begin!.center, end: end!.center);
    _dirty = false;
  }

  /// If [begin] and [end] are non-null, returns a tween that interpolates along
  /// a circular arc between [begin]'s [Rect.center] and [end]'s [Rect.center].
  MaterialPointArcTween? get centerArc {
    if (begin == null || end == null) {
      return null;
    }
    if (_dirty) {
      _initialize();
    }
    return _centerArc;
  }

  late MaterialPointArcTween _centerArc;

  @override
  set begin(Rect? value) {
    if (value != begin) {
      super.begin = value;
      _dirty = true;
    }
  }

  @override
  set end(Rect? value) {
    if (value != end) {
      super.end = value;
      _dirty = true;
    }
  }

  @override
  Rect lerp(double t) {
    if (_dirty) {
      _initialize();
    }
    if (t == 0.0) {
      return begin!;
    }
    if (t == 1.0) {
      return end!;
    }
    final Offset center = _centerArc.lerp(t);
    final double width = lerpDouble(begin!.width, end!.width, t)!;
    final double height = lerpDouble(begin!.height, end!.height, t)!;
    return Rect.fromLTWH(center.dx - width / 2.0, center.dy - height / 2.0, width, height);
  }

  @override
  String toString() {
    return '${objectRuntimeType(this, 'MaterialRectCenterArcTween')}($begin \u2192 $end; centerArc=$centerArc)';
  }
}
