// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// See https://github.com/flutter/engine/blob/main/lib/ui/geometry.dart for
// documentation of APIs.
part of ui;

double toDegrees(double radians) {
  return radians * 180 / math.pi;
}

abstract class OffsetBase {
  const OffsetBase(this._dx, this._dy);

  final double _dx;
  final double _dy;
  bool get isInfinite => _dx >= double.infinity || _dy >= double.infinity;
  bool get isFinite => _dx.isFinite && _dy.isFinite;
  bool operator <(OffsetBase other) => _dx < other._dx && _dy < other._dy;
  bool operator <=(OffsetBase other) => _dx <= other._dx && _dy <= other._dy;
  bool operator >(OffsetBase other) => _dx > other._dx && _dy > other._dy;
  bool operator >=(OffsetBase other) => _dx >= other._dx && _dy >= other._dy;
  @override
  bool operator ==(Object other) {
    return other is OffsetBase && other._dx == _dx && other._dy == _dy;
  }

  @override
  int get hashCode => Object.hash(_dx, _dy);

  @override
  String toString() => 'OffsetBase(${_dx.toStringAsFixed(1)}, ${_dy.toStringAsFixed(1)})';
}

class Offset extends OffsetBase {
  const Offset(super.dx, super.dy);
  factory Offset.fromDirection(double direction, [double distance = 1.0]) {
    return Offset(distance * math.cos(direction), distance * math.sin(direction));
  }
  double get dx => _dx;
  double get dy => _dy;
  double get distance => math.sqrt(dx * dx + dy * dy);
  double get distanceSquared => dx * dx + dy * dy;
  double get direction => math.atan2(dy, dx);
  static const Offset zero = Offset(0.0, 0.0);
  // This is included for completeness, because [Size.infinite] exists.
  static const Offset infinite = Offset(double.infinity, double.infinity);
  Offset scale(double scaleX, double scaleY) => Offset(dx * scaleX, dy * scaleY);
  Offset translate(double translateX, double translateY) =>
      Offset(dx + translateX, dy + translateY);
  Offset operator -() => Offset(-dx, -dy);
  Offset operator -(Offset other) => Offset(dx - other.dx, dy - other.dy);
  Offset operator +(Offset other) => Offset(dx + other.dx, dy + other.dy);
  Offset operator *(double operand) => Offset(dx * operand, dy * operand);
  Offset operator /(double operand) => Offset(dx / operand, dy / operand);
  Offset operator ~/(double operand) =>
      Offset((dx ~/ operand).toDouble(), (dy ~/ operand).toDouble());
  Offset operator %(double operand) => Offset(dx % operand, dy % operand);
  Rect operator &(Size other) => Rect.fromLTWH(dx, dy, other.width, other.height);
  static Offset? lerp(Offset? a, Offset? b, double t) {
    if (b == null) {
      if (a == null) {
        return null;
      } else {
        return a * (1.0 - t);
      }
    } else {
      if (a == null) {
        return b * t;
      } else {
        return Offset(_lerpDouble(a.dx, b.dx, t), _lerpDouble(a.dy, b.dy, t));
      }
    }
  }

  @override
  bool operator ==(Object other) {
    return other is Offset && other.dx == dx && other.dy == dy;
  }

  @override
  int get hashCode => Object.hash(dx, dy);

  @override
  String toString() => 'Offset(${dx.toStringAsFixed(1)}, ${dy.toStringAsFixed(1)})';
}

class Size extends OffsetBase {
  const Size(super.width, super.height);
  // Used by the rendering library's _DebugSize hack.
  Size.copy(Size source) : super(source.width, source.height);
  const Size.square(double dimension) : super(dimension, dimension); // ignore: use_super_parameters
  const Size.fromWidth(double width) : super(width, double.infinity);
  const Size.fromHeight(double height) : super(double.infinity, height);
  const Size.fromRadius(double radius) : super(radius * 2.0, radius * 2.0);
  double get width => _dx;
  double get height => _dy;
  double get aspectRatio {
    if (height != 0.0) {
      return width / height;
    }
    if (width > 0.0) {
      return double.infinity;
    }
    if (width < 0.0) {
      return double.negativeInfinity;
    }
    return 0.0;
  }

  static const Size zero = Size(0.0, 0.0);
  static const Size infinite = Size(double.infinity, double.infinity);
  bool get isEmpty => width <= 0.0 || height <= 0.0;
  OffsetBase operator -(OffsetBase other) {
    if (other is Size) {
      return Offset(width - other.width, height - other.height);
    }
    if (other is Offset) {
      return Size(width - other.dx, height - other.dy);
    }
    throw ArgumentError(other);
  }

  Size operator +(Offset other) => Size(width + other.dx, height + other.dy);
  Size operator *(double operand) => Size(width * operand, height * operand);
  Size operator /(double operand) => Size(width / operand, height / operand);
  Size operator ~/(double operand) =>
      Size((width ~/ operand).toDouble(), (height ~/ operand).toDouble());
  Size operator %(double operand) => Size(width % operand, height % operand);
  double get shortestSide => math.min(width.abs(), height.abs());
  double get longestSide => math.max(width.abs(), height.abs());

  // Convenience methods that do the equivalent of calling the similarly named
  // methods on a Rect constructed from the given origin and this size.
  Offset topLeft(Offset origin) => origin;
  Offset topCenter(Offset origin) => Offset(origin.dx + width / 2.0, origin.dy);
  Offset topRight(Offset origin) => Offset(origin.dx + width, origin.dy);
  Offset centerLeft(Offset origin) => Offset(origin.dx, origin.dy + height / 2.0);
  Offset center(Offset origin) => Offset(origin.dx + width / 2.0, origin.dy + height / 2.0);
  Offset centerRight(Offset origin) => Offset(origin.dx + width, origin.dy + height / 2.0);
  Offset bottomLeft(Offset origin) => Offset(origin.dx, origin.dy + height);
  Offset bottomCenter(Offset origin) => Offset(origin.dx + width / 2.0, origin.dy + height);
  Offset bottomRight(Offset origin) => Offset(origin.dx + width, origin.dy + height);
  bool contains(Offset offset) {
    return offset.dx >= 0.0 && offset.dx < width && offset.dy >= 0.0 && offset.dy < height;
  }

  Size get flipped => Size(height, width);
  static Size? lerp(Size? a, Size? b, double t) {
    if (b == null) {
      if (a == null) {
        return null;
      } else {
        return a * (1.0 - t);
      }
    } else {
      if (a == null) {
        return b * t;
      } else {
        return Size(_lerpDouble(a.width, b.width, t), _lerpDouble(a.height, b.height, t));
      }
    }
  }

  // We don't compare the runtimeType because of _DebugSize in the framework.
  @override
  bool operator ==(Object other) {
    return other is Size && other._dx == _dx && other._dy == _dy;
  }

  @override
  int get hashCode => Object.hash(_dx, _dy);

  @override
  String toString() => 'Size(${width.toStringAsFixed(1)}, ${height.toStringAsFixed(1)})';
}

class Rect {
  const Rect.fromLTRB(this.left, this.top, this.right, this.bottom);

  const Rect.fromLTWH(double left, double top, double width, double height)
    : this.fromLTRB(left, top, left + width, top + height);

  Rect.fromCircle({required Offset center, required double radius})
    : this.fromCenter(center: center, width: radius * 2, height: radius * 2);

  Rect.fromCenter({required Offset center, required double width, required double height})
    : this.fromLTRB(
        center.dx - width / 2,
        center.dy - height / 2,
        center.dx + width / 2,
        center.dy + height / 2,
      );

  Rect.fromPoints(Offset a, Offset b)
    : this.fromLTRB(
        math.min(a.dx, b.dx),
        math.min(a.dy, b.dy),
        math.max(a.dx, b.dx),
        math.max(a.dy, b.dy),
      );

  final double left;
  final double top;
  final double right;
  final double bottom;
  double get width => right - left;
  double get height => bottom - top;
  Size get size => Size(width, height);
  bool get hasNaN => left.isNaN || top.isNaN || right.isNaN || bottom.isNaN;
  static const Rect zero = Rect.fromLTRB(0.0, 0.0, 0.0, 0.0);

  static const double _giantScalar = 1.0E+9; // matches kGiantRect from layer.h
  static const Rect largest = Rect.fromLTRB(
    -_giantScalar,
    -_giantScalar,
    _giantScalar,
    _giantScalar,
  );
  // included for consistency with Offset and Size
  bool get isInfinite {
    return left >= double.infinity ||
        top >= double.infinity ||
        right >= double.infinity ||
        bottom >= double.infinity;
  }

  bool get isFinite => left.isFinite && top.isFinite && right.isFinite && bottom.isFinite;
  bool get isEmpty => left >= right || top >= bottom;
  Rect shift(Offset offset) {
    return Rect.fromLTRB(left + offset.dx, top + offset.dy, right + offset.dx, bottom + offset.dy);
  }

  Rect translate(double translateX, double translateY) {
    return Rect.fromLTRB(
      left + translateX,
      top + translateY,
      right + translateX,
      bottom + translateY,
    );
  }

  Rect inflate(double delta) {
    return Rect.fromLTRB(left - delta, top - delta, right + delta, bottom + delta);
  }

  Rect deflate(double delta) => inflate(-delta);
  Rect intersect(Rect other) {
    return Rect.fromLTRB(
      math.max(left, other.left),
      math.max(top, other.top),
      math.min(right, other.right),
      math.min(bottom, other.bottom),
    );
  }

  Rect expandToInclude(Rect other) {
    return Rect.fromLTRB(
      math.min(left, other.left),
      math.min(top, other.top),
      math.max(right, other.right),
      math.max(bottom, other.bottom),
    );
  }

  bool overlaps(Rect other) {
    if (right <= other.left || other.right <= left) {
      return false;
    }
    if (bottom <= other.top || other.bottom <= top) {
      return false;
    }
    return true;
  }

  double get shortestSide => math.min(width.abs(), height.abs());
  double get longestSide => math.max(width.abs(), height.abs());
  Offset get topLeft => Offset(left, top);
  Offset get topCenter => Offset(left + width / 2.0, top);
  Offset get topRight => Offset(right, top);
  Offset get centerLeft => Offset(left, top + height / 2.0);
  Offset get center => Offset(left + width / 2.0, top + height / 2.0);
  Offset get centerRight => Offset(right, top + height / 2.0);
  Offset get bottomLeft => Offset(left, bottom);
  Offset get bottomCenter => Offset(left + width / 2.0, bottom);
  Offset get bottomRight => Offset(right, bottom);
  bool contains(Offset offset) {
    return offset.dx >= left && offset.dx < right && offset.dy >= top && offset.dy < bottom;
  }

  static Rect? lerp(Rect? a, Rect? b, double t) {
    if (b == null) {
      if (a == null) {
        return null;
      } else {
        final double k = 1.0 - t;
        return Rect.fromLTRB(a.left * k, a.top * k, a.right * k, a.bottom * k);
      }
    } else {
      if (a == null) {
        return Rect.fromLTRB(b.left * t, b.top * t, b.right * t, b.bottom * t);
      } else {
        return Rect.fromLTRB(
          _lerpDouble(a.left, b.left, t),
          _lerpDouble(a.top, b.top, t),
          _lerpDouble(a.right, b.right, t),
          _lerpDouble(a.bottom, b.bottom, t),
        );
      }
    }
  }

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (runtimeType != other.runtimeType) {
      return false;
    }
    return other is Rect &&
        other.left == left &&
        other.top == top &&
        other.right == right &&
        other.bottom == bottom;
  }

  @override
  int get hashCode => Object.hash(left, top, right, bottom);

  @override
  String toString() =>
      'Rect.fromLTRB(${left.toStringAsFixed(1)}, ${top.toStringAsFixed(1)}, ${right.toStringAsFixed(1)}, ${bottom.toStringAsFixed(1)})';
}

class Radius {
  const Radius.circular(double radius) : this.elliptical(radius, radius);
  const Radius.elliptical(this.x, this.y);
  final double x;
  final double y;
  static const Radius zero = Radius.circular(0.0);
  Radius clamp({Radius? minimum, Radius? maximum}) {
    minimum ??= const Radius.circular(-double.infinity);
    maximum ??= const Radius.circular(double.infinity);
    return Radius.elliptical(
      clampDouble(x, minimum.x, maximum.x),
      clampDouble(y, minimum.y, maximum.y),
    );
  }

  Radius clampValues({double? minimumX, double? minimumY, double? maximumX, double? maximumY}) {
    return Radius.elliptical(
      clampDouble(x, minimumX ?? -double.infinity, maximumX ?? double.infinity),
      clampDouble(y, minimumY ?? -double.infinity, maximumY ?? double.infinity),
    );
  }

  Radius operator -() => Radius.elliptical(-x, -y);
  Radius operator -(Radius other) => Radius.elliptical(x - other.x, y - other.y);
  Radius operator +(Radius other) => Radius.elliptical(x + other.x, y + other.y);
  Radius operator *(double operand) => Radius.elliptical(x * operand, y * operand);
  Radius operator /(double operand) => Radius.elliptical(x / operand, y / operand);
  Radius operator ~/(double operand) =>
      Radius.elliptical((x ~/ operand).toDouble(), (y ~/ operand).toDouble());
  Radius operator %(double operand) => Radius.elliptical(x % operand, y % operand);
  static Radius? lerp(Radius? a, Radius? b, double t) {
    if (b == null) {
      if (a == null) {
        return null;
      } else {
        final double k = 1.0 - t;
        return Radius.elliptical(a.x * k, a.y * k);
      }
    } else {
      if (a == null) {
        return Radius.elliptical(b.x * t, b.y * t);
      } else {
        return Radius.elliptical(_lerpDouble(a.x, b.x, t), _lerpDouble(a.y, b.y, t));
      }
    }
  }

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (runtimeType != other.runtimeType) {
      return false;
    }

    return other is Radius && other.x == x && other.y == y;
  }

  @override
  int get hashCode => Object.hash(x, y);

  @override
  String toString() {
    return x == y
        ? 'Radius.circular(${x.toStringAsFixed(1)})'
        : 'Radius.elliptical(${x.toStringAsFixed(1)}, '
            '${y.toStringAsFixed(1)})';
  }
}

class RRect {
  const RRect.fromLTRBXY(
    double left,
    double top,
    double right,
    double bottom,
    double radiusX,
    double radiusY,
  ) : this._raw(
        top: top,
        left: left,
        right: right,
        bottom: bottom,
        tlRadiusX: radiusX,
        tlRadiusY: radiusY,
        trRadiusX: radiusX,
        trRadiusY: radiusY,
        blRadiusX: radiusX,
        blRadiusY: radiusY,
        brRadiusX: radiusX,
        brRadiusY: radiusY,
        uniformRadii: radiusX == radiusY,
      );

  RRect.fromLTRBR(double left, double top, double right, double bottom, Radius radius)
    : this._raw(
        top: top,
        left: left,
        right: right,
        bottom: bottom,
        tlRadiusX: radius.x,
        tlRadiusY: radius.y,
        trRadiusX: radius.x,
        trRadiusY: radius.y,
        blRadiusX: radius.x,
        blRadiusY: radius.y,
        brRadiusX: radius.x,
        brRadiusY: radius.y,
        uniformRadii: radius.x == radius.y,
      );

  RRect.fromRectXY(Rect rect, double radiusX, double radiusY)
    : this._raw(
        top: rect.top,
        left: rect.left,
        right: rect.right,
        bottom: rect.bottom,
        tlRadiusX: radiusX,
        tlRadiusY: radiusY,
        trRadiusX: radiusX,
        trRadiusY: radiusY,
        blRadiusX: radiusX,
        blRadiusY: radiusY,
        brRadiusX: radiusX,
        brRadiusY: radiusY,
        uniformRadii: radiusX == radiusY,
      );

  RRect.fromRectAndRadius(Rect rect, Radius radius)
    : this._raw(
        top: rect.top,
        left: rect.left,
        right: rect.right,
        bottom: rect.bottom,
        tlRadiusX: radius.x,
        tlRadiusY: radius.y,
        trRadiusX: radius.x,
        trRadiusY: radius.y,
        blRadiusX: radius.x,
        blRadiusY: radius.y,
        brRadiusX: radius.x,
        brRadiusY: radius.y,
        uniformRadii: radius.x == radius.y,
      );

  RRect.fromLTRBAndCorners(
    double left,
    double top,
    double right,
    double bottom, {
    Radius topLeft = Radius.zero,
    Radius topRight = Radius.zero,
    Radius bottomRight = Radius.zero,
    Radius bottomLeft = Radius.zero,
  }) : this._raw(
         top: top,
         left: left,
         right: right,
         bottom: bottom,
         tlRadiusX: topLeft.x,
         tlRadiusY: topLeft.y,
         trRadiusX: topRight.x,
         trRadiusY: topRight.y,
         blRadiusX: bottomLeft.x,
         blRadiusY: bottomLeft.y,
         brRadiusX: bottomRight.x,
         brRadiusY: bottomRight.y,
         uniformRadii:
             topLeft.x == topLeft.y &&
             topLeft.x == topRight.x &&
             topLeft.x == topRight.y &&
             topLeft.x == bottomLeft.x &&
             topLeft.x == bottomLeft.y &&
             topLeft.x == bottomRight.x &&
             topLeft.x == bottomRight.y,
       );

  RRect.fromRectAndCorners(
    Rect rect, {
    Radius topLeft = Radius.zero,
    Radius topRight = Radius.zero,
    Radius bottomRight = Radius.zero,
    Radius bottomLeft = Radius.zero,
  }) : this._raw(
         top: rect.top,
         left: rect.left,
         right: rect.right,
         bottom: rect.bottom,
         tlRadiusX: topLeft.x,
         tlRadiusY: topLeft.y,
         trRadiusX: topRight.x,
         trRadiusY: topRight.y,
         blRadiusX: bottomLeft.x,
         blRadiusY: bottomLeft.y,
         brRadiusX: bottomRight.x,
         brRadiusY: bottomRight.y,
         uniformRadii:
             topLeft.x == topLeft.y &&
             topLeft.x == topRight.x &&
             topLeft.x == topRight.y &&
             topLeft.x == bottomLeft.x &&
             topLeft.x == bottomLeft.y &&
             topLeft.x == bottomRight.x &&
             topLeft.x == bottomRight.y,
       );

  const RRect._raw({
    this.left = 0.0,
    this.top = 0.0,
    this.right = 0.0,
    this.bottom = 0.0,
    this.tlRadiusX = 0.0,
    this.tlRadiusY = 0.0,
    this.trRadiusX = 0.0,
    this.trRadiusY = 0.0,
    this.brRadiusX = 0.0,
    this.brRadiusY = 0.0,
    this.blRadiusX = 0.0,
    this.blRadiusY = 0.0,
    bool uniformRadii = false,
  }) : assert(tlRadiusX >= 0),
       assert(tlRadiusY >= 0),
       assert(trRadiusX >= 0),
       assert(trRadiusY >= 0),
       assert(brRadiusX >= 0),
       assert(brRadiusY >= 0),
       assert(blRadiusX >= 0),
       assert(blRadiusY >= 0),
       webOnlyUniformRadii = uniformRadii;

  final double left;
  final double top;
  final double right;
  final double bottom;
  final double tlRadiusX;
  final double tlRadiusY;
  Radius get tlRadius => Radius.elliptical(tlRadiusX, tlRadiusY);
  final double trRadiusX;
  final double trRadiusY;
  Radius get trRadius => Radius.elliptical(trRadiusX, trRadiusY);
  final double brRadiusX;
  final double brRadiusY;
  Radius get brRadius => Radius.elliptical(brRadiusX, brRadiusY);
  final double blRadiusX;
  final double blRadiusY;
  // webOnly
  final bool webOnlyUniformRadii;
  Radius get blRadius => Radius.elliptical(blRadiusX, blRadiusY);
  static const RRect zero = RRect._raw();

  RRect shift(Offset offset) {
    return RRect._raw(
      left: left + offset.dx,
      top: top + offset.dy,
      right: right + offset.dx,
      bottom: bottom + offset.dy,
      tlRadiusX: tlRadiusX,
      tlRadiusY: tlRadiusY,
      trRadiusX: trRadiusX,
      trRadiusY: trRadiusY,
      blRadiusX: blRadiusX,
      blRadiusY: blRadiusY,
      brRadiusX: brRadiusX,
      brRadiusY: brRadiusY,
    );
  }

  RRect inflate(double delta) {
    return RRect._raw(
      left: left - delta,
      top: top - delta,
      right: right + delta,
      bottom: bottom + delta,
      tlRadiusX: math.max(0, tlRadiusX + delta),
      tlRadiusY: math.max(0, tlRadiusY + delta),
      trRadiusX: math.max(0, trRadiusX + delta),
      trRadiusY: math.max(0, trRadiusY + delta),
      blRadiusX: math.max(0, blRadiusX + delta),
      blRadiusY: math.max(0, blRadiusY + delta),
      brRadiusX: math.max(0, brRadiusX + delta),
      brRadiusY: math.max(0, brRadiusY + delta),
    );
  }

  RRect deflate(double delta) => inflate(-delta);
  double get width => right - left;
  double get height => bottom - top;
  Rect get outerRect => Rect.fromLTRB(left, top, right, bottom);
  Rect get safeInnerRect {
    const double kInsetFactor = 0.29289321881; // 1-cos(pi/4)

    final double leftRadius = math.max(blRadiusX, tlRadiusX);
    final double topRadius = math.max(tlRadiusY, trRadiusY);
    final double rightRadius = math.max(trRadiusX, brRadiusX);
    final double bottomRadius = math.max(brRadiusY, blRadiusY);

    return Rect.fromLTRB(
      left + leftRadius * kInsetFactor,
      top + topRadius * kInsetFactor,
      right - rightRadius * kInsetFactor,
      bottom - bottomRadius * kInsetFactor,
    );
  }

  Rect get middleRect {
    final double leftRadius = math.max(blRadiusX, tlRadiusX);
    final double topRadius = math.max(tlRadiusY, trRadiusY);
    final double rightRadius = math.max(trRadiusX, brRadiusX);
    final double bottomRadius = math.max(brRadiusY, blRadiusY);
    return Rect.fromLTRB(
      left + leftRadius,
      top + topRadius,
      right - rightRadius,
      bottom - bottomRadius,
    );
  }

  Rect get wideMiddleRect {
    final double topRadius = math.max(tlRadiusY, trRadiusY);
    final double bottomRadius = math.max(brRadiusY, blRadiusY);
    return Rect.fromLTRB(left, top + topRadius, right, bottom - bottomRadius);
  }

  Rect get tallMiddleRect {
    final double leftRadius = math.max(blRadiusX, tlRadiusX);
    final double rightRadius = math.max(trRadiusX, brRadiusX);
    return Rect.fromLTRB(left + leftRadius, top, right - rightRadius, bottom);
  }

  bool get isEmpty => left >= right || top >= bottom;
  bool get isFinite => left.isFinite && top.isFinite && right.isFinite && bottom.isFinite;
  bool get isRect {
    return (tlRadiusX == 0.0 || tlRadiusY == 0.0) &&
        (trRadiusX == 0.0 || trRadiusY == 0.0) &&
        (blRadiusX == 0.0 || blRadiusY == 0.0) &&
        (brRadiusX == 0.0 || brRadiusY == 0.0);
  }

  bool get isStadium {
    return tlRadius == trRadius &&
        trRadius == brRadius &&
        brRadius == blRadius &&
        (width <= 2.0 * tlRadiusX || height <= 2.0 * tlRadiusY);
  }

  bool get isEllipse {
    return tlRadius == trRadius &&
        trRadius == brRadius &&
        brRadius == blRadius &&
        width <= 2.0 * tlRadiusX &&
        height <= 2.0 * tlRadiusY;
  }

  bool get isCircle => width == height && isEllipse;
  double get shortestSide => math.min(width.abs(), height.abs());
  double get longestSide => math.max(width.abs(), height.abs());
  bool get hasNaN =>
      left.isNaN ||
      top.isNaN ||
      right.isNaN ||
      bottom.isNaN ||
      trRadiusX.isNaN ||
      trRadiusY.isNaN ||
      tlRadiusX.isNaN ||
      tlRadiusY.isNaN ||
      brRadiusX.isNaN ||
      brRadiusY.isNaN ||
      blRadiusX.isNaN ||
      blRadiusY.isNaN;
  Offset get center => Offset(left + width / 2.0, top + height / 2.0);

  // Returns the minimum between min and scale to which radius1 and radius2
  // should be scaled with in order not to exceed the limit.
  double _getMin(double min, double radius1, double radius2, double limit) {
    final double sum = radius1 + radius2;
    if (sum > limit && sum != 0.0) {
      return math.min(min, limit / sum);
    }
    return min;
  }

  RRect scaleRadii() {
    double scale = 1.0;
    final double absWidth = width.abs();
    final double absHeight = height.abs();
    scale = _getMin(scale, blRadiusY, tlRadiusY, absHeight);
    scale = _getMin(scale, tlRadiusX, trRadiusX, absWidth);
    scale = _getMin(scale, trRadiusY, brRadiusY, absHeight);
    scale = _getMin(scale, brRadiusX, blRadiusX, absWidth);

    if (scale < 1.0) {
      return RRect._raw(
        top: top,
        left: left,
        right: right,
        bottom: bottom,
        tlRadiusX: tlRadiusX * scale,
        tlRadiusY: tlRadiusY * scale,
        trRadiusX: trRadiusX * scale,
        trRadiusY: trRadiusY * scale,
        blRadiusX: blRadiusX * scale,
        blRadiusY: blRadiusY * scale,
        brRadiusX: brRadiusX * scale,
        brRadiusY: brRadiusY * scale,
      );
    }

    return RRect._raw(
      top: top,
      left: left,
      right: right,
      bottom: bottom,
      tlRadiusX: tlRadiusX,
      tlRadiusY: tlRadiusY,
      trRadiusX: trRadiusX,
      trRadiusY: trRadiusY,
      blRadiusX: blRadiusX,
      blRadiusY: blRadiusY,
      brRadiusX: brRadiusX,
      brRadiusY: brRadiusY,
    );
  }

  bool contains(Offset point) {
    if (point.dx < left || point.dx >= right || point.dy < top || point.dy >= bottom) {
      return false;
    } // outside bounding box

    final RRect scaled = scaleRadii();

    double x;
    double y;
    double radiusX;
    double radiusY;
    // check whether point is in one of the rounded corner areas
    // x, y -> translate to ellipse center
    if (point.dx < left + scaled.tlRadiusX && point.dy < top + scaled.tlRadiusY) {
      x = point.dx - left - scaled.tlRadiusX;
      y = point.dy - top - scaled.tlRadiusY;
      radiusX = scaled.tlRadiusX;
      radiusY = scaled.tlRadiusY;
    } else if (point.dx > right - scaled.trRadiusX && point.dy < top + scaled.trRadiusY) {
      x = point.dx - right + scaled.trRadiusX;
      y = point.dy - top - scaled.trRadiusY;
      radiusX = scaled.trRadiusX;
      radiusY = scaled.trRadiusY;
    } else if (point.dx > right - scaled.brRadiusX && point.dy > bottom - scaled.brRadiusY) {
      x = point.dx - right + scaled.brRadiusX;
      y = point.dy - bottom + scaled.brRadiusY;
      radiusX = scaled.brRadiusX;
      radiusY = scaled.brRadiusY;
    } else if (point.dx < left + scaled.blRadiusX && point.dy > bottom - scaled.blRadiusY) {
      x = point.dx - left - scaled.blRadiusX;
      y = point.dy - bottom + scaled.blRadiusY;
      radiusX = scaled.blRadiusX;
      radiusY = scaled.blRadiusY;
    } else {
      return true; // inside and not within the rounded corner area
    }

    x = x / radiusX;
    y = y / radiusY;
    // check if the point is outside the unit circle
    if (x * x + y * y > 1.0) {
      return false;
    }
    return true;
  }

  static RRect? lerp(RRect? a, RRect? b, double t) {
    if (b == null) {
      if (a == null) {
        return null;
      } else {
        final double k = 1.0 - t;
        return RRect._raw(
          left: a.left * k,
          top: a.top * k,
          right: a.right * k,
          bottom: a.bottom * k,
          tlRadiusX: math.max(0, a.tlRadiusX * k),
          tlRadiusY: math.max(0, a.tlRadiusY * k),
          trRadiusX: math.max(0, a.trRadiusX * k),
          trRadiusY: math.max(0, a.trRadiusY * k),
          brRadiusX: math.max(0, a.brRadiusX * k),
          brRadiusY: math.max(0, a.brRadiusY * k),
          blRadiusX: math.max(0, a.blRadiusX * k),
          blRadiusY: math.max(0, a.blRadiusY * k),
        );
      }
    } else {
      if (a == null) {
        return RRect._raw(
          left: b.left * t,
          top: b.top * t,
          right: b.right * t,
          bottom: b.bottom * t,
          tlRadiusX: math.max(0, b.tlRadiusX * t),
          tlRadiusY: math.max(0, b.tlRadiusY * t),
          trRadiusX: math.max(0, b.trRadiusX * t),
          trRadiusY: math.max(0, b.trRadiusY * t),
          brRadiusX: math.max(0, b.brRadiusX * t),
          brRadiusY: math.max(0, b.brRadiusY * t),
          blRadiusX: math.max(0, b.blRadiusX * t),
          blRadiusY: math.max(0, b.blRadiusY * t),
        );
      } else {
        return RRect._raw(
          left: _lerpDouble(a.left, b.left, t),
          top: _lerpDouble(a.top, b.top, t),
          right: _lerpDouble(a.right, b.right, t),
          bottom: _lerpDouble(a.bottom, b.bottom, t),
          tlRadiusX: math.max(0, _lerpDouble(a.tlRadiusX, b.tlRadiusX, t)),
          tlRadiusY: math.max(0, _lerpDouble(a.tlRadiusY, b.tlRadiusY, t)),
          trRadiusX: math.max(0, _lerpDouble(a.trRadiusX, b.trRadiusX, t)),
          trRadiusY: math.max(0, _lerpDouble(a.trRadiusY, b.trRadiusY, t)),
          brRadiusX: math.max(0, _lerpDouble(a.brRadiusX, b.brRadiusX, t)),
          brRadiusY: math.max(0, _lerpDouble(a.brRadiusY, b.brRadiusY, t)),
          blRadiusX: math.max(0, _lerpDouble(a.blRadiusX, b.blRadiusX, t)),
          blRadiusY: math.max(0, _lerpDouble(a.blRadiusY, b.blRadiusY, t)),
        );
      }
    }
  }

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (runtimeType != other.runtimeType) {
      return false;
    }
    return other is RRect &&
        other.left == left &&
        other.top == top &&
        other.right == right &&
        other.bottom == bottom &&
        other.tlRadiusX == tlRadiusX &&
        other.tlRadiusY == tlRadiusY &&
        other.trRadiusX == trRadiusX &&
        other.trRadiusY == trRadiusY &&
        other.blRadiusX == blRadiusX &&
        other.blRadiusY == blRadiusY &&
        other.brRadiusX == brRadiusX &&
        other.brRadiusY == brRadiusY;
  }

  @override
  int get hashCode => Object.hash(
    left,
    top,
    right,
    bottom,
    tlRadiusX,
    tlRadiusY,
    trRadiusX,
    trRadiusY,
    blRadiusX,
    blRadiusY,
    brRadiusX,
    brRadiusY,
  );

  @override
  String toString() {
    final String rect =
        '${left.toStringAsFixed(1)}, '
        '${top.toStringAsFixed(1)}, '
        '${right.toStringAsFixed(1)}, '
        '${bottom.toStringAsFixed(1)}';
    if (tlRadius == trRadius && trRadius == brRadius && brRadius == blRadius) {
      if (tlRadius.x == tlRadius.y) {
        return 'RRect.fromLTRBR($rect, ${tlRadius.x.toStringAsFixed(1)})';
      }
      return 'RRect.fromLTRBXY($rect, ${tlRadius.x.toStringAsFixed(1)}, ${tlRadius.y.toStringAsFixed(1)})';
    }
    return 'RRect.fromLTRBAndCorners('
        '$rect, '
        'topLeft: $tlRadius, '
        'topRight: $trRadius, '
        'bottomRight: $brRadius, '
        'bottomLeft: $blRadius'
        ')';
  }
}
// Modeled after Skia's SkRSXform.

class RSTransform {
  RSTransform(double scos, double ssin, double tx, double ty) {
    _value
      ..[0] = scos
      ..[1] = ssin
      ..[2] = tx
      ..[3] = ty;
  }
  factory RSTransform.fromComponents({
    required double rotation,
    required double scale,
    required double anchorX,
    required double anchorY,
    required double translateX,
    required double translateY,
  }) {
    final double scos = math.cos(rotation) * scale;
    final double ssin = math.sin(rotation) * scale;
    final double tx = translateX + -scos * anchorX + ssin * anchorY;
    final double ty = translateY + -ssin * anchorX - scos * anchorY;
    return RSTransform(scos, ssin, tx, ty);
  }

  final Float32List _value = Float32List(4);
  double get scos => _value[0];
  double get ssin => _value[1];
  double get tx => _value[2];
  double get ty => _value[3];
}
