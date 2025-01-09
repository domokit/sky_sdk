// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/widgets.dart';
///
/// @docImport 'box_border.dart';
/// @docImport 'box_decoration.dart';
library;

import 'package:flutter/foundation.dart';

import 'basic_types.dart';

/// Base class for [BorderRadius] that allows for text-direction aware resolution.
///
/// A property or argument of this type accepts classes created either with [
/// BorderRadius.only] and its variants, or [BorderRadiusDirectional.only]
/// and its variants.
///
/// To convert a [BorderRadiusGeometry] object of indeterminate type into a
/// [BorderRadius] object, call the [resolve] method.
@immutable
abstract class BorderRadiusGeometry {
  /// Abstract const constructor. This constructor enables subclasses to provide
  /// const constructors so that they can be used in const expressions.
  const BorderRadiusGeometry();

  Radius get _topLeft;
  Radius get _topRight;
  Radius get _bottomLeft;
  Radius get _bottomRight;
  Radius get _topStart;
  Radius get _topEnd;
  Radius get _bottomStart;
  Radius get _bottomEnd;

  /// Returns the difference between two [BorderRadiusGeometry] objects.
  ///
  /// If you know you are applying this to two [BorderRadius] or two
  /// [BorderRadiusDirectional] objects, consider using the binary infix `-`
  /// operator instead, which always returns an object of the same type as the
  /// operands, and is typed accordingly.
  ///
  /// If [subtract] is applied to two objects of the same type ([BorderRadius] or
  /// [BorderRadiusDirectional]), an object of that type will be returned (though
  /// this is not reflected in the type system). Otherwise, an object
  /// representing a combination of both is returned. That object can be turned
  /// into a concrete [BorderRadius] using [resolve].
  ///
  /// This method returns the same result as [add] applied to the result of
  /// negating the argument (using the prefix unary `-` operator or multiplying
  /// the argument by -1.0 using the `*` operator).
  BorderRadiusGeometry subtract(BorderRadiusGeometry other) {
    return _MixedBorderRadius(
      _topLeft - other._topLeft,
      _topRight - other._topRight,
      _bottomLeft - other._bottomLeft,
      _bottomRight - other._bottomRight,
      _topStart - other._topStart,
      _topEnd - other._topEnd,
      _bottomStart - other._bottomStart,
      _bottomEnd - other._bottomEnd,
    );
  }

  /// Returns the sum of two [BorderRadiusGeometry] objects.
  ///
  /// If you know you are adding two [BorderRadius] or two [BorderRadiusDirectional]
  /// objects, consider using the `+` operator instead, which always returns an
  /// object of the same type as the operands, and is typed accordingly.
  ///
  /// If [add] is applied to two objects of the same type ([BorderRadius] or
  /// [BorderRadiusDirectional]), an object of that type will be returned (though
  /// this is not reflected in the type system). Otherwise, an object
  /// representing a combination of both is returned. That object can be turned
  /// into a concrete [BorderRadius] using [resolve].
  BorderRadiusGeometry add(BorderRadiusGeometry other) {
    return _MixedBorderRadius(
      _topLeft + other._topLeft,
      _topRight + other._topRight,
      _bottomLeft + other._bottomLeft,
      _bottomRight + other._bottomRight,
      _topStart + other._topStart,
      _topEnd + other._topEnd,
      _bottomStart + other._bottomStart,
      _bottomEnd + other._bottomEnd,
    );
  }

  /// Returns the [BorderRadiusGeometry] object with each corner radius negated.
  ///
  /// This is the same as multiplying the object by -1.0.
  ///
  /// This operator returns an object of the same type as the operand.
  BorderRadiusGeometry operator -();

  /// Scales the [BorderRadiusGeometry] object's corners by the given factor.
  ///
  /// This operator returns an object of the same type as the operand.
  BorderRadiusGeometry operator *(double other);

  /// Divides the [BorderRadiusGeometry] object's corners by the given factor.
  ///
  /// This operator returns an object of the same type as the operand.
  BorderRadiusGeometry operator /(double other);

  /// Integer divides the [BorderRadiusGeometry] object's corners by the given factor.
  ///
  /// This operator returns an object of the same type as the operand.
  ///
  /// This operator may have unexpected results when applied to a mixture of
  /// [BorderRadius] and [BorderRadiusDirectional] objects.
  BorderRadiusGeometry operator ~/(double other);

  /// Computes the remainder of each corner by the given factor.
  ///
  /// This operator returns an object of the same type as the operand.
  ///
  /// This operator may have unexpected results when applied to a mixture of
  /// [BorderRadius] and [BorderRadiusDirectional] objects.
  BorderRadiusGeometry operator %(double other);

  /// Linearly interpolate between two [BorderRadiusGeometry] objects.
  ///
  /// If either is null, this function interpolates from [BorderRadius.zero],
  /// and the result is an object of the same type as the non-null argument. (If
  /// both are null, this returns null.)
  ///
  /// If [lerp] is applied to two objects of the same type ([BorderRadius] or
  /// [BorderRadiusDirectional]), an object of that type will be returned (though
  /// this is not reflected in the type system). Otherwise, an object
  /// representing a combination of both is returned. That object can be turned
  /// into a concrete [BorderRadius] using [resolve].
  ///
  /// {@macro dart.ui.shadow.lerp}
  static BorderRadiusGeometry? lerp(BorderRadiusGeometry? a, BorderRadiusGeometry? b, double t) {
    if (identical(a, b)) {
      return a;
    }
    a ??= BorderRadius.zero;
    b ??= BorderRadius.zero;
    return a.add((b.subtract(a)) * t);
  }

  /// Convert this instance into a [BorderRadius], so that the radii are
  /// expressed for specific physical corners (top-left, top-right, etc) rather
  /// than in a direction-dependent manner.
  ///
  /// See also:
  ///
  ///  * [BorderRadius], for which this is a no-op (returns itself).
  ///  * [BorderRadiusDirectional], which flips the horizontal direction
  ///    based on the `direction` argument.
  BorderRadius resolve(TextDirection? direction);

  @override
  String toString() {
    String? visual, logical;
    if (_topLeft == _topRight && _topRight == _bottomLeft && _bottomLeft == _bottomRight) {
      if (_topLeft != Radius.zero) {
        if (_topLeft.x == _topLeft.y) {
          visual = 'BorderRadius.circular(${_topLeft.x.toStringAsFixed(1)})';
        } else {
          visual = 'BorderRadius.all($_topLeft)';
        }
      }
    } else {
      // visuals aren't the same and at least one isn't zero
      final StringBuffer result = StringBuffer();
      result.write('BorderRadius.only(');
      bool comma = false;
      if (_topLeft != Radius.zero) {
        result.write('topLeft: $_topLeft');
        comma = true;
      }
      if (_topRight != Radius.zero) {
        if (comma) {
          result.write(', ');
        }
        result.write('topRight: $_topRight');
        comma = true;
      }
      if (_bottomLeft != Radius.zero) {
        if (comma) {
          result.write(', ');
        }
        result.write('bottomLeft: $_bottomLeft');
        comma = true;
      }
      if (_bottomRight != Radius.zero) {
        if (comma) {
          result.write(', ');
        }
        result.write('bottomRight: $_bottomRight');
      }
      result.write(')');
      visual = result.toString();
    }
    if (_topStart == _topEnd && _topEnd == _bottomEnd && _bottomEnd == _bottomStart) {
      if (_topStart != Radius.zero) {
        if (_topStart.x == _topStart.y) {
          logical = 'BorderRadiusDirectional.circular(${_topStart.x.toStringAsFixed(1)})';
        } else {
          logical = 'BorderRadiusDirectional.all($_topStart)';
        }
      }
    } else {
      // logicals aren't the same and at least one isn't zero
      final StringBuffer result = StringBuffer();
      result.write('BorderRadiusDirectional.only(');
      bool comma = false;
      if (_topStart != Radius.zero) {
        result.write('topStart: $_topStart');
        comma = true;
      }
      if (_topEnd != Radius.zero) {
        if (comma) {
          result.write(', ');
        }
        result.write('topEnd: $_topEnd');
        comma = true;
      }
      if (_bottomStart != Radius.zero) {
        if (comma) {
          result.write(', ');
        }
        result.write('bottomStart: $_bottomStart');
        comma = true;
      }
      if (_bottomEnd != Radius.zero) {
        if (comma) {
          result.write(', ');
        }
        result.write('bottomEnd: $_bottomEnd');
      }
      result.write(')');
      logical = result.toString();
    }
    if (visual != null && logical != null) {
      return '$visual + $logical';
    }
    return visual ?? logical ?? 'BorderRadius.zero';
  }

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is BorderRadiusGeometry &&
        other._topLeft == _topLeft &&
        other._topRight == _topRight &&
        other._bottomLeft == _bottomLeft &&
        other._bottomRight == _bottomRight &&
        other._topStart == _topStart &&
        other._topEnd == _topEnd &&
        other._bottomStart == _bottomStart &&
        other._bottomEnd == _bottomEnd;
  }

  @override
  int get hashCode => Object.hash(
    _topLeft,
    _topRight,
    _bottomLeft,
    _bottomRight,
    _topStart,
    _topEnd,
    _bottomStart,
    _bottomEnd,
  );
}

/// An immutable set of radii for each corner of a rectangle.
///
/// Used by [BoxDecoration] when the shape is a [BoxShape.rectangle].
///
/// The [BorderRadius] class specifies offsets in terms of visual corners, e.g.
/// [topLeft]. These values are not affected by the [TextDirection]. To support
/// both left-to-right and right-to-left layouts, consider using
/// [BorderRadiusDirectional], which is expressed in terms that are relative to
/// a [TextDirection] (typically obtained from the ambient [Directionality]).
class BorderRadius extends BorderRadiusGeometry {
  /// Creates a border radius where all radii are [radius].
  const BorderRadius.all(Radius radius)
    : this.only(topLeft: radius, topRight: radius, bottomLeft: radius, bottomRight: radius);

  /// Creates a border radius where all radii are [Radius.circular(radius)].
  BorderRadius.circular(double radius) : this.all(Radius.circular(radius));

  /// Creates a vertically symmetric border radius where the top and bottom
  /// sides of the rectangle have the same radii.
  const BorderRadius.vertical({Radius top = Radius.zero, Radius bottom = Radius.zero})
    : this.only(topLeft: top, topRight: top, bottomLeft: bottom, bottomRight: bottom);

  /// Creates a horizontally symmetrical border radius where the left and right
  /// sides of the rectangle have the same radii.
  const BorderRadius.horizontal({Radius left = Radius.zero, Radius right = Radius.zero})
    : this.only(topLeft: left, topRight: right, bottomLeft: left, bottomRight: right);

  /// Creates a border radius with only the given non-zero values. The other
  /// corners will be right angles.
  const BorderRadius.only({
    this.topLeft = Radius.zero,
    this.topRight = Radius.zero,
    this.bottomLeft = Radius.zero,
    this.bottomRight = Radius.zero,
  });

  /// Returns a copy of this BorderRadius with the given fields replaced with
  /// the new values.
  BorderRadius copyWith({
    Radius? topLeft,
    Radius? topRight,
    Radius? bottomLeft,
    Radius? bottomRight,
  }) {
    return BorderRadius.only(
      topLeft: topLeft ?? this.topLeft,
      topRight: topRight ?? this.topRight,
      bottomLeft: bottomLeft ?? this.bottomLeft,
      bottomRight: bottomRight ?? this.bottomRight,
    );
  }

  /// A border radius with all zero radii.
  static const BorderRadius zero = BorderRadius.all(Radius.zero);

  /// The top-left [Radius].
  final Radius topLeft;

  @override
  Radius get _topLeft => topLeft;

  /// The top-right [Radius].
  final Radius topRight;

  @override
  Radius get _topRight => topRight;

  /// The bottom-left [Radius].
  final Radius bottomLeft;

  @override
  Radius get _bottomLeft => bottomLeft;

  /// The bottom-right [Radius].
  final Radius bottomRight;

  @override
  Radius get _bottomRight => bottomRight;

  @override
  Radius get _topStart => Radius.zero;

  @override
  Radius get _topEnd => Radius.zero;

  @override
  Radius get _bottomStart => Radius.zero;

  @override
  Radius get _bottomEnd => Radius.zero;

  /// Creates an [RRect] from the current border radius and a [Rect].
  ///
  /// If any of the radii have negative values in x or y, those values will be
  /// clamped to zero in order to produce a valid [RRect].
  RRect toRRect(Rect rect) {
    // Because the current radii could be negative, we must clamp them before
    // converting them to an RRect to be rendered, since negative radii on
    // RRects don't make sense.
    return RRect.fromRectAndCorners(
      rect,
      topLeft: topLeft.clamp(minimum: Radius.zero),
      topRight: topRight.clamp(minimum: Radius.zero),
      bottomLeft: bottomLeft.clamp(minimum: Radius.zero),
      bottomRight: bottomRight.clamp(minimum: Radius.zero),
    );
  }

  @override
  BorderRadiusGeometry subtract(BorderRadiusGeometry other) {
    if (other is BorderRadius) {
      return this - other;
    }
    return super.subtract(other);
  }

  @override
  BorderRadiusGeometry add(BorderRadiusGeometry other) {
    if (other is BorderRadius) {
      return this + other;
    }
    return super.add(other);
  }

  /// Returns the difference between two [BorderRadius] objects.
  BorderRadius operator -(BorderRadius other) {
    return BorderRadius.only(
      topLeft: topLeft - other.topLeft,
      topRight: topRight - other.topRight,
      bottomLeft: bottomLeft - other.bottomLeft,
      bottomRight: bottomRight - other.bottomRight,
    );
  }

  /// Returns the sum of two [BorderRadius] objects.
  BorderRadius operator +(BorderRadius other) {
    return BorderRadius.only(
      topLeft: topLeft + other.topLeft,
      topRight: topRight + other.topRight,
      bottomLeft: bottomLeft + other.bottomLeft,
      bottomRight: bottomRight + other.bottomRight,
    );
  }

  /// Returns the [BorderRadius] object with each corner negated.
  ///
  /// This is the same as multiplying the object by -1.0.
  @override
  BorderRadius operator -() {
    return BorderRadius.only(
      topLeft: -topLeft,
      topRight: -topRight,
      bottomLeft: -bottomLeft,
      bottomRight: -bottomRight,
    );
  }

  /// Scales each corner of the [BorderRadius] by the given factor.
  @override
  BorderRadius operator *(double other) {
    return BorderRadius.only(
      topLeft: topLeft * other,
      topRight: topRight * other,
      bottomLeft: bottomLeft * other,
      bottomRight: bottomRight * other,
    );
  }

  /// Divides each corner of the [BorderRadius] by the given factor.
  @override
  BorderRadius operator /(double other) {
    return BorderRadius.only(
      topLeft: topLeft / other,
      topRight: topRight / other,
      bottomLeft: bottomLeft / other,
      bottomRight: bottomRight / other,
    );
  }

  /// Integer divides each corner of the [BorderRadius] by the given factor.
  @override
  BorderRadius operator ~/(double other) {
    return BorderRadius.only(
      topLeft: topLeft ~/ other,
      topRight: topRight ~/ other,
      bottomLeft: bottomLeft ~/ other,
      bottomRight: bottomRight ~/ other,
    );
  }

  /// Computes the remainder of each corner by the given factor.
  @override
  BorderRadius operator %(double other) {
    return BorderRadius.only(
      topLeft: topLeft % other,
      topRight: topRight % other,
      bottomLeft: bottomLeft % other,
      bottomRight: bottomRight % other,
    );
  }

  /// Linearly interpolate between two [BorderRadius] objects.
  ///
  /// If either is null, this function interpolates from [BorderRadius.zero].
  ///
  /// {@macro dart.ui.shadow.lerp}
  static BorderRadius? lerp(BorderRadius? a, BorderRadius? b, double t) {
    if (identical(a, b)) {
      return a;
    }
    if (a == null) {
      return b! * t;
    }
    if (b == null) {
      return a * (1.0 - t);
    }
    return BorderRadius.only(
      topLeft: Radius.lerp(a.topLeft, b.topLeft, t)!,
      topRight: Radius.lerp(a.topRight, b.topRight, t)!,
      bottomLeft: Radius.lerp(a.bottomLeft, b.bottomLeft, t)!,
      bottomRight: Radius.lerp(a.bottomRight, b.bottomRight, t)!,
    );
  }

  @override
  BorderRadius resolve(TextDirection? direction) => this;
}

/// An immutable set of radii for each corner of a rectangle, but with the
/// corners specified in a manner dependent on the writing direction.
///
/// This can be used to specify a corner radius on the leading or trailing edge
/// of a box, so that it flips to the other side when the text alignment flips
/// (e.g. being on the top right in English text but the top left in Arabic
/// text).
///
/// See also:
///
///  * [BorderRadius], a variant that uses physical labels (`topLeft` and
///    `topRight` instead of `topStart` and `topEnd`).
class BorderRadiusDirectional extends BorderRadiusGeometry {
  /// Creates a border radius where all radii are [radius].
  const BorderRadiusDirectional.all(Radius radius)
    : this.only(topStart: radius, topEnd: radius, bottomStart: radius, bottomEnd: radius);

  /// Creates a border radius where all radii are [Radius.circular(radius)].
  BorderRadiusDirectional.circular(double radius) : this.all(Radius.circular(radius));

  /// Creates a vertically symmetric border radius where the top and bottom
  /// sides of the rectangle have the same radii.
  const BorderRadiusDirectional.vertical({Radius top = Radius.zero, Radius bottom = Radius.zero})
    : this.only(topStart: top, topEnd: top, bottomStart: bottom, bottomEnd: bottom);

  /// Creates a horizontally symmetrical border radius where the start and end
  /// sides of the rectangle have the same radii.
  const BorderRadiusDirectional.horizontal({Radius start = Radius.zero, Radius end = Radius.zero})
    : this.only(topStart: start, topEnd: end, bottomStart: start, bottomEnd: end);

  /// Creates a border radius with only the given non-zero values. The other
  /// corners will be right angles.
  const BorderRadiusDirectional.only({
    this.topStart = Radius.zero,
    this.topEnd = Radius.zero,
    this.bottomStart = Radius.zero,
    this.bottomEnd = Radius.zero,
  });

  /// A border radius with all zero radii.
  ///
  /// Consider using [BorderRadius.zero] instead, since that object has the same
  /// effect, but will be cheaper to [resolve].
  static const BorderRadiusDirectional zero = BorderRadiusDirectional.all(Radius.zero);

  /// The top-start [Radius].
  final Radius topStart;

  @override
  Radius get _topStart => topStart;

  /// The top-end [Radius].
  final Radius topEnd;

  @override
  Radius get _topEnd => topEnd;

  /// The bottom-start [Radius].
  final Radius bottomStart;

  @override
  Radius get _bottomStart => bottomStart;

  /// The bottom-end [Radius].
  final Radius bottomEnd;

  @override
  Radius get _bottomEnd => bottomEnd;

  @override
  Radius get _topLeft => Radius.zero;

  @override
  Radius get _topRight => Radius.zero;

  @override
  Radius get _bottomLeft => Radius.zero;

  @override
  Radius get _bottomRight => Radius.zero;

  @override
  BorderRadiusGeometry subtract(BorderRadiusGeometry other) {
    if (other is BorderRadiusDirectional) {
      return this - other;
    }
    return super.subtract(other);
  }

  @override
  BorderRadiusGeometry add(BorderRadiusGeometry other) {
    if (other is BorderRadiusDirectional) {
      return this + other;
    }
    return super.add(other);
  }

  /// Returns the difference between two [BorderRadiusDirectional] objects.
  BorderRadiusDirectional operator -(BorderRadiusDirectional other) {
    return BorderRadiusDirectional.only(
      topStart: topStart - other.topStart,
      topEnd: topEnd - other.topEnd,
      bottomStart: bottomStart - other.bottomStart,
      bottomEnd: bottomEnd - other.bottomEnd,
    );
  }

  /// Returns the sum of two [BorderRadiusDirectional] objects.
  BorderRadiusDirectional operator +(BorderRadiusDirectional other) {
    return BorderRadiusDirectional.only(
      topStart: topStart + other.topStart,
      topEnd: topEnd + other.topEnd,
      bottomStart: bottomStart + other.bottomStart,
      bottomEnd: bottomEnd + other.bottomEnd,
    );
  }

  /// Returns the [BorderRadiusDirectional] object with each corner negated.
  ///
  /// This is the same as multiplying the object by -1.0.
  @override
  BorderRadiusDirectional operator -() {
    return BorderRadiusDirectional.only(
      topStart: -topStart,
      topEnd: -topEnd,
      bottomStart: -bottomStart,
      bottomEnd: -bottomEnd,
    );
  }

  /// Scales each corner of the [BorderRadiusDirectional] by the given factor.
  @override
  BorderRadiusDirectional operator *(double other) {
    return BorderRadiusDirectional.only(
      topStart: topStart * other,
      topEnd: topEnd * other,
      bottomStart: bottomStart * other,
      bottomEnd: bottomEnd * other,
    );
  }

  /// Divides each corner of the [BorderRadiusDirectional] by the given factor.
  @override
  BorderRadiusDirectional operator /(double other) {
    return BorderRadiusDirectional.only(
      topStart: topStart / other,
      topEnd: topEnd / other,
      bottomStart: bottomStart / other,
      bottomEnd: bottomEnd / other,
    );
  }

  /// Integer divides each corner of the [BorderRadiusDirectional] by the given factor.
  @override
  BorderRadiusDirectional operator ~/(double other) {
    return BorderRadiusDirectional.only(
      topStart: topStart ~/ other,
      topEnd: topEnd ~/ other,
      bottomStart: bottomStart ~/ other,
      bottomEnd: bottomEnd ~/ other,
    );
  }

  /// Computes the remainder of each corner by the given factor.
  @override
  BorderRadiusDirectional operator %(double other) {
    return BorderRadiusDirectional.only(
      topStart: topStart % other,
      topEnd: topEnd % other,
      bottomStart: bottomStart % other,
      bottomEnd: bottomEnd % other,
    );
  }

  /// Linearly interpolate between two [BorderRadiusDirectional] objects.
  ///
  /// If either is null, this function interpolates from [BorderRadiusDirectional.zero].
  ///
  /// {@macro dart.ui.shadow.lerp}
  static BorderRadiusDirectional? lerp(
    BorderRadiusDirectional? a,
    BorderRadiusDirectional? b,
    double t,
  ) {
    if (identical(a, b)) {
      return a;
    }
    if (a == null) {
      return b! * t;
    }
    if (b == null) {
      return a * (1.0 - t);
    }
    return BorderRadiusDirectional.only(
      topStart: Radius.lerp(a.topStart, b.topStart, t)!,
      topEnd: Radius.lerp(a.topEnd, b.topEnd, t)!,
      bottomStart: Radius.lerp(a.bottomStart, b.bottomStart, t)!,
      bottomEnd: Radius.lerp(a.bottomEnd, b.bottomEnd, t)!,
    );
  }

  @override
  BorderRadius resolve(TextDirection? direction) {
    assert(direction != null);
    switch (direction!) {
      case TextDirection.rtl:
        return BorderRadius.only(
          topLeft: topEnd,
          topRight: topStart,
          bottomLeft: bottomEnd,
          bottomRight: bottomStart,
        );
      case TextDirection.ltr:
        return BorderRadius.only(
          topLeft: topStart,
          topRight: topEnd,
          bottomLeft: bottomStart,
          bottomRight: bottomEnd,
        );
    }
  }
}

class _MixedBorderRadius extends BorderRadiusGeometry {
  const _MixedBorderRadius(
    this._topLeft,
    this._topRight,
    this._bottomLeft,
    this._bottomRight,
    this._topStart,
    this._topEnd,
    this._bottomStart,
    this._bottomEnd,
  );

  @override
  final Radius _topLeft;

  @override
  final Radius _topRight;

  @override
  final Radius _bottomLeft;

  @override
  final Radius _bottomRight;

  @override
  final Radius _topStart;

  @override
  final Radius _topEnd;

  @override
  final Radius _bottomStart;

  @override
  final Radius _bottomEnd;

  @override
  _MixedBorderRadius operator -() {
    return _MixedBorderRadius(
      -_topLeft,
      -_topRight,
      -_bottomLeft,
      -_bottomRight,
      -_topStart,
      -_topEnd,
      -_bottomStart,
      -_bottomEnd,
    );
  }

  /// Scales each corner of the [_MixedBorderRadius] by the given factor.
  @override
  _MixedBorderRadius operator *(double other) {
    return _MixedBorderRadius(
      _topLeft * other,
      _topRight * other,
      _bottomLeft * other,
      _bottomRight * other,
      _topStart * other,
      _topEnd * other,
      _bottomStart * other,
      _bottomEnd * other,
    );
  }

  @override
  _MixedBorderRadius operator /(double other) {
    return _MixedBorderRadius(
      _topLeft / other,
      _topRight / other,
      _bottomLeft / other,
      _bottomRight / other,
      _topStart / other,
      _topEnd / other,
      _bottomStart / other,
      _bottomEnd / other,
    );
  }

  @override
  _MixedBorderRadius operator ~/(double other) {
    return _MixedBorderRadius(
      _topLeft ~/ other,
      _topRight ~/ other,
      _bottomLeft ~/ other,
      _bottomRight ~/ other,
      _topStart ~/ other,
      _topEnd ~/ other,
      _bottomStart ~/ other,
      _bottomEnd ~/ other,
    );
  }

  @override
  _MixedBorderRadius operator %(double other) {
    return _MixedBorderRadius(
      _topLeft % other,
      _topRight % other,
      _bottomLeft % other,
      _bottomRight % other,
      _topStart % other,
      _topEnd % other,
      _bottomStart % other,
      _bottomEnd % other,
    );
  }

  @override
  BorderRadius resolve(TextDirection? direction) {
    assert(direction != null);
    switch (direction!) {
      case TextDirection.rtl:
        return BorderRadius.only(
          topLeft: _topLeft + _topEnd,
          topRight: _topRight + _topStart,
          bottomLeft: _bottomLeft + _bottomEnd,
          bottomRight: _bottomRight + _bottomStart,
        );
      case TextDirection.ltr:
        return BorderRadius.only(
          topLeft: _topLeft + _topStart,
          topRight: _topRight + _topEnd,
          bottomLeft: _bottomLeft + _bottomStart,
          bottomRight: _bottomRight + _bottomEnd,
        );
    }
  }
}
