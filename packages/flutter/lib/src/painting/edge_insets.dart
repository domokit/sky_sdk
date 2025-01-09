// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'dart:ui';
///
/// @docImport 'package:flutter/widgets.dart';
library;

import 'dart:ui' as ui show ViewPadding, lerpDouble;

import 'package:flutter/foundation.dart';

import 'basic_types.dart';

/// Base class for [EdgeInsets] that allows for text-direction aware
/// resolution.
///
/// A property or argument of this type accepts classes created either with [
/// EdgeInsets.fromLTRB] and its variants, or [
/// EdgeInsetsDirectional.fromSTEB] and its variants.
///
/// To convert an [EdgeInsetsGeometry] object of indeterminate type into a
/// [EdgeInsets] object, call the [resolve] method.
///
/// See also:
///
///  * [Padding], a widget that describes margins using [EdgeInsetsGeometry].
@immutable
abstract class EdgeInsetsGeometry {
  /// Abstract const constructor. This constructor enables subclasses to provide
  /// const constructors so that they can be used in const expressions.
  const EdgeInsetsGeometry();

  double get _bottom;
  double get _end;
  double get _left;
  double get _right;
  double get _start;
  double get _top;

  /// An [EdgeInsetsGeometry] with infinite offsets in each direction.
  ///
  /// Can be used as an infinite upper bound for [clamp].
  static const EdgeInsetsGeometry infinity = _MixedEdgeInsets.fromLRSETB(
    double.infinity,
    double.infinity,
    double.infinity,
    double.infinity,
    double.infinity,
    double.infinity,
  );

  /// Whether every dimension is non-negative.
  bool get isNonNegative {
    return _left >= 0.0 &&
        _right >= 0.0 &&
        _start >= 0.0 &&
        _end >= 0.0 &&
        _top >= 0.0 &&
        _bottom >= 0.0;
  }

  /// The total offset in the horizontal direction.
  double get horizontal => _left + _right + _start + _end;

  /// The total offset in the vertical direction.
  double get vertical => _top + _bottom;

  /// The total offset in the given direction.
  double along(Axis axis) {
    return switch (axis) {
      Axis.horizontal => horizontal,
      Axis.vertical => vertical,
    };
  }

  /// The size that this [EdgeInsets] would occupy with an empty interior.
  Size get collapsedSize => Size(horizontal, vertical);

  /// An [EdgeInsetsGeometry] with top and bottom, left and right, and start and end flipped.
  EdgeInsetsGeometry get flipped =>
      _MixedEdgeInsets.fromLRSETB(_right, _left, _end, _start, _bottom, _top);

  /// Returns a new size that is bigger than the given size by the amount of
  /// inset in the horizontal and vertical directions.
  ///
  /// See also:
  ///
  ///  * [EdgeInsets.inflateRect], to inflate a [Rect] rather than a [Size] (for
  ///    [EdgeInsetsDirectional], requires first calling [resolve] to establish
  ///    how the start and end map to the left or right).
  ///  * [deflateSize], to deflate a [Size] rather than inflating it.
  Size inflateSize(Size size) {
    return Size(size.width + horizontal, size.height + vertical);
  }

  /// Returns a new size that is smaller than the given size by the amount of
  /// inset in the horizontal and vertical directions.
  ///
  /// If the argument is smaller than [collapsedSize], then the resulting size
  /// will have negative dimensions.
  ///
  /// See also:
  ///
  ///  * [EdgeInsets.deflateRect], to deflate a [Rect] rather than a [Size]. (for
  ///    [EdgeInsetsDirectional], requires first calling [resolve] to establish
  ///    how the start and end map to the left or right).
  ///  * [inflateSize], to inflate a [Size] rather than deflating it.
  Size deflateSize(Size size) {
    return Size(size.width - horizontal, size.height - vertical);
  }

  /// Returns the difference between two [EdgeInsetsGeometry] objects.
  ///
  /// If you know you are applying this to two [EdgeInsets] or two
  /// [EdgeInsetsDirectional] objects, consider using the binary infix `-`
  /// operator instead, which always returns an object of the same type as the
  /// operands, and is typed accordingly.
  ///
  /// If [subtract] is applied to two objects of the same type ([EdgeInsets] or
  /// [EdgeInsetsDirectional]), an object of that type will be returned (though
  /// this is not reflected in the type system). Otherwise, an object
  /// representing a combination of both is returned. That object can be turned
  /// into a concrete [EdgeInsets] using [resolve].
  ///
  /// This method returns the same result as [add] applied to the result of
  /// negating the argument (using the prefix unary `-` operator or multiplying
  /// the argument by -1.0 using the `*` operator).
  EdgeInsetsGeometry subtract(EdgeInsetsGeometry other) {
    return _MixedEdgeInsets.fromLRSETB(
      _left - other._left,
      _right - other._right,
      _start - other._start,
      _end - other._end,
      _top - other._top,
      _bottom - other._bottom,
    );
  }

  /// Returns the sum of two [EdgeInsetsGeometry] objects.
  ///
  /// If you know you are adding two [EdgeInsets] or two [EdgeInsetsDirectional]
  /// objects, consider using the `+` operator instead, which always returns an
  /// object of the same type as the operands, and is typed accordingly.
  ///
  /// If [add] is applied to two objects of the same type ([EdgeInsets] or
  /// [EdgeInsetsDirectional]), an object of that type will be returned (though
  /// this is not reflected in the type system). Otherwise, an object
  /// representing a combination of both is returned. That object can be turned
  /// into a concrete [EdgeInsets] using [resolve].
  EdgeInsetsGeometry add(EdgeInsetsGeometry other) {
    return _MixedEdgeInsets.fromLRSETB(
      _left + other._left,
      _right + other._right,
      _start + other._start,
      _end + other._end,
      _top + other._top,
      _bottom + other._bottom,
    );
  }

  /// Returns a new [EdgeInsetsGeometry] object with all values greater than
  /// or equal to `min`, and less than or equal to `max`.
  EdgeInsetsGeometry clamp(EdgeInsetsGeometry min, EdgeInsetsGeometry max) {
    return _MixedEdgeInsets.fromLRSETB(
      clampDouble(_left, min._left, max._left),
      clampDouble(_right, min._right, max._right),
      clampDouble(_start, min._start, max._start),
      clampDouble(_end, min._end, max._end),
      clampDouble(_top, min._top, max._top),
      clampDouble(_bottom, min._bottom, max._bottom),
    );
  }

  /// Returns the [EdgeInsetsGeometry] object with each dimension negated.
  ///
  /// This is the same as multiplying the object by -1.0.
  ///
  /// This operator returns an object of the same type as the operand.
  EdgeInsetsGeometry operator -();

  /// Scales the [EdgeInsetsGeometry] object in each dimension by the given factor.
  ///
  /// This operator returns an object of the same type as the operand.
  EdgeInsetsGeometry operator *(double other);

  /// Divides the [EdgeInsetsGeometry] object in each dimension by the given factor.
  ///
  /// This operator returns an object of the same type as the operand.
  EdgeInsetsGeometry operator /(double other);

  /// Integer divides the [EdgeInsetsGeometry] object in each dimension by the given factor.
  ///
  /// This operator returns an object of the same type as the operand.
  ///
  /// This operator may have unexpected results when applied to a mixture of
  /// [EdgeInsets] and [EdgeInsetsDirectional] objects.
  EdgeInsetsGeometry operator ~/(double other);

  /// Computes the remainder in each dimension by the given factor.
  ///
  /// This operator returns an object of the same type as the operand.
  ///
  /// This operator may have unexpected results when applied to a mixture of
  /// [EdgeInsets] and [EdgeInsetsDirectional] objects.
  EdgeInsetsGeometry operator %(double other);

  /// Linearly interpolate between two [EdgeInsetsGeometry] objects.
  ///
  /// If either is null, this function interpolates from [EdgeInsets.zero], and
  /// the result is an object of the same type as the non-null argument.
  ///
  /// If [lerp] is applied to two objects of the same type ([EdgeInsets] or
  /// [EdgeInsetsDirectional]), an object of that type will be returned (though
  /// this is not reflected in the type system). Otherwise, an object
  /// representing a combination of both is returned. That object can be turned
  /// into a concrete [EdgeInsets] using [resolve].
  ///
  /// {@macro dart.ui.shadow.lerp}
  static EdgeInsetsGeometry? lerp(EdgeInsetsGeometry? a, EdgeInsetsGeometry? b, double t) {
    if (identical(a, b)) {
      return a;
    }
    if (a == null) {
      return b! * t;
    }
    if (b == null) {
      return a * (1.0 - t);
    }
    if (a is EdgeInsets && b is EdgeInsets) {
      return EdgeInsets.lerp(a, b, t);
    }
    if (a is EdgeInsetsDirectional && b is EdgeInsetsDirectional) {
      return EdgeInsetsDirectional.lerp(a, b, t);
    }
    return _MixedEdgeInsets.fromLRSETB(
      ui.lerpDouble(a._left, b._left, t)!,
      ui.lerpDouble(a._right, b._right, t)!,
      ui.lerpDouble(a._start, b._start, t)!,
      ui.lerpDouble(a._end, b._end, t)!,
      ui.lerpDouble(a._top, b._top, t)!,
      ui.lerpDouble(a._bottom, b._bottom, t)!,
    );
  }

  /// Convert this instance into an [EdgeInsets], which uses literal coordinates
  /// (i.e. the `left` coordinate being explicitly a distance from the left, and
  /// the `right` coordinate being explicitly a distance from the right).
  ///
  /// See also:
  ///
  ///  * [EdgeInsets], for which this is a no-op (returns itself).
  ///  * [EdgeInsetsDirectional], which flips the horizontal direction
  ///    based on the `direction` argument.
  EdgeInsets resolve(TextDirection? direction);

  @override
  String toString() {
    if (_start == 0.0 && _end == 0.0) {
      if (_left == 0.0 && _right == 0.0 && _top == 0.0 && _bottom == 0.0) {
        return 'EdgeInsets.zero';
      }
      if (_left == _right && _right == _top && _top == _bottom) {
        return 'EdgeInsets.all(${_left.toStringAsFixed(1)})';
      }
      return 'EdgeInsets(${_left.toStringAsFixed(1)}, '
          '${_top.toStringAsFixed(1)}, '
          '${_right.toStringAsFixed(1)}, '
          '${_bottom.toStringAsFixed(1)})';
    }
    if (_left == 0.0 && _right == 0.0) {
      return 'EdgeInsetsDirectional(${_start.toStringAsFixed(1)}, '
          '${_top.toStringAsFixed(1)}, '
          '${_end.toStringAsFixed(1)}, '
          '${_bottom.toStringAsFixed(1)})';
    }
    return 'EdgeInsets(${_left.toStringAsFixed(1)}, '
        '${_top.toStringAsFixed(1)}, '
        '${_right.toStringAsFixed(1)}, '
        '${_bottom.toStringAsFixed(1)})'
        ' + '
        'EdgeInsetsDirectional(${_start.toStringAsFixed(1)}, '
        '0.0, '
        '${_end.toStringAsFixed(1)}, '
        '0.0)';
  }

  @override
  bool operator ==(Object other) {
    return other is EdgeInsetsGeometry &&
        other._left == _left &&
        other._right == _right &&
        other._start == _start &&
        other._end == _end &&
        other._top == _top &&
        other._bottom == _bottom;
  }

  @override
  int get hashCode => Object.hash(_left, _right, _start, _end, _top, _bottom);
}

/// An immutable set of offsets in each of the four cardinal directions.
///
/// Typically used for an offset from each of the four sides of a box. For
/// example, the padding inside a box can be represented using this class.
///
/// The [EdgeInsets] class specifies offsets in terms of visual edges, left,
/// top, right, and bottom. These values are not affected by the
/// [TextDirection]. To support both left-to-right and right-to-left layouts,
/// consider using [EdgeInsetsDirectional], which is expressed in terms of
/// _start_, top, _end_, and bottom, where start and end are resolved in terms
/// of a [TextDirection] (typically obtained from the ambient [Directionality]).
///
/// {@tool snippet}
///
/// Here are some examples of how to create [EdgeInsets] instances:
///
/// Typical eight-pixel margin on all sides:
///
/// ```dart
/// const EdgeInsets.all(8.0)
/// ```
/// {@end-tool}
/// {@tool snippet}
///
/// Eight pixel margin above and below, no horizontal margins:
///
/// ```dart
/// const EdgeInsets.symmetric(vertical: 8.0)
/// ```
/// {@end-tool}
/// {@tool snippet}
///
/// Left margin indent of 40 pixels:
///
/// ```dart
/// const EdgeInsets.only(left: 40.0)
/// ```
/// {@end-tool}
///
/// See also:
///
///  * [Padding], a widget that accepts [EdgeInsets] to describe its margins.
///  * [EdgeInsetsDirectional], which (for properties and arguments that accept
///    the type [EdgeInsetsGeometry]) allows the horizontal insets to be
///    specified in a [TextDirection]-aware manner.
class EdgeInsets extends EdgeInsetsGeometry {
  /// Creates insets from offsets from the left, top, right, and bottom.
  const EdgeInsets.fromLTRB(this.left, this.top, this.right, this.bottom);

  /// Creates insets where all the offsets are `value`.
  ///
  /// {@tool snippet}
  ///
  /// Typical eight-pixel margin on all sides:
  ///
  /// ```dart
  /// const EdgeInsets.all(8.0)
  /// ```
  /// {@end-tool}
  const EdgeInsets.all(double value) : left = value, top = value, right = value, bottom = value;

  /// Creates insets with only the given values non-zero.
  ///
  /// {@tool snippet}
  ///
  /// Left margin indent of 40 pixels:
  ///
  /// ```dart
  /// const EdgeInsets.only(left: 40.0)
  /// ```
  /// {@end-tool}
  const EdgeInsets.only({this.left = 0.0, this.top = 0.0, this.right = 0.0, this.bottom = 0.0});

  /// Creates insets with symmetrical vertical and horizontal offsets.
  ///
  /// {@tool snippet}
  ///
  /// Eight pixel margin above and below, no horizontal margins:
  ///
  /// ```dart
  /// const EdgeInsets.symmetric(vertical: 8.0)
  /// ```
  /// {@end-tool}
  const EdgeInsets.symmetric({double vertical = 0.0, double horizontal = 0.0})
    : left = horizontal,
      top = vertical,
      right = horizontal,
      bottom = vertical;

  /// Creates insets that match the given view padding.
  ///
  /// If you need the current system padding or view insets in the context of a
  /// widget, consider using [MediaQuery.paddingOf] to obtain these values rather than
  /// using the value from a [FlutterView] directly, so that you get notified of
  /// changes.
  EdgeInsets.fromViewPadding(ui.ViewPadding padding, double devicePixelRatio)
    : left = padding.left / devicePixelRatio,
      top = padding.top / devicePixelRatio,
      right = padding.right / devicePixelRatio,
      bottom = padding.bottom / devicePixelRatio;

  /// Deprecated. Will be removed in a future version of Flutter.
  ///
  /// Use [EdgeInsets.fromViewPadding] instead.
  @Deprecated(
    'Use EdgeInsets.fromViewPadding instead. '
    'This feature was deprecated after v3.8.0-14.0.pre.',
  )
  factory EdgeInsets.fromWindowPadding(ui.ViewPadding padding, double devicePixelRatio) =
      EdgeInsets.fromViewPadding;

  /// An [EdgeInsets] with zero offsets in each direction.
  static const EdgeInsets zero = EdgeInsets.only();

  /// The offset from the left.
  final double left;

  @override
  double get _left => left;

  /// The offset from the top.
  final double top;

  @override
  double get _top => top;

  /// The offset from the right.
  final double right;

  @override
  double get _right => right;

  /// The offset from the bottom.
  final double bottom;

  @override
  double get _bottom => bottom;

  @override
  double get _start => 0.0;

  @override
  double get _end => 0.0;

  /// An Offset describing the vector from the top left of a rectangle to the
  /// top left of that rectangle inset by this object.
  Offset get topLeft => Offset(left, top);

  /// An Offset describing the vector from the top right of a rectangle to the
  /// top right of that rectangle inset by this object.
  Offset get topRight => Offset(-right, top);

  /// An Offset describing the vector from the bottom left of a rectangle to the
  /// bottom left of that rectangle inset by this object.
  Offset get bottomLeft => Offset(left, -bottom);

  /// An Offset describing the vector from the bottom right of a rectangle to the
  /// bottom right of that rectangle inset by this object.
  Offset get bottomRight => Offset(-right, -bottom);

  /// An [EdgeInsets] with top and bottom as well as left and right flipped.
  @override
  EdgeInsets get flipped => EdgeInsets.fromLTRB(right, bottom, left, top);

  /// Returns a new rect that is bigger than the given rect in each direction by
  /// the amount of inset in each direction. Specifically, the left edge of the
  /// rect is moved left by [left], the top edge of the rect is moved up by
  /// [top], the right edge of the rect is moved right by [right], and the
  /// bottom edge of the rect is moved down by [bottom].
  ///
  /// See also:
  ///
  ///  * [inflateSize], to inflate a [Size] rather than a [Rect].
  ///  * [deflateRect], to deflate a [Rect] rather than inflating it.
  Rect inflateRect(Rect rect) {
    return Rect.fromLTRB(
      rect.left - left,
      rect.top - top,
      rect.right + right,
      rect.bottom + bottom,
    );
  }

  /// Returns a new rect that is smaller than the given rect in each direction by
  /// the amount of inset in each direction. Specifically, the left edge of the
  /// rect is moved right by [left], the top edge of the rect is moved down by
  /// [top], the right edge of the rect is moved left by [right], and the
  /// bottom edge of the rect is moved up by [bottom].
  ///
  /// If the argument's [Rect.size] is smaller than [collapsedSize], then the
  /// resulting rectangle will have negative dimensions.
  ///
  /// See also:
  ///
  ///  * [deflateSize], to deflate a [Size] rather than a [Rect].
  ///  * [inflateRect], to inflate a [Rect] rather than deflating it.
  Rect deflateRect(Rect rect) {
    return Rect.fromLTRB(
      rect.left + left,
      rect.top + top,
      rect.right - right,
      rect.bottom - bottom,
    );
  }

  @override
  EdgeInsetsGeometry subtract(EdgeInsetsGeometry other) {
    if (other is EdgeInsets) {
      return this - other;
    }
    return super.subtract(other);
  }

  @override
  EdgeInsetsGeometry add(EdgeInsetsGeometry other) {
    if (other is EdgeInsets) {
      return this + other;
    }
    return super.add(other);
  }

  @override
  EdgeInsetsGeometry clamp(EdgeInsetsGeometry min, EdgeInsetsGeometry max) {
    return EdgeInsets.fromLTRB(
      clampDouble(_left, min._left, max._left),
      clampDouble(_top, min._top, max._top),
      clampDouble(_right, min._right, max._right),
      clampDouble(_bottom, min._bottom, max._bottom),
    );
  }

  /// Returns the difference between two [EdgeInsets].
  EdgeInsets operator -(EdgeInsets other) {
    return EdgeInsets.fromLTRB(
      left - other.left,
      top - other.top,
      right - other.right,
      bottom - other.bottom,
    );
  }

  /// Returns the sum of two [EdgeInsets].
  EdgeInsets operator +(EdgeInsets other) {
    return EdgeInsets.fromLTRB(
      left + other.left,
      top + other.top,
      right + other.right,
      bottom + other.bottom,
    );
  }

  /// Returns the [EdgeInsets] object with each dimension negated.
  ///
  /// This is the same as multiplying the object by -1.0.
  @override
  EdgeInsets operator -() {
    return EdgeInsets.fromLTRB(-left, -top, -right, -bottom);
  }

  /// Scales the [EdgeInsets] in each dimension by the given factor.
  @override
  EdgeInsets operator *(double other) {
    return EdgeInsets.fromLTRB(left * other, top * other, right * other, bottom * other);
  }

  /// Divides the [EdgeInsets] in each dimension by the given factor.
  @override
  EdgeInsets operator /(double other) {
    return EdgeInsets.fromLTRB(left / other, top / other, right / other, bottom / other);
  }

  /// Integer divides the [EdgeInsets] in each dimension by the given factor.
  @override
  EdgeInsets operator ~/(double other) {
    return EdgeInsets.fromLTRB(
      (left ~/ other).toDouble(),
      (top ~/ other).toDouble(),
      (right ~/ other).toDouble(),
      (bottom ~/ other).toDouble(),
    );
  }

  /// Computes the remainder in each dimension by the given factor.
  @override
  EdgeInsets operator %(double other) {
    return EdgeInsets.fromLTRB(left % other, top % other, right % other, bottom % other);
  }

  /// Linearly interpolate between two [EdgeInsets].
  ///
  /// If either is null, this function interpolates from [EdgeInsets.zero].
  ///
  /// {@macro dart.ui.shadow.lerp}
  static EdgeInsets? lerp(EdgeInsets? a, EdgeInsets? b, double t) {
    if (identical(a, b)) {
      return a;
    }
    if (a == null) {
      return b! * t;
    }
    if (b == null) {
      return a * (1.0 - t);
    }
    return EdgeInsets.fromLTRB(
      ui.lerpDouble(a.left, b.left, t)!,
      ui.lerpDouble(a.top, b.top, t)!,
      ui.lerpDouble(a.right, b.right, t)!,
      ui.lerpDouble(a.bottom, b.bottom, t)!,
    );
  }

  @override
  EdgeInsets resolve(TextDirection? direction) => this;

  /// Creates a copy of this EdgeInsets but with the given fields replaced
  /// with the new values.
  EdgeInsets copyWith({double? left, double? top, double? right, double? bottom}) {
    return EdgeInsets.only(
      left: left ?? this.left,
      top: top ?? this.top,
      right: right ?? this.right,
      bottom: bottom ?? this.bottom,
    );
  }
}

/// An immutable set of offsets in each of the four cardinal directions, but
/// whose horizontal components are dependent on the writing direction.
///
/// This can be used to indicate padding from the left in [TextDirection.ltr]
/// text and padding from the right in [TextDirection.rtl] text without having
/// to be aware of the current text direction.
///
/// See also:
///
///  * [EdgeInsets], a variant that uses physical labels (left and right instead
///    of start and end).
class EdgeInsetsDirectional extends EdgeInsetsGeometry {
  /// Creates insets from offsets from the start, top, end, and bottom.
  const EdgeInsetsDirectional.fromSTEB(this.start, this.top, this.end, this.bottom);

  /// Creates insets with only the given values non-zero.
  ///
  /// {@tool snippet}
  ///
  /// A margin indent of 40 pixels on the leading side:
  ///
  /// ```dart
  /// const EdgeInsetsDirectional.only(start: 40.0)
  /// ```
  /// {@end-tool}
  const EdgeInsetsDirectional.only({
    this.start = 0.0,
    this.top = 0.0,
    this.end = 0.0,
    this.bottom = 0.0,
  });

  /// Creates insets with symmetric vertical and horizontal offsets.
  ///
  /// This is equivalent to [EdgeInsets.symmetric], since the inset is the same
  /// with either [TextDirection]. This constructor is just a convenience for
  /// type compatibility.
  ///
  /// {@tool snippet}
  /// Eight pixel margin above and below, no horizontal margins:
  ///
  /// ```dart
  /// const EdgeInsetsDirectional.symmetric(vertical: 8.0)
  /// ```
  /// {@end-tool}
  const EdgeInsetsDirectional.symmetric({double horizontal = 0.0, double vertical = 0.0})
    : start = horizontal,
      end = horizontal,
      top = vertical,
      bottom = vertical;

  /// Creates insets where all the offsets are `value`.
  ///
  /// {@tool snippet}
  ///
  /// Typical eight-pixel margin on all sides:
  ///
  /// ```dart
  /// const EdgeInsetsDirectional.all(8.0)
  /// ```
  /// {@end-tool}
  const EdgeInsetsDirectional.all(double value)
    : start = value,
      top = value,
      end = value,
      bottom = value;

  /// An [EdgeInsetsDirectional] with zero offsets in each direction.
  ///
  /// Consider using [EdgeInsets.zero] instead, since that object has the same
  /// effect, but will be cheaper to [resolve].
  static const EdgeInsetsDirectional zero = EdgeInsetsDirectional.only();

  /// The offset from the start side, the side from which the user will start
  /// reading text.
  ///
  /// This value is normalized into an [EdgeInsets.left] or [EdgeInsets.right]
  /// value by the [resolve] method.
  final double start;

  @override
  double get _start => start;

  /// The offset from the top.
  ///
  /// This value is passed through to [EdgeInsets.top] unmodified by the
  /// [resolve] method.
  final double top;

  @override
  double get _top => top;

  /// The offset from the end side, the side on which the user ends reading
  /// text.
  ///
  /// This value is normalized into an [EdgeInsets.left] or [EdgeInsets.right]
  /// value by the [resolve] method.
  final double end;

  @override
  double get _end => end;

  /// The offset from the bottom.
  ///
  /// This value is passed through to [EdgeInsets.bottom] unmodified by the
  /// [resolve] method.
  final double bottom;

  @override
  double get _bottom => bottom;

  @override
  double get _left => 0.0;

  @override
  double get _right => 0.0;

  @override
  bool get isNonNegative => start >= 0.0 && top >= 0.0 && end >= 0.0 && bottom >= 0.0;

  /// An [EdgeInsetsDirectional] with [top] and [bottom] as well as [start] and [end] flipped.
  @override
  EdgeInsetsDirectional get flipped => EdgeInsetsDirectional.fromSTEB(end, bottom, start, top);

  @override
  EdgeInsetsGeometry subtract(EdgeInsetsGeometry other) {
    if (other is EdgeInsetsDirectional) {
      return this - other;
    }
    return super.subtract(other);
  }

  @override
  EdgeInsetsGeometry add(EdgeInsetsGeometry other) {
    if (other is EdgeInsetsDirectional) {
      return this + other;
    }
    return super.add(other);
  }

  /// Returns the difference between two [EdgeInsetsDirectional] objects.
  EdgeInsetsDirectional operator -(EdgeInsetsDirectional other) {
    return EdgeInsetsDirectional.fromSTEB(
      start - other.start,
      top - other.top,
      end - other.end,
      bottom - other.bottom,
    );
  }

  /// Returns the sum of two [EdgeInsetsDirectional] objects.
  EdgeInsetsDirectional operator +(EdgeInsetsDirectional other) {
    return EdgeInsetsDirectional.fromSTEB(
      start + other.start,
      top + other.top,
      end + other.end,
      bottom + other.bottom,
    );
  }

  /// Returns the [EdgeInsetsDirectional] object with each dimension negated.
  ///
  /// This is the same as multiplying the object by -1.0.
  @override
  EdgeInsetsDirectional operator -() {
    return EdgeInsetsDirectional.fromSTEB(-start, -top, -end, -bottom);
  }

  /// Scales the [EdgeInsetsDirectional] object in each dimension by the given factor.
  @override
  EdgeInsetsDirectional operator *(double other) {
    return EdgeInsetsDirectional.fromSTEB(start * other, top * other, end * other, bottom * other);
  }

  /// Divides the [EdgeInsetsDirectional] object in each dimension by the given factor.
  @override
  EdgeInsetsDirectional operator /(double other) {
    return EdgeInsetsDirectional.fromSTEB(start / other, top / other, end / other, bottom / other);
  }

  /// Integer divides the [EdgeInsetsDirectional] object in each dimension by the given factor.
  @override
  EdgeInsetsDirectional operator ~/(double other) {
    return EdgeInsetsDirectional.fromSTEB(
      (start ~/ other).toDouble(),
      (top ~/ other).toDouble(),
      (end ~/ other).toDouble(),
      (bottom ~/ other).toDouble(),
    );
  }

  /// Computes the remainder in each dimension by the given factor.
  @override
  EdgeInsetsDirectional operator %(double other) {
    return EdgeInsetsDirectional.fromSTEB(start % other, top % other, end % other, bottom % other);
  }

  /// Linearly interpolate between two [EdgeInsetsDirectional].
  ///
  /// If either is null, this function interpolates from [EdgeInsetsDirectional.zero].
  ///
  /// To interpolate between two [EdgeInsetsGeometry] objects of arbitrary type
  /// (either [EdgeInsets] or [EdgeInsetsDirectional]), consider the
  /// [EdgeInsetsGeometry.lerp] static method.
  ///
  /// {@macro dart.ui.shadow.lerp}
  static EdgeInsetsDirectional? lerp(EdgeInsetsDirectional? a, EdgeInsetsDirectional? b, double t) {
    if (identical(a, b)) {
      return a;
    }
    if (a == null) {
      return b! * t;
    }
    if (b == null) {
      return a * (1.0 - t);
    }
    return EdgeInsetsDirectional.fromSTEB(
      ui.lerpDouble(a.start, b.start, t)!,
      ui.lerpDouble(a.top, b.top, t)!,
      ui.lerpDouble(a.end, b.end, t)!,
      ui.lerpDouble(a.bottom, b.bottom, t)!,
    );
  }

  @override
  EdgeInsets resolve(TextDirection? direction) {
    assert(direction != null);
    return switch (direction!) {
      TextDirection.rtl => EdgeInsets.fromLTRB(end, top, start, bottom),
      TextDirection.ltr => EdgeInsets.fromLTRB(start, top, end, bottom),
    };
  }

  /// Creates a copy of this EdgeInsetsDirectional but with the given
  /// fields replaced with the new values.
  EdgeInsetsDirectional copyWith({double? start, double? top, double? end, double? bottom}) {
    return EdgeInsetsDirectional.only(
      start: start ?? this.start,
      top: top ?? this.top,
      end: end ?? this.end,
      bottom: bottom ?? this.bottom,
    );
  }
}

class _MixedEdgeInsets extends EdgeInsetsGeometry {
  const _MixedEdgeInsets.fromLRSETB(
    this._left,
    this._right,
    this._start,
    this._end,
    this._top,
    this._bottom,
  );

  @override
  final double _left;

  @override
  final double _right;

  @override
  final double _start;

  @override
  final double _end;

  @override
  final double _top;

  @override
  final double _bottom;

  @override
  bool get isNonNegative {
    return _left >= 0.0 &&
        _right >= 0.0 &&
        _start >= 0.0 &&
        _end >= 0.0 &&
        _top >= 0.0 &&
        _bottom >= 0.0;
  }

  @override
  _MixedEdgeInsets operator -() {
    return _MixedEdgeInsets.fromLRSETB(-_left, -_right, -_start, -_end, -_top, -_bottom);
  }

  @override
  _MixedEdgeInsets operator *(double other) {
    return _MixedEdgeInsets.fromLRSETB(
      _left * other,
      _right * other,
      _start * other,
      _end * other,
      _top * other,
      _bottom * other,
    );
  }

  @override
  _MixedEdgeInsets operator /(double other) {
    return _MixedEdgeInsets.fromLRSETB(
      _left / other,
      _right / other,
      _start / other,
      _end / other,
      _top / other,
      _bottom / other,
    );
  }

  @override
  _MixedEdgeInsets operator ~/(double other) {
    return _MixedEdgeInsets.fromLRSETB(
      (_left ~/ other).toDouble(),
      (_right ~/ other).toDouble(),
      (_start ~/ other).toDouble(),
      (_end ~/ other).toDouble(),
      (_top ~/ other).toDouble(),
      (_bottom ~/ other).toDouble(),
    );
  }

  @override
  _MixedEdgeInsets operator %(double other) {
    return _MixedEdgeInsets.fromLRSETB(
      _left % other,
      _right % other,
      _start % other,
      _end % other,
      _top % other,
      _bottom % other,
    );
  }

  @override
  EdgeInsets resolve(TextDirection? direction) {
    assert(direction != null);
    return switch (direction!) {
      TextDirection.rtl => EdgeInsets.fromLTRB(_end + _left, _top, _start + _right, _bottom),
      TextDirection.ltr => EdgeInsets.fromLTRB(_start + _left, _top, _end + _right, _bottom),
    };
  }
}
