// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' as ui show lerpDouble;

import 'package:flutter/foundation.dart';

import 'alignment.dart';
import 'basic_types.dart';

/// An offset that's expressed as a fraction of a [Size].
///
/// `FractionalOffset(1.0, 0.0)` represents the top right of the [Size].
///
/// `FractionalOffset(0.0, 1.0)` represents the bottom left of the [Size].
///
/// `FractionalOffset(0.5, 2.0)` represents a point half way across the [Size],
/// below the bottom of the rectangle by the height of the [Size].
///
/// The [FractionalOffset] class specifies offsets in terms of a distance from
/// the top left, regardless of the [TextDirection].
///
/// ## Design discussion
///
/// [FractionalOffset] and [Alignment] are two different representations of the
/// same information: the location within a rectangle relative to the size of
/// the rectangle. The difference between the two classes is in the coordinate
/// system they use to represent the location.
///
/// [FractionalOffset] uses a coordinate system with an origin in the top-left
/// corner of the rectangle whereas [Alignment] uses a coordinate system with an
/// origin in the center of the rectangle.
///
/// Historically, [FractionalOffset] predates [Alignment]. When we attempted to
/// make a version of [FractionalOffset] that adapted to the [TextDirection], we
/// ran into difficulty because placing the origin in the top-left corner
/// introduced a left-to-right bias that was hard to remove.
///
/// By placing the origin in the center, [Alignment] and [AlignmentDirectional]
/// are able to use the same origin, which means we can use a linear function to
/// resolve an [AlignmentDirectional] into an [Alignment] in both
/// [TextDirection.rtl] and [TextDirection.ltr].
///
/// [Alignment] is better for most purposes than [FractionalOffset] and should
/// be used instead of [FractionalOffset]. We continue to implement
/// [FractionalOffset] to support code that predates [Alignment].
///
/// See also:
///
///  * [Alignment], which uses a coordinate system based on the center of the
///    rectangle instead of the top left corner of the rectangle.
@immutable
class FractionalOffset extends Alignment {
  /// Creates a fractional offset.
  const FractionalOffset(double dx, double dy) : super(dx * 2.0 - 1.0, dy * 2.0 - 1.0);

  /// Creates a fractional offset from a specific offset and size.
  ///
  /// The returned [FractionalOffset] describes the position of the
  /// [Offset] in the [Size], as a fraction of the [Size].
  factory FractionalOffset.fromOffsetAndSize(Offset offset, Size size) {
    return FractionalOffset(offset.dx / size.width, offset.dy / size.height);
  }

  /// Creates a fractional offset from a specific offset and rectangle.
  ///
  /// The offset is assumed to be relative to the same origin as the rectangle.
  ///
  /// If the offset is relative to the top left of the rectangle, use [
  /// FractionalOffset.fromOffsetAndSize] instead, passing `rect.size`.
  ///
  /// The returned [FractionalOffset] describes the position of the
  /// [Offset] in the [Rect], as a fraction of the [Rect].
  factory FractionalOffset.fromOffsetAndRect(Offset offset, Rect rect) {
    return FractionalOffset.fromOffsetAndSize(offset - rect.topLeft, rect.size);
  }

  /// The distance fraction in the horizontal direction.
  ///
  /// A value of 0.0 corresponds to the leftmost edge. A value of 1.0
  /// corresponds to the rightmost edge. Values are not limited to that range;
  /// negative values represent positions to the left of the left edge, and
  /// values greater than 1.0 represent positions to the right of the right
  /// edge.
  double get dx => (x + 1.0) / 2.0;

  /// The distance fraction in the vertical direction.
  ///
  /// A value of 0.0 corresponds to the topmost edge. A value of 1.0 corresponds
  /// to the bottommost edge. Values are not limited to that range; negative
  /// values represent positions above the top, and values greater than 1.0
  /// represent positions below the bottom.
  double get dy => (y + 1.0) / 2.0;

  /// The top left corner.
  static const FractionalOffset topLeft = FractionalOffset(0.0, 0.0);

  /// The center point along the top edge.
  static const FractionalOffset topCenter = FractionalOffset(0.5, 0.0);

  /// The top right corner.
  static const FractionalOffset topRight = FractionalOffset(1.0, 0.0);

  /// The center point along the left edge.
  static const FractionalOffset centerLeft = FractionalOffset(0.0, 0.5);

  /// The center point, both horizontally and vertically.
  static const FractionalOffset center = FractionalOffset(0.5, 0.5);

  /// The center point along the right edge.
  static const FractionalOffset centerRight = FractionalOffset(1.0, 0.5);

  /// The bottom left corner.
  static const FractionalOffset bottomLeft = FractionalOffset(0.0, 1.0);

  /// The center point along the bottom edge.
  static const FractionalOffset bottomCenter = FractionalOffset(0.5, 1.0);

  /// The bottom right corner.
  static const FractionalOffset bottomRight = FractionalOffset(1.0, 1.0);

  @override
  Alignment operator -(Alignment other) {
    if (other is! FractionalOffset) {
      return super - other;
    }
    return FractionalOffset(dx - other.dx, dy - other.dy);
  }

  @override
  Alignment operator +(Alignment other) {
    if (other is! FractionalOffset) {
      return super + other;
    }
    return FractionalOffset(dx + other.dx, dy + other.dy);
  }

  @override
  FractionalOffset operator -() {
    return FractionalOffset(-dx, -dy);
  }

  @override
  FractionalOffset operator *(double other) {
    return FractionalOffset(dx * other, dy * other);
  }

  @override
  FractionalOffset operator /(double other) {
    return FractionalOffset(dx / other, dy / other);
  }

  @override
  FractionalOffset operator ~/(double other) {
    return FractionalOffset((dx ~/ other).toDouble(), (dy ~/ other).toDouble());
  }

  @override
  FractionalOffset operator %(double other) {
    return FractionalOffset(dx % other, dy % other);
  }

  /// Linearly interpolate between two [FractionalOffset]s.
  ///
  /// If either is null, this function interpolates from [FractionalOffset.center].
  ///
  /// {@macro dart.ui.shadow.lerp}
  static FractionalOffset? lerp(FractionalOffset? a, FractionalOffset? b, double t) {
    if (identical(a, b)) {
      return a;
    }
    if (a == null) {
      return FractionalOffset(ui.lerpDouble(0.5, b!.dx, t)!, ui.lerpDouble(0.5, b.dy, t)!);
    }
    if (b == null) {
      return FractionalOffset(ui.lerpDouble(a.dx, 0.5, t)!, ui.lerpDouble(a.dy, 0.5, t)!);
    }
    return FractionalOffset(ui.lerpDouble(a.dx, b.dx, t)!, ui.lerpDouble(a.dy, b.dy, t)!);
  }

  @override
  String toString() {
    return 'FractionalOffset(${dx.toStringAsFixed(1)}, '
        '${dy.toStringAsFixed(1)})';
  }
}
