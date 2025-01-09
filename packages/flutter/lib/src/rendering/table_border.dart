// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
library;

import 'package:flutter/foundation.dart';
import 'package:flutter/painting.dart' hide Border;

/// Border specification for [Table] widgets.
///
/// This is like [Border], with the addition of two sides: the inner horizontal
/// borders between rows and the inner vertical borders between columns.
///
/// The sides are represented by [BorderSide] objects.
@immutable
class TableBorder {
  /// Creates a border for a table.
  ///
  /// All the sides of the border default to [BorderSide.none].
  const TableBorder({
    this.top = BorderSide.none,
    this.right = BorderSide.none,
    this.bottom = BorderSide.none,
    this.left = BorderSide.none,
    this.horizontalInside = BorderSide.none,
    this.verticalInside = BorderSide.none,
    this.borderRadius = BorderRadius.zero,
  });

  /// A uniform border with all sides the same color and width.
  ///
  /// The sides default to black solid borders, one logical pixel wide.
  factory TableBorder.all({
    Color color = const Color(0xFF000000),
    double width = 1.0,
    BorderStyle style = BorderStyle.solid,
    BorderRadius borderRadius = BorderRadius.zero,
  }) {
    final BorderSide side = BorderSide(color: color, width: width, style: style);
    return TableBorder(
      top: side,
      right: side,
      bottom: side,
      left: side,
      horizontalInside: side,
      verticalInside: side,
      borderRadius: borderRadius,
    );
  }

  /// Creates a border for a table where all the interior sides use the same
  /// styling and all the exterior sides use the same styling.
  const TableBorder.symmetric({
    BorderSide inside = BorderSide.none,
    BorderSide outside = BorderSide.none,
    this.borderRadius = BorderRadius.zero,
  }) : top = outside,
       right = outside,
       bottom = outside,
       left = outside,
       horizontalInside = inside,
       verticalInside = inside;

  /// The top side of this border.
  final BorderSide top;

  /// The right side of this border.
  final BorderSide right;

  /// The bottom side of this border.
  final BorderSide bottom;

  /// The left side of this border.
  final BorderSide left;

  /// The horizontal interior sides of this border.
  final BorderSide horizontalInside;

  /// The vertical interior sides of this border.
  final BorderSide verticalInside;

  /// The [BorderRadius] to use when painting the corners of this border.
  ///
  /// It is also applied to [DataTable]'s [Material].
  final BorderRadius borderRadius;

  /// The widths of the sides of this border represented as an [EdgeInsets].
  ///
  /// This can be used, for example, with a [Padding] widget to inset a box by
  /// the size of these borders.
  EdgeInsets get dimensions {
    return EdgeInsets.fromLTRB(left.width, top.width, right.width, bottom.width);
  }

  /// Whether all the sides of the border (outside and inside) are identical.
  /// Uniform borders are typically more efficient to paint.
  bool get isUniform {
    final Color topColor = top.color;
    if (right.color != topColor ||
        bottom.color != topColor ||
        left.color != topColor ||
        horizontalInside.color != topColor ||
        verticalInside.color != topColor) {
      return false;
    }

    final double topWidth = top.width;
    if (right.width != topWidth ||
        bottom.width != topWidth ||
        left.width != topWidth ||
        horizontalInside.width != topWidth ||
        verticalInside.width != topWidth) {
      return false;
    }

    final BorderStyle topStyle = top.style;
    if (right.style != topStyle ||
        bottom.style != topStyle ||
        left.style != topStyle ||
        horizontalInside.style != topStyle ||
        verticalInside.style != topStyle) {
      return false;
    }

    return true;
  }

  /// Creates a copy of this border but with the widths scaled by the factor `t`.
  ///
  /// The `t` argument represents the multiplicand, or the position on the
  /// timeline for an interpolation from nothing to `this`, with 0.0 meaning
  /// that the object returned should be the nil variant of this object, 1.0
  /// meaning that no change should be applied, returning `this` (or something
  /// equivalent to `this`), and other values meaning that the object should be
  /// multiplied by `t`. Negative values are treated like zero.
  ///
  /// Values for `t` are usually obtained from an [Animation<double>], such as
  /// an [AnimationController].
  ///
  /// See also:
  ///
  ///  * [BorderSide.scale], which is used to implement this method.
  TableBorder scale(double t) {
    return TableBorder(
      top: top.scale(t),
      right: right.scale(t),
      bottom: bottom.scale(t),
      left: left.scale(t),
      horizontalInside: horizontalInside.scale(t),
      verticalInside: verticalInside.scale(t),
    );
  }

  /// Linearly interpolate between two table borders.
  ///
  /// If a border is null, it is treated as having only [BorderSide.none]
  /// borders.
  ///
  /// {@macro dart.ui.shadow.lerp}
  static TableBorder? lerp(TableBorder? a, TableBorder? b, double t) {
    if (identical(a, b)) {
      return a;
    }
    if (a == null) {
      return b!.scale(t);
    }
    if (b == null) {
      return a.scale(1.0 - t);
    }
    return TableBorder(
      top: BorderSide.lerp(a.top, b.top, t),
      right: BorderSide.lerp(a.right, b.right, t),
      bottom: BorderSide.lerp(a.bottom, b.bottom, t),
      left: BorderSide.lerp(a.left, b.left, t),
      horizontalInside: BorderSide.lerp(a.horizontalInside, b.horizontalInside, t),
      verticalInside: BorderSide.lerp(a.verticalInside, b.verticalInside, t),
    );
  }

  /// Paints the border around the given [Rect] on the given [Canvas], with the
  /// given rows and columns.
  ///
  /// Uniform borders are more efficient to paint than more complex borders.
  ///
  /// The `rows` argument specifies the vertical positions between the rows,
  /// relative to the given rectangle. For example, if the table contained two
  /// rows of height 100.0 each, then `rows` would contain a single value,
  /// 100.0, which is the vertical position between the two rows (relative to
  /// the top edge of `rect`).
  ///
  /// The `columns` argument specifies the horizontal positions between the
  /// columns, relative to the given rectangle. For example, if the table
  /// contained two columns of height 100.0 each, then `columns` would contain a
  /// single value, 100.0, which is the vertical position between the two
  /// columns (relative to the left edge of `rect`).
  ///
  /// The [verticalInside] border is only drawn if there are at least two
  /// columns. The [horizontalInside] border is only drawn if there are at least
  /// two rows. The horizontal borders are drawn after the vertical borders.
  ///
  /// The outer borders (in the order [top], [right], [bottom], [left], with
  /// [left] above the others) are painted after the inner borders.
  ///
  /// The paint order is particularly notable in the case of
  /// partially-transparent borders.
  void paint(
    Canvas canvas,
    Rect rect, {
    required Iterable<double> rows,
    required Iterable<double> columns,
  }) {
    // properties can't be null

    // arguments can't be null
    assert(rows.isEmpty || (rows.first >= 0.0 && rows.last <= rect.height));
    assert(columns.isEmpty || (columns.first >= 0.0 && columns.last <= rect.width));

    if (columns.isNotEmpty || rows.isNotEmpty) {
      final Paint paint = Paint();
      final Path path = Path();

      if (columns.isNotEmpty) {
        switch (verticalInside.style) {
          case BorderStyle.solid:
            paint
              ..color = verticalInside.color
              ..strokeWidth = verticalInside.width
              ..style = PaintingStyle.stroke;
            path.reset();
            for (final double x in columns) {
              path.moveTo(rect.left + x, rect.top);
              path.lineTo(rect.left + x, rect.bottom);
            }
            canvas.drawPath(path, paint);
          case BorderStyle.none:
            break;
        }
      }

      if (rows.isNotEmpty) {
        switch (horizontalInside.style) {
          case BorderStyle.solid:
            paint
              ..color = horizontalInside.color
              ..strokeWidth = horizontalInside.width
              ..style = PaintingStyle.stroke;
            path.reset();
            for (final double y in rows) {
              path.moveTo(rect.left, rect.top + y);
              path.lineTo(rect.right, rect.top + y);
            }
            canvas.drawPath(path, paint);
          case BorderStyle.none:
            break;
        }
      }
    }
    if (!isUniform || borderRadius == BorderRadius.zero) {
      paintBorder(canvas, rect, top: top, right: right, bottom: bottom, left: left);
    } else {
      final RRect outer = borderRadius.toRRect(rect);
      final RRect inner = outer.deflate(top.width);
      final Paint paint = Paint()..color = top.color;
      canvas.drawDRRect(outer, inner, paint);
    }
  }

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is TableBorder &&
        other.top == top &&
        other.right == right &&
        other.bottom == bottom &&
        other.left == left &&
        other.horizontalInside == horizontalInside &&
        other.verticalInside == verticalInside &&
        other.borderRadius == borderRadius;
  }

  @override
  int get hashCode =>
      Object.hash(top, right, bottom, left, horizontalInside, verticalInside, borderRadius);

  @override
  String toString() =>
      'TableBorder($top, $right, $bottom, $left, $horizontalInside, $verticalInside, $borderRadius)';
}
