// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
library;

import 'dart:ui' show lerpDouble;

import 'package:flutter/foundation.dart';

import 'basic_types.dart';
import 'borders.dart';
import 'edge_insets.dart';

/// Defines the relative size and alignment of one [LinearBorder] edge.
///
/// A [LinearBorder] defines a box outline as zero to four edges, each
/// of which is rendered as a single line. The width and color of the
/// lines is defined by [LinearBorder.side].
///
/// Each line's length is defined by [size], a value between 0.0 and 1.0
/// (the default) which defines the length as a percentage of the
/// length of a box edge.
///
/// When [size] is less than 1.0, the line is aligned within the
/// available space according to [alignment], a value between -1.0 and
/// 1.0.  The default is 0.0, which means centered, -1.0 means align on the
/// "start" side, and 1.0 means align on the "end" side. The meaning of
/// start and end depend on the current [TextDirection], see
/// [Directionality].
@immutable
class LinearBorderEdge {
  /// Defines one side of a [LinearBorder].
  ///
  /// The values of [size] and [alignment] must be between
  /// 0.0 and 1.0, and -1.0 and 1.0 respectively.
  const LinearBorderEdge({this.size = 1.0, this.alignment = 0.0})
    : assert(size >= 0.0 && size <= 1.0);

  /// A value between 0.0 and 1.0 that defines the length of the edge as a
  /// percentage of the length of the corresponding box
  /// edge. Default is 1.0.
  final double size;

  /// A value between -1.0 and 1.0 that defines how edges for which [size]
  /// is less than 1.0 are aligned relative to the corresponding box edge.
  ///
  ///  * -1.0, aligned in the "start" direction. That's left
  ///    for [TextDirection.ltr] and right for [TextDirection.rtl].
  ///  * 0.0, centered.
  ///  * 1.0, aligned in the "end" direction. That's right
  ///    for [TextDirection.ltr] and left for [TextDirection.rtl].
  final double alignment;

  /// Linearly interpolates between two [LinearBorder]s.
  ///
  /// If both `a` and `b` are null then null is returned. If `a` is null
  /// then we interpolate to `b` varying [size] from 0.0 to `b.size`. If `b`
  /// is null then we interpolate from `a` varying size from `a.size` to zero.
  /// Otherwise both values are interpolated.
  static LinearBorderEdge? lerp(LinearBorderEdge? a, LinearBorderEdge? b, double t) {
    if (identical(a, b)) {
      return a;
    }

    a ??= LinearBorderEdge(alignment: b!.alignment, size: 0);
    b ??= LinearBorderEdge(alignment: a.alignment, size: 0);

    return LinearBorderEdge(
      size: lerpDouble(a.size, b.size, t)!,
      alignment: lerpDouble(a.alignment, b.alignment, t)!,
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
    return other is LinearBorderEdge && other.size == size && other.alignment == alignment;
  }

  @override
  int get hashCode => Object.hash(size, alignment);

  @override
  String toString() {
    final StringBuffer s = StringBuffer('${objectRuntimeType(this, 'LinearBorderEdge')}(');
    if (size != 1.0) {
      s.write('size: $size');
    }
    if (alignment != 0) {
      final String comma = size != 1.0 ? ', ' : '';
      s.write('${comma}alignment: $alignment');
    }
    s.write(')');
    return s.toString();
  }
}

/// An [OutlinedBorder] like [BoxBorder] that allows one to define a rectangular (box) border
/// in terms of zero to four [LinearBorderEdge]s, each of which is rendered as a single line.
///
/// The color and width of each line are defined by [side]. When [LinearBorder] is used
/// with a class whose border sides and shape are defined by a [ButtonStyle], then a non-null
/// [ButtonStyle.side] will override the one specified here. For example the [LinearBorder]
/// in the [TextButton] example below adds a red underline to the button. This is because
/// TextButton's `side` parameter overrides the `side` property of its [ButtonStyle.shape].
///
/// ```dart
///  TextButton(
///    style: TextButton.styleFrom(
///      side: const BorderSide(color: Colors.red),
///      shape: const LinearBorder(
///        side: BorderSide(color: Colors.blue),
///        bottom: LinearBorderEdge(),
///      ),
///    ),
///    onPressed: () { },
///    child: const Text('Red LinearBorder'),
///  )
///```
///
/// This class resolves itself against the current [TextDirection] (see [Directionality]).
/// Start and end values resolve to left and right for [TextDirection.ltr] and to
/// right and left for [TextDirection.rtl].
///
/// Convenience constructors are included for the common case where just one edge is specified:
/// [LinearBorder.start], [LinearBorder.end], [LinearBorder.top], [LinearBorder.bottom].
///
/// {@tool dartpad}
/// This example shows how to draw different kinds of [LinearBorder]s.
///
/// ** See code in examples/api/lib/painting/linear_border/linear_border.0.dart **
/// {@end-tool}
class LinearBorder extends OutlinedBorder {
  /// Creates a rectangular box border that's rendered as zero to four lines.
  const LinearBorder({super.side, this.start, this.end, this.top, this.bottom});

  /// Creates a rectangular box border with an edge on the left for [TextDirection.ltr]
  /// or on the right for [TextDirection.rtl].
  LinearBorder.start({super.side, double alignment = 0.0, double size = 1.0})
    : start = LinearBorderEdge(alignment: alignment, size: size),
      end = null,
      top = null,
      bottom = null;

  /// Creates a rectangular box border with an edge on the right for [TextDirection.ltr]
  /// or on the left for [TextDirection.rtl].
  LinearBorder.end({super.side, double alignment = 0.0, double size = 1.0})
    : start = null,
      end = LinearBorderEdge(alignment: alignment, size: size),
      top = null,
      bottom = null;

  /// Creates a rectangular box border with an edge on the top.
  LinearBorder.top({super.side, double alignment = 0.0, double size = 1.0})
    : start = null,
      end = null,
      top = LinearBorderEdge(alignment: alignment, size: size),
      bottom = null;

  /// Creates a rectangular box border with an edge on the bottom.
  LinearBorder.bottom({super.side, double alignment = 0.0, double size = 1.0})
    : start = null,
      end = null,
      top = null,
      bottom = LinearBorderEdge(alignment: alignment, size: size);

  /// No border.
  static const LinearBorder none = LinearBorder();

  /// Defines the left edge for [TextDirection.ltr] or the right
  /// for [TextDirection.rtl].
  final LinearBorderEdge? start;

  /// Defines the right edge for [TextDirection.ltr] or the left
  /// for [TextDirection.rtl].
  final LinearBorderEdge? end;

  /// Defines the top edge.
  final LinearBorderEdge? top;

  /// Defines the bottom edge.
  final LinearBorderEdge? bottom;

  @override
  LinearBorder scale(double t) {
    return LinearBorder(side: side.scale(t));
  }

  @override
  EdgeInsetsGeometry get dimensions {
    final double width = side.width;
    return EdgeInsetsDirectional.fromSTEB(
      start == null ? 0.0 : width,
      top == null ? 0.0 : width,
      end == null ? 0.0 : width,
      bottom == null ? 0.0 : width,
    );
  }

  @override
  ShapeBorder? lerpFrom(ShapeBorder? a, double t) {
    if (a is LinearBorder) {
      return LinearBorder(
        side: BorderSide.lerp(a.side, side, t),
        start: LinearBorderEdge.lerp(a.start, start, t),
        end: LinearBorderEdge.lerp(a.end, end, t),
        top: LinearBorderEdge.lerp(a.top, top, t),
        bottom: LinearBorderEdge.lerp(a.bottom, bottom, t),
      );
    }
    return super.lerpFrom(a, t);
  }

  @override
  ShapeBorder? lerpTo(ShapeBorder? b, double t) {
    if (b is LinearBorder) {
      return LinearBorder(
        side: BorderSide.lerp(side, b.side, t),
        start: LinearBorderEdge.lerp(start, b.start, t),
        end: LinearBorderEdge.lerp(end, b.end, t),
        top: LinearBorderEdge.lerp(top, b.top, t),
        bottom: LinearBorderEdge.lerp(bottom, b.bottom, t),
      );
    }
    return super.lerpTo(b, t);
  }

  /// Returns a copy of this LinearBorder with the given fields replaced with
  /// the new values.
  @override
  LinearBorder copyWith({
    BorderSide? side,
    LinearBorderEdge? start,
    LinearBorderEdge? end,
    LinearBorderEdge? top,
    LinearBorderEdge? bottom,
  }) {
    return LinearBorder(
      side: side ?? this.side,
      start: start ?? this.start,
      end: end ?? this.end,
      top: top ?? this.top,
      bottom: bottom ?? this.bottom,
    );
  }

  @override
  Path getInnerPath(Rect rect, {TextDirection? textDirection}) {
    final Rect adjustedRect = dimensions.resolve(textDirection).deflateRect(rect);
    return Path()..addRect(adjustedRect);
  }

  @override
  Path getOuterPath(Rect rect, {TextDirection? textDirection}) {
    return Path()..addRect(rect);
  }

  @override
  void paint(Canvas canvas, Rect rect, {TextDirection? textDirection}) {
    final EdgeInsets insets = dimensions.resolve(textDirection);
    final bool rtl = textDirection == TextDirection.rtl;

    final Path path = Path();
    final Paint paint = Paint()..strokeWidth = 0.0;

    void drawEdge(Rect rect, Color color) {
      paint.color = color;
      path.reset();
      path.moveTo(rect.left, rect.top);
      if (rect.width == 0.0) {
        paint.style = PaintingStyle.stroke;
        path.lineTo(rect.left, rect.bottom);
      } else if (rect.height == 0.0) {
        paint.style = PaintingStyle.stroke;
        path.lineTo(rect.right, rect.top);
      } else {
        paint.style = PaintingStyle.fill;
        path.lineTo(rect.right, rect.top);
        path.lineTo(rect.right, rect.bottom);
        path.lineTo(rect.left, rect.bottom);
      }
      canvas.drawPath(path, paint);
    }

    if (start != null && start!.size != 0.0 && side.style != BorderStyle.none) {
      final Rect insetRect = Rect.fromLTWH(
        rect.left,
        rect.top + insets.top,
        rect.width,
        rect.height - insets.vertical,
      );
      final double x = rtl ? rect.right - insets.right : rect.left;
      final double width = rtl ? insets.right : insets.left;
      final double height = insetRect.height * start!.size;
      final double y = (insetRect.height - height) * ((start!.alignment + 1.0) / 2.0);
      final Rect r = Rect.fromLTWH(x, y, width, height);
      drawEdge(r, side.color);
    }

    if (end != null && end!.size != 0.0 && side.style != BorderStyle.none) {
      final Rect insetRect = Rect.fromLTWH(
        rect.left,
        rect.top + insets.top,
        rect.width,
        rect.height - insets.vertical,
      );
      final double x = rtl ? rect.left : rect.right - insets.right;
      final double width = rtl ? insets.left : insets.right;
      final double height = insetRect.height * end!.size;
      final double y = (insetRect.height - height) * ((end!.alignment + 1.0) / 2.0);
      final Rect r = Rect.fromLTWH(x, y, width, height);
      drawEdge(r, side.color);
    }

    if (top != null && top!.size != 0.0 && side.style != BorderStyle.none) {
      final double width = rect.width * top!.size;
      final double startX = (rect.width - width) * ((top!.alignment + 1.0) / 2.0);
      final double x = rtl ? rect.width - startX - width : startX;
      final Rect r = Rect.fromLTWH(x, rect.top, width, insets.top);
      drawEdge(r, side.color);
    }

    if (bottom != null && bottom!.size != 0.0 && side.style != BorderStyle.none) {
      final double width = rect.width * bottom!.size;
      final double startX = (rect.width - width) * ((bottom!.alignment + 1.0) / 2.0);
      final double x = rtl ? rect.width - startX - width : startX;
      final Rect r = Rect.fromLTWH(x, rect.bottom - insets.bottom, width, side.width);
      drawEdge(r, side.color);
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
    return other is LinearBorder &&
        other.side == side &&
        other.start == start &&
        other.end == end &&
        other.top == top &&
        other.bottom == bottom;
  }

  @override
  int get hashCode => Object.hash(side, start, end, top, bottom);

  @override
  String toString() {
    if (this == LinearBorder.none) {
      return 'LinearBorder.none';
    }

    final StringBuffer s = StringBuffer('${objectRuntimeType(this, 'LinearBorder')}(side: $side');

    if (start != null) {
      s.write(', start: $start');
    }
    if (end != null) {
      s.write(', end: $end');
    }
    if (top != null) {
      s.write(', top: $top');
    }
    if (bottom != null) {
      s.write(', bottom: $bottom');
    }
    s.write(')');
    return s.toString();
  }
}
