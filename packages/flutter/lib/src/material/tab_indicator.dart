// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'tabs.dart';
library;

import 'package:flutter/widgets.dart';

import 'colors.dart';

/// Used with [TabBar.indicator] to draw a horizontal line below the
/// selected tab.
///
/// The selected tab underline is inset from the tab's boundary by [insets].
/// The [borderSide] defines the line's color and weight.
///
/// The [TabBar.indicatorSize] property can be used to define the indicator's
/// bounds in terms of its (centered) widget with [TabBarIndicatorSize.label],
/// or the entire tab with [TabBarIndicatorSize.tab].
class UnderlineTabIndicator extends Decoration {
  /// Create an underline style selected tab indicator.
  const UnderlineTabIndicator({
    this.borderRadius,
    this.borderSide = const BorderSide(width: 2.0, color: Colors.white),
    this.insets = EdgeInsets.zero,
  });

  /// The radius of the indicator's corners.
  ///
  /// If this value is non-null, rounded rectangular tab indicator is
  /// drawn, otherwise rectangular tab indicator is drawn.
  final BorderRadius? borderRadius;

  /// The color and weight of the horizontal line drawn below the selected tab.
  final BorderSide borderSide;

  /// Locates the selected tab's underline relative to the tab's boundary.
  ///
  /// The [TabBar.indicatorSize] property can be used to define the tab
  /// indicator's bounds in terms of its (centered) tab widget with
  /// [TabBarIndicatorSize.label], or the entire tab with
  /// [TabBarIndicatorSize.tab].
  final EdgeInsetsGeometry insets;

  @override
  Decoration? lerpFrom(Decoration? a, double t) {
    if (a is UnderlineTabIndicator) {
      return UnderlineTabIndicator(
        borderSide: BorderSide.lerp(a.borderSide, borderSide, t),
        insets: EdgeInsetsGeometry.lerp(a.insets, insets, t)!,
      );
    }
    return super.lerpFrom(a, t);
  }

  @override
  Decoration? lerpTo(Decoration? b, double t) {
    if (b is UnderlineTabIndicator) {
      return UnderlineTabIndicator(
        borderSide: BorderSide.lerp(borderSide, b.borderSide, t),
        insets: EdgeInsetsGeometry.lerp(insets, b.insets, t)!,
      );
    }
    return super.lerpTo(b, t);
  }

  @override
  BoxPainter createBoxPainter([VoidCallback? onChanged]) {
    return _UnderlinePainter(this, borderRadius, onChanged);
  }

  Rect _indicatorRectFor(Rect rect, TextDirection textDirection) {
    final Rect indicator = insets.resolve(textDirection).deflateRect(rect);
    return Rect.fromLTWH(
      indicator.left,
      indicator.bottom - borderSide.width,
      indicator.width,
      borderSide.width,
    );
  }

  @override
  Path getClipPath(Rect rect, TextDirection textDirection) {
    if (borderRadius != null) {
      return Path()..addRRect(borderRadius!.toRRect(_indicatorRectFor(rect, textDirection)));
    }
    return Path()..addRect(_indicatorRectFor(rect, textDirection));
  }
}

class _UnderlinePainter extends BoxPainter {
  _UnderlinePainter(this.decoration, this.borderRadius, super.onChanged);

  final UnderlineTabIndicator decoration;
  final BorderRadius? borderRadius;

  @override
  void paint(Canvas canvas, Offset offset, ImageConfiguration configuration) {
    assert(configuration.size != null);
    final Rect rect = offset & configuration.size!;
    final TextDirection textDirection = configuration.textDirection!;
    final Paint paint;
    if (borderRadius != null) {
      paint = Paint()..color = decoration.borderSide.color;
      final Rect indicator = decoration._indicatorRectFor(rect, textDirection);
      final RRect rrect = RRect.fromRectAndCorners(
        indicator,
        topLeft: borderRadius!.topLeft,
        topRight: borderRadius!.topRight,
        bottomRight: borderRadius!.bottomRight,
        bottomLeft: borderRadius!.bottomLeft,
      );
      canvas.drawRRect(rrect, paint);
    } else {
      paint = decoration.borderSide.toPaint()..strokeCap = StrokeCap.square;
      final Rect indicator = decoration
          ._indicatorRectFor(rect, textDirection)
          .deflate(decoration.borderSide.width / 2.0);
      canvas.drawLine(indicator.bottomLeft, indicator.bottomRight, paint);
    }
  }
}
