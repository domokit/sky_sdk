// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/cupertino.dart';
/// @docImport 'package:flutter/material.dart';
library;

import 'dart:math' as math;

import 'basic.dart';
import 'debug.dart';
import 'framework.dart';

/// [NavigationToolbar] is a layout helper to position 3 widgets or groups of
/// widgets along a horizontal axis that's sensible for an application's
/// navigation bar such as in Material Design and in iOS.
///
/// The [leading] and [trailing] widgets occupy the edges of the widget with
/// reasonable size constraints while the [middle] widget occupies the remaining
/// space in either a center aligned or start aligned fashion.
///
/// Either directly use the themed app bars such as the Material [AppBar] or
/// the iOS [CupertinoNavigationBar] or wrap this widget with more theming
/// specifications for your own custom app bar.
class NavigationToolbar extends StatelessWidget {
  /// Creates a widget that lays out its children in a manner suitable for a
  /// toolbar.
  const NavigationToolbar({
    super.key,
    this.leading,
    this.middle,
    this.trailing,
    this.centerMiddle = true,
    this.middleSpacing = kMiddleSpacing,
  });

  /// The default spacing around the [middle] widget in dp.
  static const double kMiddleSpacing = 16.0;

  /// Widget to place at the start of the horizontal toolbar.
  final Widget? leading;

  /// Widget to place in the middle of the horizontal toolbar, occupying
  /// as much remaining space as possible.
  final Widget? middle;

  /// Widget to place at the end of the horizontal toolbar.
  final Widget? trailing;

  /// Whether to align the [middle] widget to the center of this widget or
  /// next to the [leading] widget when false.
  final bool centerMiddle;

  /// The spacing around the [middle] widget on horizontal axis.
  ///
  /// Defaults to [kMiddleSpacing].
  final double middleSpacing;

  @override
  Widget build(BuildContext context) {
    assert(debugCheckHasDirectionality(context));
    final TextDirection textDirection = Directionality.of(context);
    return CustomMultiChildLayout(
      delegate: _ToolbarLayout(
        centerMiddle: centerMiddle,
        middleSpacing: middleSpacing,
        textDirection: textDirection,
      ),
      children: <Widget>[
        if (leading != null) LayoutId(id: _ToolbarSlot.leading, child: leading!),
        if (middle != null) LayoutId(id: _ToolbarSlot.middle, child: middle!),
        if (trailing != null) LayoutId(id: _ToolbarSlot.trailing, child: trailing!),
      ],
    );
  }
}

enum _ToolbarSlot { leading, middle, trailing }

class _ToolbarLayout extends MultiChildLayoutDelegate {
  _ToolbarLayout({
    required this.centerMiddle,
    required this.middleSpacing,
    required this.textDirection,
  });

  // If false the middle widget should be start-justified within the space
  // between the leading and trailing widgets.
  // If true the middle widget is centered within the toolbar (not within the horizontal
  // space between the leading and trailing widgets).
  final bool centerMiddle;

  /// The spacing around middle widget on horizontal axis.
  final double middleSpacing;

  final TextDirection textDirection;

  @override
  void performLayout(Size size) {
    double leadingWidth = 0.0;
    double trailingWidth = 0.0;

    if (hasChild(_ToolbarSlot.leading)) {
      final BoxConstraints constraints = BoxConstraints(
        maxWidth: size.width,
        minHeight: size.height, // The height should be exactly the height of the bar.
        maxHeight: size.height,
      );
      leadingWidth = layoutChild(_ToolbarSlot.leading, constraints).width;
      final double leadingX = switch (textDirection) {
        TextDirection.rtl => size.width - leadingWidth,
        TextDirection.ltr => 0.0,
      };
      positionChild(_ToolbarSlot.leading, Offset(leadingX, 0.0));
    }

    if (hasChild(_ToolbarSlot.trailing)) {
      final BoxConstraints constraints = BoxConstraints.loose(size);
      final Size trailingSize = layoutChild(_ToolbarSlot.trailing, constraints);
      final double trailingX = switch (textDirection) {
        TextDirection.rtl => 0.0,
        TextDirection.ltr => size.width - trailingSize.width,
      };
      final double trailingY = (size.height - trailingSize.height) / 2.0;
      trailingWidth = trailingSize.width;
      positionChild(_ToolbarSlot.trailing, Offset(trailingX, trailingY));
    }

    if (hasChild(_ToolbarSlot.middle)) {
      final double maxWidth = math.max(
        size.width - leadingWidth - trailingWidth - middleSpacing * 2.0,
        0.0,
      );
      final BoxConstraints constraints = BoxConstraints.loose(size).copyWith(maxWidth: maxWidth);
      final Size middleSize = layoutChild(_ToolbarSlot.middle, constraints);

      final double middleStartMargin = leadingWidth + middleSpacing;
      double middleStart = middleStartMargin;
      final double middleY = (size.height - middleSize.height) / 2.0;
      // If the centered middle will not fit between the leading and trailing
      // widgets, then align its left or right edge with the adjacent boundary.
      if (centerMiddle) {
        middleStart = (size.width - middleSize.width) / 2.0;
        if (middleStart + middleSize.width > size.width - trailingWidth) {
          middleStart = size.width - trailingWidth - middleSize.width - middleSpacing;
        } else if (middleStart < middleStartMargin) {
          middleStart = middleStartMargin;
        }
      }

      final double middleX = switch (textDirection) {
        TextDirection.rtl => size.width - middleSize.width - middleStart,
        TextDirection.ltr => middleStart,
      };

      positionChild(_ToolbarSlot.middle, Offset(middleX, middleY));
    }
  }

  @override
  bool shouldRelayout(_ToolbarLayout oldDelegate) {
    return oldDelegate.centerMiddle != centerMiddle ||
        oldDelegate.middleSpacing != middleSpacing ||
        oldDelegate.textDirection != textDirection;
  }
}
