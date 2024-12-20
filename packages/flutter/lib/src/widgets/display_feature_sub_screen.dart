// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/cupertino.dart';
/// @docImport 'package:flutter/material.dart';
///
/// @docImport 'safe_area.dart';
library;

import 'dart:math' as math;
import 'dart:ui' show DisplayFeature, DisplayFeatureState;

import 'basic.dart';
import 'debug.dart';
import 'framework.dart';
import 'media_query.dart';

/// Positions [child] such that it avoids overlapping any [DisplayFeature] that
/// splits the screen into sub-screens.
///
/// A [DisplayFeature] splits the screen into sub-screens when both these
/// conditions are met:
///
///   * it obstructs the screen, meaning the area it occupies is not 0 or the
///     `state` is [DisplayFeatureState.postureHalfOpened].
///   * it is at least as tall as the screen, producing a left and right
///     sub-screen or it is at least as wide as the screen, producing a top and
///     bottom sub-screen
///
/// After determining the sub-screens, the closest one to [anchorPoint] is used
/// to render the content.
///
/// If no [anchorPoint] is provided, then [Directionality] is used:
///
///   * for [TextDirection.ltr], [anchorPoint] is `Offset.zero`, which will
///     cause the content to appear in the top-left sub-screen.
///   * for [TextDirection.rtl], [anchorPoint] is `Offset(double.maxFinite, 0)`,
///     which will cause the content to appear in the top-right sub-screen.
///
/// If no [anchorPoint] is provided, and there is no [Directionality] ancestor
/// widget in the tree, then the widget asserts during build in debug mode.
///
/// Similarly to [SafeArea], this widget assumes there is no added padding
/// between it and the first [MediaQuery] ancestor. The [child] is wrapped in a
/// new [MediaQuery] instance containing the [DisplayFeature]s that exist in the
/// selected sub-screen, with coordinates relative to the sub-screen. Padding is
/// also adjusted to zero out any sides that were avoided by this widget.
///
/// See also:
///
///  * [showDialog], which is a way to display a [DialogRoute].
///  * [showCupertinoDialog], which displays an iOS-style dialog.
class DisplayFeatureSubScreen extends StatelessWidget {
  /// Creates a widget that positions its child so that it avoids display
  /// features.
  const DisplayFeatureSubScreen({super.key, this.anchorPoint, required this.child});

  /// {@template flutter.widgets.DisplayFeatureSubScreen.anchorPoint}
  /// The anchor point used to pick the closest sub-screen.
  ///
  /// If the anchor point sits inside one of these sub-screens, then that
  /// sub-screen is picked. If not, then the sub-screen with the closest edge to
  /// the point is used.
  ///
  /// [Offset.zero] is the top-left corner of the available screen space. For a
  /// vertically split dual-screen device, this is the top-left corner of the
  /// left screen.
  ///
  /// When this is null, [Directionality] is used:
  ///
  ///   * for [TextDirection.ltr], [anchorPoint] is [Offset.zero], which will
  ///     cause the top-left sub-screen to be picked.
  ///   * for [TextDirection.rtl], [anchorPoint] is
  ///     `Offset(double.maxFinite, 0)`, which will cause the top-right
  ///     sub-screen to be picked.
  /// {@endtemplate}
  final Offset? anchorPoint;

  /// The widget below this widget in the tree.
  ///
  /// The padding on the [MediaQuery] for the [child] will be suitably adjusted
  /// to zero out any sides that were avoided by this widget. The [MediaQuery]
  /// for the [child] will no longer contain any display features that split the
  /// screen into sub-screens.
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget child;

  @override
  Widget build(BuildContext context) {
    assert(
      anchorPoint != null ||
          debugCheckHasDirectionality(
            context,
            why: 'to determine which sub-screen DisplayFeatureSubScreen uses',
            alternative:
                "Alternatively, consider specifying the 'anchorPoint' argument on the DisplayFeatureSubScreen.",
          ),
    );
    final MediaQueryData mediaQuery = MediaQuery.of(context);
    final Size parentSize = mediaQuery.size;
    final Rect wantedBounds = Offset.zero & parentSize;
    final Offset resolvedAnchorPoint = _capOffset(
      anchorPoint ?? _fallbackAnchorPoint(context),
      parentSize,
    );
    final Iterable<Rect> subScreens = subScreensInBounds(wantedBounds, avoidBounds(mediaQuery));
    final Rect closestSubScreen = _closestToAnchorPoint(subScreens, resolvedAnchorPoint);

    return Padding(
      padding: EdgeInsets.only(
        left: closestSubScreen.left,
        top: closestSubScreen.top,
        right: parentSize.width - closestSubScreen.right,
        bottom: parentSize.height - closestSubScreen.bottom,
      ),
      child: MediaQuery(data: mediaQuery.removeDisplayFeatures(closestSubScreen), child: child),
    );
  }

  static Offset _fallbackAnchorPoint(BuildContext context) {
    return switch (Directionality.of(context)) {
      TextDirection.rtl => const Offset(double.maxFinite, 0),
      TextDirection.ltr => Offset.zero,
    };
  }

  /// Returns the areas of the screen that are obstructed by display features.
  ///
  /// A [DisplayFeature] obstructs the screen when the area it occupies is
  /// not 0 or the `state` is [DisplayFeatureState.postureHalfOpened].
  static Iterable<Rect> avoidBounds(MediaQueryData mediaQuery) {
    return mediaQuery.displayFeatures
        .where(
          (DisplayFeature d) =>
              d.bounds.shortestSide > 0 || d.state == DisplayFeatureState.postureHalfOpened,
        )
        .map((DisplayFeature d) => d.bounds);
  }

  /// Returns the closest sub-screen to the [anchorPoint].
  static Rect _closestToAnchorPoint(Iterable<Rect> subScreens, Offset anchorPoint) {
    Rect closestScreen = subScreens.first;
    double closestDistance = _distanceFromPointToRect(anchorPoint, closestScreen);
    for (final Rect screen in subScreens) {
      final double subScreenDistance = _distanceFromPointToRect(anchorPoint, screen);
      if (subScreenDistance < closestDistance) {
        closestScreen = screen;
        closestDistance = subScreenDistance;
      }
    }
    return closestScreen;
  }

  static double _distanceFromPointToRect(Offset point, Rect rect) {
    // Cases for point position relative to rect:
    // 1  2  3
    // 4 [R] 5
    // 6  7  8
    if (point.dx < rect.left) {
      if (point.dy < rect.top) {
        // Case 1
        return (point - rect.topLeft).distance;
      } else if (point.dy > rect.bottom) {
        // Case 6
        return (point - rect.bottomLeft).distance;
      } else {
        // Case 4
        return rect.left - point.dx;
      }
    } else if (point.dx > rect.right) {
      if (point.dy < rect.top) {
        // Case 3
        return (point - rect.topRight).distance;
      } else if (point.dy > rect.bottom) {
        // Case 8
        return (point - rect.bottomRight).distance;
      } else {
        // Case 5
        return point.dx - rect.right;
      }
    } else {
      if (point.dy < rect.top) {
        // Case 2
        return rect.top - point.dy;
      } else if (point.dy > rect.bottom) {
        // Case 7
        return point.dy - rect.bottom;
      } else {
        // Case R
        return 0;
      }
    }
  }

  /// Returns sub-screens resulted by dividing [wantedBounds] along items of
  /// [avoidBounds] that are at least as tall or as wide.
  static Iterable<Rect> subScreensInBounds(Rect wantedBounds, Iterable<Rect> avoidBounds) {
    Iterable<Rect> subScreens = <Rect>[wantedBounds];
    for (final Rect bounds in avoidBounds) {
      final List<Rect> newSubScreens = <Rect>[];
      for (final Rect screen in subScreens) {
        if (screen.top >= bounds.top && screen.bottom <= bounds.bottom) {
          // Display feature splits the screen vertically
          if (screen.left < bounds.left) {
            // There is a smaller sub-screen, left of the display feature
            newSubScreens.add(
              Rect.fromLTWH(screen.left, screen.top, bounds.left - screen.left, screen.height),
            );
          }
          if (screen.right > bounds.right) {
            // There is a smaller sub-screen, right of the display feature
            newSubScreens.add(
              Rect.fromLTWH(bounds.right, screen.top, screen.right - bounds.right, screen.height),
            );
          }
        } else if (screen.left >= bounds.left && screen.right <= bounds.right) {
          // Display feature splits the sub-screen horizontally
          if (screen.top < bounds.top) {
            // There is a smaller sub-screen, above the display feature
            newSubScreens.add(
              Rect.fromLTWH(screen.left, screen.top, screen.width, bounds.top - screen.top),
            );
          }
          if (screen.bottom > bounds.bottom) {
            // There is a smaller sub-screen, below the display feature
            newSubScreens.add(
              Rect.fromLTWH(
                screen.left,
                bounds.bottom,
                screen.width,
                screen.bottom - bounds.bottom,
              ),
            );
          }
        } else {
          newSubScreens.add(screen);
        }
      }
      subScreens = newSubScreens;
    }
    return subScreens;
  }

  static Offset _capOffset(Offset offset, Size maximum) {
    if (offset.dx >= 0 &&
        offset.dx <= maximum.width &&
        offset.dy >= 0 &&
        offset.dy <= maximum.height) {
      return offset;
    } else {
      return Offset(
        math.min(math.max(0, offset.dx), maximum.width),
        math.min(math.max(0, offset.dy), maximum.height),
      );
    }
  }
}
