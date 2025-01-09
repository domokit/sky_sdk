// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math';

import 'package:flutter/material.dart';
import 'animation.dart';

const double contentHeight = 80.0;
const double contentWidth = 300.0;
const double contentHorizontalPadding = 40.0;
const double tapTargetRadius = 44.0;
const double tapTargetToContentDistance = 20.0;
const double gutterHeight = 88.0;

/// Background of the overlay.
class Background extends StatelessWidget {
  const Background({
    super.key,
    required this.animations,
    required this.center,
    required this.color,
    required this.deviceSize,
    required this.status,
    required this.textDirection,
  });

  /// Animations.
  final Animations animations;

  /// Overlay center position.
  final Offset center;

  /// Color of the background.
  final Color color;

  /// Device size.
  final Size deviceSize;

  /// Status of the parent overlay.
  final FeatureDiscoveryStatus status;

  /// Directionality of content.
  final TextDirection textDirection;

  static const double horizontalShift = 20.0;
  static const double padding = 40.0;

  /// Compute the center position of the background.
  ///
  /// If [center] is near the top or bottom edges of the screen, then
  /// background is centered there.
  /// Otherwise, background center is calculated and upon opening, animated
  /// from [center] to the new calculated position.
  Offset get centerPosition {
    if (_isNearTopOrBottomEdges(center, deviceSize)) {
      return center;
    } else {
      final Offset start = center;

      // dy of centerPosition is calculated to be the furthest point in
      // [Content] from the [center].
      double endY;
      if (_isOnTopHalfOfScreen(center, deviceSize)) {
        endY = center.dy - tapTargetRadius - tapTargetToContentDistance - contentHeight;
        if (endY < 0.0) {
          endY = center.dy + tapTargetRadius + tapTargetToContentDistance;
        }
      } else {
        endY = center.dy + tapTargetRadius + tapTargetToContentDistance;
        if (endY + contentHeight > deviceSize.height) {
          endY = center.dy - tapTargetRadius - tapTargetToContentDistance - contentHeight;
        }
      }

      // Horizontal background center shift based on whether the tap target is
      // on the left, center, or right side of the screen.
      double shift;
      if (_isOnLeftHalfOfScreen(center, deviceSize)) {
        shift = horizontalShift;
      } else if (center.dx == deviceSize.width / 2) {
        shift = textDirection == TextDirection.ltr ? -horizontalShift : horizontalShift;
      } else {
        shift = -horizontalShift;
      }

      // dx of centerPosition is calculated to be the middle point of the
      // [Content] bounds shifted by [horizontalShift].
      final Rect textBounds = _getContentBounds(deviceSize, center);
      final double left = min(textBounds.left, center.dx - 88.0);
      final double right = max(textBounds.right, center.dx + 88.0);
      final double endX = (left + right) / 2 + shift;
      final Offset end = Offset(endX, endY);

      return animations.backgroundCenter(status, start, end).value;
    }
  }

  /// Compute the radius.
  ///
  /// Radius is a function of the greatest distance from [center] to one of
  /// the corners of [Content].
  double get radius {
    final Rect textBounds = _getContentBounds(deviceSize, center);
    final double textRadius = _maxDistance(center, textBounds) + padding;
    if (_isNearTopOrBottomEdges(center, deviceSize)) {
      return animations.backgroundRadius(status, textRadius).value;
    } else {
      // Scale down radius if icon is towards the middle of the screen.
      return animations.backgroundRadius(status, textRadius).value * 0.8;
    }
  }

  double get opacity => animations.backgroundOpacity(status).value;

  @override
  Widget build(BuildContext context) {
    return Positioned(
      left: centerPosition.dx,
      top: centerPosition.dy,
      child: FractionalTranslation(
        translation: const Offset(-0.5, -0.5),
        child: Opacity(
          opacity: opacity,
          child: Container(
            height: radius * 2,
            width: radius * 2,
            decoration: BoxDecoration(shape: BoxShape.circle, color: color),
          ),
        ),
      ),
    );
  }

  /// Compute the maximum distance from [point] to the four corners of [bounds].
  double _maxDistance(Offset point, Rect bounds) {
    double distance(double x1, double y1, double x2, double y2) {
      return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
    }

    final double tl = distance(point.dx, point.dy, bounds.left, bounds.top);
    final double tr = distance(point.dx, point.dy, bounds.right, bounds.top);
    final double bl = distance(point.dx, point.dy, bounds.left, bounds.bottom);
    final double br = distance(point.dx, point.dy, bounds.right, bounds.bottom);
    return max(tl, max(tr, max(bl, br)));
  }
}

/// Widget that represents the text to show in the overlay.
class Content extends StatelessWidget {
  const Content({
    super.key,
    required this.animations,
    required this.center,
    required this.description,
    required this.deviceSize,
    required this.status,
    required this.title,
    required this.textTheme,
  });

  /// Animations.
  final Animations animations;

  /// Overlay center position.
  final Offset center;

  /// Description.
  final String description;

  /// Device size.
  final Size deviceSize;

  /// Status of the parent overlay.
  final FeatureDiscoveryStatus status;

  /// Title.
  final String title;

  /// [TextTheme] to use for drawing the [title] and the [description].
  final TextTheme textTheme;

  double get opacity => animations.contentOpacity(status).value;

  @override
  Widget build(BuildContext context) {
    final Rect position = _getContentBounds(deviceSize, center);

    return Positioned(
      left: position.left,
      height: position.bottom - position.top,
      width: position.right - position.left,
      top: position.top,
      child: Opacity(
        opacity: opacity,
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: <Widget>[
            _buildTitle(textTheme),
            const SizedBox(height: 12.0),
            _buildDescription(textTheme),
          ],
        ),
      ),
    );
  }

  Widget _buildTitle(TextTheme theme) {
    return Text(
      title,
      maxLines: 1,
      overflow: TextOverflow.ellipsis,
      style: theme.titleLarge?.copyWith(color: Colors.white),
    );
  }

  Widget _buildDescription(TextTheme theme) {
    return Text(
      description,
      maxLines: 2,
      overflow: TextOverflow.ellipsis,
      style: theme.titleMedium?.copyWith(color: Colors.white70),
    );
  }
}

/// Widget that represents the ripple effect of [TapTarget].
class Ripple extends StatelessWidget {
  const Ripple({super.key, required this.animations, required this.center, required this.status});

  /// Animations.
  final Animations animations;

  /// Overlay center position.
  final Offset center;

  /// Status of the parent overlay.
  final FeatureDiscoveryStatus status;

  double get radius => animations.rippleRadius(status).value;
  double get opacity => animations.rippleOpacity(status).value;

  @override
  Widget build(BuildContext context) {
    return Positioned(
      left: center.dx,
      top: center.dy,
      child: FractionalTranslation(
        translation: const Offset(-0.5, -0.5),
        child: Opacity(
          opacity: opacity,
          child: Container(
            height: radius * 2,
            width: radius * 2,
            decoration: const BoxDecoration(color: Colors.white, shape: BoxShape.circle),
          ),
        ),
      ),
    );
  }
}

/// Wrapper widget around [child] representing the anchor of the overlay.
class TapTarget extends StatelessWidget {
  const TapTarget({
    super.key,
    required this.animations,
    required this.center,
    required this.status,
    required this.onTap,
    required this.child,
  });

  /// Animations.
  final Animations animations;

  /// Device size.
  final Offset center;

  /// Status of the parent overlay.
  final FeatureDiscoveryStatus status;

  /// Callback invoked when the user taps on the [TapTarget].
  final void Function() onTap;

  /// Child widget that will be promoted by the overlay.
  final Icon child;

  double get radius => animations.tapTargetRadius(status).value;
  double get opacity => animations.tapTargetOpacity(status).value;

  @override
  Widget build(BuildContext context) {
    final ThemeData theme = Theme.of(context);
    return Positioned(
      left: center.dx,
      top: center.dy,
      child: FractionalTranslation(
        translation: const Offset(-0.5, -0.5),
        child: InkWell(
          onTap: onTap,
          child: Opacity(
            opacity: opacity,
            child: Container(
              height: radius * 2,
              width: radius * 2,
              decoration: BoxDecoration(
                color:
                    theme.brightness == Brightness.dark ? theme.colorScheme.primary : Colors.white,
                shape: BoxShape.circle,
              ),
              child: child,
            ),
          ),
        ),
      ),
    );
  }
}

/// Method to compute the bounds of the content.
///
/// This is exposed so it can be used for calculating the background radius
/// and center and for laying out the content.
Rect _getContentBounds(Size deviceSize, Offset overlayCenter) {
  double top;
  if (_isOnTopHalfOfScreen(overlayCenter, deviceSize)) {
    top = overlayCenter.dy - tapTargetRadius - tapTargetToContentDistance - contentHeight;
    if (top < 0) {
      top = overlayCenter.dy + tapTargetRadius + tapTargetToContentDistance;
    }
  } else {
    top = overlayCenter.dy + tapTargetRadius + tapTargetToContentDistance;
    if (top + contentHeight > deviceSize.height) {
      top = overlayCenter.dy - tapTargetRadius - tapTargetToContentDistance - contentHeight;
    }
  }

  final double left = max(contentHorizontalPadding, overlayCenter.dx - contentWidth);
  final double right = min(deviceSize.width - contentHorizontalPadding, left + contentWidth);
  return Rect.fromLTRB(left, top, right, top + contentHeight);
}

bool _isNearTopOrBottomEdges(Offset position, Size deviceSize) {
  return position.dy <= gutterHeight || (deviceSize.height - position.dy) <= gutterHeight;
}

bool _isOnTopHalfOfScreen(Offset position, Size deviceSize) {
  return position.dy < (deviceSize.height / 2.0);
}

bool _isOnLeftHalfOfScreen(Offset position, Size deviceSize) {
  return position.dx < (deviceSize.width / 2.0);
}
