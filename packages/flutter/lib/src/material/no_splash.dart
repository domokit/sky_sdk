// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'button_style.dart';
/// @docImport 'elevated_button.dart';
/// @docImport 'theme.dart';
library;

import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';

import 'ink_well.dart';
import 'material.dart';

class _NoSplashFactory extends InteractiveInkFeatureFactory {
  const _NoSplashFactory();

  @override
  InteractiveInkFeature create({
    required MaterialInkController controller,
    required RenderBox referenceBox,
    required Offset position,
    required Color color,
    required TextDirection textDirection,
    bool containedInkWell = false,
    RectCallback? rectCallback,
    BorderRadius? borderRadius,
    ShapeBorder? customBorder,
    double? radius,
    VoidCallback? onRemoved,
  }) {
    return NoSplash(
      controller: controller,
      referenceBox: referenceBox,
      color: color,
      onRemoved: onRemoved,
    );
  }
}

/// An [InteractiveInkFeature] that doesn't paint a splash.
///
/// Use [NoSplash.splashFactory] to defeat the default ink splash drawn by
/// an [InkWell] or [ButtonStyle]. For example, to create an [ElevatedButton]
/// that does not draw the default "ripple" ink splash when it's tapped:
///
/// ```dart
/// ElevatedButton(
///   style: ElevatedButton.styleFrom(
///     splashFactory: NoSplash.splashFactory,
///   ),
///   onPressed: () { },
///   child: const Text('No Splash'),
/// )
/// ```
class NoSplash extends InteractiveInkFeature {
  /// Create an [InteractiveInkFeature] that doesn't paint a splash.
  NoSplash({
    required super.controller,
    required super.referenceBox,
    required super.color,
    super.onRemoved,
  });

  /// Used to specify this type of ink splash for an [InkWell], [InkResponse]
  /// material [Theme], or [ButtonStyle].
  static const InteractiveInkFeatureFactory splashFactory = _NoSplashFactory();

  @override
  void paintFeature(Canvas canvas, Matrix4 transform) {}

  @override
  void confirm() {
    super.confirm();
    dispose();
  }

  @override
  void cancel() {
    super.cancel();
    dispose();
  }
}
