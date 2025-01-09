// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'ink_decoration.dart';
/// @docImport 'ink_splash.dart';
/// @docImport 'ink_well.dart';
library;

import 'package:flutter/widgets.dart';

import 'ink_well.dart' show InteractiveInkFeature;
import 'material.dart';

const Duration _kDefaultHighlightFadeDuration = Duration(milliseconds: 200);

/// A visual emphasis on a part of a [Material] receiving user interaction.
///
/// This object is rarely created directly. Instead of creating an ink highlight
/// directly, consider using an [InkResponse] or [InkWell] widget, which uses
/// gestures (such as tap and long-press) to trigger ink highlights.
///
/// See also:
///
///  * [InkResponse], which uses gestures to trigger ink highlights and ink
///    splashes in the parent [Material].
///  * [InkWell], which is a rectangular [InkResponse] (the most common type of
///    ink response).
///  * [Material], which is the widget on which the ink highlight is painted.
///  * [InkSplash], which is an ink feature that shows a reaction to user input
///    on a [Material].
///  * [Ink], a convenience widget for drawing images and other decorations on
///    Material widgets.
class InkHighlight extends InteractiveInkFeature {
  /// Begin a highlight animation.
  ///
  /// The [controller] argument is typically obtained via
  /// `Material.of(context)`.
  ///
  /// If a `rectCallback` is given, then it provides the highlight rectangle,
  /// otherwise, the highlight rectangle is coincident with the [referenceBox].
  ///
  /// When the highlight is removed, `onRemoved` will be called.
  InkHighlight({
    required super.controller,
    required super.referenceBox,
    required super.color,
    required TextDirection textDirection,
    BoxShape shape = BoxShape.rectangle,
    double? radius,
    BorderRadius? borderRadius,
    super.customBorder,
    RectCallback? rectCallback,
    super.onRemoved,
    Duration fadeDuration = _kDefaultHighlightFadeDuration,
  }) : _shape = shape,
       _radius = radius,
       _borderRadius = borderRadius ?? BorderRadius.zero,

       _textDirection = textDirection,
       _rectCallback = rectCallback {
    _alphaController =
        AnimationController(duration: fadeDuration, vsync: controller.vsync)
          ..addListener(controller.markNeedsPaint)
          ..addStatusListener(_handleAlphaStatusChanged)
          ..forward();
    _alpha = _alphaController.drive(IntTween(begin: 0, end: color.alpha));

    controller.addInkFeature(this);
  }

  final BoxShape _shape;
  final double? _radius;
  final BorderRadius _borderRadius;
  final RectCallback? _rectCallback;
  final TextDirection _textDirection;

  late Animation<int> _alpha;
  late AnimationController _alphaController;

  /// Whether this part of the material is being visually emphasized.
  bool get active => _active;
  bool _active = true;

  /// Start visually emphasizing this part of the material.
  void activate() {
    _active = true;
    _alphaController.forward();
  }

  /// Stop visually emphasizing this part of the material.
  void deactivate() {
    _active = false;
    _alphaController.reverse();
  }

  void _handleAlphaStatusChanged(AnimationStatus status) {
    if (status.isDismissed && !_active) {
      dispose();
    }
  }

  @override
  void dispose() {
    _alphaController.dispose();
    super.dispose();
  }

  void _paintHighlight(Canvas canvas, Rect rect, Paint paint) {
    canvas.save();
    if (customBorder != null) {
      canvas.clipPath(customBorder!.getOuterPath(rect, textDirection: _textDirection));
    }
    switch (_shape) {
      case BoxShape.circle:
        canvas.drawCircle(rect.center, _radius ?? Material.defaultSplashRadius, paint);
      case BoxShape.rectangle:
        if (_borderRadius != BorderRadius.zero) {
          final RRect clipRRect = RRect.fromRectAndCorners(
            rect,
            topLeft: _borderRadius.topLeft,
            topRight: _borderRadius.topRight,
            bottomLeft: _borderRadius.bottomLeft,
            bottomRight: _borderRadius.bottomRight,
          );
          canvas.drawRRect(clipRRect, paint);
        } else {
          canvas.drawRect(rect, paint);
        }
    }
    canvas.restore();
  }

  @override
  void paintFeature(Canvas canvas, Matrix4 transform) {
    final Paint paint = Paint()..color = color.withAlpha(_alpha.value);
    final Offset? originOffset = MatrixUtils.getAsTranslation(transform);
    final Rect rect = _rectCallback != null ? _rectCallback() : Offset.zero & referenceBox.size;
    if (originOffset == null) {
      canvas.save();
      canvas.transform(transform.storage);
      _paintHighlight(canvas, rect, paint);
      canvas.restore();
    } else {
      _paintHighlight(canvas, rect.shift(originOffset), paint);
    }
  }
}
