// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'basic.dart';
/// @docImport 'color_filter.dart';
library;

import 'dart:ui';

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';

import 'framework.dart';

/// Applies an [ImageFilter] to its child.
///
/// An image filter will always apply its filter operation to the child widget,
/// even if said filter is conceptually a "no-op", such as an ImageFilter.blur
/// with a radius of 0 or an ImageFilter.matrix with an identity matrix. Setting
/// [ImageFiltered.enabled] to `false` is a more efficient manner of disabling
/// an image filter.
///
/// The framework does not attempt to optimize out "no-op" filters because it
/// cannot tell the difference between an intentional no-op and a filter that is
/// only incidentally a no-op. Consider an ImageFilter.matrix that is animated
/// and happens to pass through the identity matrix. If the framework identified it
/// as a no-op it would drop and then recreate the layer during the animation which
/// would be more expensive than keeping it around.
///
/// {@youtube 560 315 https://www.youtube.com/watch?v=7Lftorq4i2o}
///
/// See also:
///
/// * [BackdropFilter], which applies an [ImageFilter] to everything
///   beneath its child.
/// * [ColorFiltered], which applies a [ColorFilter] to its child.
@immutable
class ImageFiltered extends SingleChildRenderObjectWidget {
  /// Creates a widget that applies an [ImageFilter] to its child.
  const ImageFiltered({super.key, required this.imageFilter, super.child, this.enabled = true});

  /// The image filter to apply to the child of this widget.
  final ImageFilter imageFilter;

  /// Whether or not to apply the image filter operation to the child of this
  /// widget.
  ///
  /// Prefer setting enabled to `false` instead of creating a "no-op" filter
  /// type for performance reasons.
  final bool enabled;

  @override
  RenderObject createRenderObject(BuildContext context) =>
      _ImageFilterRenderObject(imageFilter, enabled);

  @override
  void updateRenderObject(BuildContext context, RenderObject renderObject) {
    (renderObject as _ImageFilterRenderObject)
      ..enabled = enabled
      ..imageFilter = imageFilter;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<ImageFilter>('imageFilter', imageFilter));
  }
}

class _ImageFilterRenderObject extends RenderProxyBox {
  _ImageFilterRenderObject(this._imageFilter, this._enabled);

  bool get enabled => _enabled;
  bool _enabled;
  set enabled(bool value) {
    if (enabled == value) {
      return;
    }
    final bool wasRepaintBoundary = isRepaintBoundary;
    _enabled = value;
    if (isRepaintBoundary != wasRepaintBoundary) {
      markNeedsCompositingBitsUpdate();
    }
    markNeedsPaint();
  }

  ImageFilter get imageFilter => _imageFilter;
  ImageFilter _imageFilter;
  set imageFilter(ImageFilter value) {
    if (value != _imageFilter) {
      _imageFilter = value;
      markNeedsCompositedLayerUpdate();
    }
  }

  @override
  bool get alwaysNeedsCompositing => child != null && enabled;

  @override
  bool get isRepaintBoundary => alwaysNeedsCompositing;

  @override
  OffsetLayer updateCompositedLayer({required covariant ImageFilterLayer? oldLayer}) {
    final ImageFilterLayer layer = oldLayer ?? ImageFilterLayer();
    layer.imageFilter = imageFilter;
    return layer;
  }
}
