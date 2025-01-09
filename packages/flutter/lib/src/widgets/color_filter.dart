// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'basic.dart';
library;

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';

import 'framework.dart';

/// Applies a [ColorFilter] to its child.
///
/// This widget applies a function independently to each pixel of [child]'s
/// content, according to the [ColorFilter] specified.
/// Use the [ColorFilter.mode] constructor to apply a [Color] using a [BlendMode].
/// Use the [BackdropFilter] widget instead, if the [ColorFilter]
/// needs to be applied onto the content beneath [child].
///
/// {@youtube 560 315 https://www.youtube.com/watch?v=F7Cll22Dno8}
///
/// {@tool dartpad}
/// These two images have two [ColorFilter]s applied with different [BlendMode]s,
/// one with red color and [BlendMode.modulate] another with a grey color and [BlendMode.saturation].
///
/// ** See code in examples/api/lib/widgets/color_filter/color_filtered.0.dart **
///{@end-tool}
///
/// See also:
///
///  * [BlendMode], describes how to blend a source image with the destination image.
///  * [ColorFilter], which describes a function that modify a color to a different color.

@immutable
class ColorFiltered extends SingleChildRenderObjectWidget {
  /// Creates a widget that applies a [ColorFilter] to its child.
  const ColorFiltered({required this.colorFilter, super.child, super.key});

  /// The color filter to apply to the child of this widget.
  final ColorFilter colorFilter;

  @override
  RenderObject createRenderObject(BuildContext context) => _ColorFilterRenderObject(colorFilter);

  @override
  void updateRenderObject(BuildContext context, RenderObject renderObject) {
    (renderObject as _ColorFilterRenderObject).colorFilter = colorFilter;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<ColorFilter>('colorFilter', colorFilter));
  }
}

class _ColorFilterRenderObject extends RenderProxyBox {
  _ColorFilterRenderObject(this._colorFilter);

  ColorFilter get colorFilter => _colorFilter;
  ColorFilter _colorFilter;
  set colorFilter(ColorFilter value) {
    if (value != _colorFilter) {
      _colorFilter = value;
      markNeedsPaint();
    }
  }

  @override
  bool get alwaysNeedsCompositing => child != null;

  @override
  void paint(PaintingContext context, Offset offset) {
    layer = context.pushColorFilter(
      offset,
      colorFilter,
      super.paint,
      oldLayer: layer as ColorFilterLayer?,
    );
    assert(() {
      layer!.debugCreator = debugCreator;
      return true;
    }());
  }
}
