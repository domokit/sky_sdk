// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;

import 'package:meta/meta.dart';
import 'package:ui/ui.dart' as ui;

import 'dom.dart';

/// How far is the light source from the surface of the UI.
///
/// Originally based on the constant in `flow/layers/physical_shape_layer.cc`.
const double kLightHeight = 600.0;

/// The radius of the light source. The positive radius creates a penumbra in
/// the shadow, which we express using a blur effect.
///
/// Originally based on the constant in  `flow/layers/physical_shape_layer.cc`.
const double kLightRadius = 800.0;

/// The X offset of the list source relative to the center of the shape.
///
/// This shifts the shadow along the X asix as if the light beams at an angle.
const double kLightOffsetX = -200.0;

/// The Y offset of the list source relative to the center of the shape.
///
/// This shifts the shadow along the Y asix as if the light beams at an angle.
const double kLightOffsetY = -400.0;

/// Computes the offset that moves the shadow due to the light hitting the
/// shape at an angle.
///
///     ------ light
///        \
///         \
///          \
///           \
///            \
///         --------- shape
///             |\
///             | \
///             |  \
/// ------------x---x------------
///             |<->| offset
///
/// This is not a complete physical model. For example, this does not take into
/// account the size of the shape (this function doesn't even take the shape as
/// a parameter). It's just a good enough approximation.
ui.Offset computeShadowOffset(double elevation) {
  if (elevation == 0.0) {
    return ui.Offset.zero;
  }

  final double dx = -kLightOffsetX * elevation / kLightHeight;
  final double dy = -kLightOffsetY * elevation / kLightHeight;
  return ui.Offset(dx, dy);
}

/// Computes the rectangle that contains the penumbra of the shadow cast by
/// the [shape] that's elevated above the surface of the screen at [elevation].
ui.Rect computePenumbraBounds(ui.Rect shape, double elevation) {
  if (elevation == 0.0) {
    return shape;
  }

  // tangent for x
  final double tx = (kLightRadius + shape.width * 0.5) / kLightHeight;
  // tangent for y
  final double ty = (kLightRadius + shape.height * 0.5) / kLightHeight;
  final double dx = elevation * tx;
  final double dy = elevation * ty;
  final ui.Offset offset = computeShadowOffset(elevation);
  final ui.Rect bounds = ui.Rect.fromLTRB(
    shape.left - dx,
    shape.top - dy,
    shape.right + dx,
    shape.bottom + dy,
  ).shift(offset);

  // Expand the bounds rectangle to compensate for inaccuracy in the shadow
  // calculation.  This is similar to a workaround that had previously been
  // used in the Flutter framework to adjust the bounds for shadows drawn
  // by Skia.
  // (See https://github.com/flutter/flutter/pull/127052)
  return bounds.inflate(20);
}

/// Information needed to render a shadow using CSS or canvas.
@immutable
class SurfaceShadowData {
  const SurfaceShadowData({required this.blurWidth, required this.offset});

  /// The length in pixels of the shadow.
  ///
  /// This is different from the `sigma` used by blur filters. This value
  /// contains the entire shadow, so, for example, to compute the shadow
  /// bounds it is sufficient to add this value to the width of the shape
  /// that casts it.
  final double blurWidth;

  /// The offset of the shadow relative to the shape as computed by
  /// [computeShadowOffset].
  final ui.Offset offset;
}

/// Computes the shadow for [shape] based on its [elevation] from the surface
/// of the screen.
///
/// The algorithm approximates the math done by the C++ implementation from
/// `physical_shape_layer.cc` but it's not exact, since on the Web we do not
/// (cannot) use Skia's shadow API directly. However, this algorithms is
/// consistent with [computePenumbraBounds] used by [RecordingCanvas] during
/// bounds estimation.
SurfaceShadowData? computeShadow(ui.Rect shape, double elevation) {
  if (elevation == 0.0) {
    return null;
  }

  final double penumbraTangentX = (kLightRadius + shape.width * 0.5) / kLightHeight;
  final double penumbraTangentY = (kLightRadius + shape.height * 0.5) / kLightHeight;
  final double penumbraWidth = elevation * penumbraTangentX;
  final double penumbraHeight = elevation * penumbraTangentY;
  return SurfaceShadowData(
    // There's no way to express different blur along different dimensions, so
    // we use the narrower of the two to prevent the shadow blur from being longer
    // than the shape itself, using min instead of average of penumbra values.
    blurWidth: math.min(penumbraWidth, penumbraHeight),
    offset: computeShadowOffset(elevation),
  );
}

/// Applies a CSS shadow to the [shape].
void applyCssShadow(DomElement? element, ui.Rect shape, double elevation, ui.Color color) {
  final SurfaceShadowData? shadow = computeShadow(shape, elevation);
  if (shadow == null) {
    element!.style.boxShadow = 'none';
  } else {
    color = toShadowColor(color);
    element!.style.boxShadow =
        '${shadow.offset.dx}px ${shadow.offset.dy}px '
        '${shadow.blurWidth}px 0px rgba(${color.red}, ${color.green}, ${color.blue}, ${color.alpha / 255})';
  }
}

/// Converts a shadow color specified by the framework to the color that should
/// actually be applied when rendering the shadow.
///
/// Flutter shadows look softer than the color specified by the developer. For
/// example, it is common to get a solid black for a shadow and see a very soft
/// shadow. This function softens the color by reducing its alpha by a constant
/// factor.
ui.Color toShadowColor(ui.Color color) {
  // Reduce alpha to make shadows less aggressive:
  //
  // - https://github.com/flutter/flutter/issues/52734
  // - https://github.com/flutter/gallery/issues/118
  final int reducedAlpha = (0.3 * color.alpha).round();
  return ui.Color((reducedAlpha & 0xff) << 24 | (color.value & 0x00ffffff));
}
