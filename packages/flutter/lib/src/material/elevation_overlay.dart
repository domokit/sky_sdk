// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'color_scheme.dart';
/// @docImport 'material.dart';
library;

import 'dart:math' as math;

import 'package:flutter/widgets.dart';

import 'colors.dart';
import 'theme.dart';

/// A utility class for dealing with the overlay color needed
/// to indicate elevation of surfaces.
abstract final class ElevationOverlay {
  /// Applies a surface tint color to a given container color to indicate
  /// the level of its elevation.
  ///
  /// With Material Design 3, some components will use a "surface tint" color
  /// overlay with an opacity applied to their base color to indicate they are
  /// elevated. The amount of opacity will vary with the elevation as described
  /// in: https://m3.material.io/styles/color/the-color-system/color-roles.
  ///
  /// If [surfaceTint] is not null and not completely transparent ([Color.alpha]
  /// is 0), then the returned color will be the given [color] with the
  /// [surfaceTint] of the appropriate opacity applied to it. Otherwise it will
  /// just return [color] unmodified.
  static Color applySurfaceTint(Color color, Color? surfaceTint, double elevation) {
    if (surfaceTint != null && surfaceTint != Colors.transparent) {
      return Color.alphaBlend(
        surfaceTint.withOpacity(_surfaceTintOpacityForElevation(elevation)),
        color,
      );
    }
    return color;
  }

  // Calculates the opacity of the surface tint color from the elevation by
  // looking it up in the token generated table of opacities, interpolating
  // between values as needed. If the elevation is outside the range of values
  // in the table it will clamp to the smallest or largest opacity.
  static double _surfaceTintOpacityForElevation(double elevation) {
    if (elevation < _surfaceTintElevationOpacities[0].elevation) {
      // Elevation less than the first entry, so just clamp it to the first one.
      return _surfaceTintElevationOpacities[0].opacity;
    }

    // Walk the opacity list and find the closest match(es) for the elevation.
    int index = 0;
    while (elevation >= _surfaceTintElevationOpacities[index].elevation) {
      // If we found it exactly or walked off the end of the list just return it.
      if (elevation == _surfaceTintElevationOpacities[index].elevation ||
          index + 1 == _surfaceTintElevationOpacities.length) {
        return _surfaceTintElevationOpacities[index].opacity;
      }
      index += 1;
    }

    // Interpolate between the two opacity values
    final _ElevationOpacity lower = _surfaceTintElevationOpacities[index - 1];
    final _ElevationOpacity upper = _surfaceTintElevationOpacities[index];
    final double t = (elevation - lower.elevation) / (upper.elevation - lower.elevation);
    return lower.opacity + t * (upper.opacity - lower.opacity);
  }

  /// Applies an overlay color to a surface color to indicate
  /// the level of its elevation in a dark theme.
  ///
  /// If using Material Design 3, this type of color overlay is no longer used.
  /// Instead a "surface tint" overlay is used instead. See [applySurfaceTint],
  /// [ThemeData.useMaterial3] for more information.
  ///
  /// Material drop shadows can be difficult to see in a dark theme, so the
  /// elevation of a surface should be portrayed with an "overlay" in addition
  /// to the shadow. As the elevation of the component increases, the
  /// overlay increases in opacity. This function computes and applies this
  /// overlay to a given color as needed.
  ///
  /// If the ambient theme is dark ([ThemeData.brightness] is [Brightness.dark]),
  /// and [ThemeData.applyElevationOverlayColor] is true, and the given
  /// [color] is [ColorScheme.surface] then this will return a version of
  /// the [color] with a semi-transparent [ColorScheme.onSurface] overlaid
  /// on top of it. The opacity of the overlay is computed based on the
  /// [elevation].
  ///
  /// Otherwise it will just return the [color] unmodified.
  ///
  /// See also:
  ///
  ///  * [ThemeData.applyElevationOverlayColor] which controls the whether
  ///    an overlay color will be applied to indicate elevation.
  ///  * [overlayColor] which computes the needed overlay color.
  ///  * [Material] which uses this to apply an elevation overlay to its surface.
  ///  * <https://material.io/design/color/dark-theme.html>, which specifies how
  ///    the overlay should be applied.
  static Color applyOverlay(BuildContext context, Color color, double elevation) {
    final ThemeData theme = Theme.of(context);
    if (elevation > 0.0 &&
        theme.applyElevationOverlayColor &&
        theme.brightness == Brightness.dark &&
        color.withOpacity(1.0) == theme.colorScheme.surface.withOpacity(1.0)) {
      return colorWithOverlay(color, theme.colorScheme.onSurface, elevation);
    }
    return color;
  }

  /// Computes the appropriate overlay color used to indicate elevation in
  /// dark themes.
  ///
  /// If using Material Design 3, this type of color overlay is no longer used.
  /// Instead a "surface tint" overlay is used instead. See [applySurfaceTint],
  /// [ThemeData.useMaterial3] for more information.
  ///
  /// See also:
  ///
  ///  * https://material.io/design/color/dark-theme.html#properties which
  ///    specifies the exact overlay values for a given elevation.
  static Color overlayColor(BuildContext context, double elevation) {
    final ThemeData theme = Theme.of(context);
    return _overlayColor(theme.colorScheme.onSurface, elevation);
  }

  /// Returns a color blended by laying a semi-transparent overlay (using the
  /// [overlay] color) on top of a surface (using the [surface] color).
  ///
  /// If using Material Design 3, this type of color overlay is no longer used.
  /// Instead a "surface tint" overlay is used instead. See [applySurfaceTint],
  /// [ThemeData.useMaterial3] for more information.
  ///
  /// The opacity of the overlay depends on [elevation]. As [elevation]
  /// increases, the opacity will also increase.
  ///
  /// See https://material.io/design/color/dark-theme.html#properties.
  static Color colorWithOverlay(Color surface, Color overlay, double elevation) {
    return Color.alphaBlend(_overlayColor(overlay, elevation), surface);
  }

  /// Applies an opacity to [color] based on [elevation].
  static Color _overlayColor(Color color, double elevation) {
    // Compute the opacity for the given elevation
    // This formula matches the values in the spec:
    // https://material.io/design/color/dark-theme.html#properties
    final double opacity = (4.5 * math.log(elevation + 1) + 2) / 100.0;
    return color.withOpacity(opacity);
  }
}

// A data class to hold the opacity at a given elevation.
class _ElevationOpacity {
  const _ElevationOpacity(this.elevation, this.opacity);

  final double elevation;
  final double opacity;
}

// BEGIN GENERATED TOKEN PROPERTIES - SurfaceTint

// Do not edit by hand. The code between the "BEGIN GENERATED" and
// "END GENERATED" comments are generated from data in the Material
// Design token database by the script:
//   dev/tools/gen_defaults/bin/gen_defaults.dart.

// dart format off
// Surface tint opacities based on elevations according to the
// Material Design 3 specification:
//   https://m3.material.io/styles/color/the-color-system/color-roles
// Ordered by increasing elevation.
const List<_ElevationOpacity> _surfaceTintElevationOpacities = <_ElevationOpacity>[
  _ElevationOpacity(0.0, 0.0),   // Elevation level 0
  _ElevationOpacity(1.0, 0.05),  // Elevation level 1
  _ElevationOpacity(3.0, 0.08),  // Elevation level 2
  _ElevationOpacity(6.0, 0.11),  // Elevation level 3
  _ElevationOpacity(8.0, 0.12),  // Elevation level 4
  _ElevationOpacity(12.0, 0.14), // Elevation level 5
];
// dart format on

// END GENERATED TOKEN PROPERTIES - SurfaceTint
