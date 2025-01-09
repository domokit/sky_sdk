// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('applySurfaceTint with null surface tint returns given color', () {
    final Color result = ElevationOverlay.applySurfaceTint(const Color(0xff888888), null, 42.0);

    expect(result, equals(const Color(0xFF888888)));
  });

  test('applySurfaceTint with exact elevation levels uses the right opacity overlay', () {
    const Color baseColor = Color(0xff888888);
    const Color surfaceTintColor = Color(0xff44CCFF);

    Color overlayWithOpacity(double opacity) {
      return Color.alphaBlend(surfaceTintColor.withOpacity(opacity), baseColor);
    }

    // Based on values from the spec:
    //   https://m3.material.io/styles/color/the-color-system/color-roles

    // Elevation level 0 (0.0) - should have opacity 0.0.
    expect(
      ElevationOverlay.applySurfaceTint(baseColor, surfaceTintColor, 0.0),
      equals(overlayWithOpacity(0.0)),
    );

    // Elevation level 1 (1.0) - should have opacity 0.05.
    expect(
      ElevationOverlay.applySurfaceTint(baseColor, surfaceTintColor, 1.0),
      equals(overlayWithOpacity(0.05)),
    );

    // Elevation level 2 (3.0) - should have opacity 0.08.
    expect(
      ElevationOverlay.applySurfaceTint(baseColor, surfaceTintColor, 3.0),
      equals(overlayWithOpacity(0.08)),
    );

    // Elevation level 3 (6.0) - should have opacity 0.11`.
    expect(
      ElevationOverlay.applySurfaceTint(baseColor, surfaceTintColor, 6.0),
      equals(overlayWithOpacity(0.11)),
    );

    // Elevation level 4 (8.0) - should have opacity 0.12.
    expect(
      ElevationOverlay.applySurfaceTint(baseColor, surfaceTintColor, 8.0),
      equals(overlayWithOpacity(0.12)),
    );

    // Elevation level 5 (12.0) - should have opacity 0.14.
    expect(
      ElevationOverlay.applySurfaceTint(baseColor, surfaceTintColor, 12.0),
      equals(overlayWithOpacity(0.14)),
    );
  });

  test('applySurfaceTint with elevation lower than level 0 should have no overlay', () {
    const Color baseColor = Color(0xff888888);
    const Color surfaceTintColor = Color(0xff44CCFF);

    Color overlayWithOpacity(double opacity) {
      return Color.alphaBlend(surfaceTintColor.withOpacity(opacity), baseColor);
    }

    expect(
      ElevationOverlay.applySurfaceTint(baseColor, surfaceTintColor, -42.0),
      equals(overlayWithOpacity(0.0)),
    );
  });

  test('applySurfaceTint with elevation higher than level 5 should have no level 5 overlay', () {
    const Color baseColor = Color(0xff888888);
    const Color surfaceTintColor = Color(0xff44CCFF);

    Color overlayWithOpacity(double opacity) {
      return Color.alphaBlend(surfaceTintColor.withOpacity(opacity), baseColor);
    }

    // Elevation level 5 (12.0) - should have opacity 0.14.
    expect(
      ElevationOverlay.applySurfaceTint(baseColor, surfaceTintColor, 42.0),
      equals(overlayWithOpacity(0.14)),
    );
  });

  test('applySurfaceTint with elevation between two levels should interpolate the opacity', () {
    const Color baseColor = Color(0xff888888);
    const Color surfaceTintColor = Color(0xff44CCFF);

    Color overlayWithOpacity(double opacity) {
      return Color.alphaBlend(surfaceTintColor.withOpacity(opacity), baseColor);
    }

    // Elevation between level 4 (8.0) and level 5 (12.0) should be interpolated
    // between the opacities 0.12 and 0.14.

    // One third (0.3): (elevation 9.2) -> (opacity 0.126)
    expect(
      ElevationOverlay.applySurfaceTint(baseColor, surfaceTintColor, 9.2),
      equals(overlayWithOpacity(0.126)),
    );

    // Half way (0.5): (elevation 10.0) -> (opacity 0.13)
    expect(
      ElevationOverlay.applySurfaceTint(baseColor, surfaceTintColor, 10.0),
      equals(overlayWithOpacity(0.13)),
    );

    // Two thirds (0.6): (elevation 10.4) -> (opacity 0.132)
    expect(
      ElevationOverlay.applySurfaceTint(baseColor, surfaceTintColor, 10.4),
      equals(overlayWithOpacity(0.132)),
    );
  });
}
