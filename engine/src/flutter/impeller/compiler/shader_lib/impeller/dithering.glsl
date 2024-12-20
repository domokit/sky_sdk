// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef DITHERING_GLSL_
#define DITHERING_GLSL_

#include <impeller/types.glsl>

/// The dithering rate, which is 1.0 / 64.0, or 0.015625.
const float kDitherRate = 1.0 / 64.0;

/// Returns the closest color to the input color using 8x8 ordered dithering.
///
/// Ordered dithering divides the output into a grid of cells, and then assigns
/// a different threshold to each cell. The threshold is used to determine if
/// the color should be rounded up (white) or down (black).
//
/// This technique was chosen mostly because Skia also uses it:
/// https://github.com/google/skia/blob/f9de059517a6f58951510fc7af0cba21e13dd1a8/src/opts/SkRasterPipeline_opts.h#L1717
///
/// See also:
/// - https://en.wikipedia.org/wiki/Ordered_dithering
/// - https://surma.dev/things/ditherpunk/
/// - https://shader-tutorial.dev/advanced/color-banding-dithering/
vec4 IPOrderedDither8x8(vec4 color, vec2 dest) {
  // Get the x and y coordinates of the pixel in the 8x8 grid.
  uint x = uint(dest.x) % 8;
  uint y = uint(dest.y);
  y ^= x;

  // Get the dither value from the matrix.
  uint m = (y & 1) << 5 |  //
           (x & 1) << 4 |  //
           (y & 2) << 2 |  //
           (x & 2) << 1 |  //
           (y & 4) >> 1 |  //
           (x & 4) >> 2;   //

  // Scale that dither to [0,1), then (-0.5,+0.5), here using 63/128 = 0.4921875
  // as 0.5-epsilon. We want to make sure our dither is less than 0.5 in either
  // direction to keep exact values like 0 and 1 unchanged after rounding.
  float dither = float(m) * (2.0 / 128.0) - (63.0 / 128.0);

  // Apply the dither to the color.
  color.rgb += dither * kDitherRate;

  return color;
}

#endif
