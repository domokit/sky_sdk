// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

precision mediump float;

#include <impeller/color.glsl>
#include <impeller/texture.glsl>
#include <impeller/types.glsl>

// A color filter that transforms colors through a 4x5 color matrix.
//
// This filter can be used to change the saturation of pixels, convert from YUV
// to RGB, etc.
//
// 4x5 matrix for transforming the color and alpha components of a Bitmap.
// The matrix can be passed as single array, and is treated as follows:
//
//   [ a, b, c, d, e,
//     f, g, h, i, j,
//     k, l, m, n, o,
//     p, q, r, s, t ]
//
// When applied to a color [R, G, B, A], the resulting color is computed as:
//
//    R’ = a*R + b*G + c*B + d*A + e;
//    G’ = f*R + g*G + h*B + i*A + j;
//    B’ = k*R + l*G + m*B + n*A + o;
//    A’ = p*R + q*G + r*B + s*A + t;
//
// That resulting color [R’, G’, B’, A’] then has each channel clamped to the 0
// to 255 range.

uniform FragInfo {
  mat4 color_m;
  f16vec4 color_v;
  float16_t input_alpha;
}
frag_info;

uniform f16sampler2D input_texture;

in highp vec2 v_texture_coords;
out f16vec4 frag_color;

void main() {
  f16vec4 input_color =
      texture(input_texture, v_texture_coords) * frag_info.input_alpha;

  // unpremultiply first, as filter inputs are premultiplied.
  f16vec4 color = IPHalfUnpremultiply(input_color);

  color = clamp(f16mat4(frag_info.color_m) * color + frag_info.color_v,
                float16_t(0), float16_t(1.0));

  // premultiply the outputs
  frag_color = f16vec4(color.rgb * color.a, color.a);
}
