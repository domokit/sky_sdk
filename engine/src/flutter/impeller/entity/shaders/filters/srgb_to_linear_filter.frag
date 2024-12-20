// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

precision mediump float;

#include <impeller/color.glsl>

// Creates a color filter that applies the inverse of the sRGB gamma curve
// to the RGB channels.

#include <impeller/types.glsl>

uniform f16sampler2D input_texture;

uniform FragInfo {
  float16_t input_alpha;
}
frag_info;

in highp vec2 v_texture_coords;
out f16vec4 frag_color;

void main() {
  f16vec4 input_color =
      texture(input_texture, v_texture_coords) * frag_info.input_alpha;

  f16vec4 color = IPHalfUnpremultiply(input_color);
  for (int i = 0; i < 3; i++) {
    if (color[i] <= 0.04045hf) {
      color[i] = color[i] / 12.92hf;
    } else {
      color[i] = pow((color[i] + 0.055hf) / 1.055hf, 2.4hf);
    }
  }

  frag_color = IPHalfPremultiply(color);
}
