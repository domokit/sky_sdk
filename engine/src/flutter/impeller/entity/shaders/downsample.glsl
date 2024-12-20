// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

precision highp float;

#include <impeller/constants.glsl>
#include <impeller/types.glsl>

uniform f16sampler2D texture_sampler;

uniform FragInfo {
  float edge;
  float ratio;
  vec2 pixel_size;
}
frag_info;

in highp vec2 v_texture_coords;

out vec4 frag_color;

vec4 Sample(vec2 uv) {
#ifdef SUPPORTS_DECAL
  return texture(texture_sampler, uv, float16_t(kDefaultMipBias));
#else
  if ((uv.x < 0 || uv.y < 0 || uv.x > 1 || uv.y > 1)) {
    return vec4(0);
  } else {
    return texture(texture_sampler, uv, float16_t(kDefaultMipBias));
  }
#endif
}

void main() {
  vec4 total = vec4(0.0);
  for (float i = -frag_info.edge; i <= frag_info.edge; i += 2) {
    for (float j = -frag_info.edge; j <= frag_info.edge; j += 2) {
      total += (Sample(v_texture_coords + frag_info.pixel_size * vec2(i, j)) *
                frag_info.ratio);
    }
  }
  frag_color = total;
}
