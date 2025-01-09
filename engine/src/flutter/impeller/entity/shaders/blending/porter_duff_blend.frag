// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

precision mediump float;

#include <impeller/blending.glsl>
#include <impeller/color.glsl>
#include <impeller/texture.glsl>
#include <impeller/types.glsl>

// see GetPorterDuffSpecConstants in content_context.cc for actual constants
layout(constant_id = 0) const float supports_decal = 1.0;
layout(constant_id = 1) const float src_coeff = 1.0;
layout(constant_id = 2) const float src_coeff_dst_alpha = 1.0;
layout(constant_id = 3) const float dst_coeff = 1.0;
layout(constant_id = 4) const float dst_coeff_src_alpha = 1.0;
layout(constant_id = 5) const float dst_coeff_src_color = 1.0;

uniform f16sampler2D texture_sampler_dst;

uniform FragInfo {
  float16_t input_alpha;
  float16_t output_alpha;
  float tmx;
  float tmy;
}
frag_info;

in vec2 v_texture_coords;
in f16vec4 v_color;

out vec4 frag_color;

f16vec4 Sample(f16sampler2D texture_sampler,
               vec2 texture_coords,
               float tmx,
               float tmy) {
  if (supports_decal > 0.0) {
    return texture(texture_sampler, texture_coords);
  }
  return IPHalfSampleWithTileMode(texture_sampler, texture_coords, tmx, tmy);
}

void main() {
  f16vec4 dst = Sample(texture_sampler_dst, v_texture_coords, frag_info.tmx,
                       frag_info.tmy) *
                frag_info.input_alpha;
  f16vec4 src = v_color;
  frag_color = f16vec4(src * (src_coeff + dst.a * src_coeff_dst_alpha) +
                       dst * (dst_coeff + src.a * dst_coeff_src_alpha +
                              src * dst_coeff_src_color));
  frag_color *= frag_info.output_alpha;
}
