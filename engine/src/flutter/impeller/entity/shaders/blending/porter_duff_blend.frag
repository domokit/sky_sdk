// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

precision mediump float;

#include <impeller/blending.glsl>
#include <impeller/color.glsl>
#include <impeller/texture.glsl>
#include <impeller/types.glsl>

layout(constant_id = 0) const float supports_decal = 1.0;

uniform f16sampler2D texture_sampler_dst;

uniform FragInfo {
  float16_t src_coeff;
  float16_t src_coeff_dst_alpha;
  float16_t dst_coeff;
  float16_t dst_coeff_src_alpha;
  float16_t dst_coeff_src_color;
  float16_t input_alpha;
  float16_t output_alpha;
  float tmx;
  float tmy;
}
frag_info;

in vec2 v_texture_coords;
in f16vec4 v_color;

out f16vec4 frag_color;

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
  frag_color =
      src * (frag_info.src_coeff + dst.a * frag_info.src_coeff_dst_alpha) +
      dst * (frag_info.dst_coeff + src.a * frag_info.dst_coeff_src_alpha +
             src * frag_info.dst_coeff_src_color);
  frag_color *= frag_info.output_alpha;
}
