// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <impeller/blending.glsl>
#include <impeller/color.glsl>
#include <impeller/texture.glsl>
#include <impeller/types.glsl>
#include "blend_select.glsl"

layout(constant_id = 0) const float blend_type = 0.0;
layout(constant_id = 1) const float supports_decal = 1.0;

uniform BlendInfo {
  float16_t dst_input_alpha;
  float16_t src_input_alpha;
  float16_t color_factor;
  f16vec4 color;  // This color input is expected to be unpremultiplied.
  float supports_decal_sampler_address_mode;
}
blend_info;

uniform f16sampler2D texture_sampler_dst;
uniform f16sampler2D texture_sampler_src;

in vec2 v_dst_texture_coords;
in vec2 v_src_texture_coords;

out f16vec4 frag_color;

f16vec4 Sample(f16sampler2D texture_sampler, vec2 texture_coords) {
  if (supports_decal > 0.0) {
    return texture(texture_sampler, texture_coords);
  }
  return IPHalfSampleDecal(texture_sampler, texture_coords);
}

void main() {
  f16vec4 dst =
      IPHalfUnpremultiply(Sample(texture_sampler_dst,  // sampler
                                 v_dst_texture_coords  // texture coordinates
                                 ));
  dst.a *= blend_info.dst_input_alpha;
  f16vec4 src = blend_info.color_factor > 0.0hf
                    ? blend_info.color
                    : IPHalfUnpremultiply(Sample(
                          texture_sampler_src,  // sampler
                          v_src_texture_coords  // texture coordinates
                          ));
  if (blend_info.color_factor == 0.0hf) {
    src.a *= blend_info.src_input_alpha;
  }

  f16vec3 blend_result = AdvancedBlend(dst.rgb, src.rgb, int(blend_type));

  frag_color = IPApplyBlendedColor(dst, src, blend_result);
}
