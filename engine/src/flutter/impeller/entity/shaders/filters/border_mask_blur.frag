// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

precision mediump float;

#include <impeller/gaussian.glsl>
#include <impeller/texture.glsl>
#include <impeller/types.glsl>

// Constant time mask blur for image borders.
//
// This mask blur extends the geometry of the source image (with clamp border
// sampling) and applies a Gaussian blur to the alpha mask at the edges.
//
// The blur itself works by mapping the Gaussian distribution's indefinite
// integral (using an erf approximation) to the 4 edges of the UV rectangle and
// multiplying them.

uniform f16sampler2D texture_sampler;

uniform FragInfo {
  float16_t src_factor;
  float16_t inner_blur_factor;
  float16_t outer_blur_factor;
  float16_t alpha;

  f16vec2 sigma_uv;
}
frag_info;

in highp vec2 v_texture_coords;

out f16vec4 frag_color;

float16_t BoxBlurMask(f16vec2 uv) {
  // LTRB
  return IPGaussianIntegral(uv.x, frag_info.sigma_uv.x) *          //
         IPGaussianIntegral(uv.y, frag_info.sigma_uv.y) *          //
         IPGaussianIntegral(1.0hf - uv.x, frag_info.sigma_uv.x) *  //
         IPGaussianIntegral(1.0hf - uv.y, frag_info.sigma_uv.y);
}

void main() {
  f16vec4 image_color = texture(texture_sampler, v_texture_coords);
  float16_t blur_factor = BoxBlurMask(f16vec2(v_texture_coords));

  float16_t within_bounds =
      float16_t(v_texture_coords.x >= 0.0 && v_texture_coords.y >= 0.0 &&
                v_texture_coords.x < 1.0 && v_texture_coords.y < 1.0);
  float16_t inner_factor =
      (frag_info.inner_blur_factor * blur_factor + frag_info.src_factor) *
      within_bounds;
  float16_t outer_factor =
      frag_info.outer_blur_factor * blur_factor * (1.0hf - within_bounds);

  float16_t mask_factor = inner_factor + outer_factor;
  frag_color = image_color * mask_factor;
}
