// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

precision mediump float;

#include <impeller/external_texture_oes.glsl>

uniform sampler2D SAMPLER_EXTERNAL_OES_texture_sampler;

uniform FragInfo {
  float x_tile_mode;
  float y_tile_mode;
  float alpha;
}
frag_info;

in highp vec2 v_texture_coords;

out vec4 frag_color;

void main() {
  frag_color =
      IPSampleWithTileModeOES(SAMPLER_EXTERNAL_OES_texture_sampler,  // sampler
                              v_texture_coords,       // texture coordinates
                              frag_info.x_tile_mode,  // x tile mode
                              frag_info.y_tile_mode   // y tile mode
                              ) *
      frag_info.alpha;
}
