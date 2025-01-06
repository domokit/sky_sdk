// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <impeller/conversions.glsl>
#include <impeller/types.glsl>

uniform FrameInfo {
  mat4 mvp;
  float texture_sampler_y_coord_scale;
}
frame_info;

in vec2 position;
in vec2 texture_coords;

out highp vec2 v_texture_coords;

void main() {
  gl_Position = frame_info.mvp * vec4(position, 0.0, 1.0);
  v_texture_coords =
      IPRemapCoords(texture_coords, frame_info.texture_sampler_y_coord_scale);
}
