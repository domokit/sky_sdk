// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

precision mediump float;

#include <impeller/color.glsl>
#include <impeller/dithering.glsl>
#include <impeller/gradient.glsl>
#include <impeller/texture.glsl>
#include <impeller/types.glsl>

struct ColorPoint {
  vec4 color;
  float stop;
};

layout(std140) readonly buffer ColorData {
  ColorPoint colors[];
}
color_data;

uniform FragInfo {
  highp vec2 center;
  float radius;
  float tile_mode;
  vec4 decal_border_color;
  float alpha;
  int colors_length;
  vec2 focus;
  float focus_radius;
}
frag_info;

highp in vec2 v_position;

out vec4 frag_color;

void main() {
  vec2 res = IPComputeConicalT(frag_info.focus, frag_info.focus_radius,
                               frag_info.center, frag_info.radius, v_position);

  float t = res.x;
  vec4 result_color = vec4(0);
  if (res.y < 0.0 ||
      ((t < 0.0 || t > 1.0) && frag_info.tile_mode == kTileModeDecal)) {
    result_color = frag_info.decal_border_color;
  } else {
    t = IPFloatTile(t, frag_info.tile_mode);

    for (int i = 1; i < frag_info.colors_length; i++) {
      ColorPoint prev_point = color_data.colors[i - 1];
      ColorPoint current_point = color_data.colors[i];
      if (t >= prev_point.stop && t <= current_point.stop) {
        float delta = (current_point.stop - prev_point.stop);
        if (delta < 0.001) {
          result_color = current_point.color;
        } else {
          float ratio = (t - prev_point.stop) / delta;
          result_color = mix(prev_point.color, current_point.color, ratio);
        }
        break;
      }
    }
  }

  frag_color = IPPremultiply(result_color) * frag_info.alpha;
  frag_color = IPOrderedDither8x8(frag_color, gl_FragCoord.xy);
}
