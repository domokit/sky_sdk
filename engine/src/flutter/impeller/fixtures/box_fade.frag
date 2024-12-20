// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

uniform FrameInfo {
  float current_time;
  vec2 cursor_position;
  vec2 window_size;
}
frame_info;

in vec2 interpolated_texture_coordinates;

out vec4 frag_color;

uniform sampler2D contents1;
uniform sampler2D contents2;

void main() {
  vec4 tex1 = texture(contents1, interpolated_texture_coordinates);
  vec4 tex2 = texture(contents2, interpolated_texture_coordinates);
  frag_color = mix(
      tex1, tex2,
      clamp(frame_info.cursor_position.x / frame_info.window_size.x, 0.0, 1.0));
}
