// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
#include <impeller/transform.glsl>
#include <impeller/types.glsl>

uniform FrameInfo {
  mat4 mvp;
  float should_round;
  vec2 size;
}
frame_info;

in vec2 uv;
in vec2 position;

out vec2 v_uv;

vec4 impRound(vec4 value) {
  return floor(value + vec4(0.5));
}

void main() {
  gl_Position = frame_info.mvp * vec4(position, 0, 1);
  if (frame_info.should_round > 0) {
    vec4 size4 = vec4(frame_info.size.x, frame_info.size.y, 1, 1);
    gl_Position = impRound(gl_Position * size4) / size4;
  }
  v_uv = uv;
}
