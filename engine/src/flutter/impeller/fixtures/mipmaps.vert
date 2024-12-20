// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

uniform FrameInfo {
  mat4 mvp;
}
frame_info;

in vec2 vertex_position;
in vec2 uv;

out vec2 v_uv;

void main() {
  gl_Position = frame_info.mvp * vec4(vertex_position, 0.0, 1.0);
  v_uv = uv;
}
