// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

uniform VertInfo {  // 128 bytes (alignment = NPOT(largest member))
  mat4 mvp;         // offset 0 bytes, size 64 bytes
  vec4 color;       // offset 64 bytes, size 16 bytes
}
vert_info;

in vec2 position;
out vec4 v_color;

void main() {
  v_color = vert_info.color;
  gl_Position = vert_info.mvp * vec4(position, 0.0, 1.0);
}
