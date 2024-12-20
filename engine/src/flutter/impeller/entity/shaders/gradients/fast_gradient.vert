// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <impeller/transform.glsl>
#include <impeller/types.glsl>

uniform FrameInfo {
  mat4 mvp;
}
frame_info;

in vec2 position;
in mediump vec4 color;

// The geometry of the fast gradient draws is designed so that the
// varying unit will perform the correct color interpolation.
out mediump vec4 v_color;

void main() {
  gl_Position = frame_info.mvp * vec4(position, 0.0, 1.0);
  v_color = color;
}
