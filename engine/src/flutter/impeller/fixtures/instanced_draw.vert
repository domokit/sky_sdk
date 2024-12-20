// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifdef IMPELLER_TARGET_OPENGLES

void main() {
  // Instancing is not supported on legacy targets and test will be disabled.
}

#else  // IMPELLER_TARGET_OPENGLES

uniform FrameInfo {
  mat4 mvp;
}
frame_info;

readonly buffer InstanceInfo {
  vec4 colors[];
}
instance_info;

in vec2 vtx;

out vec4 v_color;

void main() {
  gl_Position =
      frame_info.mvp * vec4(vtx.x + 105.0 * gl_InstanceIndex,
                            vtx.y + 105.0 * gl_InstanceIndex, 0.0, 1.0);
  v_color = instance_info.colors[gl_InstanceIndex];
}

#endif  // IMPELLER_TARGET_OPENGLES
