// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <impeller/types.glsl>

uniform FrameInfo {
  mat4 mvp;
}
frame_info;

in vec2 position;
// Note: The GLES backend uses name matching for attribute locations. This name
// must match the name of the attribute input in:
// impeller/compiler/shader_lib/flutter/runtime_effect.glsl
out vec2 _fragCoord;

void main() {
  gl_Position = frame_info.mvp * vec4(position, 0.0, 1.0);
  _fragCoord = position;
}
