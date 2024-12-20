// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <flutter/runtime_effect.glsl>

uniform sampler2D u_texture;

out vec4 frag_color;

void main() {
  frag_color = texture(u_texture, FlutterFragCoord().xy / vec2(100)).bgra;
}
