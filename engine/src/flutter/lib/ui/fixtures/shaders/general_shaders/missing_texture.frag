// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <flutter/runtime_effect.glsl>

uniform vec2 u_size;

out vec4 frag_color;

void main() {
  frag_color = vec4(u_size.x, u_size.y, 0, 1);
}
