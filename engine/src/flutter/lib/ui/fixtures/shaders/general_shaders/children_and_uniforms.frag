#version 320 es

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

precision highp float;

layout(location = 0) out vec4 color;

layout(location = 0) uniform sampler2D child1;
layout(location = 1) uniform float a;
layout(location = 2) uniform sampler2D child2;
layout(location = 3) uniform float b;

void main() {
  // child1 is a 10x10 image where the left half is blue and the right
  // half is green, and b should be 1, so c1 should be vec4(0, 1, 0, 1)
  vec4 c1 = texture(child1, vec2(b, 0));

  // child2 only contains vec4(0, 1, 0, 1).
  vec4 c2 = texture(child2, vec2(0));

  color = c1 * c2;
}
