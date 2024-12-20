#version 320 es

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

precision highp float;

layout(location = 0) out vec4 fragColor;

layout(location = 0) uniform float a;

void main() {
  vec2 ones = vec2(a);
  vec3 zeros = vec3(0.0, 0.0, 0.0);
  fragColor = vec4(zeros[0], ones[0], zeros[2], ones[1]);
}
