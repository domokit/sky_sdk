#version 320 es

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

precision highp float;

layout(location = 0) out vec4 fragColor;

layout(location = 0) uniform float a;

void main() {
  fragColor = vec4(
      // length of a zero vector is 0.0
      length(vec3(a - 1.0, 0.0, 0.0)),
      // sqrt(3.0^2.0 + 4.0^2.0) - 4.0 = 5.0 - 4.0 = 1.0
      length(vec2(a * 3.0, 4.0)) - 4.0, 0.0,
      // sqrt(4.0^2.0 + (-4.0)^2.0 + (-4.0)^2.0) + 4.0^2.0) - 7.0 = sqrt(16.0
      // + 16.0 + 16.0 + 16.0) - 7.0 = sqrt(64.0) - 7.0 = 8.0 - 7.0 = 1.0
      length(vec4(a * 4.0, -4.0, -4.0, 4.0)) - 7.0);
}
