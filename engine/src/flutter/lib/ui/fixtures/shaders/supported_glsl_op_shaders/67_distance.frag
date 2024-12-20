#version 320 es

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

precision highp float;

layout(location = 0) out vec4 fragColor;

layout(location = 0) uniform float a;

void main() {
  fragColor = vec4(
      // distance between same value is 0.0
      distance(a * 7.0, 7.0),
      // 7.0 - 6.0 = 1.0
      distance(a * 7.0, 6.0), 0.0,
      // sqrt(7.0 * 7.0 - 6.0 * 8.0) = sqrt(49.0 - 48.0) = sqrt(1.0) = 1.0
      distance(vec2(a * 7.0, 6.0), vec2(7.0, 8.0)));
}
