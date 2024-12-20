#version 320 es

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

precision highp float;

layout(location = 0) out vec4 fragColor;

layout(location = 0) uniform float a;

void main() {
  float sum = 0.0;
  for (float i = 0.0; i < 6.0; i++) {
    if (i > a * 5.0) {
      break;
    }
    if (i < 1.0) {
      continue;
    }
    if (a > 0.0) {
      sum += a * 0.25;
    }
  }
  fragColor = vec4(0.0, sum, 0.0, 1.0);
}
