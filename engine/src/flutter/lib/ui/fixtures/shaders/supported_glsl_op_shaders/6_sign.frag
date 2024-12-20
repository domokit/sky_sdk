#version 320 es

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

precision highp float;

layout(location = 0) out vec4 fragColor;

layout(location = 0) uniform float a;

void main() {
  fragColor = vec4(
      // sign is negative which results to -1.0, and -1.0 + 1.0 is 0.0
      sign(-72.45) + a,
      // sign is negative which results to -1.0, and -1.0 + 2.0 is 1.0
      sign(-12.34) + 2.0 * a, 0.0,
      // sign is positive which results to 1.0
      sign(a * 0.1234));
}
