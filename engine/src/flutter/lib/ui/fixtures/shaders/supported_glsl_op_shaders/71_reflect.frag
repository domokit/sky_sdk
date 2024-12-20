#version 320 es

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

precision highp float;

layout(location = 0) out vec4 fragColor;

layout(location = 0) uniform float a;

// For a given incident vector I and surface normal N reflect returns the
// reflection direction calculated as I - 2.0 * dot(N, I) * N.
void main() {
  // To get [0.0, 1.0] as the output, choose [0.6, 0.8] as N, and solve for I.
  // Since the reflection is symmetric:
  // I’ = reflect(I)
  // I’ = I - 2 dot(N, I) N
  // I = I’ - 2 dot(N, I’) N
  // N = [0.6, 0.8]
  // I’ = [0, 1]
  // I = [0, 1] - 2 * 0.8 [0.6, 0.8]
  // I = [-0.96, -0.28]
  fragColor =
      vec4(reflect(vec2(a * -0.96, -0.28), vec2(0.6, 0.8))[0],
           reflect(vec2(a * -0.96, -0.28), vec2(0.6, 0.8))[1], 0.0, 1.0);
}
