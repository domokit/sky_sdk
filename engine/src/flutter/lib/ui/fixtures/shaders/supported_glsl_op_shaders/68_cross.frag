#version 320 es

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

precision highp float;

layout(location = 0) out vec4 fragColor;

layout(location = 0) uniform float a;

void main() {
  fragColor = vec4(
      /* cross product of parallel vectors is a zero vector */
      cross(vec3(a, 2.0, 3.0), vec3(2.0, 4.0, 6.0))[0], 1.0,
      // cross product of parallel vectors is a zero vector
      cross(vec3(a, 2.0, 3.0), vec3(2.0, 4.0, 6.0))[2], 1.0);
}
