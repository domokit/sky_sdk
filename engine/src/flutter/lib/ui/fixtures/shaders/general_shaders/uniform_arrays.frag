#version 100 core

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

precision highp float;

layout(location = 0) out vec4 oColor;

layout(location = 1) uniform float floatArray[2];
layout(location = 3) uniform vec2 vec2Array[2];
layout(location = 7) uniform vec3 vec3Array[2];
layout(location = 13) uniform mat2 mat2Array[2];

void main() {
  vec4 badColor = vec4(1.0, 0, 0, 1.0);
  vec4 goodColor = vec4(0, 1.0, 0, 1.0);

  // The test populates the uniforms with strictly increasing values, so if
  // out-of-order values are read out of the uniforms, then the bad color that
  // causes the test to fail is returned.
  if (floatArray[0] >= floatArray[1] || floatArray[1] >= vec2Array[0].x ||
      vec2Array[0].x >= vec2Array[0].y || vec2Array[0].y >= vec2Array[1].x ||
      vec2Array[1].x >= vec2Array[1].y || vec2Array[1].y >= vec3Array[0].x ||
      vec3Array[0].x >= vec3Array[0].y || vec3Array[0].y >= vec3Array[0].z ||
      vec3Array[0].z >= vec3Array[1].x || vec3Array[1].x >= vec3Array[1].y ||
      vec3Array[1].y >= vec3Array[1].z ||
      vec3Array[1].z >= mat2Array[0][0][0] ||
      mat2Array[0][0][0] >= mat2Array[0][0][1] ||
      mat2Array[0][0][1] >= mat2Array[0][1][0] ||
      mat2Array[0][1][0] >= mat2Array[0][1][1] ||
      mat2Array[0][1][1] >= mat2Array[1][0][0] ||
      mat2Array[1][0][0] >= mat2Array[1][0][1] ||
      mat2Array[1][0][1] >= mat2Array[1][1][0] ||
      mat2Array[1][1][0] >= mat2Array[1][1][1]) {
    oColor = badColor;
  } else {
    oColor = goodColor;
  }
}
