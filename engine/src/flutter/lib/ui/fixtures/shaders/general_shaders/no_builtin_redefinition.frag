#version 320 es

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

precision highp float;

layout(location = 0) out vec4 oColor;

layout(location = 0) uniform float a;  // should be 1.0

float saturate(float x) {
  return clamp(x, 0.0, 1.0);
}

float addA(float x) {
  return x + a;
}

vec2 pairWithA(float x) {
  return vec2(x, a);
}

vec3 composedFunction(float x) {
  return vec3(addA(x), pairWithA(x));
}

float multiParam(float x, float y, float z) {
  return x * y * z * a;
}

void main() {
  float x = saturate(addA(0.0));          // x = 0 + 1;
  vec3 v3 = composedFunction(x);          // v3 = vec3(2, 1, 1);
  x = multiParam(v3.x, v3.y, v3.z);       // x = 2 * 1 * 1 * 1;
  oColor = vec4(0.0, x / 2.0, 0.0, 1.0);  // vec4(0, 1, 0, 1);
}
