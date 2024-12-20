#version 320 es

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

precision highp float;

layout(location = 0) out vec4 fragColor;

layout(location = 0) uniform float a;

void main() {
  float one = dot(vec2(1.0), vec2(0.0, a));
  fragColor = vec4(0.0, one, 0.0, 1.0);
}
