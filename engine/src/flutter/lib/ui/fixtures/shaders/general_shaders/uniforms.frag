#version 320 es

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

precision highp float;

layout(location = 0) out vec4 oColor;

layout(location = 0) uniform float iFloatUniform;
layout(location = 1) uniform vec2 iVec2Uniform;
layout(location = 2) uniform mat2 iMat2Uniform;

void main() {
  oColor = vec4(iFloatUniform, iVec2Uniform, iMat2Uniform[1][1]);
}
