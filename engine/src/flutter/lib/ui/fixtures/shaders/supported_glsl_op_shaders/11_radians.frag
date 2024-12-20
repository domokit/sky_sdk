#version 320 es

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

precision highp float;

layout(location = 0) out vec4 fragColor;

layout(location = 0) uniform float a;

void main() {
  fragColor = vec4(0.0,
                   // 57.2957795131 * pi / 180.0 = 1.0
                   float(radians(a * 57.2957795131)), 0.0, 1.0);
}
