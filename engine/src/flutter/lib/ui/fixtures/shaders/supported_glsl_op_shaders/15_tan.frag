#version 320 es

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

precision highp float;

layout(location = 0) out vec4 fragColor;

layout(location = 0) uniform float a;

void main() {
  fragColor = vec4(
      // tan(0.0) = 0.0
      tan(0.0),
      // tan(1.57079632679) = tan(pi / 4.0) = 1.0
      tan(a * 0.785398163), 0.0, 1.0);
}
