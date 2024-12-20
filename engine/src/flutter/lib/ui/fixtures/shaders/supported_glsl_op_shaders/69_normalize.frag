#version 320 es

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

precision highp float;

layout(location = 0) out vec4 fragColor;

layout(location = 0) uniform float a;

void main() {
  fragColor =
      vec4(0.0,
           // normalized result is x = [3/5, 4/5], so add 2/5 to x1 to make 1.0
           normalize(vec2(a * 3.0, 4.0))[0] + 0.4, 0.0,
           // normalized result is x = [3/5, 4/5], so add 1/5 to x2 to make 1.0
           normalize(vec2(a * 3.0, 4.0))[1] + 0.2);
}
