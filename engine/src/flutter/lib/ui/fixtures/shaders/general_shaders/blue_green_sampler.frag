#version 320 es

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

precision highp float;

layout(location = 0) out vec4 oColor;

layout(location = 0) uniform sampler2D iChild;

void main() {
  // iChild1 is an image that is half blue, half green,
  // so oColor should be set to vec2(0, 1, 0, 1)
  oColor = texture(iChild, vec2(1, 0));
  oColor.a = 1.0;
}
