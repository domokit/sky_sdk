// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// Declare samplers in different order than usage.
uniform sampler2D textureA;
uniform sampler2D textureB;

out vec4 frag_color;

void main() {
  vec4 sample_1 = texture(textureB, vec2(1.0));
  vec4 sample_2 = texture(textureA, vec2(1.0));
  frag_color = sample_1 + sample_2;
}
