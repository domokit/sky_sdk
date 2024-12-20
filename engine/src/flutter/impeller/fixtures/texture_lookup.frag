// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

uniform sampler2D textureA;

out vec4 frag_color;

void main() {
  frag_color = texture(textureA, vec2(1.0) + gl_FragCoord.xy);
}
