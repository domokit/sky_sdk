// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

uniform FragInfo {
  vec4 unused_color;
  vec4 color;
}
frag_info;

in vec2 v_position;

out vec4 frag_color;

float SphereDistance(vec2 position, float radius) {
  return length(v_position - position) - radius;
}

void main() {
  frag_color = frag_info.color;
}
