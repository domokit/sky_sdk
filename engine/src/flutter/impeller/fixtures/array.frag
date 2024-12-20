// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

uniform FragInfo {
  vec2 circle_positions[4];
  vec4 colors[4];
}
frag_info;

in vec2 v_position;

out vec4 frag_color;

float SphereDistance(vec2 position, float radius) {
  return length(v_position - position) - radius;
}

void main() {
  for (int i = 0; i < 4; i++) {
    if (SphereDistance(frag_info.circle_positions[i].xy, 20) <= 0) {
      frag_color = frag_info.colors[i];
      return;
    }
  }
  frag_color = vec4(0);
}
