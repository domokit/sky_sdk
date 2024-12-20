// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

in vec4 v_color;
in vec4 v_color2;

out vec4 frag_color;

uniform FragInfo {
  float time;
}
frag_info;

void main() {
  float floor = floor(frag_info.time);
  float fract = frag_info.time - floor;
  if (mod(int(floor), 2) == 0) {
    fract = 1.0 - fract;
  }
  frag_color = mix(v_color, v_color2, fract);
}
