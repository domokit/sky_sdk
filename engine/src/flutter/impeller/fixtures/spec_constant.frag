// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

in vec4 v_color;
in vec4 v_color2;

layout(constant_id = 0) const float some_fraction = 1.0;

out vec4 frag_color;

uniform FragInfo {
  float time;
}
frag_info;

void main() {
  frag_color = mix(v_color, v_color2, some_fraction);
}
