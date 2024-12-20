// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

uniform FragInfo {
  float lod;
}
frag_info;

uniform sampler2D tex;

in vec2 v_uv;

out vec4 frag_color;

void main() {
  frag_color = textureLod(tex, v_uv, frag_info.lod);
}
