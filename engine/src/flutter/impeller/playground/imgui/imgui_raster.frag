// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

in vec2 frag_texture_coordinates;
in vec4 frag_vertex_color;

out vec4 frag_color;

uniform sampler2D tex;

void main() {
  frag_color = frag_vertex_color * texture(tex, frag_texture_coordinates.st);
}
