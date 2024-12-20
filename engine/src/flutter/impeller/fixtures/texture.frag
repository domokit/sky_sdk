// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

in vec2 interpolated_texture_coordinates;

out vec4 frag_color;

uniform sampler2D texture_contents;

void main() {
  frag_color = texture(texture_contents, interpolated_texture_coordinates);
}
