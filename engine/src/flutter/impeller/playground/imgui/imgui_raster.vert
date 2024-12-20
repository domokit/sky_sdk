// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

uniform UniformBuffer {
  mat4 mvp;
}
uniform_buffer;

in vec2 vertex_position;
in vec2 texture_coordinates;
in vec4 vertex_color;

out vec2 frag_texture_coordinates;
out vec4 frag_vertex_color;

void main() {
  gl_Position = uniform_buffer.mvp * vec4(vertex_position.xy, 0.0, 1.0);
  frag_texture_coordinates = texture_coordinates;
  frag_vertex_color = vertex_color;
}
