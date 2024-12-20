// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

uniform UniformBuffer {
  mat4 mvp;
}
uniform_buffer;

in vec3 vertex_position;
in vec2 texture_coordinates;

out vec2 interpolated_texture_coordinates;

void main() {
  gl_Position = uniform_buffer.mvp * vec4(vertex_position, 1.0);
  interpolated_texture_coordinates = texture_coordinates;
}
