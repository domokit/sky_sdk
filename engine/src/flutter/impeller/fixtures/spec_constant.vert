// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

in vec2 position;
in vec4 color;
in vec4 color2;

out vec4 v_color;
out vec4 v_color2;

void main() {
  gl_Position = vec4(position, 0.0, 1.0);
  v_color = color;
  v_color2 = color2;
}
