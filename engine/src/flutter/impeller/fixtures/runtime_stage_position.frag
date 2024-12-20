// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <flutter/runtime_effect.glsl>

uniform vec4 ltrb;

out vec4 frag_color;

// Output solid red if frag position is within LTRB rectangle.
void main() {
  if (FlutterFragCoord().x >= ltrb.x && FlutterFragCoord().x <= ltrb.z &&
      FlutterFragCoord().y >= ltrb.y && FlutterFragCoord().y <= ltrb.w) {
    frag_color = vec4(1.0, 0.0, 0.0, 1.0);
  } else {
    frag_color = vec4(0.0);
  }
}
