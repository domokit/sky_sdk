// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef CONVERSIONS_GLSL_
#define CONVERSIONS_GLSL_

vec2 IPRemapCoords(vec2 coords, float y_coord_scale) {
  if (y_coord_scale < 0.0) {
    coords.y = 1.0 - coords.y;
  }
  return coords;
}

#endif
