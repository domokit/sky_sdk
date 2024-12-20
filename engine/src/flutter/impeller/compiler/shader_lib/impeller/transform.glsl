// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef TRANSFORM_GLSL_
#define TRANSFORM_GLSL_

/// Returns the Cartesian coordinates of `position` in `transform` space.
vec2 IPVec2TransformPosition(mat4 matrix, vec2 point) {
  vec4 transformed = matrix * vec4(point, 0, 1);
  return transformed.xy / transformed.w;
}

// Returns the transformed gl_Position for a given glyph position in a glyph
// atlas.
vec4 IPPositionForGlyphPosition(mat4 mvp,
                                vec2 unit_position,
                                vec2 destination_position,
                                vec2 destination_size) {
  mat4 translation = mat4(1, 0, 0, 0,  //
                          0, 1, 0, 0,  //
                          0, 0, 1, 0,  //
                          destination_position.xy, 0, 1);
  return mvp * translation *
         vec4(unit_position.x * destination_size.x,
              unit_position.y * destination_size.y, 0.0, 1.0);
}

#ifdef IMPELLER_TARGET_OPENGLES

// Shim matrix `inverse` for versions that lack it.
// TODO: This could be gated on GLSL < 1.4.
mat3 IPMat3Inverse(mat3 m) {
  float a00 = m[0][0], a01 = m[0][1], a02 = m[0][2];
  float a10 = m[1][0], a11 = m[1][1], a12 = m[1][2];
  float a20 = m[2][0], a21 = m[2][1], a22 = m[2][2];

  float b01 = a22 * a11 - a12 * a21;
  float b11 = -a22 * a10 + a12 * a20;
  float b21 = a21 * a10 - a11 * a20;

  float det = a00 * b01 + a01 * b11 + a02 * b21;

  return mat3(b01, (-a22 * a01 + a02 * a21), (a12 * a01 - a02 * a11), b11,
              (a22 * a00 - a02 * a20), (-a12 * a00 + a02 * a10), b21,
              (-a21 * a00 + a01 * a20), (a11 * a00 - a01 * a10)) /
         det;
}

#else  // IMPELLER_TARGET_OPENGLES

mat3 IPMat3Inverse(mat3 m) {
  return inverse(m);
}

#endif  // IMPELLER_TARGET_OPENGLES

#endif  // TRANSFORM_GLSL_
