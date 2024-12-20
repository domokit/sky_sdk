// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef COLOR_GLSL_
#define COLOR_GLSL_

#include <impeller/branching.glsl>
#include <impeller/types.glsl>

/// Convert a premultiplied color (a color which has its color components
/// multiplied with its alpha value) to an unpremultiplied color.
///
/// Returns (0, 0, 0, 0) if the alpha component is 0.
vec4 IPUnpremultiply(vec4 color) {
  if (color.a == 0) {
    return vec4(0);
  }
  return vec4(color.rgb / color.a, color.a);
}

/// Convert a premultiplied color (a color which has its color components
/// multiplied with its alpha value) to an unpremultiplied color.
///
/// Returns (0, 0, 0, 0) if the alpha component is 0.
f16vec4 IPHalfUnpremultiply(f16vec4 color) {
  if (color.a == 0.0hf) {
    return f16vec4(0.0hf);
  }
  return f16vec4(color.rgb / color.a, color.a);
}

/// Convert an unpremultiplied color (a color which has its color components
/// separated from its alpha value) to a premultiplied color.
///
/// Returns (0, 0, 0, 0) if the alpha component is 0.
vec4 IPPremultiply(vec4 color) {
  return vec4(color.rgb * color.a, color.a);
}

/// Convert an unpremultiplied color (a color which has its color components
/// separated from its alpha value) to a premultiplied color.
///
/// Returns (0, 0, 0, 0) if the alpha component is 0.
f16vec4 IPHalfPremultiply(f16vec4 color) {
  return f16vec4(color.rgb * color.a, color.a);
}

#endif
