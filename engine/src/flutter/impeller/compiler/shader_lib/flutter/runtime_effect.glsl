// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef RUNTIME_EFFECT_GLSL_
#define RUNTIME_EFFECT_GLSL_

#if defined(IMPELLER_GRAPHICS_BACKEND)

// Note: The GLES backend uses name matching for attribute locations. This name
// must match the name of the attribute output in:
// impeller/entity/shaders/runtime_effect.vert
in vec2 _fragCoord;
vec2 FlutterFragCoord() {
  return _fragCoord;
}

#elif defined(SKIA_GRAPHICS_BACKEND)

vec2 FlutterFragCoord() {
  return gl_FragCoord.xy;
}

#else
#error "Runtime effect builtins are not supported for this graphics backend."
#endif

#endif
