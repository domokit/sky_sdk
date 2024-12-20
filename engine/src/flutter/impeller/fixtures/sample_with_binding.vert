// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "types.h"

layout(set = 3, binding = 17) uniform UniformBufferObject {
  Uniforms uniforms;
}
ubo;

uniform sampler2D world;

in vec2 inPosition;
in vec3 inPosition22;
in vec4 inAnotherPosition;
in float stuff;

out vec4 outStuff;

void main() {
  gl_Position = ubo.uniforms.projection * ubo.uniforms.view *
                ubo.uniforms.model * vec4(inPosition22, 1.0) *
                inAnotherPosition;
  outStuff = texture(world, inPosition);
}
