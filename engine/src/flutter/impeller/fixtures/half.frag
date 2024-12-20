// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#extension GL_AMD_gpu_shader_half_float : enable
#extension GL_AMD_gpu_shader_half_float_fetch : enable
#extension GL_EXT_shader_explicit_arithmetic_types_float16 : enable

uniform FragInfo {
  float16_t half_1;
  f16vec2 half_2;
  f16vec3 half_3;
  f16vec4 half_4;
}
frag_info;

out vec4 frag_color;

void main() {
  frag_color = vec4(0);
}
