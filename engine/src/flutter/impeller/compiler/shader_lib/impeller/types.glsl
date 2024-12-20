// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef TYPES_GLSL_
#define TYPES_GLSL_

#extension GL_AMD_gpu_shader_half_float : enable
#extension GL_AMD_gpu_shader_half_float_fetch : enable
#extension GL_EXT_shader_explicit_arithmetic_types_float16 : enable

#ifndef IMPELLER_TARGET_METAL_IOS

precision mediump sampler2D;

#define float16_t float
#define f16vec2 vec2
#define f16vec3 vec3
#define f16vec4 vec4
#define f16mat4 mat4
#define f16sampler2D sampler2D

#endif  // IMPELLER_TARGET_METAL

#define BoolF float
#define BoolV2 vec2
#define BoolV3 vec3
#define BoolV4 vec4

#endif
