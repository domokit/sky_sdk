// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef BRANCHING_GLSL_
#define BRANCHING_GLSL_

#include <impeller/constants.glsl>
#include <impeller/types.glsl>

/// Perform an equality check for each vec3 component.
///
/// Returns 1.0 if x == y, otherwise 0.0.
BoolV3 IPVec3IsEqual(vec3 x, float y) {
  vec3 diff = abs(x - y);
  return vec3(diff.r < kEhCloseEnough,  //
              diff.g < kEhCloseEnough,  //
              diff.b < kEhCloseEnough);
}

/// Perform a branchless greater than check.
///
/// Returns 1.0 if x > y, otherwise 0.0.
BoolF IPFloatIsGreaterThan(float x, float y) {
  return max(sign(x - y), 0);
}

/// Perform a branchless greater than check for each vec3 component.
///
/// Returns 1.0 if x > y, otherwise 0.0.
BoolV3 IPVec3IsGreaterThan(vec3 x, vec3 y) {
  return max(sign(x - y), 0);
}

/// Perform a branchless less than check.
///
/// Returns 1.0 if x < y, otherwise 0.0.
BoolF IPFloatIsLessThan(float x, float y) {
  return max(sign(y - x), 0);
}

/// For each vec3 component, if value > cutoff, return b, otherwise return a.
vec3 IPVec3ChooseCutoff(vec3 a, vec3 b, vec3 value, float cutoff) {
  return mix(a, b, IPVec3IsGreaterThan(value, vec3(cutoff)));
}

/// For each vec3 component, if value > 0.5, return b, otherwise return a.
vec3 IPVec3Choose(vec3 a, vec3 b, vec3 value) {
  return IPVec3ChooseCutoff(a, b, value, 0.5);
}

/// Perform a branchless greater than check for each vec3 component.
///
/// Returns 1.0 if x > y, otherwise 0.0.
f16vec3 IPHalfVec3IsGreaterThan(f16vec3 x, f16vec3 y) {
  return max(sign(x - y), float16_t(0.0hf));
}

/// For each vec3 component, if value > cutoff, return b, otherwise return a.
f16vec3 IPHalfVec3ChooseCutoff(f16vec3 a,
                               f16vec3 b,
                               f16vec3 value,
                               float16_t cutoff) {
  return mix(a, b, IPHalfVec3IsGreaterThan(value, f16vec3(cutoff)));
}

/// For each vec3 component, if value > 0.5, return b, otherwise return a.
f16vec3 IPHalfVec3Choose(f16vec3 a, f16vec3 b, f16vec3 value) {
  return IPHalfVec3ChooseCutoff(a, b, value, float16_t(0.5hf));
}

#endif
