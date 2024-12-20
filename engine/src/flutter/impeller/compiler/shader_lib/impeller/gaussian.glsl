// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef GAUSSIAN_GLSL_
#define GAUSSIAN_GLSL_

#include <impeller/constants.glsl>
#include <impeller/types.glsl>

/// Gaussian distribution function.
float IPGaussian(float x, float sigma) {
  float variance = sigma * sigma;
  return exp(-0.5f * x * x / variance) / (kSqrtTwoPi * sigma);
}

/// Equivalent to `IPGaussian(float x, float sigma)` but executed 4x.
vec4 IPGaussian(vec4 x, float sigma) {
  float variance = sigma * sigma;
  return exp(-0.5f * x * x / variance) / (kSqrtTwoPi * sigma);
}

/// Gaussian distribution function.
float16_t IPHalfGaussian(float16_t x, float16_t sigma) {
  float16_t variance = sigma * sigma;
  return exp(-0.5hf * x * x / variance) / (float16_t(kSqrtTwoPi) * sigma);
}

/// Abramowitz and Stegun erf approximation.
float16_t IPErf(float16_t x) {
  float16_t a = abs(x);
  // 0.278393*x + 0.230389*x^2 + 0.078108*x^4 + 1
  float16_t b =
      (0.278393hf + (0.230389hf + 0.078108hf * a * a) * a) * a + 1.0hf;
  return sign(x) * (1.0hf - 1.0hf / (b * b * b * b));
}

/// Vec2 variation for the Abramowitz and Stegun erf approximation.
f16vec2 IPVec2Erf(f16vec2 x) {
  f16vec2 a = abs(x);
  // 0.278393*x + 0.230389*x^2 + 0.078108*x^4 + 1
  f16vec2 b = (0.278393hf + (0.230389hf + 0.078108hf * a * a) * a) * a + 1.0hf;
  return sign(x) * (1.0hf - 1.0hf / (b * b * b * b));
}

/// The indefinite integral of the Gaussian function.
/// Uses a very close approximation of Erf.
float16_t IPGaussianIntegral(float16_t x, float16_t sigma) {
  // ( 1 + erf( x * (sqrt(2) / (2 * sigma) ) ) / 2
  return (1.0hf + IPErf(x * (float16_t(kHalfSqrtTwo) / sigma))) * 0.5hf;
}

/// Vec2 variation for the indefinite integral of the Gaussian function.
/// Uses a very close approximation of Erf.
f16vec2 IPVec2GaussianIntegral(f16vec2 x, float16_t sigma) {
  // ( 1 + erf( x * (sqrt(2) / (2 * sigma) ) ) / 2
  return (1.0hf + IPVec2Erf(x * (float16_t(kHalfSqrtTwo) / sigma))) * 0.5hf;
}

/// Simpler (but less accurate) approximation of the Gaussian integral.
vec2 IPVec2FastGaussianIntegral(vec2 x, float sigma) {
  return 1.0 / (1.0 + exp(-kSqrtThree / sigma * x));
}

/// Equivalent to `IPVec2FastGaussianIntegral(vec2 x, float sigma)` but operated
/// 4x instead of 2x.
vec4 IPVec4FastGaussianIntegral(vec4 x, float sigma) {
  return 1.0 / (1.0 + exp(-kSqrtThree / sigma * x));
}

/// Simpler (but less accurate) approximation of the Gaussian integral.
f16vec2 IPHalfVec2FastGaussianIntegral(f16vec2 x, float16_t sigma) {
  return 1.0hf / (1.0hf + exp(float16_t(-kSqrtThree) / sigma * x));
}

/// Simple logistic sigmoid with a domain of [-1, 1] and range of [0, 1].
float16_t IPSigmoid(float16_t x) {
  return 1.03731472073hf / (1.0hf + exp(-4.0hf * x)) - 0.0186573603638hf;
}

#endif
