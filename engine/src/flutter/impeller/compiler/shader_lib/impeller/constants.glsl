// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef CONSTANTS_GLSL_
#define CONSTANTS_GLSL_

#include <impeller/types.glsl>

const float kEhCloseEnough = 0.000001;

// 1/1024.
const float16_t kEhCloseEnoughHalf = 0.0009765625hf;

// 1 / (2 * pi)
const float k1Over2Pi = 0.1591549430918;

// sqrt(2 * pi)
const float kSqrtTwoPi = 2.50662827463;

// sqrt(2) / 2 == 1 / sqrt(2)
const float kHalfSqrtTwo = 0.70710678118;

// sqrt(3)
const float kSqrtThree = 1.73205080757;

// The default mip bias to use when sampling textures.
// This value biases towards sampling from a lower mip level (bigger size),
// which results in sharper looking images when mip sampling is enabled. This is
// the same constant that Skia uses.
const float kDefaultMipBias = -0.475;

#endif
