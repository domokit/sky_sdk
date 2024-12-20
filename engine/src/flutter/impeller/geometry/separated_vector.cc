// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "separated_vector.h"

namespace impeller {

SeparatedVector2::SeparatedVector2() = default;

SeparatedVector2::SeparatedVector2(Vector2 direction, Scalar magnitude)
    : direction(direction), magnitude(magnitude){};

SeparatedVector2::SeparatedVector2(Vector2 vector)
    : direction(vector.Normalize()), magnitude(vector.GetLength()){};

Vector2 SeparatedVector2::GetVector() const {
  return direction * magnitude;
}

Scalar SeparatedVector2::GetAlignment(const SeparatedVector2& other) const {
  return direction.Dot(other.direction);
}

Radians SeparatedVector2::AngleTo(const SeparatedVector2& other) const {
  return direction.AngleTo(other.direction);
}

}  // namespace impeller
