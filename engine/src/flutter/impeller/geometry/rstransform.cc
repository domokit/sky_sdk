// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/impeller/geometry/rstransform.h"

#include "flutter/impeller/geometry/matrix.h"

namespace impeller {

bool RSTransform::IsAxisAligned() const {
  return scaled_cos == 0.0f || scaled_sin == 0.0f;
}

Matrix RSTransform::GetMatrix() const {
  // clang-format off
  return Matrix::MakeRow(scaled_cos, -scaled_sin, 0.0f, translate_x,
                         scaled_sin,  scaled_cos, 0.0f, translate_y,
                            0.0f,       0.0f,     1.0f,     0.0f,
                            0.0f,       0.0f,     0.0f,     1.0f);
  // clang-format on
}

void RSTransform::GetQuad(Scalar width, Scalar height, Quad& quad) const {
  Point origin = {translate_x, translate_y};
  Point dx = width * Point{scaled_cos, scaled_sin};
  Point dy = height * Point{-scaled_sin, scaled_cos};
  quad = {
      // Ordered in the same Z pattern as Rect::GetPoints()
      origin,
      origin + dx,
      origin + dy,
      origin + dx + dy,
  };
}

Quad RSTransform::GetQuad(Scalar width, Scalar height) const {
  Quad quad;
  GetQuad(width, height, quad);
  return quad;
}

Quad RSTransform::GetQuad(Size size) const {
  Quad quad;
  GetQuad(size.width, size.height, quad);
  return quad;
}

std::optional<Rect> RSTransform::GetBounds(Scalar width, Scalar height) const {
  return Rect::MakePointBounds(GetQuad(width, height));
}

std::optional<Rect> RSTransform::GetBounds(Size size) const {
  return Rect::MakePointBounds(GetQuad(size));
}

}  // namespace impeller
