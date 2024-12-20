// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "transform.h"

#include "base/string_utils.h"

namespace gfx {

Transform::Transform()
    : matrix_{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1},
      is_identity_(true) {}

Transform::Transform(float col1row1,
                     float col2row1,
                     float col3row1,
                     float col4row1,
                     float col1row2,
                     float col2row2,
                     float col3row2,
                     float col4row2,
                     float col1row3,
                     float col2row3,
                     float col3row3,
                     float col4row3,
                     float col1row4,
                     float col2row4,
                     float col3row4,
                     float col4row4)
    : matrix_{col1row1, col2row1, col3row1, col4row1, col1row2, col2row2,
              col3row2, col4row2, col1row3, col2row3, col3row3, col4row3,
              col4row4, col2row4, col3row4, col4row4} {
  UpdateIdentity();
}

Transform::Transform(float col1row1,
                     float col2row1,
                     float col1row2,
                     float col2row2,
                     float x_translation,
                     float y_translation)
    : matrix_{col1row1,
              col2row1,
              x_translation,
              0,
              col1row2,
              col2row2,
              y_translation,
              0,
              0,
              0,
              1,
              0,
              0,
              0,
              0,
              1} {
  UpdateIdentity();
}

bool Transform::operator==(const Transform& rhs) const {
  return matrix_[0] == rhs[0] && matrix_[1] == rhs[1] && matrix_[2] == rhs[2] &&
         matrix_[3] == rhs[3] && matrix_[4] == rhs[4] && matrix_[5] == rhs[5] &&
         matrix_[6] == rhs[6] && matrix_[7] == rhs[7] && matrix_[8] == rhs[8] &&
         matrix_[9] == rhs[9] && matrix_[10] == rhs[10] &&
         matrix_[11] == rhs[11] && matrix_[12] == rhs[12] &&
         matrix_[13] == rhs[13] && matrix_[14] == rhs[14] &&
         matrix_[15] == rhs[15];
}

bool Transform::IsIdentity() const {
  return is_identity_;
}

std::string Transform::ToString() const {
  return base::StringPrintf(
      "[ %+0.4f %+0.4f %+0.4f %+0.4f  \n"
      "  %+0.4f %+0.4f %+0.4f %+0.4f  \n"
      "  %+0.4f %+0.4f %+0.4f %+0.4f  \n"
      "  %+0.4f %+0.4f %+0.4f %+0.4f ]\n",
      matrix_[0], matrix_[1], matrix_[2], matrix_[3], matrix_[4], matrix_[5],
      matrix_[6], matrix_[7], matrix_[8], matrix_[9], matrix_[10], matrix_[11],
      matrix_[12], matrix_[13], matrix_[14], matrix_[15]);
}

void Transform::Scale(float x, float y) {
  matrix_[0] *= x;
  matrix_[5] *= y;
  UpdateIdentity();
}

void Transform::TransformRect(RectF* rect) const {
  if (IsIdentity())
    return;
  PointF origin = rect->origin();
  PointF top_right = rect->top_right();
  PointF bottom_left = rect->bottom_left();
  TransformPoint(&origin);
  TransformPoint(&top_right);
  TransformPoint(&bottom_left);
  rect->set_origin(origin);
  rect->set_width(top_right.x() - origin.x());
  rect->set_height(bottom_left.y() - origin.y());
}

void Transform::TransformPoint(PointF* point) const {
  if (IsIdentity())
    return;
  float x = point->x();
  float y = point->y();
  point->SetPoint(x * matrix_[0] + y * matrix_[1] + matrix_[2],
                  x * matrix_[4] + y * matrix_[5] + matrix_[6]);
  return;
}

void Transform::UpdateIdentity() {
  for (size_t i = 0; i < 16; i++) {
    if (i == 0 || i == 5 || i == 10 || i == 15) {
      if (matrix_[i] != 1) {
        is_identity_ = false;
        return;
      }
    } else {
      if (matrix_[i] != 0) {
        is_identity_ = false;
        return;
      }
    }
  }
  is_identity_ = true;
}

}  // namespace gfx
