// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "gtest/gtest.h"

#include "flutter/impeller/geometry/rstransform.h"

#include "flutter/impeller/geometry/geometry_asserts.h"

namespace impeller {
namespace testing {

TEST(RSTransformTest, Construction) {
  RSTransform transform = RSTransform::Make({10.0f, 12.0f}, 2.0f, Degrees(90));

  EXPECT_EQ(transform.scaled_cos, 0.0f);
  EXPECT_EQ(transform.scaled_sin, 2.0f);
  EXPECT_EQ(transform.translate_x, 10.0f);
  EXPECT_EQ(transform.translate_y, 12.0f);

  EXPECT_EQ(transform.GetBounds(20.0f, 30.0f),
            // relative corners are at
            // 0, 0
            // 0, 40
            // -60, 0
            // -60, 40
            // then add 10, 12 to all values
            Rect::MakeLTRB(10 + -2 * 30, 12 + 0, 10 + 0, 12 + 40));
}

TEST(RSTransformTest, CompareToMatrix) {
  for (int tx = 0; tx <= 100; tx += 10) {
    for (int ty = 0; ty <= 100; ty += 10) {
      Point origin(tx, ty);
      for (int scale = 1; scale <= 20; scale += 5) {
        // Overshoot a full circle by 30 degrees
        for (int degrees = 0; degrees <= 390; degrees += 45) {
          auto matrix = Matrix::MakeTranslation(origin) *
                        Matrix::MakeRotationZ(Degrees(degrees)) *
                        Matrix::MakeScale(Vector2(scale, scale));
          auto rst = RSTransform::Make(origin, scale, Degrees(degrees));
          EXPECT_MATRIX_NEAR(rst.GetMatrix(), matrix);
          for (int w = 10; w <= 100; w += 10) {
            for (int h = 10; h <= 100; h += 10) {
              Quad q = rst.GetQuad(w, h);
              auto points = Rect::MakeWH(w, h).GetTransformedPoints(matrix);
              for (int i = 0; i < 4; i++) {
                EXPECT_NEAR(q[i].x, points[i].x, kEhCloseEnough);
                EXPECT_NEAR(q[i].y, points[i].y, kEhCloseEnough);
              }
            }
          }
        }
      }
    }
  }
}

}  // namespace testing
}  // namespace impeller
