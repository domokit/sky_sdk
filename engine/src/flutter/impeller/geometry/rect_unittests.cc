// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "gtest/gtest.h"

#include "flutter/impeller/geometry/rect.h"

#include "flutter/impeller/geometry/geometry_asserts.h"

namespace impeller {
namespace testing {

TEST(RectTest, RectEmptyDeclaration) {
  Rect rect;

  EXPECT_EQ(rect.GetLeft(), 0.0f);
  EXPECT_EQ(rect.GetTop(), 0.0f);
  EXPECT_EQ(rect.GetRight(), 0.0f);
  EXPECT_EQ(rect.GetBottom(), 0.0f);
  EXPECT_EQ(rect.GetX(), 0.0f);
  EXPECT_EQ(rect.GetY(), 0.0f);
  EXPECT_EQ(rect.GetWidth(), 0.0f);
  EXPECT_EQ(rect.GetHeight(), 0.0f);
  EXPECT_TRUE(rect.IsEmpty());
  EXPECT_TRUE(rect.IsFinite());
}

TEST(RectTest, IRectEmptyDeclaration) {
  IRect rect;

  EXPECT_EQ(rect.GetLeft(), 0);
  EXPECT_EQ(rect.GetTop(), 0);
  EXPECT_EQ(rect.GetRight(), 0);
  EXPECT_EQ(rect.GetBottom(), 0);
  EXPECT_EQ(rect.GetX(), 0);
  EXPECT_EQ(rect.GetY(), 0);
  EXPECT_EQ(rect.GetWidth(), 0);
  EXPECT_EQ(rect.GetHeight(), 0);
  EXPECT_TRUE(rect.IsEmpty());
  // EXPECT_TRUE(rect.IsFinite());  // should fail to compile
}

TEST(RectTest, RectDefaultConstructor) {
  Rect rect = Rect();

  EXPECT_EQ(rect.GetLeft(), 0.0f);
  EXPECT_EQ(rect.GetTop(), 0.0f);
  EXPECT_EQ(rect.GetRight(), 0.0f);
  EXPECT_EQ(rect.GetBottom(), 0.0f);
  EXPECT_EQ(rect.GetX(), 0.0f);
  EXPECT_EQ(rect.GetY(), 0.0f);
  EXPECT_EQ(rect.GetWidth(), 0.0f);
  EXPECT_EQ(rect.GetHeight(), 0.0f);
  EXPECT_TRUE(rect.IsEmpty());
  EXPECT_TRUE(rect.IsFinite());
}

TEST(RectTest, IRectDefaultConstructor) {
  IRect rect = IRect();

  EXPECT_EQ(rect.GetLeft(), 0);
  EXPECT_EQ(rect.GetTop(), 0);
  EXPECT_EQ(rect.GetRight(), 0);
  EXPECT_EQ(rect.GetBottom(), 0);
  EXPECT_EQ(rect.GetX(), 0);
  EXPECT_EQ(rect.GetY(), 0);
  EXPECT_EQ(rect.GetWidth(), 0);
  EXPECT_EQ(rect.GetHeight(), 0);
  EXPECT_TRUE(rect.IsEmpty());
}

TEST(RectTest, RectSimpleLTRB) {
  // Using fractional-power-of-2 friendly values for equality tests
  Rect rect = Rect::MakeLTRB(5.125f, 10.25f, 20.625f, 25.375f);

  EXPECT_EQ(rect.GetLeft(), 5.125f);
  EXPECT_EQ(rect.GetTop(), 10.25f);
  EXPECT_EQ(rect.GetRight(), 20.625f);
  EXPECT_EQ(rect.GetBottom(), 25.375f);
  EXPECT_EQ(rect.GetX(), 5.125f);
  EXPECT_EQ(rect.GetY(), 10.25f);
  EXPECT_EQ(rect.GetWidth(), 15.5f);
  EXPECT_EQ(rect.GetHeight(), 15.125f);
  EXPECT_FALSE(rect.IsEmpty());
  EXPECT_TRUE(rect.IsFinite());
}

TEST(RectTest, IRectSimpleLTRB) {
  IRect rect = IRect::MakeLTRB(5, 10, 20, 25);

  EXPECT_EQ(rect.GetLeft(), 5);
  EXPECT_EQ(rect.GetTop(), 10);
  EXPECT_EQ(rect.GetRight(), 20);
  EXPECT_EQ(rect.GetBottom(), 25);
  EXPECT_EQ(rect.GetX(), 5);
  EXPECT_EQ(rect.GetY(), 10);
  EXPECT_EQ(rect.GetWidth(), 15);
  EXPECT_EQ(rect.GetHeight(), 15);
  EXPECT_FALSE(rect.IsEmpty());
}

TEST(RectTest, RectSimpleXYWH) {
  // Using fractional-power-of-2 friendly values for equality tests
  Rect rect = Rect::MakeXYWH(5.125f, 10.25f, 15.5f, 15.125f);

  EXPECT_EQ(rect.GetLeft(), 5.125f);
  EXPECT_EQ(rect.GetTop(), 10.25f);
  EXPECT_EQ(rect.GetRight(), 20.625f);
  EXPECT_EQ(rect.GetBottom(), 25.375f);
  EXPECT_EQ(rect.GetX(), 5.125f);
  EXPECT_EQ(rect.GetY(), 10.25f);
  EXPECT_EQ(rect.GetWidth(), 15.5f);
  EXPECT_EQ(rect.GetHeight(), 15.125f);
  EXPECT_FALSE(rect.IsEmpty());
  EXPECT_TRUE(rect.IsFinite());
}

TEST(RectTest, IRectSimpleXYWH) {
  IRect rect = IRect::MakeXYWH(5, 10, 15, 16);

  EXPECT_EQ(rect.GetLeft(), 5);
  EXPECT_EQ(rect.GetTop(), 10);
  EXPECT_EQ(rect.GetRight(), 20);
  EXPECT_EQ(rect.GetBottom(), 26);
  EXPECT_EQ(rect.GetX(), 5);
  EXPECT_EQ(rect.GetY(), 10);
  EXPECT_EQ(rect.GetWidth(), 15);
  EXPECT_EQ(rect.GetHeight(), 16);
  EXPECT_FALSE(rect.IsEmpty());
}

TEST(RectTest, RectSimpleWH) {
  // Using fractional-power-of-2 friendly values for equality tests
  Rect rect = Rect::MakeWH(15.5f, 15.125f);

  EXPECT_EQ(rect.GetLeft(), 0.0f);
  EXPECT_EQ(rect.GetTop(), 0.0f);
  EXPECT_EQ(rect.GetRight(), 15.5f);
  EXPECT_EQ(rect.GetBottom(), 15.125f);
  EXPECT_EQ(rect.GetX(), 0.0f);
  EXPECT_EQ(rect.GetY(), 0.0f);
  EXPECT_EQ(rect.GetWidth(), 15.5f);
  EXPECT_EQ(rect.GetHeight(), 15.125f);
  EXPECT_FALSE(rect.IsEmpty());
  EXPECT_TRUE(rect.IsFinite());
}

TEST(RectTest, IRectSimpleWH) {
  // Using fractional-power-of-2 friendly values for equality tests
  IRect rect = IRect::MakeWH(15, 25);

  EXPECT_EQ(rect.GetLeft(), 0);
  EXPECT_EQ(rect.GetTop(), 0);
  EXPECT_EQ(rect.GetRight(), 15);
  EXPECT_EQ(rect.GetBottom(), 25);
  EXPECT_EQ(rect.GetX(), 0);
  EXPECT_EQ(rect.GetY(), 0);
  EXPECT_EQ(rect.GetWidth(), 15);
  EXPECT_EQ(rect.GetHeight(), 25);
  EXPECT_FALSE(rect.IsEmpty());
}

TEST(RectTest, RectFromIRect) {
  IRect irect = IRect::MakeLTRB(10, 20, 30, 40);
  Rect rect = Rect::Make(irect);

  EXPECT_EQ(rect.GetLeft(), 10);
  EXPECT_EQ(rect.GetTop(), 20);
  EXPECT_EQ(rect.GetRight(), 30);
  EXPECT_EQ(rect.GetBottom(), 40);

  // The following do not compile
  // IRect irect2 = IRect::Make(rect);
  // IRect irect2 = IRect::Make(irect);
}

TEST(RectTest, RectOverflowXYWH) {
  auto min = std::numeric_limits<Scalar>::lowest();
  auto max = std::numeric_limits<Scalar>::max();
  auto inf = std::numeric_limits<Scalar>::infinity();

  // 8 cases:
  //   finite X, max W
  //   max X, max W
  //   finite Y, max H
  //   max Y, max H
  //   finite X, min W
  //   min X, min W
  //   finite Y, min H
  //   min Y, min H

  // a small finite value added to a max value will remain max
  // a very large finite value (like max) added to max will go to infinity

  {
    Rect rect = Rect::MakeXYWH(5.0, 10.0f, max, 15.0f);

    EXPECT_EQ(rect.GetLeft(), 5.0f);
    EXPECT_EQ(rect.GetTop(), 10.0f);
    EXPECT_EQ(rect.GetRight(), max);
    EXPECT_EQ(rect.GetBottom(), 25.0f);
    EXPECT_EQ(rect.GetX(), 5.0f);
    EXPECT_EQ(rect.GetY(), 10.0f);
    EXPECT_EQ(rect.GetWidth(), max);
    EXPECT_EQ(rect.GetHeight(), 15.0f);
    EXPECT_FALSE(rect.IsEmpty());
    EXPECT_TRUE(rect.IsFinite());
  }

  {
    Rect rect = Rect::MakeXYWH(max, 10.0f, max, 15.0f);

    EXPECT_EQ(rect.GetLeft(), max);
    EXPECT_EQ(rect.GetTop(), 10.0f);
    EXPECT_EQ(rect.GetRight(), inf);
    EXPECT_EQ(rect.GetBottom(), 25.0f);
    EXPECT_EQ(rect.GetX(), max);
    EXPECT_EQ(rect.GetY(), 10.0f);
    EXPECT_EQ(rect.GetWidth(), inf);
    EXPECT_EQ(rect.GetHeight(), 15.0f);
    EXPECT_FALSE(rect.IsEmpty());
    EXPECT_FALSE(rect.IsFinite());
  }

  {
    Rect rect = Rect::MakeXYWH(5.0f, 10.0f, 20.0f, max);

    EXPECT_EQ(rect.GetLeft(), 5.0f);
    EXPECT_EQ(rect.GetTop(), 10.0f);
    EXPECT_EQ(rect.GetRight(), 25.0f);
    EXPECT_EQ(rect.GetBottom(), max);
    EXPECT_EQ(rect.GetX(), 5.0f);
    EXPECT_EQ(rect.GetY(), 10.0f);
    EXPECT_EQ(rect.GetWidth(), 20.0f);
    EXPECT_EQ(rect.GetHeight(), max);
    EXPECT_FALSE(rect.IsEmpty());
    EXPECT_TRUE(rect.IsFinite());
  }

  {
    Rect rect = Rect::MakeXYWH(5.0f, max, 20.0f, max);

    EXPECT_EQ(rect.GetLeft(), 5.0f);
    EXPECT_EQ(rect.GetTop(), max);
    EXPECT_EQ(rect.GetRight(), 25.0f);
    EXPECT_EQ(rect.GetBottom(), inf);
    EXPECT_EQ(rect.GetX(), 5.0f);
    EXPECT_EQ(rect.GetY(), max);
    EXPECT_EQ(rect.GetWidth(), 20.0f);
    EXPECT_EQ(rect.GetHeight(), inf);
    EXPECT_FALSE(rect.IsEmpty());
    EXPECT_FALSE(rect.IsFinite());
  }

  {
    Rect rect = Rect::MakeXYWH(5.0, 10.0f, min, 15.0f);

    EXPECT_EQ(rect.GetLeft(), 5.0f);
    EXPECT_EQ(rect.GetTop(), 10.0f);
    EXPECT_EQ(rect.GetRight(), min);
    EXPECT_EQ(rect.GetBottom(), 25.0f);
    EXPECT_EQ(rect.GetX(), 5.0f);
    EXPECT_EQ(rect.GetY(), 10.0f);
    EXPECT_EQ(rect.GetWidth(), min);
    EXPECT_EQ(rect.GetHeight(), 15.0f);
    EXPECT_TRUE(rect.IsEmpty());
    EXPECT_TRUE(rect.IsFinite());
  }

  {
    Rect rect = Rect::MakeXYWH(min, 10.0f, min, 15.0f);

    EXPECT_EQ(rect.GetLeft(), min);
    EXPECT_EQ(rect.GetTop(), 10.0f);
    EXPECT_EQ(rect.GetRight(), -inf);
    EXPECT_EQ(rect.GetBottom(), 25.0f);
    EXPECT_EQ(rect.GetX(), min);
    EXPECT_EQ(rect.GetY(), 10.0f);
    EXPECT_EQ(rect.GetWidth(), -inf);
    EXPECT_EQ(rect.GetHeight(), 15.0f);
    EXPECT_TRUE(rect.IsEmpty());
    EXPECT_FALSE(rect.IsFinite());
  }

  {
    Rect rect = Rect::MakeXYWH(5.0f, 10.0f, 20.0f, min);

    EXPECT_EQ(rect.GetLeft(), 5.0f);
    EXPECT_EQ(rect.GetTop(), 10.0f);
    EXPECT_EQ(rect.GetRight(), 25.0f);
    EXPECT_EQ(rect.GetBottom(), min);
    EXPECT_EQ(rect.GetX(), 5.0f);
    EXPECT_EQ(rect.GetY(), 10.0f);
    EXPECT_EQ(rect.GetWidth(), 20.0f);
    EXPECT_EQ(rect.GetHeight(), min);
    EXPECT_TRUE(rect.IsEmpty());
    EXPECT_TRUE(rect.IsFinite());
  }

  {
    Rect rect = Rect::MakeXYWH(5.0f, min, 20.0f, min);

    EXPECT_EQ(rect.GetLeft(), 5.0f);
    EXPECT_EQ(rect.GetTop(), min);
    EXPECT_EQ(rect.GetRight(), 25.0f);
    EXPECT_EQ(rect.GetBottom(), -inf);
    EXPECT_EQ(rect.GetX(), 5.0f);
    EXPECT_EQ(rect.GetY(), min);
    EXPECT_EQ(rect.GetWidth(), 20.0f);
    EXPECT_EQ(rect.GetHeight(), -inf);
    EXPECT_TRUE(rect.IsEmpty());
    EXPECT_FALSE(rect.IsFinite());
  }
}

TEST(RectTest, IRectOverflowXYWH) {
  auto min = std::numeric_limits<int64_t>::min();
  auto max = std::numeric_limits<int64_t>::max();

  // 4 cases
  //   x near max, positive w takes it past max
  //   x near min, negative w takes it below min
  //   y near max, positive h takes it past max
  //   y near min, negative h takes it below min

  {
    IRect rect = IRect::MakeXYWH(max - 5, 10, 10, 16);

    EXPECT_EQ(rect.GetLeft(), max - 5);
    EXPECT_EQ(rect.GetTop(), 10);
    EXPECT_EQ(rect.GetRight(), max);
    EXPECT_EQ(rect.GetBottom(), 26);
    EXPECT_EQ(rect.GetX(), max - 5);
    EXPECT_EQ(rect.GetY(), 10);
    EXPECT_EQ(rect.GetWidth(), 5);
    EXPECT_EQ(rect.GetHeight(), 16);
    EXPECT_FALSE(rect.IsEmpty());
  }

  {
    IRect rect = IRect::MakeXYWH(min + 5, 10, -10, 16);

    EXPECT_EQ(rect.GetLeft(), min + 5);
    EXPECT_EQ(rect.GetTop(), 10);
    EXPECT_EQ(rect.GetRight(), min);
    EXPECT_EQ(rect.GetBottom(), 26);
    EXPECT_EQ(rect.GetX(), min + 5);
    EXPECT_EQ(rect.GetY(), 10);
    EXPECT_EQ(rect.GetWidth(), -5);
    EXPECT_EQ(rect.GetHeight(), 16);
    EXPECT_TRUE(rect.IsEmpty());
  }

  {
    IRect rect = IRect::MakeXYWH(5, max - 10, 10, 16);

    EXPECT_EQ(rect.GetLeft(), 5);
    EXPECT_EQ(rect.GetTop(), max - 10);
    EXPECT_EQ(rect.GetRight(), 15);
    EXPECT_EQ(rect.GetBottom(), max);
    EXPECT_EQ(rect.GetX(), 5);
    EXPECT_EQ(rect.GetY(), max - 10);
    EXPECT_EQ(rect.GetWidth(), 10);
    EXPECT_EQ(rect.GetHeight(), 10);
    EXPECT_FALSE(rect.IsEmpty());
  }

  {
    IRect rect = IRect::MakeXYWH(5, min + 10, 10, -16);

    EXPECT_EQ(rect.GetLeft(), 5);
    EXPECT_EQ(rect.GetTop(), min + 10);
    EXPECT_EQ(rect.GetRight(), 15);
    EXPECT_EQ(rect.GetBottom(), min);
    EXPECT_EQ(rect.GetX(), 5);
    EXPECT_EQ(rect.GetY(), min + 10);
    EXPECT_EQ(rect.GetWidth(), 10);
    EXPECT_EQ(rect.GetHeight(), -10);
    EXPECT_TRUE(rect.IsEmpty());
  }
}

TEST(RectTest, RectOverflowLTRB) {
  auto min = std::numeric_limits<Scalar>::lowest();
  auto max = std::numeric_limits<Scalar>::max();
  auto inf = std::numeric_limits<Scalar>::infinity();

  // 8 cases:
  //   finite negative X, max W
  //   ~min X, ~max W
  //   finite negative Y, max H
  //   ~min Y, ~max H
  //   finite positive X, min W
  //   ~min X, ~min W
  //   finite positive Y, min H
  //   ~min Y, ~min H

  // a small finite value subtracted from a max value will remain max
  // a very large finite value (like min) subtracted from max will go to inf

  {
    Rect rect = Rect::MakeLTRB(-5.0f, 10.0f, max, 25.0f);

    EXPECT_EQ(rect.GetLeft(), -5.0f);
    EXPECT_EQ(rect.GetTop(), 10.0f);
    EXPECT_EQ(rect.GetRight(), max);
    EXPECT_EQ(rect.GetBottom(), 25.0f);
    EXPECT_EQ(rect.GetX(), -5.0f);
    EXPECT_EQ(rect.GetY(), 10.0f);
    EXPECT_EQ(rect.GetWidth(), max);
    EXPECT_EQ(rect.GetHeight(), 15.0f);
    EXPECT_FALSE(rect.IsEmpty());
    EXPECT_TRUE(rect.IsFinite());
  }

  {
    Rect rect = Rect::MakeLTRB(min + 5.0f, 10.0f, max - 5.0f, 25.0f);

    EXPECT_EQ(rect.GetLeft(), min + 5.0f);
    EXPECT_EQ(rect.GetTop(), 10.0f);
    EXPECT_EQ(rect.GetRight(), max - 5.0f);
    EXPECT_EQ(rect.GetBottom(), 25.0f);
    EXPECT_EQ(rect.GetX(), min + 5.0f);
    EXPECT_EQ(rect.GetY(), 10.0f);
    EXPECT_EQ(rect.GetWidth(), inf);
    EXPECT_EQ(rect.GetHeight(), 15.0f);
    EXPECT_FALSE(rect.IsEmpty());
    EXPECT_TRUE(rect.IsFinite());
  }

  {
    Rect rect = Rect::MakeLTRB(5.0f, -10.0f, 20.0f, max);

    EXPECT_EQ(rect.GetLeft(), 5.0f);
    EXPECT_EQ(rect.GetTop(), -10.0f);
    EXPECT_EQ(rect.GetRight(), 20.0f);
    EXPECT_EQ(rect.GetBottom(), max);
    EXPECT_EQ(rect.GetX(), 5.0f);
    EXPECT_EQ(rect.GetY(), -10.0f);
    EXPECT_EQ(rect.GetWidth(), 15.0f);
    EXPECT_EQ(rect.GetHeight(), max);
    EXPECT_FALSE(rect.IsEmpty());
    EXPECT_TRUE(rect.IsFinite());
  }

  {
    Rect rect = Rect::MakeLTRB(5.0f, min + 10.0f, 20.0f, max - 15.0f);

    EXPECT_EQ(rect.GetLeft(), 5.0f);
    EXPECT_EQ(rect.GetTop(), min + 10.0f);
    EXPECT_EQ(rect.GetRight(), 20.0f);
    EXPECT_EQ(rect.GetBottom(), max - 15.0f);
    EXPECT_EQ(rect.GetX(), 5.0f);
    EXPECT_EQ(rect.GetY(), min + 10.0f);
    EXPECT_EQ(rect.GetWidth(), 15.0f);
    EXPECT_EQ(rect.GetHeight(), inf);
    EXPECT_FALSE(rect.IsEmpty());
    EXPECT_TRUE(rect.IsFinite());
  }

  {
    Rect rect = Rect::MakeLTRB(5.0f, 10.0f, min, 25.0f);

    EXPECT_EQ(rect.GetLeft(), 5.0f);
    EXPECT_EQ(rect.GetTop(), 10.0f);
    EXPECT_EQ(rect.GetRight(), min);
    EXPECT_EQ(rect.GetBottom(), 25.0f);
    EXPECT_EQ(rect.GetX(), 5.0f);
    EXPECT_EQ(rect.GetY(), 10.0f);
    EXPECT_EQ(rect.GetWidth(), min);
    EXPECT_EQ(rect.GetHeight(), 15.0f);
    EXPECT_TRUE(rect.IsEmpty());
    EXPECT_TRUE(rect.IsFinite());
  }

  {
    Rect rect = Rect::MakeLTRB(max - 5.0f, 10.0f, min + 10.0f, 25.0f);

    EXPECT_EQ(rect.GetLeft(), max - 5.0f);
    EXPECT_EQ(rect.GetTop(), 10.0f);
    EXPECT_EQ(rect.GetRight(), min + 10.0f);
    EXPECT_EQ(rect.GetBottom(), 25.0f);
    EXPECT_EQ(rect.GetX(), max - 5.0f);
    EXPECT_EQ(rect.GetY(), 10.0f);
    EXPECT_EQ(rect.GetWidth(), -inf);
    EXPECT_EQ(rect.GetHeight(), 15.0f);
    EXPECT_TRUE(rect.IsEmpty());
    EXPECT_TRUE(rect.IsFinite());
  }

  {
    Rect rect = Rect::MakeLTRB(5.0f, 10.0f, 20.0f, min);

    EXPECT_EQ(rect.GetLeft(), 5.0f);
    EXPECT_EQ(rect.GetTop(), 10.0f);
    EXPECT_EQ(rect.GetRight(), 20.0f);
    EXPECT_EQ(rect.GetBottom(), min);
    EXPECT_EQ(rect.GetX(), 5.0f);
    EXPECT_EQ(rect.GetY(), 10.0f);
    EXPECT_EQ(rect.GetWidth(), 15.0f);
    EXPECT_EQ(rect.GetHeight(), min);
    EXPECT_TRUE(rect.IsEmpty());
    EXPECT_TRUE(rect.IsFinite());
  }

  {
    Rect rect = Rect::MakeLTRB(5.0f, max - 5.0f, 20.0f, min + 10.0f);

    EXPECT_EQ(rect.GetLeft(), 5.0f);
    EXPECT_EQ(rect.GetTop(), max - 5.0f);
    EXPECT_EQ(rect.GetRight(), 20.0f);
    EXPECT_EQ(rect.GetBottom(), min + 10.0f);
    EXPECT_EQ(rect.GetX(), 5.0f);
    EXPECT_EQ(rect.GetY(), max - 5.0f);
    EXPECT_EQ(rect.GetWidth(), 15.0f);
    EXPECT_EQ(rect.GetHeight(), -inf);
    EXPECT_TRUE(rect.IsEmpty());
    EXPECT_TRUE(rect.IsFinite());
  }
}

TEST(RectTest, IRectOverflowLTRB) {
  auto min = std::numeric_limits<int64_t>::min();
  auto max = std::numeric_limits<int64_t>::max();

  // 4 cases
  //   negative l, r near max takes width past max
  //   positive l, r near min takes width below min
  //   negative t, b near max takes width past max
  //   positive t, b near min takes width below min

  {
    IRect rect = IRect::MakeLTRB(-10, 10, max - 5, 26);

    EXPECT_EQ(rect.GetLeft(), -10);
    EXPECT_EQ(rect.GetTop(), 10);
    EXPECT_EQ(rect.GetRight(), max - 5);
    EXPECT_EQ(rect.GetBottom(), 26);
    EXPECT_EQ(rect.GetX(), -10);
    EXPECT_EQ(rect.GetY(), 10);
    EXPECT_EQ(rect.GetWidth(), max);
    EXPECT_EQ(rect.GetHeight(), 16);
    EXPECT_FALSE(rect.IsEmpty());
  }

  {
    IRect rect = IRect::MakeLTRB(10, 10, min + 5, 26);

    EXPECT_EQ(rect.GetLeft(), 10);
    EXPECT_EQ(rect.GetTop(), 10);
    EXPECT_EQ(rect.GetRight(), min + 5);
    EXPECT_EQ(rect.GetBottom(), 26);
    EXPECT_EQ(rect.GetX(), 10);
    EXPECT_EQ(rect.GetY(), 10);
    EXPECT_EQ(rect.GetWidth(), min);
    EXPECT_EQ(rect.GetHeight(), 16);
    EXPECT_TRUE(rect.IsEmpty());
  }

  {
    IRect rect = IRect::MakeLTRB(5, -10, 15, max - 5);

    EXPECT_EQ(rect.GetLeft(), 5);
    EXPECT_EQ(rect.GetTop(), -10);
    EXPECT_EQ(rect.GetRight(), 15);
    EXPECT_EQ(rect.GetBottom(), max - 5);
    EXPECT_EQ(rect.GetX(), 5);
    EXPECT_EQ(rect.GetY(), -10);
    EXPECT_EQ(rect.GetWidth(), 10);
    EXPECT_EQ(rect.GetHeight(), max);
    EXPECT_FALSE(rect.IsEmpty());
  }

  {
    IRect rect = IRect::MakeLTRB(5, 10, 15, min + 5);

    EXPECT_EQ(rect.GetLeft(), 5);
    EXPECT_EQ(rect.GetTop(), 10);
    EXPECT_EQ(rect.GetRight(), 15);
    EXPECT_EQ(rect.GetBottom(), min + 5);
    EXPECT_EQ(rect.GetX(), 5);
    EXPECT_EQ(rect.GetY(), 10);
    EXPECT_EQ(rect.GetWidth(), 10);
    EXPECT_EQ(rect.GetHeight(), min);
    EXPECT_TRUE(rect.IsEmpty());
  }
}

TEST(RectTest, RectMakeSize) {
  {
    Size s(100, 200);
    Rect r = Rect::MakeSize(s);
    Rect expected = Rect::MakeLTRB(0, 0, 100, 200);
    EXPECT_RECT_NEAR(r, expected);
  }

  {
    ISize s(100, 200);
    Rect r = Rect::MakeSize(s);
    Rect expected = Rect::MakeLTRB(0, 0, 100, 200);
    EXPECT_RECT_NEAR(r, expected);
  }

  {
    Size s(100, 200);
    IRect r = IRect::MakeSize(s);
    IRect expected = IRect::MakeLTRB(0, 0, 100, 200);
    EXPECT_EQ(r, expected);
  }

  {
    ISize s(100, 200);
    IRect r = IRect::MakeSize(s);
    IRect expected = IRect::MakeLTRB(0, 0, 100, 200);
    EXPECT_EQ(r, expected);
  }
}

TEST(RectTest, RectMakeMaximum) {
  Rect rect = Rect::MakeMaximum();
  auto inf = std::numeric_limits<Scalar>::infinity();
  auto min = std::numeric_limits<Scalar>::lowest();
  auto max = std::numeric_limits<Scalar>::max();

  EXPECT_EQ(rect.GetLeft(), min);
  EXPECT_EQ(rect.GetTop(), min);
  EXPECT_EQ(rect.GetRight(), max);
  EXPECT_EQ(rect.GetBottom(), max);
  EXPECT_EQ(rect.GetX(), min);
  EXPECT_EQ(rect.GetY(), min);
  EXPECT_EQ(rect.GetWidth(), inf);
  EXPECT_EQ(rect.GetHeight(), inf);
  EXPECT_FALSE(rect.IsEmpty());
  EXPECT_TRUE(rect.IsFinite());
}

TEST(RectTest, IRectMakeMaximum) {
  IRect rect = IRect::MakeMaximum();
  auto min = std::numeric_limits<int64_t>::min();
  auto max = std::numeric_limits<int64_t>::max();

  EXPECT_EQ(rect.GetLeft(), min);
  EXPECT_EQ(rect.GetTop(), min);
  EXPECT_EQ(rect.GetRight(), max);
  EXPECT_EQ(rect.GetBottom(), max);
  EXPECT_EQ(rect.GetX(), min);
  EXPECT_EQ(rect.GetY(), min);
  EXPECT_EQ(rect.GetWidth(), max);
  EXPECT_EQ(rect.GetHeight(), max);
  EXPECT_FALSE(rect.IsEmpty());
}

TEST(RectTest, RectFromRect) {
  EXPECT_EQ(Rect(Rect::MakeXYWH(2, 3, 7, 15)),
            Rect::MakeXYWH(2.0, 3.0, 7.0, 15.0));
  EXPECT_EQ(Rect(Rect::MakeLTRB(2, 3, 7, 15)),
            Rect::MakeLTRB(2.0, 3.0, 7.0, 15.0));
}

TEST(RectTest, IRectFromIRect) {
  EXPECT_EQ(IRect(IRect::MakeXYWH(2, 3, 7, 15)),  //
            IRect::MakeXYWH(2, 3, 7, 15));
  EXPECT_EQ(IRect(IRect::MakeLTRB(2, 3, 7, 15)),  //
            IRect::MakeLTRB(2, 3, 7, 15));
}

TEST(RectTest, RectCopy) {
  // Using fractional-power-of-2 friendly values for equality tests
  Rect rect = Rect::MakeLTRB(5.125f, 10.25f, 20.625f, 25.375f);
  Rect copy = rect;

  EXPECT_EQ(rect, copy);
  EXPECT_EQ(copy.GetLeft(), 5.125f);
  EXPECT_EQ(copy.GetTop(), 10.25f);
  EXPECT_EQ(copy.GetRight(), 20.625f);
  EXPECT_EQ(copy.GetBottom(), 25.375f);
  EXPECT_EQ(copy.GetX(), 5.125f);
  EXPECT_EQ(copy.GetY(), 10.25f);
  EXPECT_EQ(copy.GetWidth(), 15.5f);
  EXPECT_EQ(copy.GetHeight(), 15.125f);
  EXPECT_FALSE(copy.IsEmpty());
  EXPECT_TRUE(copy.IsFinite());
}

TEST(RectTest, IRectCopy) {
  IRect rect = IRect::MakeLTRB(5, 10, 20, 25);
  IRect copy = rect;

  EXPECT_EQ(rect, copy);
  EXPECT_EQ(copy.GetLeft(), 5);
  EXPECT_EQ(copy.GetTop(), 10);
  EXPECT_EQ(copy.GetRight(), 20);
  EXPECT_EQ(copy.GetBottom(), 25);
  EXPECT_EQ(copy.GetX(), 5);
  EXPECT_EQ(copy.GetY(), 10);
  EXPECT_EQ(copy.GetWidth(), 15);
  EXPECT_EQ(copy.GetHeight(), 15);
  EXPECT_FALSE(copy.IsEmpty());
}

TEST(RectTest, RectOriginSizeXYWHGetters) {
  {
    Rect r = Rect::MakeOriginSize({10, 20}, {50, 40});
    EXPECT_EQ(r.GetOrigin(), Point(10, 20));
    EXPECT_EQ(r.GetSize(), Size(50, 40));
    EXPECT_EQ(r.GetX(), 10);
    EXPECT_EQ(r.GetY(), 20);
    EXPECT_EQ(r.GetWidth(), 50);
    EXPECT_EQ(r.GetHeight(), 40);
    auto expected_array = std::array<Scalar, 4>{10, 20, 50, 40};
    EXPECT_EQ(r.GetXYWH(), expected_array);
  }

  {
    Rect r = Rect::MakeLTRB(10, 20, 50, 40);
    EXPECT_EQ(r.GetOrigin(), Point(10, 20));
    EXPECT_EQ(r.GetSize(), Size(40, 20));
    EXPECT_EQ(r.GetX(), 10);
    EXPECT_EQ(r.GetY(), 20);
    EXPECT_EQ(r.GetWidth(), 40);
    EXPECT_EQ(r.GetHeight(), 20);
    auto expected_array = std::array<Scalar, 4>{10, 20, 40, 20};
    EXPECT_EQ(r.GetXYWH(), expected_array);
  }
}

TEST(RectTest, IRectOriginSizeXYWHGetters) {
  {
    IRect r = IRect::MakeOriginSize({10, 20}, {50, 40});
    EXPECT_EQ(r.GetOrigin(), IPoint(10, 20));
    EXPECT_EQ(r.GetSize(), ISize(50, 40));
    EXPECT_EQ(r.GetX(), 10);
    EXPECT_EQ(r.GetY(), 20);
    EXPECT_EQ(r.GetWidth(), 50);
    EXPECT_EQ(r.GetHeight(), 40);
    auto expected_array = std::array<int64_t, 4>{10, 20, 50, 40};
    EXPECT_EQ(r.GetXYWH(), expected_array);
  }

  {
    IRect r = IRect::MakeLTRB(10, 20, 50, 40);
    EXPECT_EQ(r.GetOrigin(), IPoint(10, 20));
    EXPECT_EQ(r.GetSize(), ISize(40, 20));
    EXPECT_EQ(r.GetX(), 10);
    EXPECT_EQ(r.GetY(), 20);
    EXPECT_EQ(r.GetWidth(), 40);
    EXPECT_EQ(r.GetHeight(), 20);
    auto expected_array = std::array<int64_t, 4>{10, 20, 40, 20};
    EXPECT_EQ(r.GetXYWH(), expected_array);
  }
}

TEST(RectTest, RectRoundOutEmpty) {
  Rect rect;

  EXPECT_EQ(Rect::RoundOut(rect), Rect());

  EXPECT_EQ(IRect::RoundOut(rect), IRect());
}

TEST(RectTest, RectRoundOutSimple) {
  Rect rect = Rect::MakeLTRB(5.125f, 10.75f, 20.625f, 25.375f);

  EXPECT_EQ(Rect::RoundOut(rect), Rect::MakeLTRB(5.0f, 10.0f, 21.0f, 26.0f));

  EXPECT_EQ(IRect::RoundOut(rect), IRect::MakeLTRB(5, 10, 21, 26));
}

TEST(RectTest, RectRoundOutToIRectHuge) {
  auto test = [](int corners) {
    EXPECT_TRUE(corners >= 0 && corners <= 0xf);
    Scalar l, t, r, b;
    int64_t il, it, ir, ib;
    l = il = 50;
    t = it = 50;
    r = ir = 80;
    b = ib = 80;
    if ((corners & (1 << 0)) != 0) {
      l = -1E20;
      il = std::numeric_limits<int64_t>::min();
    }
    if ((corners & (1 << 1)) != 0) {
      t = -1E20;
      it = std::numeric_limits<int64_t>::min();
    }
    if ((corners & (1 << 2)) != 0) {
      r = +1E20;
      ir = std::numeric_limits<int64_t>::max();
    }
    if ((corners & (1 << 3)) != 0) {
      b = +1E20;
      ib = std::numeric_limits<int64_t>::max();
    }

    Rect rect = Rect::MakeLTRB(l, t, r, b);
    IRect irect = IRect::RoundOut(rect);
    EXPECT_EQ(irect.GetLeft(), il) << corners;
    EXPECT_EQ(irect.GetTop(), it) << corners;
    EXPECT_EQ(irect.GetRight(), ir) << corners;
    EXPECT_EQ(irect.GetBottom(), ib) << corners;
  };

  for (int corners = 0; corners <= 15; corners++) {
    test(corners);
  }
}

TEST(RectTest, RectDoesNotIntersectEmpty) {
  Rect rect = Rect::MakeLTRB(50, 50, 100, 100);

  auto test = [&rect](Scalar l, Scalar t, Scalar r, Scalar b,
                      const std::string& label) {
    EXPECT_FALSE(rect.IntersectsWithRect(Rect::MakeLTRB(l, b, r, t)))
        << label << " with Top/Bottom swapped";
    EXPECT_FALSE(rect.IntersectsWithRect(Rect::MakeLTRB(r, b, l, t)))
        << label << " with Left/Right swapped";
    EXPECT_FALSE(rect.IntersectsWithRect(Rect::MakeLTRB(r, t, l, b)))
        << label << " with all sides swapped";
  };

  test(20, 20, 30, 30, "Above and Left");
  test(70, 20, 80, 30, "Above");
  test(120, 20, 130, 30, "Above and Right");
  test(120, 70, 130, 80, "Right");
  test(120, 120, 130, 130, "Below and Right");
  test(70, 120, 80, 130, "Below");
  test(20, 120, 30, 130, "Below and Left");
  test(20, 70, 30, 80, "Left");

  test(70, 70, 80, 80, "Inside");

  test(40, 70, 60, 80, "Straddling Left");
  test(70, 40, 80, 60, "Straddling Top");
  test(90, 70, 110, 80, "Straddling Right");
  test(70, 90, 80, 110, "Straddling Bottom");
}

TEST(RectTest, IRectDoesNotIntersectEmpty) {
  IRect rect = IRect::MakeLTRB(50, 50, 100, 100);

  auto test = [&rect](int64_t l, int64_t t, int64_t r, int64_t b,
                      const std::string& label) {
    EXPECT_FALSE(rect.IntersectsWithRect(IRect::MakeLTRB(l, b, r, t)))
        << label << " with Top/Bottom swapped";
    EXPECT_FALSE(rect.IntersectsWithRect(IRect::MakeLTRB(r, b, l, t)))
        << label << " with Left/Right swapped";
    EXPECT_FALSE(rect.IntersectsWithRect(IRect::MakeLTRB(r, t, l, b)))
        << label << " with all sides swapped";
  };

  test(20, 20, 30, 30, "Above and Left");
  test(70, 20, 80, 30, "Above");
  test(120, 20, 130, 30, "Above and Right");
  test(120, 70, 130, 80, "Right");
  test(120, 120, 130, 130, "Below and Right");
  test(70, 120, 80, 130, "Below");
  test(20, 120, 30, 130, "Below and Left");
  test(20, 70, 30, 80, "Left");

  test(70, 70, 80, 80, "Inside");

  test(40, 70, 60, 80, "Straddling Left");
  test(70, 40, 80, 60, "Straddling Top");
  test(90, 70, 110, 80, "Straddling Right");
  test(70, 90, 80, 110, "Straddling Bottom");
}

TEST(RectTest, EmptyRectDoesNotIntersect) {
  Rect rect = Rect::MakeLTRB(50, 50, 100, 100);

  auto test = [&rect](Scalar l, Scalar t, Scalar r, Scalar b,
                      const std::string& label) {
    EXPECT_FALSE(Rect::MakeLTRB(l, b, r, t).IntersectsWithRect(rect))
        << label << " with Top/Bottom swapped";
    EXPECT_FALSE(Rect::MakeLTRB(r, b, l, t).IntersectsWithRect(rect))
        << label << " with Left/Right swapped";
    EXPECT_FALSE(Rect::MakeLTRB(r, t, l, b).IntersectsWithRect(rect))
        << label << " with all sides swapped";
  };

  test(20, 20, 30, 30, "Above and Left");
  test(70, 20, 80, 30, "Above");
  test(120, 20, 130, 30, "Above and Right");
  test(120, 70, 130, 80, "Right");
  test(120, 120, 130, 130, "Below and Right");
  test(70, 120, 80, 130, "Below");
  test(20, 120, 30, 130, "Below and Left");
  test(20, 70, 30, 80, "Left");

  test(70, 70, 80, 80, "Inside");

  test(40, 70, 60, 80, "Straddling Left");
  test(70, 40, 80, 60, "Straddling Top");
  test(90, 70, 110, 80, "Straddling Right");
  test(70, 90, 80, 110, "Straddling Bottom");
}

TEST(RectTest, EmptyIRectDoesNotIntersect) {
  IRect rect = IRect::MakeLTRB(50, 50, 100, 100);

  auto test = [&rect](int64_t l, int64_t t, int64_t r, int64_t b,
                      const std::string& label) {
    EXPECT_FALSE(IRect::MakeLTRB(l, b, r, t).IntersectsWithRect(rect))
        << label << " with Top/Bottom swapped";
    EXPECT_FALSE(IRect::MakeLTRB(r, b, l, t).IntersectsWithRect(rect))
        << label << " with Left/Right swapped";
    EXPECT_FALSE(IRect::MakeLTRB(r, t, l, b).IntersectsWithRect(rect))
        << label << " with all sides swapped";
  };

  test(20, 20, 30, 30, "Above and Left");
  test(70, 20, 80, 30, "Above");
  test(120, 20, 130, 30, "Above and Right");
  test(120, 70, 130, 80, "Right");
  test(120, 120, 130, 130, "Below and Right");
  test(70, 120, 80, 130, "Below");
  test(20, 120, 30, 130, "Below and Left");
  test(20, 70, 30, 80, "Left");

  test(70, 70, 80, 80, "Inside");

  test(40, 70, 60, 80, "Straddling Left");
  test(70, 40, 80, 60, "Straddling Top");
  test(90, 70, 110, 80, "Straddling Right");
  test(70, 90, 80, 110, "Straddling Bottom");
}

TEST(RectTest, RectScale) {
  auto test1 = [](Rect rect, Scalar scale) {
    Rect expected = Rect::MakeXYWH(rect.GetX() * scale,      //
                                   rect.GetY() * scale,      //
                                   rect.GetWidth() * scale,  //
                                   rect.GetHeight() * scale);

    EXPECT_RECT_NEAR(rect.Scale(scale), expected)  //
        << rect << " * " << scale;
    EXPECT_RECT_NEAR(rect.Scale(scale, scale), expected)  //
        << rect << " * " << scale;
    EXPECT_RECT_NEAR(rect.Scale(Point(scale, scale)), expected)  //
        << rect << " * " << scale;
    EXPECT_RECT_NEAR(rect.Scale(Size(scale, scale)), expected)  //
        << rect << " * " << scale;
  };

  auto test2 = [&test1](Rect rect, Scalar scale_x, Scalar scale_y) {
    Rect expected = Rect::MakeXYWH(rect.GetX() * scale_x,      //
                                   rect.GetY() * scale_y,      //
                                   rect.GetWidth() * scale_x,  //
                                   rect.GetHeight() * scale_y);

    EXPECT_RECT_NEAR(rect.Scale(scale_x, scale_y), expected)  //
        << rect << " * " << scale_x << ", " << scale_y;
    EXPECT_RECT_NEAR(rect.Scale(Point(scale_x, scale_y)), expected)  //
        << rect << " * " << scale_x << ", " << scale_y;
    EXPECT_RECT_NEAR(rect.Scale(Size(scale_x, scale_y)), expected)  //
        << rect << " * " << scale_x << ", " << scale_y;

    test1(rect, scale_x);
    test1(rect, scale_y);
  };

  test2(Rect::MakeLTRB(10, 15, 100, 150), 1.0, 0.0);
  test2(Rect::MakeLTRB(10, 15, 100, 150), 0.0, 1.0);
  test2(Rect::MakeLTRB(10, 15, 100, 150), 0.0, 0.0);
  test2(Rect::MakeLTRB(10, 15, 100, 150), 2.5, 3.5);
  test2(Rect::MakeLTRB(10, 15, 100, 150), 3.5, 2.5);
  test2(Rect::MakeLTRB(10, 15, -100, 150), 2.5, 3.5);
  test2(Rect::MakeLTRB(10, 15, 100, -150), 2.5, 3.5);
  test2(Rect::MakeLTRB(10, 15, 100, 150), -2.5, 3.5);
  test2(Rect::MakeLTRB(10, 15, 100, 150), 2.5, -3.5);
}

TEST(RectTest, IRectScale) {
  auto test1 = [](IRect rect, int64_t scale) {
    IRect expected = IRect::MakeXYWH(rect.GetX() * scale,      //
                                     rect.GetY() * scale,      //
                                     rect.GetWidth() * scale,  //
                                     rect.GetHeight() * scale);

    EXPECT_EQ(rect.Scale(scale), expected)  //
        << rect << " * " << scale;
    EXPECT_EQ(rect.Scale(scale, scale), expected)  //
        << rect << " * " << scale;
    EXPECT_EQ(rect.Scale(IPoint(scale, scale)), expected)  //
        << rect << " * " << scale;
    EXPECT_EQ(rect.Scale(ISize(scale, scale)), expected)  //
        << rect << " * " << scale;
  };

  auto test2 = [&test1](IRect rect, int64_t scale_x, int64_t scale_y) {
    IRect expected = IRect::MakeXYWH(rect.GetX() * scale_x,      //
                                     rect.GetY() * scale_y,      //
                                     rect.GetWidth() * scale_x,  //
                                     rect.GetHeight() * scale_y);

    EXPECT_EQ(rect.Scale(scale_x, scale_y), expected)  //
        << rect << " * " << scale_x << ", " << scale_y;
    EXPECT_EQ(rect.Scale(IPoint(scale_x, scale_y)), expected)  //
        << rect << " * " << scale_x << ", " << scale_y;
    EXPECT_EQ(rect.Scale(ISize(scale_x, scale_y)), expected)  //
        << rect << " * " << scale_x << ", " << scale_y;

    test1(rect, scale_x);
    test1(rect, scale_y);
  };

  test2(IRect::MakeLTRB(10, 15, 100, 150), 2, 3);
  test2(IRect::MakeLTRB(10, 15, 100, 150), 3, 2);
  test2(IRect::MakeLTRB(10, 15, -100, 150), 2, 3);
  test2(IRect::MakeLTRB(10, 15, 100, -150), 2, 3);
  test2(IRect::MakeLTRB(10, 15, 100, 150), -2, 3);
  test2(IRect::MakeLTRB(10, 15, 100, 150), 2, -3);
}

TEST(RectTest, RectArea) {
  EXPECT_EQ(Rect::MakeXYWH(0, 0, 100, 200).Area(), 20000);
  EXPECT_EQ(Rect::MakeXYWH(10, 20, 100, 200).Area(), 20000);
  EXPECT_EQ(Rect::MakeXYWH(0, 0, 200, 100).Area(), 20000);
  EXPECT_EQ(Rect::MakeXYWH(10, 20, 200, 100).Area(), 20000);
  EXPECT_EQ(Rect::MakeXYWH(0, 0, 100, 100).Area(), 10000);
  EXPECT_EQ(Rect::MakeXYWH(10, 20, 100, 100).Area(), 10000);
}

TEST(RectTest, IRectArea) {
  EXPECT_EQ(IRect::MakeXYWH(0, 0, 100, 200).Area(), 20000);
  EXPECT_EQ(IRect::MakeXYWH(10, 20, 100, 200).Area(), 20000);
  EXPECT_EQ(IRect::MakeXYWH(0, 0, 200, 100).Area(), 20000);
  EXPECT_EQ(IRect::MakeXYWH(10, 20, 200, 100).Area(), 20000);
  EXPECT_EQ(IRect::MakeXYWH(0, 0, 100, 100).Area(), 10000);
  EXPECT_EQ(IRect::MakeXYWH(10, 20, 100, 100).Area(), 10000);
}

TEST(RectTest, RectGetNormalizingTransform) {
  {
    // Checks for expected matrix values

    auto r = Rect::MakeXYWH(100, 200, 200, 400);

    EXPECT_EQ(r.GetNormalizingTransform(),
              Matrix::MakeScale({0.005, 0.0025, 1.0}) *
                  Matrix::MakeTranslation({-100, -200}));
  }

  {
    // Checks for expected transform of points relative to the rect

    auto r = Rect::MakeLTRB(300, 500, 400, 700);
    auto m = r.GetNormalizingTransform();

    // The 4 corners of the rect => (0, 0) to (1, 1)
    EXPECT_EQ(m * Point(300, 500), Point(0, 0));
    EXPECT_EQ(m * Point(400, 500), Point(1, 0));
    EXPECT_EQ(m * Point(400, 700), Point(1, 1));
    EXPECT_EQ(m * Point(300, 700), Point(0, 1));

    // The center => (0.5, 0.5)
    EXPECT_EQ(m * Point(350, 600), Point(0.5, 0.5));

    // Outside the 4 corners => (-1, -1) to (2, 2)
    EXPECT_EQ(m * Point(200, 300), Point(-1, -1));
    EXPECT_EQ(m * Point(500, 300), Point(2, -1));
    EXPECT_EQ(m * Point(500, 900), Point(2, 2));
    EXPECT_EQ(m * Point(200, 900), Point(-1, 2));
  }

  {
    // Checks for behavior with empty rects

    auto zero = Matrix::MakeScale({0.0, 0.0, 1.0});

    // Empty for width and/or height == 0
    EXPECT_EQ(Rect::MakeXYWH(10, 10, 0, 10).GetNormalizingTransform(), zero);
    EXPECT_EQ(Rect::MakeXYWH(10, 10, 10, 0).GetNormalizingTransform(), zero);
    EXPECT_EQ(Rect::MakeXYWH(10, 10, 0, 0).GetNormalizingTransform(), zero);

    // Empty for width and/or height < 0
    EXPECT_EQ(Rect::MakeXYWH(10, 10, -1, 10).GetNormalizingTransform(), zero);
    EXPECT_EQ(Rect::MakeXYWH(10, 10, 10, -1).GetNormalizingTransform(), zero);
    EXPECT_EQ(Rect::MakeXYWH(10, 10, -1, -1).GetNormalizingTransform(), zero);
  }

  {
    // Checks for behavior with non-finite rects

    auto z = Matrix::MakeScale({0.0, 0.0, 1.0});
    auto nan = std::numeric_limits<Scalar>::quiet_NaN();
    auto inf = std::numeric_limits<Scalar>::infinity();

    // Non-finite for width and/or height == nan
    EXPECT_EQ(Rect::MakeXYWH(10, 10, nan, 10).GetNormalizingTransform(), z);
    EXPECT_EQ(Rect::MakeXYWH(10, 10, 10, nan).GetNormalizingTransform(), z);
    EXPECT_EQ(Rect::MakeXYWH(10, 10, nan, nan).GetNormalizingTransform(), z);

    // Non-finite for width and/or height == inf
    EXPECT_EQ(Rect::MakeXYWH(10, 10, inf, 10).GetNormalizingTransform(), z);
    EXPECT_EQ(Rect::MakeXYWH(10, 10, 10, inf).GetNormalizingTransform(), z);
    EXPECT_EQ(Rect::MakeXYWH(10, 10, inf, inf).GetNormalizingTransform(), z);

    // Non-finite for width and/or height == -inf
    EXPECT_EQ(Rect::MakeXYWH(10, 10, -inf, 10).GetNormalizingTransform(), z);
    EXPECT_EQ(Rect::MakeXYWH(10, 10, 10, -inf).GetNormalizingTransform(), z);
    EXPECT_EQ(Rect::MakeXYWH(10, 10, -inf, -inf).GetNormalizingTransform(), z);

    // Non-finite for origin X and/or Y == nan
    EXPECT_EQ(Rect::MakeXYWH(nan, 10, 10, 10).GetNormalizingTransform(), z);
    EXPECT_EQ(Rect::MakeXYWH(10, nan, 10, 10).GetNormalizingTransform(), z);
    EXPECT_EQ(Rect::MakeXYWH(nan, nan, 10, 10).GetNormalizingTransform(), z);

    // Non-finite for origin X and/or Y == inf
    EXPECT_EQ(Rect::MakeXYWH(inf, 10, 10, 10).GetNormalizingTransform(), z);
    EXPECT_EQ(Rect::MakeXYWH(10, inf, 10, 10).GetNormalizingTransform(), z);
    EXPECT_EQ(Rect::MakeXYWH(inf, inf, 10, 10).GetNormalizingTransform(), z);

    // Non-finite for origin X and/or Y == -inf
    EXPECT_EQ(Rect::MakeXYWH(-inf, 10, 10, 10).GetNormalizingTransform(), z);
    EXPECT_EQ(Rect::MakeXYWH(10, -inf, 10, 10).GetNormalizingTransform(), z);
    EXPECT_EQ(Rect::MakeXYWH(-inf, -inf, 10, 10).GetNormalizingTransform(), z);
  }
}

TEST(RectTest, IRectGetNormalizingTransform) {
  {
    // Checks for expected matrix values

    auto r = IRect::MakeXYWH(100, 200, 200, 400);

    EXPECT_EQ(r.GetNormalizingTransform(),
              Matrix::MakeScale({0.005, 0.0025, 1.0}) *
                  Matrix::MakeTranslation({-100, -200}));
  }

  {
    // Checks for expected transform of points relative to the rect

    auto r = IRect::MakeLTRB(300, 500, 400, 700);
    auto m = r.GetNormalizingTransform();

    // The 4 corners of the rect => (0, 0) to (1, 1)
    EXPECT_EQ(m * Point(300, 500), Point(0, 0));
    EXPECT_EQ(m * Point(400, 500), Point(1, 0));
    EXPECT_EQ(m * Point(400, 700), Point(1, 1));
    EXPECT_EQ(m * Point(300, 700), Point(0, 1));

    // The center => (0.5, 0.5)
    EXPECT_EQ(m * Point(350, 600), Point(0.5, 0.5));

    // Outside the 4 corners => (-1, -1) to (2, 2)
    EXPECT_EQ(m * Point(200, 300), Point(-1, -1));
    EXPECT_EQ(m * Point(500, 300), Point(2, -1));
    EXPECT_EQ(m * Point(500, 900), Point(2, 2));
    EXPECT_EQ(m * Point(200, 900), Point(-1, 2));
  }

  {
    // Checks for behavior with empty rects

    auto zero = Matrix::MakeScale({0.0, 0.0, 1.0});

    // Empty for width and/or height == 0
    EXPECT_EQ(IRect::MakeXYWH(10, 10, 0, 10).GetNormalizingTransform(), zero);
    EXPECT_EQ(IRect::MakeXYWH(10, 10, 10, 0).GetNormalizingTransform(), zero);
    EXPECT_EQ(IRect::MakeXYWH(10, 10, 0, 0).GetNormalizingTransform(), zero);

    // Empty for width and/or height < 0
    EXPECT_EQ(IRect::MakeXYWH(10, 10, -1, 10).GetNormalizingTransform(), zero);
    EXPECT_EQ(IRect::MakeXYWH(10, 10, 10, -1).GetNormalizingTransform(), zero);
    EXPECT_EQ(IRect::MakeXYWH(10, 10, -1, -1).GetNormalizingTransform(), zero);
  }
}

TEST(RectTest, RectXYWHIsEmpty) {
  auto nan = std::numeric_limits<Scalar>::quiet_NaN();

  // Non-empty
  EXPECT_FALSE(Rect::MakeXYWH(1.5, 2.3, 10.5, 7.2).IsEmpty());

  // Empty both width and height both 0 or negative, in all combinations
  EXPECT_TRUE(Rect::MakeXYWH(1.5, 2.3, 0.0, 0.0).IsEmpty());
  EXPECT_TRUE(Rect::MakeXYWH(1.5, 2.3, -1.0, -1.0).IsEmpty());
  EXPECT_TRUE(Rect::MakeXYWH(1.5, 2.3, 0.0, -1.0).IsEmpty());
  EXPECT_TRUE(Rect::MakeXYWH(1.5, 2.3, -1.0, 0.0).IsEmpty());

  // Empty for 0 or negative width or height (but not both at the same time)
  EXPECT_TRUE(Rect::MakeXYWH(1.5, 2.3, 10.5, 0.0).IsEmpty());
  EXPECT_TRUE(Rect::MakeXYWH(1.5, 2.3, 10.5, -1.0).IsEmpty());
  EXPECT_TRUE(Rect::MakeXYWH(1.5, 2.3, 0.0, 7.2).IsEmpty());
  EXPECT_TRUE(Rect::MakeXYWH(1.5, 2.3, -1.0, 7.2).IsEmpty());

  // Empty for NaN in width or height or both
  EXPECT_TRUE(Rect::MakeXYWH(1.5, 2.3, 10.5, nan).IsEmpty());
  EXPECT_TRUE(Rect::MakeXYWH(1.5, 2.3, nan, 7.2).IsEmpty());
  EXPECT_TRUE(Rect::MakeXYWH(1.5, 2.3, nan, nan).IsEmpty());
}

TEST(RectTest, IRectXYWHIsEmpty) {
  // Non-empty
  EXPECT_FALSE(IRect::MakeXYWH(1, 2, 10, 7).IsEmpty());

  // Empty both width and height both 0 or negative, in all combinations
  EXPECT_TRUE(IRect::MakeXYWH(1, 2, 0, 0).IsEmpty());
  EXPECT_TRUE(IRect::MakeXYWH(1, 2, -1, -1).IsEmpty());
  EXPECT_TRUE(IRect::MakeXYWH(1, 2, -1, 0).IsEmpty());
  EXPECT_TRUE(IRect::MakeXYWH(1, 2, 0, -1).IsEmpty());

  // Empty for 0 or negative width or height (but not both at the same time)
  EXPECT_TRUE(IRect::MakeXYWH(1, 2, 10, 0).IsEmpty());
  EXPECT_TRUE(IRect::MakeXYWH(1, 2, 10, -1).IsEmpty());
  EXPECT_TRUE(IRect::MakeXYWH(1, 2, 0, 7).IsEmpty());
  EXPECT_TRUE(IRect::MakeXYWH(1, 2, -1, 7).IsEmpty());
}

TEST(RectTest, MakePointBoundsQuad) {
  Quad quad = {
      Point(10, 10),
      Point(20, 10),
      Point(10, 20),
      Point(20, 20),
  };
  std::optional<Rect> bounds = Rect::MakePointBounds(quad);
  EXPECT_TRUE(bounds.has_value());
  if (bounds.has_value()) {
    EXPECT_TRUE(RectNear(bounds.value(), Rect::MakeLTRB(10, 10, 20, 20)));
  }
}

TEST(RectTest, IsSquare) {
  EXPECT_TRUE(Rect::MakeXYWH(10, 30, 20, 20).IsSquare());
  EXPECT_FALSE(Rect::MakeXYWH(10, 30, 20, 19).IsSquare());
  EXPECT_FALSE(Rect::MakeXYWH(10, 30, 19, 20).IsSquare());
  EXPECT_TRUE(Rect::MakeMaximum().IsSquare());

  EXPECT_TRUE(IRect::MakeXYWH(10, 30, 20, 20).IsSquare());
  EXPECT_FALSE(IRect::MakeXYWH(10, 30, 20, 19).IsSquare());
  EXPECT_FALSE(IRect::MakeXYWH(10, 30, 19, 20).IsSquare());
  EXPECT_TRUE(IRect::MakeMaximum().IsSquare());
}

TEST(RectTest, GetCenter) {
  EXPECT_EQ(Rect::MakeXYWH(10, 30, 20, 20).GetCenter(), Point(20, 40));
  EXPECT_EQ(Rect::MakeXYWH(10, 30, 20, 19).GetCenter(), Point(20, 39.5));
  EXPECT_EQ(Rect::MakeMaximum().GetCenter(), Point(0, 0));

  // Note that we expect a Point as the answer from an IRect
  EXPECT_EQ(IRect::MakeXYWH(10, 30, 20, 20).GetCenter(), Point(20, 40));
  EXPECT_EQ(IRect::MakeXYWH(10, 30, 20, 19).GetCenter(), Point(20, 39.5));
  EXPECT_EQ(IRect::MakeMaximum().GetCenter(), Point(0, 0));
}

TEST(RectTest, RectExpand) {
  auto rect = Rect::MakeLTRB(100, 100, 200, 200);

  // Expand(T amount)
  EXPECT_EQ(rect.Expand(10), Rect::MakeLTRB(90, 90, 210, 210));
  EXPECT_EQ(rect.Expand(-10), Rect::MakeLTRB(110, 110, 190, 190));

  // Expand(amount, amount)
  EXPECT_EQ(rect.Expand(10, 10), Rect::MakeLTRB(90, 90, 210, 210));
  EXPECT_EQ(rect.Expand(10, -10), Rect::MakeLTRB(90, 110, 210, 190));
  EXPECT_EQ(rect.Expand(-10, 10), Rect::MakeLTRB(110, 90, 190, 210));
  EXPECT_EQ(rect.Expand(-10, -10), Rect::MakeLTRB(110, 110, 190, 190));

  // Expand(amount, amount, amount, amount)
  EXPECT_EQ(rect.Expand(10, 20, 30, 40), Rect::MakeLTRB(90, 80, 230, 240));
  EXPECT_EQ(rect.Expand(-10, 20, 30, 40), Rect::MakeLTRB(110, 80, 230, 240));
  EXPECT_EQ(rect.Expand(10, -20, 30, 40), Rect::MakeLTRB(90, 120, 230, 240));
  EXPECT_EQ(rect.Expand(10, 20, -30, 40), Rect::MakeLTRB(90, 80, 170, 240));
  EXPECT_EQ(rect.Expand(10, 20, 30, -40), Rect::MakeLTRB(90, 80, 230, 160));

  // Expand(Point amount)
  EXPECT_EQ(rect.Expand(Point{10, 10}), Rect::MakeLTRB(90, 90, 210, 210));
  EXPECT_EQ(rect.Expand(Point{10, -10}), Rect::MakeLTRB(90, 110, 210, 190));
  EXPECT_EQ(rect.Expand(Point{-10, 10}), Rect::MakeLTRB(110, 90, 190, 210));
  EXPECT_EQ(rect.Expand(Point{-10, -10}), Rect::MakeLTRB(110, 110, 190, 190));

  // Expand(Size amount)
  EXPECT_EQ(rect.Expand(Size{10, 10}), Rect::MakeLTRB(90, 90, 210, 210));
  EXPECT_EQ(rect.Expand(Size{10, -10}), Rect::MakeLTRB(90, 110, 210, 190));
  EXPECT_EQ(rect.Expand(Size{-10, 10}), Rect::MakeLTRB(110, 90, 190, 210));
  EXPECT_EQ(rect.Expand(Size{-10, -10}), Rect::MakeLTRB(110, 110, 190, 190));
}

TEST(RectTest, IRectExpand) {
  auto rect = IRect::MakeLTRB(100, 100, 200, 200);

  // Expand(T amount)
  EXPECT_EQ(rect.Expand(10), IRect::MakeLTRB(90, 90, 210, 210));
  EXPECT_EQ(rect.Expand(-10), IRect::MakeLTRB(110, 110, 190, 190));

  // Expand(amount, amount)
  EXPECT_EQ(rect.Expand(10, 10), IRect::MakeLTRB(90, 90, 210, 210));
  EXPECT_EQ(rect.Expand(10, -10), IRect::MakeLTRB(90, 110, 210, 190));
  EXPECT_EQ(rect.Expand(-10, 10), IRect::MakeLTRB(110, 90, 190, 210));
  EXPECT_EQ(rect.Expand(-10, -10), IRect::MakeLTRB(110, 110, 190, 190));

  // Expand(amount, amount, amount, amount)
  EXPECT_EQ(rect.Expand(10, 20, 30, 40), IRect::MakeLTRB(90, 80, 230, 240));
  EXPECT_EQ(rect.Expand(-10, 20, 30, 40), IRect::MakeLTRB(110, 80, 230, 240));
  EXPECT_EQ(rect.Expand(10, -20, 30, 40), IRect::MakeLTRB(90, 120, 230, 240));
  EXPECT_EQ(rect.Expand(10, 20, -30, 40), IRect::MakeLTRB(90, 80, 170, 240));
  EXPECT_EQ(rect.Expand(10, 20, 30, -40), IRect::MakeLTRB(90, 80, 230, 160));

  // Expand(IPoint amount)
  EXPECT_EQ(rect.Expand(IPoint{10, 10}), IRect::MakeLTRB(90, 90, 210, 210));
  EXPECT_EQ(rect.Expand(IPoint{10, -10}), IRect::MakeLTRB(90, 110, 210, 190));
  EXPECT_EQ(rect.Expand(IPoint{-10, 10}), IRect::MakeLTRB(110, 90, 190, 210));
  EXPECT_EQ(rect.Expand(IPoint{-10, -10}), IRect::MakeLTRB(110, 110, 190, 190));

  // Expand(ISize amount)
  EXPECT_EQ(rect.Expand(ISize{10, 10}), IRect::MakeLTRB(90, 90, 210, 210));
  EXPECT_EQ(rect.Expand(ISize{10, -10}), IRect::MakeLTRB(90, 110, 210, 190));
  EXPECT_EQ(rect.Expand(ISize{-10, 10}), IRect::MakeLTRB(110, 90, 190, 210));
  EXPECT_EQ(rect.Expand(ISize{-10, -10}), IRect::MakeLTRB(110, 110, 190, 190));
}

TEST(RectTest, ContainsFloatingPoint) {
  auto rect1 =
      Rect::MakeXYWH(472.599945f, 440.999969f, 1102.80005f, 654.000061f);
  auto rect2 = Rect::MakeXYWH(724.f, 618.f, 600.f, 300.f);
  EXPECT_TRUE(rect1.Contains(rect2));
}

template <typename R>
static constexpr inline R flip_lr(R rect) {
  return R::MakeLTRB(rect.GetRight(), rect.GetTop(),  //
                     rect.GetLeft(), rect.GetBottom());
}

template <typename R>
static constexpr inline R flip_tb(R rect) {
  return R::MakeLTRB(rect.GetLeft(), rect.GetBottom(),  //
                     rect.GetRight(), rect.GetTop());
}

template <typename R>
static constexpr inline R flip_lrtb(R rect) {
  return flip_lr(flip_tb(rect));
}

static constexpr inline Rect swap_nan(const Rect& rect, int index) {
  Scalar nan = std::numeric_limits<Scalar>::quiet_NaN();
  FML_DCHECK(index >= 0 && index <= 15);
  Scalar l = ((index & (1 << 0)) != 0) ? nan : rect.GetLeft();
  Scalar t = ((index & (1 << 1)) != 0) ? nan : rect.GetTop();
  Scalar r = ((index & (1 << 2)) != 0) ? nan : rect.GetRight();
  Scalar b = ((index & (1 << 3)) != 0) ? nan : rect.GetBottom();
  return Rect::MakeLTRB(l, t, r, b);
}

static constexpr inline Point swap_nan(const Point& point, int index) {
  Scalar nan = std::numeric_limits<Scalar>::quiet_NaN();
  FML_DCHECK(index >= 0 && index <= 3);
  Scalar x = ((index & (1 << 0)) != 0) ? nan : point.x;
  Scalar y = ((index & (1 << 1)) != 0) ? nan : point.y;
  return Point(x, y);
}

TEST(RectTest, RectUnion) {
  auto check_nans = [](const Rect& a, const Rect& b, const std::string& label) {
    ASSERT_TRUE(a.IsFinite()) << label;
    ASSERT_TRUE(b.IsFinite()) << label;
    ASSERT_FALSE(a.Union(b).IsEmpty());

    for (int i = 1; i < 16; i++) {
      // NaN in a produces b
      EXPECT_EQ(swap_nan(a, i).Union(b), b) << label << ", index = " << i;
      // NaN in b produces a
      EXPECT_EQ(a.Union(swap_nan(b, i)), a) << label << ", index = " << i;
      // NaN in both is empty
      for (int j = 1; j < 16; j++) {
        EXPECT_TRUE(swap_nan(a, i).Union(swap_nan(b, j)).IsEmpty())
            << label << ", indices = " << i << ", " << j;
      }
    }
  };

  auto check_empty_flips = [](const Rect& a, const Rect& b,
                              const std::string& label) {
    ASSERT_FALSE(a.IsEmpty());
    // b is allowed to be empty

    // unflipped a vs flipped (empty) b yields a
    EXPECT_EQ(a.Union(flip_lr(b)), a) << label;
    EXPECT_EQ(a.Union(flip_tb(b)), a) << label;
    EXPECT_EQ(a.Union(flip_lrtb(b)), a) << label;

    // flipped (empty) a vs unflipped b yields b
    EXPECT_EQ(flip_lr(a).Union(b), b) << label;
    EXPECT_EQ(flip_tb(a).Union(b), b) << label;
    EXPECT_EQ(flip_lrtb(a).Union(b), b) << label;

    // flipped (empty) a vs flipped (empty) b yields empty
    EXPECT_TRUE(flip_lr(a).Union(flip_lr(b)).IsEmpty()) << label;
    EXPECT_TRUE(flip_tb(a).Union(flip_tb(b)).IsEmpty()) << label;
    EXPECT_TRUE(flip_lrtb(a).Union(flip_lrtb(b)).IsEmpty()) << label;
  };

  auto test = [&check_nans, &check_empty_flips](const Rect& a, const Rect& b,
                                                const Rect& result) {
    ASSERT_FALSE(a.IsEmpty()) << a;
    // b is allowed to be empty

    std::stringstream stream;
    stream << a << " union " << b;
    auto label = stream.str();

    EXPECT_EQ(a.Union(b), result) << label;
    EXPECT_EQ(b.Union(a), result) << label;
    check_empty_flips(a, b, label);
    check_nans(a, b, label);
  };

  {
    auto a = Rect::MakeXYWH(100, 100, 100, 100);
    auto b = Rect::MakeXYWH(0, 0, 0, 0);
    auto expected = Rect::MakeXYWH(100, 100, 100, 100);
    test(a, b, expected);
  }

  {
    auto a = Rect::MakeXYWH(100, 100, 100, 100);
    auto b = Rect::MakeXYWH(0, 0, 1, 1);
    auto expected = Rect::MakeXYWH(0, 0, 200, 200);
    test(a, b, expected);
  }

  {
    auto a = Rect::MakeXYWH(100, 100, 100, 100);
    auto b = Rect::MakeXYWH(10, 10, 1, 1);
    auto expected = Rect::MakeXYWH(10, 10, 190, 190);
    test(a, b, expected);
  }

  {
    auto a = Rect::MakeXYWH(0, 0, 100, 100);
    auto b = Rect::MakeXYWH(10, 10, 100, 100);
    auto expected = Rect::MakeXYWH(0, 0, 110, 110);
    test(a, b, expected);
  }

  {
    auto a = Rect::MakeXYWH(0, 0, 100, 100);
    auto b = Rect::MakeXYWH(100, 100, 100, 100);
    auto expected = Rect::MakeXYWH(0, 0, 200, 200);
    test(a, b, expected);
  }
}

TEST(RectTest, OptRectUnion) {
  auto a = Rect::MakeLTRB(0, 0, 100, 100);
  auto b = Rect::MakeLTRB(100, 100, 200, 200);
  auto c = Rect::MakeLTRB(100, 0, 200, 100);

  // NullOpt, NullOpt
  EXPECT_FALSE(Rect::Union(std::nullopt, std::nullopt).has_value());
  EXPECT_EQ(Rect::Union(std::nullopt, std::nullopt), std::nullopt);

  auto test1 = [](const Rect& r) {
    // Rect, NullOpt
    EXPECT_TRUE(Rect::Union(r, std::nullopt).has_value());
    EXPECT_EQ(Rect::Union(r, std::nullopt).value(), r);

    // OptRect, NullOpt
    EXPECT_TRUE(Rect::Union(std::optional(r), std::nullopt).has_value());
    EXPECT_EQ(Rect::Union(std::optional(r), std::nullopt).value(), r);

    // NullOpt, Rect
    EXPECT_TRUE(Rect::Union(std::nullopt, r).has_value());
    EXPECT_EQ(Rect::Union(std::nullopt, r).value(), r);

    // NullOpt, OptRect
    EXPECT_TRUE(Rect::Union(std::nullopt, std::optional(r)).has_value());
    EXPECT_EQ(Rect::Union(std::nullopt, std::optional(r)).value(), r);
  };

  test1(a);
  test1(b);
  test1(c);

  auto test2 = [](const Rect& a, const Rect& b, const Rect& u) {
    ASSERT_EQ(a.Union(b), u);

    // Rect, OptRect
    EXPECT_TRUE(Rect::Union(a, std::optional(b)).has_value());
    EXPECT_EQ(Rect::Union(a, std::optional(b)).value(), u);

    // OptRect, Rect
    EXPECT_TRUE(Rect::Union(std::optional(a), b).has_value());
    EXPECT_EQ(Rect::Union(std::optional(a), b).value(), u);

    // OptRect, OptRect
    EXPECT_TRUE(Rect::Union(std::optional(a), std::optional(b)).has_value());
    EXPECT_EQ(Rect::Union(std::optional(a), std::optional(b)).value(), u);
  };

  test2(a, b, Rect::MakeLTRB(0, 0, 200, 200));
  test2(a, c, Rect::MakeLTRB(0, 0, 200, 100));
  test2(b, c, Rect::MakeLTRB(100, 0, 200, 200));
}

TEST(RectTest, IRectUnion) {
  auto check_empty_flips = [](const IRect& a, const IRect& b,
                              const std::string& label) {
    ASSERT_FALSE(a.IsEmpty());
    // b is allowed to be empty

    // unflipped a vs flipped (empty) b yields a
    EXPECT_EQ(a.Union(flip_lr(b)), a) << label;
    EXPECT_EQ(a.Union(flip_tb(b)), a) << label;
    EXPECT_EQ(a.Union(flip_lrtb(b)), a) << label;

    // flipped (empty) a vs unflipped b yields b
    EXPECT_EQ(flip_lr(a).Union(b), b) << label;
    EXPECT_EQ(flip_tb(a).Union(b), b) << label;
    EXPECT_EQ(flip_lrtb(a).Union(b), b) << label;

    // flipped (empty) a vs flipped (empty) b yields empty
    EXPECT_TRUE(flip_lr(a).Union(flip_lr(b)).IsEmpty()) << label;
    EXPECT_TRUE(flip_tb(a).Union(flip_tb(b)).IsEmpty()) << label;
    EXPECT_TRUE(flip_lrtb(a).Union(flip_lrtb(b)).IsEmpty()) << label;
  };

  auto test = [&check_empty_flips](const IRect& a, const IRect& b,
                                   const IRect& result) {
    ASSERT_FALSE(a.IsEmpty()) << a;
    // b is allowed to be empty

    std::stringstream stream;
    stream << a << " union " << b;
    auto label = stream.str();

    EXPECT_EQ(a.Union(b), result) << label;
    EXPECT_EQ(b.Union(a), result) << label;
    check_empty_flips(a, b, label);
  };

  {
    auto a = IRect::MakeXYWH(100, 100, 100, 100);
    auto b = IRect::MakeXYWH(0, 0, 0, 0);
    auto expected = IRect::MakeXYWH(100, 100, 100, 100);
    test(a, b, expected);
  }

  {
    auto a = IRect::MakeXYWH(100, 100, 100, 100);
    auto b = IRect::MakeXYWH(0, 0, 1, 1);
    auto expected = IRect::MakeXYWH(0, 0, 200, 200);
    test(a, b, expected);
  }

  {
    auto a = IRect::MakeXYWH(100, 100, 100, 100);
    auto b = IRect::MakeXYWH(10, 10, 1, 1);
    auto expected = IRect::MakeXYWH(10, 10, 190, 190);
    test(a, b, expected);
  }

  {
    auto a = IRect::MakeXYWH(0, 0, 100, 100);
    auto b = IRect::MakeXYWH(10, 10, 100, 100);
    auto expected = IRect::MakeXYWH(0, 0, 110, 110);
    test(a, b, expected);
  }

  {
    auto a = IRect::MakeXYWH(0, 0, 100, 100);
    auto b = IRect::MakeXYWH(100, 100, 100, 100);
    auto expected = IRect::MakeXYWH(0, 0, 200, 200);
    test(a, b, expected);
  }
}

TEST(RectTest, OptIRectUnion) {
  auto a = IRect::MakeLTRB(0, 0, 100, 100);
  auto b = IRect::MakeLTRB(100, 100, 200, 200);
  auto c = IRect::MakeLTRB(100, 0, 200, 100);

  // NullOpt, NullOpt
  EXPECT_FALSE(IRect::Union(std::nullopt, std::nullopt).has_value());
  EXPECT_EQ(IRect::Union(std::nullopt, std::nullopt), std::nullopt);

  auto test1 = [](const IRect& r) {
    // Rect, NullOpt
    EXPECT_TRUE(IRect::Union(r, std::nullopt).has_value());
    EXPECT_EQ(IRect::Union(r, std::nullopt).value(), r);

    // OptRect, NullOpt
    EXPECT_TRUE(IRect::Union(std::optional(r), std::nullopt).has_value());
    EXPECT_EQ(IRect::Union(std::optional(r), std::nullopt).value(), r);

    // NullOpt, Rect
    EXPECT_TRUE(IRect::Union(std::nullopt, r).has_value());
    EXPECT_EQ(IRect::Union(std::nullopt, r).value(), r);

    // NullOpt, OptRect
    EXPECT_TRUE(IRect::Union(std::nullopt, std::optional(r)).has_value());
    EXPECT_EQ(IRect::Union(std::nullopt, std::optional(r)).value(), r);
  };

  test1(a);
  test1(b);
  test1(c);

  auto test2 = [](const IRect& a, const IRect& b, const IRect& u) {
    ASSERT_EQ(a.Union(b), u);

    // Rect, OptRect
    EXPECT_TRUE(IRect::Union(a, std::optional(b)).has_value());
    EXPECT_EQ(IRect::Union(a, std::optional(b)).value(), u);

    // OptRect, Rect
    EXPECT_TRUE(IRect::Union(std::optional(a), b).has_value());
    EXPECT_EQ(IRect::Union(std::optional(a), b).value(), u);

    // OptRect, OptRect
    EXPECT_TRUE(IRect::Union(std::optional(a), std::optional(b)).has_value());
    EXPECT_EQ(IRect::Union(std::optional(a), std::optional(b)).value(), u);
  };

  test2(a, b, IRect::MakeLTRB(0, 0, 200, 200));
  test2(a, c, IRect::MakeLTRB(0, 0, 200, 100));
  test2(b, c, IRect::MakeLTRB(100, 0, 200, 200));
}

TEST(RectTest, RectIntersection) {
  auto check_nans = [](const Rect& a, const Rect& b, const std::string& label) {
    ASSERT_TRUE(a.IsFinite()) << label;
    ASSERT_TRUE(b.IsFinite()) << label;

    for (int i = 1; i < 16; i++) {
      // NaN in a produces empty
      EXPECT_FALSE(swap_nan(a, i).Intersection(b).has_value())
          << label << ", index = " << i;
      // NaN in b produces empty
      EXPECT_FALSE(a.Intersection(swap_nan(b, i)).has_value())
          << label << ", index = " << i;
      // NaN in both is empty
      for (int j = 1; j < 16; j++) {
        EXPECT_FALSE(swap_nan(a, i).Intersection(swap_nan(b, j)).has_value())
            << label << ", indices = " << i << ", " << j;
      }
    }
  };

  auto check_empty_flips = [](const Rect& a, const Rect& b,
                              const std::string& label) {
    ASSERT_FALSE(a.IsEmpty());
    // b is allowed to be empty

    // unflipped a vs flipped (empty) b yields a
    EXPECT_FALSE(a.Intersection(flip_lr(b)).has_value()) << label;
    EXPECT_TRUE(a.IntersectionOrEmpty(flip_lr(b)).IsEmpty()) << label;
    EXPECT_FALSE(a.Intersection(flip_tb(b)).has_value()) << label;
    EXPECT_TRUE(a.IntersectionOrEmpty(flip_tb(b)).IsEmpty()) << label;
    EXPECT_FALSE(a.Intersection(flip_lrtb(b)).has_value()) << label;
    EXPECT_TRUE(a.IntersectionOrEmpty(flip_lrtb(b)).IsEmpty()) << label;

    // flipped (empty) a vs unflipped b yields b
    EXPECT_FALSE(flip_lr(a).Intersection(b).has_value()) << label;
    EXPECT_TRUE(flip_lr(a).IntersectionOrEmpty(b).IsEmpty()) << label;
    EXPECT_FALSE(flip_tb(a).Intersection(b).has_value()) << label;
    EXPECT_TRUE(flip_tb(a).IntersectionOrEmpty(b).IsEmpty()) << label;
    EXPECT_FALSE(flip_lrtb(a).Intersection(b).has_value()) << label;
    EXPECT_TRUE(flip_lrtb(a).IntersectionOrEmpty(b).IsEmpty()) << label;

    // flipped (empty) a vs flipped (empty) b yields empty
    EXPECT_FALSE(flip_lr(a).Intersection(flip_lr(b)).has_value()) << label;
    EXPECT_TRUE(flip_lr(a).IntersectionOrEmpty(flip_lr(b)).IsEmpty()) << label;
    EXPECT_FALSE(flip_tb(a).Intersection(flip_tb(b)).has_value()) << label;
    EXPECT_TRUE(flip_tb(a).IntersectionOrEmpty(flip_tb(b)).IsEmpty()) << label;
    EXPECT_FALSE(flip_lrtb(a).Intersection(flip_lrtb(b)).has_value()) << label;
    EXPECT_TRUE(flip_lrtb(a).IntersectionOrEmpty(flip_lrtb(b)).IsEmpty())
        << label;
  };

  auto test_non_empty = [&check_nans, &check_empty_flips](
                            const Rect& a, const Rect& b, const Rect& result) {
    ASSERT_FALSE(a.IsEmpty()) << a;
    // b is allowed to be empty

    std::stringstream stream;
    stream << a << " union " << b;
    auto label = stream.str();

    EXPECT_TRUE(a.Intersection(b).has_value()) << label;
    EXPECT_TRUE(b.Intersection(a).has_value()) << label;
    EXPECT_EQ(a.Intersection(b), result) << label;
    EXPECT_EQ(b.Intersection(a), result) << label;
    check_empty_flips(a, b, label);
    check_nans(a, b, label);
  };

  auto test_empty = [&check_nans, &check_empty_flips](const Rect& a,
                                                      const Rect& b) {
    ASSERT_FALSE(a.IsEmpty()) << a;
    // b is allowed to be empty

    std::stringstream stream;
    stream << a << " union " << b;
    auto label = stream.str();

    EXPECT_FALSE(a.Intersection(b).has_value()) << label;
    EXPECT_TRUE(a.IntersectionOrEmpty(b).IsEmpty()) << label;
    EXPECT_FALSE(b.Intersection(a).has_value()) << label;
    EXPECT_TRUE(b.IntersectionOrEmpty(a).IsEmpty()) << label;
    check_empty_flips(a, b, label);
    check_nans(a, b, label);
  };

  {
    auto a = Rect::MakeXYWH(100, 100, 100, 100);
    auto b = Rect::MakeXYWH(0, 0, 0, 0);

    test_empty(a, b);
  }

  {
    auto a = Rect::MakeXYWH(100, 100, 100, 100);
    auto b = Rect::MakeXYWH(10, 10, 0, 0);

    test_empty(a, b);
  }

  {
    auto a = Rect::MakeXYWH(0, 0, 100, 100);
    auto b = Rect::MakeXYWH(10, 10, 100, 100);
    auto expected = Rect::MakeXYWH(10, 10, 90, 90);

    test_non_empty(a, b, expected);
  }

  {
    auto a = Rect::MakeXYWH(0, 0, 100, 100);
    auto b = Rect::MakeXYWH(100, 100, 100, 100);

    test_empty(a, b);
  }

  {
    auto a = Rect::MakeMaximum();
    auto b = Rect::MakeXYWH(10, 10, 300, 300);

    test_non_empty(a, b, b);
  }

  {
    auto a = Rect::MakeMaximum();
    auto b = Rect::MakeMaximum();

    test_non_empty(a, b, Rect::MakeMaximum());
  }
}

TEST(RectTest, OptRectIntersection) {
  auto a = Rect::MakeLTRB(0, 0, 110, 110);
  auto b = Rect::MakeLTRB(100, 100, 200, 200);
  auto c = Rect::MakeLTRB(100, 0, 200, 110);

  // NullOpt, NullOpt
  EXPECT_FALSE(Rect::Intersection(std::nullopt, std::nullopt).has_value());
  EXPECT_EQ(Rect::Intersection(std::nullopt, std::nullopt), std::nullopt);

  auto test1 = [](const Rect& r) {
    // Rect, NullOpt
    EXPECT_TRUE(Rect::Intersection(r, std::nullopt).has_value());
    EXPECT_EQ(Rect::Intersection(r, std::nullopt).value(), r);

    // OptRect, NullOpt
    EXPECT_TRUE(Rect::Intersection(std::optional(r), std::nullopt).has_value());
    EXPECT_EQ(Rect::Intersection(std::optional(r), std::nullopt).value(), r);

    // NullOpt, Rect
    EXPECT_TRUE(Rect::Intersection(std::nullopt, r).has_value());
    EXPECT_EQ(Rect::Intersection(std::nullopt, r).value(), r);

    // NullOpt, OptRect
    EXPECT_TRUE(Rect::Intersection(std::nullopt, std::optional(r)).has_value());
    EXPECT_EQ(Rect::Intersection(std::nullopt, std::optional(r)).value(), r);
  };

  test1(a);
  test1(b);
  test1(c);

  auto test2 = [](const Rect& a, const Rect& b, const Rect& i) {
    ASSERT_EQ(a.Intersection(b), i);

    // Rect, OptRect
    EXPECT_TRUE(Rect::Intersection(a, std::optional(b)).has_value());
    EXPECT_EQ(Rect::Intersection(a, std::optional(b)).value(), i);

    // OptRect, Rect
    EXPECT_TRUE(Rect::Intersection(std::optional(a), b).has_value());
    EXPECT_EQ(Rect::Intersection(std::optional(a), b).value(), i);

    // OptRect, OptRect
    EXPECT_TRUE(
        Rect::Intersection(std::optional(a), std::optional(b)).has_value());
    EXPECT_EQ(Rect::Intersection(std::optional(a), std::optional(b)).value(),
              i);
  };

  test2(a, b, Rect::MakeLTRB(100, 100, 110, 110));
  test2(a, c, Rect::MakeLTRB(100, 0, 110, 110));
  test2(b, c, Rect::MakeLTRB(100, 100, 200, 110));
}

TEST(RectTest, IRectIntersection) {
  auto check_empty_flips = [](const IRect& a, const IRect& b,
                              const std::string& label) {
    ASSERT_FALSE(a.IsEmpty());
    // b is allowed to be empty

    // unflipped a vs flipped (empty) b yields a
    EXPECT_FALSE(a.Intersection(flip_lr(b)).has_value()) << label;
    EXPECT_FALSE(a.Intersection(flip_tb(b)).has_value()) << label;
    EXPECT_FALSE(a.Intersection(flip_lrtb(b)).has_value()) << label;

    // flipped (empty) a vs unflipped b yields b
    EXPECT_FALSE(flip_lr(a).Intersection(b).has_value()) << label;
    EXPECT_FALSE(flip_tb(a).Intersection(b).has_value()) << label;
    EXPECT_FALSE(flip_lrtb(a).Intersection(b).has_value()) << label;

    // flipped (empty) a vs flipped (empty) b yields empty
    EXPECT_FALSE(flip_lr(a).Intersection(flip_lr(b)).has_value()) << label;
    EXPECT_FALSE(flip_tb(a).Intersection(flip_tb(b)).has_value()) << label;
    EXPECT_FALSE(flip_lrtb(a).Intersection(flip_lrtb(b)).has_value()) << label;
  };

  auto test_non_empty = [&check_empty_flips](const IRect& a, const IRect& b,
                                             const IRect& result) {
    ASSERT_FALSE(a.IsEmpty()) << a;
    // b is allowed to be empty

    std::stringstream stream;
    stream << a << " union " << b;
    auto label = stream.str();

    EXPECT_TRUE(a.Intersection(b).has_value()) << label;
    EXPECT_TRUE(b.Intersection(a).has_value()) << label;
    EXPECT_EQ(a.Intersection(b), result) << label;
    EXPECT_EQ(b.Intersection(a), result) << label;
    check_empty_flips(a, b, label);
  };

  auto test_empty = [&check_empty_flips](const IRect& a, const IRect& b) {
    ASSERT_FALSE(a.IsEmpty()) << a;
    // b is allowed to be empty

    std::stringstream stream;
    stream << a << " union " << b;
    auto label = stream.str();

    EXPECT_FALSE(a.Intersection(b).has_value()) << label;
    EXPECT_FALSE(b.Intersection(a).has_value()) << label;
    check_empty_flips(a, b, label);
  };

  {
    auto a = IRect::MakeXYWH(100, 100, 100, 100);
    auto b = IRect::MakeXYWH(0, 0, 0, 0);

    test_empty(a, b);
  }

  {
    auto a = IRect::MakeXYWH(100, 100, 100, 100);
    auto b = IRect::MakeXYWH(10, 10, 0, 0);

    test_empty(a, b);
  }

  {
    auto a = IRect::MakeXYWH(0, 0, 100, 100);
    auto b = IRect::MakeXYWH(10, 10, 100, 100);
    auto expected = IRect::MakeXYWH(10, 10, 90, 90);

    test_non_empty(a, b, expected);
  }

  {
    auto a = IRect::MakeXYWH(0, 0, 100, 100);
    auto b = IRect::MakeXYWH(100, 100, 100, 100);

    test_empty(a, b);
  }

  {
    auto a = IRect::MakeMaximum();
    auto b = IRect::MakeXYWH(10, 10, 300, 300);

    test_non_empty(a, b, b);
  }

  {
    auto a = IRect::MakeMaximum();
    auto b = IRect::MakeMaximum();

    test_non_empty(a, b, IRect::MakeMaximum());
  }
}

TEST(RectTest, OptIRectIntersection) {
  auto a = IRect::MakeLTRB(0, 0, 110, 110);
  auto b = IRect::MakeLTRB(100, 100, 200, 200);
  auto c = IRect::MakeLTRB(100, 0, 200, 110);

  // NullOpt, NullOpt
  EXPECT_FALSE(IRect::Intersection(std::nullopt, std::nullopt).has_value());
  EXPECT_EQ(IRect::Intersection(std::nullopt, std::nullopt), std::nullopt);

  auto test1 = [](const IRect& r) {
    // Rect, NullOpt
    EXPECT_TRUE(IRect::Intersection(r, std::nullopt).has_value());
    EXPECT_EQ(IRect::Intersection(r, std::nullopt).value(), r);

    // OptRect, NullOpt
    EXPECT_TRUE(
        IRect::Intersection(std::optional(r), std::nullopt).has_value());
    EXPECT_EQ(IRect::Intersection(std::optional(r), std::nullopt).value(), r);

    // NullOpt, Rect
    EXPECT_TRUE(IRect::Intersection(std::nullopt, r).has_value());
    EXPECT_EQ(IRect::Intersection(std::nullopt, r).value(), r);

    // NullOpt, OptRect
    EXPECT_TRUE(
        IRect::Intersection(std::nullopt, std::optional(r)).has_value());
    EXPECT_EQ(IRect::Intersection(std::nullopt, std::optional(r)).value(), r);
  };

  test1(a);
  test1(b);
  test1(c);

  auto test2 = [](const IRect& a, const IRect& b, const IRect& i) {
    ASSERT_EQ(a.Intersection(b), i);

    // Rect, OptRect
    EXPECT_TRUE(IRect::Intersection(a, std::optional(b)).has_value());
    EXPECT_EQ(IRect::Intersection(a, std::optional(b)).value(), i);

    // OptRect, Rect
    EXPECT_TRUE(IRect::Intersection(std::optional(a), b).has_value());
    EXPECT_EQ(IRect::Intersection(std::optional(a), b).value(), i);

    // OptRect, OptRect
    EXPECT_TRUE(
        IRect::Intersection(std::optional(a), std::optional(b)).has_value());
    EXPECT_EQ(IRect::Intersection(std::optional(a), std::optional(b)).value(),
              i);
  };

  test2(a, b, IRect::MakeLTRB(100, 100, 110, 110));
  test2(a, c, IRect::MakeLTRB(100, 0, 110, 110));
  test2(b, c, IRect::MakeLTRB(100, 100, 200, 110));
}

TEST(RectTest, RectIntersectsWithRect) {
  auto check_nans = [](const Rect& a, const Rect& b, const std::string& label) {
    ASSERT_TRUE(a.IsFinite()) << label;
    ASSERT_TRUE(b.IsFinite()) << label;

    for (int i = 1; i < 16; i++) {
      // NaN in a produces b
      EXPECT_FALSE(swap_nan(a, i).IntersectsWithRect(b))
          << label << ", index = " << i;
      // NaN in b produces a
      EXPECT_FALSE(a.IntersectsWithRect(swap_nan(b, i)))
          << label << ", index = " << i;
      // NaN in both is empty
      for (int j = 1; j < 16; j++) {
        EXPECT_FALSE(swap_nan(a, i).IntersectsWithRect(swap_nan(b, j)))
            << label << ", indices = " << i << ", " << j;
      }
    }
  };

  auto check_empty_flips = [](const Rect& a, const Rect& b,
                              const std::string& label) {
    ASSERT_FALSE(a.IsEmpty());
    // b is allowed to be empty

    // unflipped a vs flipped (empty) b yields a
    EXPECT_FALSE(a.IntersectsWithRect(flip_lr(b))) << label;
    EXPECT_FALSE(a.IntersectsWithRect(flip_tb(b))) << label;
    EXPECT_FALSE(a.IntersectsWithRect(flip_lrtb(b))) << label;

    // flipped (empty) a vs unflipped b yields b
    EXPECT_FALSE(flip_lr(a).IntersectsWithRect(b)) << label;
    EXPECT_FALSE(flip_tb(a).IntersectsWithRect(b)) << label;
    EXPECT_FALSE(flip_lrtb(a).IntersectsWithRect(b)) << label;

    // flipped (empty) a vs flipped (empty) b yields empty
    EXPECT_FALSE(flip_lr(a).IntersectsWithRect(flip_lr(b))) << label;
    EXPECT_FALSE(flip_tb(a).IntersectsWithRect(flip_tb(b))) << label;
    EXPECT_FALSE(flip_lrtb(a).IntersectsWithRect(flip_lrtb(b))) << label;
  };

  auto test_non_empty = [&check_nans, &check_empty_flips](const Rect& a,
                                                          const Rect& b) {
    ASSERT_FALSE(a.IsEmpty()) << a;
    // b is allowed to be empty

    std::stringstream stream;
    stream << a << " union " << b;
    auto label = stream.str();

    EXPECT_TRUE(a.IntersectsWithRect(b)) << label;
    EXPECT_TRUE(b.IntersectsWithRect(a)) << label;
    check_empty_flips(a, b, label);
    check_nans(a, b, label);
  };

  auto test_empty = [&check_nans, &check_empty_flips](const Rect& a,
                                                      const Rect& b) {
    ASSERT_FALSE(a.IsEmpty()) << a;
    // b is allowed to be empty

    std::stringstream stream;
    stream << a << " union " << b;
    auto label = stream.str();

    EXPECT_FALSE(a.IntersectsWithRect(b)) << label;
    EXPECT_FALSE(b.IntersectsWithRect(a)) << label;
    check_empty_flips(a, b, label);
    check_nans(a, b, label);
  };

  {
    auto a = Rect::MakeXYWH(100, 100, 100, 100);
    auto b = Rect::MakeXYWH(0, 0, 0, 0);

    test_empty(a, b);
  }

  {
    auto a = Rect::MakeXYWH(100, 100, 100, 100);
    auto b = Rect::MakeXYWH(10, 10, 0, 0);

    test_empty(a, b);
  }

  {
    auto a = Rect::MakeXYWH(0, 0, 100, 100);
    auto b = Rect::MakeXYWH(10, 10, 100, 100);

    test_non_empty(a, b);
  }

  {
    auto a = Rect::MakeXYWH(0, 0, 100, 100);
    auto b = Rect::MakeXYWH(100, 100, 100, 100);

    test_empty(a, b);
  }

  {
    auto a = Rect::MakeMaximum();
    auto b = Rect::MakeXYWH(10, 10, 100, 100);

    test_non_empty(a, b);
  }

  {
    auto a = Rect::MakeMaximum();
    auto b = Rect::MakeMaximum();

    test_non_empty(a, b);
  }
}

TEST(RectTest, IRectIntersectsWithRect) {
  auto check_empty_flips = [](const IRect& a, const IRect& b,
                              const std::string& label) {
    ASSERT_FALSE(a.IsEmpty());
    // b is allowed to be empty

    // unflipped a vs flipped (empty) b yields a
    EXPECT_FALSE(a.IntersectsWithRect(flip_lr(b))) << label;
    EXPECT_FALSE(a.IntersectsWithRect(flip_tb(b))) << label;
    EXPECT_FALSE(a.IntersectsWithRect(flip_lrtb(b))) << label;

    // flipped (empty) a vs unflipped b yields b
    EXPECT_FALSE(flip_lr(a).IntersectsWithRect(b)) << label;
    EXPECT_FALSE(flip_tb(a).IntersectsWithRect(b)) << label;
    EXPECT_FALSE(flip_lrtb(a).IntersectsWithRect(b)) << label;

    // flipped (empty) a vs flipped (empty) b yields empty
    EXPECT_FALSE(flip_lr(a).IntersectsWithRect(flip_lr(b))) << label;
    EXPECT_FALSE(flip_tb(a).IntersectsWithRect(flip_tb(b))) << label;
    EXPECT_FALSE(flip_lrtb(a).IntersectsWithRect(flip_lrtb(b))) << label;
  };

  auto test_non_empty = [&check_empty_flips](const IRect& a, const IRect& b) {
    ASSERT_FALSE(a.IsEmpty()) << a;
    // b is allowed to be empty

    std::stringstream stream;
    stream << a << " union " << b;
    auto label = stream.str();

    EXPECT_TRUE(a.IntersectsWithRect(b)) << label;
    EXPECT_TRUE(b.IntersectsWithRect(a)) << label;
    check_empty_flips(a, b, label);
  };

  auto test_empty = [&check_empty_flips](const IRect& a, const IRect& b) {
    ASSERT_FALSE(a.IsEmpty()) << a;
    // b is allowed to be empty

    std::stringstream stream;
    stream << a << " union " << b;
    auto label = stream.str();

    EXPECT_FALSE(a.IntersectsWithRect(b)) << label;
    EXPECT_FALSE(b.IntersectsWithRect(a)) << label;
    check_empty_flips(a, b, label);
  };

  {
    auto a = IRect::MakeXYWH(100, 100, 100, 100);
    auto b = IRect::MakeXYWH(0, 0, 0, 0);

    test_empty(a, b);
  }

  {
    auto a = IRect::MakeXYWH(100, 100, 100, 100);
    auto b = IRect::MakeXYWH(10, 10, 0, 0);

    test_empty(a, b);
  }

  {
    auto a = IRect::MakeXYWH(0, 0, 100, 100);
    auto b = IRect::MakeXYWH(10, 10, 100, 100);

    test_non_empty(a, b);
  }

  {
    auto a = IRect::MakeXYWH(0, 0, 100, 100);
    auto b = IRect::MakeXYWH(100, 100, 100, 100);

    test_empty(a, b);
  }

  {
    auto a = IRect::MakeMaximum();
    auto b = IRect::MakeXYWH(10, 10, 100, 100);

    test_non_empty(a, b);
  }

  {
    auto a = IRect::MakeMaximum();
    auto b = IRect::MakeMaximum();

    test_non_empty(a, b);
  }
}

TEST(RectTest, RectContainsPoint) {
  auto check_nans = [](const Rect& rect, const Point& point,
                       const std::string& label) {
    ASSERT_TRUE(rect.IsFinite()) << label;
    ASSERT_TRUE(point.IsFinite()) << label;

    for (int i = 1; i < 16; i++) {
      EXPECT_FALSE(swap_nan(rect, i).Contains(point))
          << label << ", index = " << i;
      for (int j = 1; j < 4; j++) {
        EXPECT_FALSE(swap_nan(rect, i).Contains(swap_nan(point, j)))
            << label << ", indices = " << i << ", " << j;
      }
    }
  };

  auto check_empty_flips = [](const Rect& rect, const Point& point,
                              const std::string& label) {
    ASSERT_FALSE(rect.IsEmpty());

    EXPECT_FALSE(flip_lr(rect).Contains(point)) << label;
    EXPECT_FALSE(flip_tb(rect).Contains(point)) << label;
    EXPECT_FALSE(flip_lrtb(rect).Contains(point)) << label;
  };

  auto test_inside = [&check_nans, &check_empty_flips](const Rect& rect,
                                                       const Point& point) {
    ASSERT_FALSE(rect.IsEmpty()) << rect;

    std::stringstream stream;
    stream << rect << " contains " << point;
    auto label = stream.str();

    EXPECT_TRUE(rect.Contains(point)) << label;
    check_empty_flips(rect, point, label);
    check_nans(rect, point, label);
  };

  auto test_outside = [&check_nans, &check_empty_flips](const Rect& rect,
                                                        const Point& point) {
    ASSERT_FALSE(rect.IsEmpty()) << rect;

    std::stringstream stream;
    stream << rect << " contains " << point;
    auto label = stream.str();

    EXPECT_FALSE(rect.Contains(point)) << label;
    check_empty_flips(rect, point, label);
    check_nans(rect, point, label);
  };

  {
    // Origin is inclusive
    auto r = Rect::MakeXYWH(100, 100, 100, 100);
    auto p = Point(100, 100);

    test_inside(r, p);
  }
  {
    // Size is exclusive
    auto r = Rect::MakeXYWH(100, 100, 100, 100);
    auto p = Point(200, 200);

    test_outside(r, p);
  }
  {
    auto r = Rect::MakeXYWH(100, 100, 100, 100);
    auto p = Point(99, 99);

    test_outside(r, p);
  }
  {
    auto r = Rect::MakeXYWH(100, 100, 100, 100);
    auto p = Point(199, 199);

    test_inside(r, p);
  }

  {
    auto r = Rect::MakeMaximum();
    auto p = Point(199, 199);

    test_inside(r, p);
  }
}

TEST(RectTest, IRectContainsIPoint) {
  auto check_empty_flips = [](const IRect& rect, const IPoint& point,
                              const std::string& label) {
    ASSERT_FALSE(rect.IsEmpty());

    EXPECT_FALSE(flip_lr(rect).Contains(point)) << label;
    EXPECT_FALSE(flip_tb(rect).Contains(point)) << label;
    EXPECT_FALSE(flip_lrtb(rect).Contains(point)) << label;
  };

  auto test_inside = [&check_empty_flips](const IRect& rect,
                                          const IPoint& point) {
    ASSERT_FALSE(rect.IsEmpty()) << rect;

    std::stringstream stream;
    stream << rect << " contains " << point;
    auto label = stream.str();

    EXPECT_TRUE(rect.Contains(point)) << label;
    check_empty_flips(rect, point, label);
  };

  auto test_outside = [&check_empty_flips](const IRect& rect,
                                           const IPoint& point) {
    ASSERT_FALSE(rect.IsEmpty()) << rect;

    std::stringstream stream;
    stream << rect << " contains " << point;
    auto label = stream.str();

    EXPECT_FALSE(rect.Contains(point)) << label;
    check_empty_flips(rect, point, label);
  };

  {
    // Origin is inclusive
    auto r = IRect::MakeXYWH(100, 100, 100, 100);
    auto p = IPoint(100, 100);

    test_inside(r, p);
  }
  {
    // Size is exclusive
    auto r = IRect::MakeXYWH(100, 100, 100, 100);
    auto p = IPoint(200, 200);

    test_outside(r, p);
  }
  {
    auto r = IRect::MakeXYWH(100, 100, 100, 100);
    auto p = IPoint(99, 99);

    test_outside(r, p);
  }
  {
    auto r = IRect::MakeXYWH(100, 100, 100, 100);
    auto p = IPoint(199, 199);

    test_inside(r, p);
  }

  {
    auto r = IRect::MakeMaximum();
    auto p = IPoint(199, 199);

    test_inside(r, p);
  }
}

TEST(RectTest, RectContainsInclusivePoint) {
  auto check_nans = [](const Rect& rect, const Point& point,
                       const std::string& label) {
    ASSERT_TRUE(rect.IsFinite()) << label;
    ASSERT_TRUE(point.IsFinite()) << label;

    for (int i = 1; i < 16; i++) {
      EXPECT_FALSE(swap_nan(rect, i).ContainsInclusive(point))
          << label << ", index = " << i;
      for (int j = 1; j < 4; j++) {
        EXPECT_FALSE(swap_nan(rect, i).ContainsInclusive(swap_nan(point, j)))
            << label << ", indices = " << i << ", " << j;
      }
    }
  };

  auto check_empty_flips = [](const Rect& rect, const Point& point,
                              const std::string& label) {
    ASSERT_FALSE(rect.IsEmpty());

    EXPECT_FALSE(flip_lr(rect).ContainsInclusive(point)) << label;
    EXPECT_FALSE(flip_tb(rect).ContainsInclusive(point)) << label;
    EXPECT_FALSE(flip_lrtb(rect).ContainsInclusive(point)) << label;
  };

  auto test_inside = [&check_nans, &check_empty_flips](const Rect& rect,
                                                       const Point& point) {
    ASSERT_FALSE(rect.IsEmpty()) << rect;

    std::stringstream stream;
    stream << rect << " contains " << point;
    auto label = stream.str();

    EXPECT_TRUE(rect.ContainsInclusive(point)) << label;
    check_empty_flips(rect, point, label);
    check_nans(rect, point, label);
  };

  auto test_outside = [&check_nans, &check_empty_flips](const Rect& rect,
                                                        const Point& point) {
    ASSERT_FALSE(rect.IsEmpty()) << rect;

    std::stringstream stream;
    stream << rect << " contains " << point;
    auto label = stream.str();

    EXPECT_FALSE(rect.ContainsInclusive(point)) << label;
    check_empty_flips(rect, point, label);
    check_nans(rect, point, label);
  };

  {
    // Origin is inclusive
    auto r = Rect::MakeXYWH(100, 100, 100, 100);
    auto p = Point(100, 100);

    test_inside(r, p);
  }
  {
    // Size is inclusive
    auto r = Rect::MakeXYWH(100, 100, 100, 100);
    auto p = Point(200, 200);

    test_inside(r, p);
  }
  {
    // Size + epsilon is exclusive
    auto r = Rect::MakeXYWH(100, 100, 100, 100);
    auto p = Point(200 + kEhCloseEnough, 200 + kEhCloseEnough);

    test_outside(r, p);
  }
  {
    auto r = Rect::MakeXYWH(100, 100, 100, 100);
    auto p = Point(99, 99);

    test_outside(r, p);
  }
  {
    auto r = Rect::MakeXYWH(100, 100, 100, 100);
    auto p = Point(199, 199);

    test_inside(r, p);
  }

  {
    auto r = Rect::MakeMaximum();
    auto p = Point(199, 199);

    test_inside(r, p);
  }
}

TEST(RectTest, IRectContainsInclusiveIPoint) {
  auto check_empty_flips = [](const IRect& rect, const IPoint& point,
                              const std::string& label) {
    ASSERT_FALSE(rect.IsEmpty());

    EXPECT_FALSE(flip_lr(rect).ContainsInclusive(point)) << label;
    EXPECT_FALSE(flip_tb(rect).ContainsInclusive(point)) << label;
    EXPECT_FALSE(flip_lrtb(rect).ContainsInclusive(point)) << label;
  };

  auto test_inside = [&check_empty_flips](const IRect& rect,
                                          const IPoint& point) {
    ASSERT_FALSE(rect.IsEmpty()) << rect;

    std::stringstream stream;
    stream << rect << " contains " << point;
    auto label = stream.str();

    EXPECT_TRUE(rect.ContainsInclusive(point)) << label;
    check_empty_flips(rect, point, label);
  };

  auto test_outside = [&check_empty_flips](const IRect& rect,
                                           const IPoint& point) {
    ASSERT_FALSE(rect.IsEmpty()) << rect;

    std::stringstream stream;
    stream << rect << " contains " << point;
    auto label = stream.str();

    EXPECT_FALSE(rect.ContainsInclusive(point)) << label;
    check_empty_flips(rect, point, label);
  };

  {
    // Origin is inclusive
    auto r = IRect::MakeXYWH(100, 100, 100, 100);
    auto p = IPoint(100, 100);

    test_inside(r, p);
  }
  {
    // Size is inclusive
    auto r = IRect::MakeXYWH(100, 100, 100, 100);
    auto p = IPoint(200, 200);

    test_inside(r, p);
  }
  {
    // Size + "epsilon" is exclusive
    auto r = IRect::MakeXYWH(100, 100, 100, 100);
    auto p = IPoint(201, 201);

    test_outside(r, p);
  }
  {
    auto r = IRect::MakeXYWH(100, 100, 100, 100);
    auto p = IPoint(99, 99);

    test_outside(r, p);
  }
  {
    auto r = IRect::MakeXYWH(100, 100, 100, 100);
    auto p = IPoint(199, 199);

    test_inside(r, p);
  }

  {
    auto r = IRect::MakeMaximum();
    auto p = IPoint(199, 199);

    test_inside(r, p);
  }
}

TEST(RectTest, RectContainsRect) {
  auto check_nans = [](const Rect& a, const Rect& b, const std::string& label) {
    ASSERT_TRUE(a.IsFinite()) << label;
    ASSERT_TRUE(b.IsFinite()) << label;
    ASSERT_FALSE(a.IsEmpty());

    for (int i = 1; i < 16; i++) {
      // NaN in a produces false
      EXPECT_FALSE(swap_nan(a, i).Contains(b)) << label << ", index = " << i;
      // NaN in b produces false
      EXPECT_TRUE(a.Contains(swap_nan(b, i))) << label << ", index = " << i;
      // NaN in both is false
      for (int j = 1; j < 16; j++) {
        EXPECT_FALSE(swap_nan(a, i).Contains(swap_nan(b, j)))
            << label << ", indices = " << i << ", " << j;
      }
    }
  };

  auto check_empty_flips = [](const Rect& a, const Rect& b,
                              const std::string& label) {
    ASSERT_FALSE(a.IsEmpty());
    // test b rects are allowed to have 0 w/h, but not be backwards
    ASSERT_FALSE(b.GetLeft() > b.GetRight() || b.GetTop() > b.GetBottom());

    // unflipped a vs flipped (empty) b yields false
    EXPECT_TRUE(a.Contains(flip_lr(b))) << label;
    EXPECT_TRUE(a.Contains(flip_tb(b))) << label;
    EXPECT_TRUE(a.Contains(flip_lrtb(b))) << label;

    // flipped (empty) a vs unflipped b yields false
    EXPECT_FALSE(flip_lr(a).Contains(b)) << label;
    EXPECT_FALSE(flip_tb(a).Contains(b)) << label;
    EXPECT_FALSE(flip_lrtb(a).Contains(b)) << label;

    // flipped (empty) a vs flipped (empty) b yields empty
    EXPECT_FALSE(flip_lr(a).Contains(flip_lr(b))) << label;
    EXPECT_FALSE(flip_tb(a).Contains(flip_tb(b))) << label;
    EXPECT_FALSE(flip_lrtb(a).Contains(flip_lrtb(b))) << label;
  };

  auto test_inside = [&check_nans, &check_empty_flips](const Rect& a,
                                                       const Rect& b) {
    ASSERT_FALSE(a.IsEmpty()) << a;
    // test b rects are allowed to have 0 w/h, but not be backwards
    ASSERT_FALSE(b.GetLeft() > b.GetRight() || b.GetTop() > b.GetBottom());

    std::stringstream stream;
    stream << a << " contains " << b;
    auto label = stream.str();

    EXPECT_TRUE(a.Contains(b)) << label;
    check_empty_flips(a, b, label);
    check_nans(a, b, label);
  };

  auto test_not_inside = [&check_nans, &check_empty_flips](const Rect& a,
                                                           const Rect& b) {
    ASSERT_FALSE(a.IsEmpty()) << a;
    // If b was empty, it would be contained and should not be tested with
    // this function - use |test_inside| instead.
    ASSERT_FALSE(b.IsEmpty()) << b;

    std::stringstream stream;
    stream << a << " contains " << b;
    auto label = stream.str();

    EXPECT_FALSE(a.Contains(b)) << label;
    check_empty_flips(a, b, label);
    check_nans(a, b, label);
  };

  {
    auto a = Rect::MakeXYWH(100, 100, 100, 100);

    test_inside(a, a);
  }
  {
    auto a = Rect::MakeXYWH(100, 100, 100, 100);
    auto b = Rect::MakeXYWH(0, 0, 0, 0);

    test_inside(a, b);
  }
  {
    auto a = Rect::MakeXYWH(100, 100, 100, 100);
    auto b = Rect::MakeXYWH(150, 150, 20, 20);

    test_inside(a, b);
  }
  {
    auto a = Rect::MakeXYWH(100, 100, 100, 100);
    auto b = Rect::MakeXYWH(150, 150, 100, 100);

    test_not_inside(a, b);
  }
  {
    auto a = Rect::MakeXYWH(100, 100, 100, 100);
    auto b = Rect::MakeXYWH(50, 50, 100, 100);

    test_not_inside(a, b);
  }
  {
    auto a = Rect::MakeXYWH(100, 100, 100, 100);
    auto b = Rect::MakeXYWH(0, 0, 300, 300);

    test_not_inside(a, b);
  }
  {
    auto a = Rect::MakeMaximum();
    auto b = Rect::MakeXYWH(0, 0, 300, 300);

    test_inside(a, b);
  }
}

TEST(RectTest, IRectContainsIRect) {
  auto check_empty_flips = [](const IRect& a, const IRect& b,
                              const std::string& label) {
    ASSERT_FALSE(a.IsEmpty());
    // test b rects are allowed to have 0 w/h, but not be backwards
    ASSERT_FALSE(b.GetLeft() > b.GetRight() || b.GetTop() > b.GetBottom());

    // unflipped a vs flipped (empty) b yields true
    EXPECT_TRUE(a.Contains(flip_lr(b))) << label;
    EXPECT_TRUE(a.Contains(flip_tb(b))) << label;
    EXPECT_TRUE(a.Contains(flip_lrtb(b))) << label;

    // flipped (empty) a vs unflipped b yields false
    EXPECT_FALSE(flip_lr(a).Contains(b)) << label;
    EXPECT_FALSE(flip_tb(a).Contains(b)) << label;
    EXPECT_FALSE(flip_lrtb(a).Contains(b)) << label;

    // flipped (empty) a vs flipped (empty) b yields empty
    EXPECT_FALSE(flip_lr(a).Contains(flip_lr(b))) << label;
    EXPECT_FALSE(flip_tb(a).Contains(flip_tb(b))) << label;
    EXPECT_FALSE(flip_lrtb(a).Contains(flip_lrtb(b))) << label;
  };

  auto test_inside = [&check_empty_flips](const IRect& a, const IRect& b) {
    ASSERT_FALSE(a.IsEmpty()) << a;
    // test b rects are allowed to have 0 w/h, but not be backwards
    ASSERT_FALSE(b.GetLeft() > b.GetRight() || b.GetTop() > b.GetBottom());

    std::stringstream stream;
    stream << a << " contains " << b;
    auto label = stream.str();

    EXPECT_TRUE(a.Contains(b)) << label;
    check_empty_flips(a, b, label);
  };

  auto test_not_inside = [&check_empty_flips](const IRect& a, const IRect& b) {
    ASSERT_FALSE(a.IsEmpty()) << a;
    // If b was empty, it would be contained and should not be tested with
    // this function - use |test_inside| instead.
    ASSERT_FALSE(b.IsEmpty()) << b;

    std::stringstream stream;
    stream << a << " contains " << b;
    auto label = stream.str();

    EXPECT_FALSE(a.Contains(b)) << label;
    check_empty_flips(a, b, label);
  };

  {
    auto a = IRect::MakeXYWH(100, 100, 100, 100);

    test_inside(a, a);
  }
  {
    auto a = IRect::MakeXYWH(100, 100, 100, 100);
    auto b = IRect::MakeXYWH(0, 0, 0, 0);

    test_inside(a, b);
  }
  {
    auto a = IRect::MakeXYWH(100, 100, 100, 100);
    auto b = IRect::MakeXYWH(150, 150, 20, 20);

    test_inside(a, b);
  }
  {
    auto a = IRect::MakeXYWH(100, 100, 100, 100);
    auto b = IRect::MakeXYWH(150, 150, 100, 100);

    test_not_inside(a, b);
  }
  {
    auto a = IRect::MakeXYWH(100, 100, 100, 100);
    auto b = IRect::MakeXYWH(50, 50, 100, 100);

    test_not_inside(a, b);
  }
  {
    auto a = IRect::MakeXYWH(100, 100, 100, 100);
    auto b = IRect::MakeXYWH(0, 0, 300, 300);

    test_not_inside(a, b);
  }
  {
    auto a = IRect::MakeMaximum();
    auto b = IRect::MakeXYWH(0, 0, 300, 300);

    test_inside(a, b);
  }
}

TEST(RectTest, RectCutOut) {
  Rect cull_rect = Rect::MakeLTRB(20, 20, 40, 40);

  auto check_nans = [&cull_rect](const Rect& diff_rect,
                                 const std::string& label) {
    EXPECT_TRUE(cull_rect.IsFinite()) << label;
    EXPECT_TRUE(diff_rect.IsFinite()) << label;

    for (int i = 1; i < 16; i++) {
      // NaN in cull_rect produces empty
      EXPECT_FALSE(swap_nan(cull_rect, i).Cutout(diff_rect).has_value())
          << label << ", index " << i;
      EXPECT_EQ(swap_nan(cull_rect, i).CutoutOrEmpty(diff_rect), Rect())
          << label << ", index " << i;

      // NaN in diff_rect is nop
      EXPECT_TRUE(cull_rect.Cutout(swap_nan(diff_rect, i)).has_value())
          << label << ", index " << i;
      EXPECT_EQ(cull_rect.CutoutOrEmpty(swap_nan(diff_rect, i)), cull_rect)
          << label << ", index " << i;

      for (int j = 1; j < 16; j++) {
        // NaN in both is also empty
        EXPECT_FALSE(
            swap_nan(cull_rect, i).Cutout(swap_nan(diff_rect, j)).has_value())
            << label << ", indices " << i << ", " << j;
        EXPECT_EQ(swap_nan(cull_rect, i).CutoutOrEmpty(swap_nan(diff_rect, j)),
                  Rect())
            << label << ", indices " << i << ", " << j;
      }
    }
  };

  auto check_empty_flips = [&cull_rect](const Rect& diff_rect,
                                        const std::string& label) {
    EXPECT_FALSE(cull_rect.IsEmpty()) << label;
    EXPECT_FALSE(diff_rect.IsEmpty()) << label;

    // unflipped cull_rect vs flipped(empty) diff_rect
    // == cull_rect
    EXPECT_TRUE(cull_rect.Cutout(flip_lr(diff_rect)).has_value()) << label;
    EXPECT_EQ(cull_rect.Cutout(flip_lr(diff_rect)), cull_rect) << label;
    EXPECT_TRUE(cull_rect.Cutout(flip_tb(diff_rect)).has_value()) << label;
    EXPECT_EQ(cull_rect.Cutout(flip_tb(diff_rect)), cull_rect) << label;
    EXPECT_TRUE(cull_rect.Cutout(flip_lrtb(diff_rect)).has_value()) << label;
    EXPECT_EQ(cull_rect.Cutout(flip_lrtb(diff_rect)), cull_rect) << label;

    // flipped(empty) cull_rect vs unflipped diff_rect
    // == empty
    EXPECT_FALSE(flip_lr(cull_rect).Cutout(diff_rect).has_value()) << label;
    EXPECT_EQ(flip_lr(cull_rect).CutoutOrEmpty(diff_rect), Rect()) << label;
    EXPECT_FALSE(flip_tb(cull_rect).Cutout(diff_rect).has_value()) << label;
    EXPECT_EQ(flip_tb(cull_rect).CutoutOrEmpty(diff_rect), Rect()) << label;
    EXPECT_FALSE(flip_lrtb(cull_rect).Cutout(diff_rect).has_value()) << label;
    EXPECT_EQ(flip_lrtb(cull_rect).CutoutOrEmpty(diff_rect), Rect()) << label;

    // flipped(empty) cull_rect vs flipped(empty) diff_rect
    // == empty
    EXPECT_FALSE(flip_lr(cull_rect).Cutout(flip_lr(diff_rect)).has_value())
        << label;
    EXPECT_EQ(flip_lr(cull_rect).CutoutOrEmpty(flip_lr(diff_rect)), Rect())
        << label;
    EXPECT_FALSE(flip_tb(cull_rect).Cutout(flip_tb(diff_rect)).has_value())
        << label;
    EXPECT_EQ(flip_tb(cull_rect).CutoutOrEmpty(flip_tb(diff_rect)), Rect())
        << label;
    EXPECT_FALSE(flip_lrtb(cull_rect).Cutout(flip_lrtb(diff_rect)).has_value())
        << label;
    EXPECT_EQ(flip_lrtb(cull_rect).CutoutOrEmpty(flip_lrtb(diff_rect)), Rect())
        << label;
  };

  auto non_reducing = [&cull_rect, &check_empty_flips, &check_nans](
                          const Rect& diff_rect, const std::string& label) {
    EXPECT_EQ(cull_rect.Cutout(diff_rect), cull_rect) << label;
    EXPECT_EQ(cull_rect.CutoutOrEmpty(diff_rect), cull_rect) << label;
    check_empty_flips(diff_rect, label);
    check_nans(diff_rect, label);
  };

  auto reducing = [&cull_rect, &check_empty_flips, &check_nans](
                      const Rect& diff_rect, const Rect& result_rect,
                      const std::string& label) {
    EXPECT_TRUE(!result_rect.IsEmpty());
    EXPECT_EQ(cull_rect.Cutout(diff_rect), result_rect) << label;
    EXPECT_EQ(cull_rect.CutoutOrEmpty(diff_rect), result_rect) << label;
    check_empty_flips(diff_rect, label);
    check_nans(diff_rect, label);
  };

  auto emptying = [&cull_rect, &check_empty_flips, &check_nans](
                      const Rect& diff_rect, const std::string& label) {
    EXPECT_FALSE(cull_rect.Cutout(diff_rect).has_value()) << label;
    EXPECT_EQ(cull_rect.CutoutOrEmpty(diff_rect), Rect()) << label;
    check_empty_flips(diff_rect, label);
    check_nans(diff_rect, label);
  };

  // Skim the corners and edge
  non_reducing(Rect::MakeLTRB(10, 10, 20, 20), "outside UL corner");
  non_reducing(Rect::MakeLTRB(20, 10, 40, 20), "Above");
  non_reducing(Rect::MakeLTRB(40, 10, 50, 20), "outside UR corner");
  non_reducing(Rect::MakeLTRB(40, 20, 50, 40), "Right");
  non_reducing(Rect::MakeLTRB(40, 40, 50, 50), "outside LR corner");
  non_reducing(Rect::MakeLTRB(20, 40, 40, 50), "Below");
  non_reducing(Rect::MakeLTRB(10, 40, 20, 50), "outside LR corner");
  non_reducing(Rect::MakeLTRB(10, 20, 20, 40), "Left");

  // Overlap corners
  non_reducing(Rect::MakeLTRB(15, 15, 25, 25), "covering UL corner");
  non_reducing(Rect::MakeLTRB(35, 15, 45, 25), "covering UR corner");
  non_reducing(Rect::MakeLTRB(35, 35, 45, 45), "covering LR corner");
  non_reducing(Rect::MakeLTRB(15, 35, 25, 45), "covering LL corner");

  // Overlap edges, but not across an entire side
  non_reducing(Rect::MakeLTRB(20, 15, 39, 25), "Top edge left-biased");
  non_reducing(Rect::MakeLTRB(21, 15, 40, 25), "Top edge, right biased");
  non_reducing(Rect::MakeLTRB(35, 20, 45, 39), "Right edge, top-biased");
  non_reducing(Rect::MakeLTRB(35, 21, 45, 40), "Right edge, bottom-biased");
  non_reducing(Rect::MakeLTRB(20, 35, 39, 45), "Bottom edge, left-biased");
  non_reducing(Rect::MakeLTRB(21, 35, 40, 45), "Bottom edge, right-biased");
  non_reducing(Rect::MakeLTRB(15, 20, 25, 39), "Left edge, top-biased");
  non_reducing(Rect::MakeLTRB(15, 21, 25, 40), "Left edge, bottom-biased");

  // Slice all the way through the middle
  non_reducing(Rect::MakeLTRB(25, 15, 35, 45), "Vertical interior slice");
  non_reducing(Rect::MakeLTRB(15, 25, 45, 35), "Horizontal interior slice");

  // Slice off each edge
  reducing(Rect::MakeLTRB(20, 15, 40, 25),  //
           Rect::MakeLTRB(20, 25, 40, 40),  //
           "Slice off top");
  reducing(Rect::MakeLTRB(35, 20, 45, 40),  //
           Rect::MakeLTRB(20, 20, 35, 40),  //
           "Slice off right");
  reducing(Rect::MakeLTRB(20, 35, 40, 45),  //
           Rect::MakeLTRB(20, 20, 40, 35),  //
           "Slice off bottom");
  reducing(Rect::MakeLTRB(15, 20, 25, 40),  //
           Rect::MakeLTRB(25, 20, 40, 40),  //
           "Slice off left");

  // cull rect contains diff rect
  non_reducing(Rect::MakeLTRB(21, 21, 39, 39), "Contained, non-covering");

  // cull rect equals diff rect
  emptying(cull_rect, "Perfectly covering");

  // diff rect contains cull rect
  emptying(Rect::MakeLTRB(15, 15, 45, 45), "Smothering");
}

TEST(RectTest, IRectCutOut) {
  IRect cull_rect = IRect::MakeLTRB(20, 20, 40, 40);

  auto check_empty_flips = [&cull_rect](const IRect& diff_rect,
                                        const std::string& label) {
    EXPECT_FALSE(diff_rect.IsEmpty());
    EXPECT_FALSE(cull_rect.IsEmpty());

    // unflipped cull_rect vs flipped(empty) diff_rect
    // == cull_rect
    EXPECT_TRUE(cull_rect.Cutout(flip_lr(diff_rect)).has_value()) << label;
    EXPECT_EQ(cull_rect.Cutout(flip_lr(diff_rect)), cull_rect) << label;
    EXPECT_TRUE(cull_rect.Cutout(flip_tb(diff_rect)).has_value()) << label;
    EXPECT_EQ(cull_rect.Cutout(flip_tb(diff_rect)), cull_rect) << label;
    EXPECT_TRUE(cull_rect.Cutout(flip_lrtb(diff_rect)).has_value()) << label;
    EXPECT_EQ(cull_rect.Cutout(flip_lrtb(diff_rect)), cull_rect) << label;

    // flipped(empty) cull_rect vs flipped(empty) diff_rect
    // == empty
    EXPECT_FALSE(flip_lr(cull_rect).Cutout(diff_rect).has_value()) << label;
    EXPECT_EQ(flip_lr(cull_rect).CutoutOrEmpty(diff_rect), IRect()) << label;
    EXPECT_FALSE(flip_tb(cull_rect).Cutout(diff_rect).has_value()) << label;
    EXPECT_EQ(flip_tb(cull_rect).CutoutOrEmpty(diff_rect), IRect()) << label;
    EXPECT_FALSE(flip_lrtb(cull_rect).Cutout(diff_rect).has_value()) << label;
    EXPECT_EQ(flip_lrtb(cull_rect).CutoutOrEmpty(diff_rect), IRect()) << label;

    // flipped(empty) cull_rect vs unflipped diff_rect
    // == empty
    EXPECT_FALSE(flip_lr(cull_rect).Cutout(flip_lr(diff_rect)).has_value())
        << label;
    EXPECT_EQ(flip_lr(cull_rect).CutoutOrEmpty(flip_lr(diff_rect)), IRect())
        << label;
    EXPECT_FALSE(flip_tb(cull_rect).Cutout(flip_tb(diff_rect)).has_value())
        << label;
    EXPECT_EQ(flip_tb(cull_rect).CutoutOrEmpty(flip_tb(diff_rect)), IRect())
        << label;
    EXPECT_FALSE(flip_lrtb(cull_rect).Cutout(flip_lrtb(diff_rect)).has_value())
        << label;
    EXPECT_EQ(flip_lrtb(cull_rect).CutoutOrEmpty(flip_lrtb(diff_rect)), IRect())
        << label;
  };

  auto non_reducing = [&cull_rect, &check_empty_flips](
                          const IRect& diff_rect, const std::string& label) {
    EXPECT_EQ(cull_rect.Cutout(diff_rect), cull_rect) << label;
    EXPECT_EQ(cull_rect.CutoutOrEmpty(diff_rect), cull_rect) << label;
    check_empty_flips(diff_rect, label);
  };

  auto reducing = [&cull_rect, &check_empty_flips](const IRect& diff_rect,
                                                   const IRect& result_rect,
                                                   const std::string& label) {
    EXPECT_TRUE(!result_rect.IsEmpty());
    EXPECT_EQ(cull_rect.Cutout(diff_rect), result_rect) << label;
    EXPECT_EQ(cull_rect.CutoutOrEmpty(diff_rect), result_rect) << label;
    check_empty_flips(diff_rect, label);
  };

  auto emptying = [&cull_rect, &check_empty_flips](const IRect& diff_rect,
                                                   const std::string& label) {
    EXPECT_FALSE(cull_rect.Cutout(diff_rect).has_value()) << label;
    EXPECT_EQ(cull_rect.CutoutOrEmpty(diff_rect), IRect()) << label;
    check_empty_flips(diff_rect, label);
  };

  // Skim the corners and edge
  non_reducing(IRect::MakeLTRB(10, 10, 20, 20), "outside UL corner");
  non_reducing(IRect::MakeLTRB(20, 10, 40, 20), "Above");
  non_reducing(IRect::MakeLTRB(40, 10, 50, 20), "outside UR corner");
  non_reducing(IRect::MakeLTRB(40, 20, 50, 40), "Right");
  non_reducing(IRect::MakeLTRB(40, 40, 50, 50), "outside LR corner");
  non_reducing(IRect::MakeLTRB(20, 40, 40, 50), "Below");
  non_reducing(IRect::MakeLTRB(10, 40, 20, 50), "outside LR corner");
  non_reducing(IRect::MakeLTRB(10, 20, 20, 40), "Left");

  // Overlap corners
  non_reducing(IRect::MakeLTRB(15, 15, 25, 25), "covering UL corner");
  non_reducing(IRect::MakeLTRB(35, 15, 45, 25), "covering UR corner");
  non_reducing(IRect::MakeLTRB(35, 35, 45, 45), "covering LR corner");
  non_reducing(IRect::MakeLTRB(15, 35, 25, 45), "covering LL corner");

  // Overlap edges, but not across an entire side
  non_reducing(IRect::MakeLTRB(20, 15, 39, 25), "Top edge left-biased");
  non_reducing(IRect::MakeLTRB(21, 15, 40, 25), "Top edge, right biased");
  non_reducing(IRect::MakeLTRB(35, 20, 45, 39), "Right edge, top-biased");
  non_reducing(IRect::MakeLTRB(35, 21, 45, 40), "Right edge, bottom-biased");
  non_reducing(IRect::MakeLTRB(20, 35, 39, 45), "Bottom edge, left-biased");
  non_reducing(IRect::MakeLTRB(21, 35, 40, 45), "Bottom edge, right-biased");
  non_reducing(IRect::MakeLTRB(15, 20, 25, 39), "Left edge, top-biased");
  non_reducing(IRect::MakeLTRB(15, 21, 25, 40), "Left edge, bottom-biased");

  // Slice all the way through the middle
  non_reducing(IRect::MakeLTRB(25, 15, 35, 45), "Vertical interior slice");
  non_reducing(IRect::MakeLTRB(15, 25, 45, 35), "Horizontal interior slice");

  // Slice off each edge
  reducing(IRect::MakeLTRB(20, 15, 40, 25),  //
           IRect::MakeLTRB(20, 25, 40, 40),  //
           "Slice off top");
  reducing(IRect::MakeLTRB(35, 20, 45, 40),  //
           IRect::MakeLTRB(20, 20, 35, 40),  //
           "Slice off right");
  reducing(IRect::MakeLTRB(20, 35, 40, 45),  //
           IRect::MakeLTRB(20, 20, 40, 35),  //
           "Slice off bottom");
  reducing(IRect::MakeLTRB(15, 20, 25, 40),  //
           IRect::MakeLTRB(25, 20, 40, 40),  //
           "Slice off left");

  // cull rect contains diff rect
  non_reducing(IRect::MakeLTRB(21, 21, 39, 39), "Contained, non-covering");

  // cull rect equals diff rect
  emptying(cull_rect, "Perfectly covering");

  // diff rect contains cull rect
  emptying(IRect::MakeLTRB(15, 15, 45, 45), "Smothering");
}

TEST(RectTest, RectGetPoints) {
  {
    Rect r = Rect::MakeXYWH(100, 200, 300, 400);
    auto points = r.GetPoints();
    EXPECT_POINT_NEAR(points[0], Point(100, 200));
    EXPECT_POINT_NEAR(points[1], Point(400, 200));
    EXPECT_POINT_NEAR(points[2], Point(100, 600));
    EXPECT_POINT_NEAR(points[3], Point(400, 600));
  }

  {
    Rect r = Rect::MakeMaximum();
    auto points = r.GetPoints();
    EXPECT_EQ(points[0], Point(std::numeric_limits<float>::lowest(),
                               std::numeric_limits<float>::lowest()));
    EXPECT_EQ(points[1], Point(std::numeric_limits<float>::max(),
                               std::numeric_limits<float>::lowest()));
    EXPECT_EQ(points[2], Point(std::numeric_limits<float>::lowest(),
                               std::numeric_limits<float>::max()));
    EXPECT_EQ(points[3], Point(std::numeric_limits<float>::max(),
                               std::numeric_limits<float>::max()));
  }
}

TEST(RectTest, RectShift) {
  auto r = Rect::MakeLTRB(0, 0, 100, 100);

  EXPECT_EQ(r.Shift(Point(10, 5)), Rect::MakeLTRB(10, 5, 110, 105));
  EXPECT_EQ(r.Shift(Point(-10, -5)), Rect::MakeLTRB(-10, -5, 90, 95));
}

TEST(RectTest, RectGetTransformedPoints) {
  Rect r = Rect::MakeXYWH(100, 200, 300, 400);
  auto points = r.GetTransformedPoints(Matrix::MakeTranslation({10, 20}));
  EXPECT_POINT_NEAR(points[0], Point(110, 220));
  EXPECT_POINT_NEAR(points[1], Point(410, 220));
  EXPECT_POINT_NEAR(points[2], Point(110, 620));
  EXPECT_POINT_NEAR(points[3], Point(410, 620));
}

TEST(RectTest, RectMakePointBounds) {
  {
    std::vector<Point> points{{1, 5}, {4, -1}, {0, 6}};
    auto r = Rect::MakePointBounds(points.begin(), points.end());
    auto expected = Rect::MakeXYWH(0, -1, 4, 7);
    EXPECT_TRUE(r.has_value());
    if (r.has_value()) {
      EXPECT_RECT_NEAR(r.value(), expected);
    }
  }
  {
    std::vector<Point> points;
    std::optional<Rect> r = Rect::MakePointBounds(points.begin(), points.end());
    EXPECT_FALSE(r.has_value());
  }
}

TEST(RectTest, RectGetPositive) {
  {
    Rect r = Rect::MakeXYWH(100, 200, 300, 400);
    auto actual = r.GetPositive();
    EXPECT_RECT_NEAR(r, actual);
  }
  {
    Rect r = Rect::MakeXYWH(100, 200, -100, -100);
    auto actual = r.GetPositive();
    Rect expected = Rect::MakeXYWH(0, 100, 100, 100);
    EXPECT_RECT_NEAR(expected, actual);
  }
}

TEST(RectTest, RectDirections) {
  auto r = Rect::MakeLTRB(1, 2, 3, 4);

  EXPECT_EQ(r.GetLeft(), 1);
  EXPECT_EQ(r.GetTop(), 2);
  EXPECT_EQ(r.GetRight(), 3);
  EXPECT_EQ(r.GetBottom(), 4);

  EXPECT_POINT_NEAR(r.GetLeftTop(), Point(1, 2));
  EXPECT_POINT_NEAR(r.GetRightTop(), Point(3, 2));
  EXPECT_POINT_NEAR(r.GetLeftBottom(), Point(1, 4));
  EXPECT_POINT_NEAR(r.GetRightBottom(), Point(3, 4));
}

TEST(RectTest, RectProject) {
  {
    auto r = Rect::MakeLTRB(-100, -100, 100, 100);
    auto actual = r.Project(r);
    auto expected = Rect::MakeLTRB(0, 0, 1, 1);
    EXPECT_RECT_NEAR(expected, actual);
  }
  {
    auto r = Rect::MakeLTRB(-100, -100, 100, 100);
    auto actual = r.Project(Rect::MakeLTRB(0, 0, 100, 100));
    auto expected = Rect::MakeLTRB(0.5, 0.5, 1, 1);
    EXPECT_RECT_NEAR(expected, actual);
  }
}

TEST(RectTest, RectRoundOut) {
  {
    auto r = Rect::MakeLTRB(-100, -200, 300, 400);
    EXPECT_EQ(Rect::RoundOut(r), r);
  }
  {
    auto r = Rect::MakeLTRB(-100.1, -200.1, 300.1, 400.1);
    EXPECT_EQ(Rect::RoundOut(r), Rect::MakeLTRB(-101, -201, 301, 401));
  }
}

TEST(RectTest, IRectRoundOut) {
  {
    auto r = Rect::MakeLTRB(-100, -200, 300, 400);
    auto ir = IRect::MakeLTRB(-100, -200, 300, 400);
    EXPECT_EQ(IRect::RoundOut(r), ir);
  }
  {
    auto r = Rect::MakeLTRB(-100.1, -200.1, 300.1, 400.1);
    auto ir = IRect::MakeLTRB(-101, -201, 301, 401);
    EXPECT_EQ(IRect::RoundOut(r), ir);
  }
}

TEST(RectTest, RectRound) {
  {
    auto r = Rect::MakeLTRB(-100, -200, 300, 400);
    EXPECT_EQ(Rect::Round(r), r);
  }
  {
    auto r = Rect::MakeLTRB(-100.4, -200.4, 300.4, 400.4);
    EXPECT_EQ(Rect::Round(r), Rect::MakeLTRB(-100, -200, 300, 400));
  }
  {
    auto r = Rect::MakeLTRB(-100.5, -200.5, 300.5, 400.5);
    EXPECT_EQ(Rect::Round(r), Rect::MakeLTRB(-101, -201, 301, 401));
  }
}

TEST(RectTest, IRectRound) {
  {
    auto r = Rect::MakeLTRB(-100, -200, 300, 400);
    auto ir = IRect::MakeLTRB(-100, -200, 300, 400);
    EXPECT_EQ(IRect::Round(r), ir);
  }
  {
    auto r = Rect::MakeLTRB(-100.4, -200.4, 300.4, 400.4);
    auto ir = IRect::MakeLTRB(-100, -200, 300, 400);
    EXPECT_EQ(IRect::Round(r), ir);
  }
  {
    auto r = Rect::MakeLTRB(-100.5, -200.5, 300.5, 400.5);
    auto ir = IRect::MakeLTRB(-101, -201, 301, 401);
    EXPECT_EQ(IRect::Round(r), ir);
  }
}

TEST(RectTest, TransformAndClipBounds) {
  {
    // This matrix should clip no corners.
    auto matrix = impeller::Matrix::MakeColumn(
        // clang-format off
        2.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 4.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 8.0f
        // clang-format on
    );
    Rect src = Rect::MakeLTRB(100.0f, 100.0f, 200.0f, 200.0f);
    // None of these should have a W<0
    EXPECT_EQ(matrix.TransformHomogenous(src.GetLeftTop()),
              Vector3(200.0f, 400.0f, 8.0f));
    EXPECT_EQ(matrix.TransformHomogenous(src.GetRightTop()),
              Vector3(400.0f, 400.0f, 8.0f));
    EXPECT_EQ(matrix.TransformHomogenous(src.GetLeftBottom()),
              Vector3(200.0f, 800.0f, 8.0f));
    EXPECT_EQ(matrix.TransformHomogenous(src.GetRightBottom()),
              Vector3(400.0f, 800.0f, 8.0f));

    Rect expect = Rect::MakeLTRB(25.0f, 50.0f, 50.0f, 100.0f);
    EXPECT_FALSE(src.TransformAndClipBounds(matrix).IsEmpty());
    EXPECT_EQ(src.TransformAndClipBounds(matrix), expect);
  }

  {
    // This matrix should clip one corner.
    auto matrix = impeller::Matrix::MakeColumn(
        // clang-format off
        2.0f, 0.0f, 0.0f, -0.01f,
        0.0f, 2.0f, 0.0f, -0.006f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 3.0f
        // clang-format on
    );
    Rect src = Rect::MakeLTRB(100.0f, 100.0f, 200.0f, 200.0f);
    // Exactly one of these should have a W<0
    EXPECT_VECTOR3_NEAR(matrix.TransformHomogenous(src.GetLeftTop()),
                        Vector3(200.0f, 200.0f, 1.4f));
    EXPECT_VECTOR3_NEAR(matrix.TransformHomogenous(src.GetRightTop()),
                        Vector3(400.0f, 200.0f, 0.4f));
    EXPECT_VECTOR3_NEAR(matrix.TransformHomogenous(src.GetLeftBottom()),
                        Vector3(200.0f, 400.0f, 0.8f));
    EXPECT_VECTOR3_NEAR(matrix.TransformHomogenous(src.GetRightBottom()),
                        Vector3(400.0f, 400.0f, -0.2f));

    Rect expect = Rect::MakeLTRB(142.85715f, 142.85715f, 6553600.f, 6553600.f);
    EXPECT_FALSE(src.TransformAndClipBounds(matrix).IsEmpty());
    EXPECT_RECT_NEAR(src.TransformAndClipBounds(matrix), expect);
  }

  {
    // This matrix should clip two corners.
    auto matrix = impeller::Matrix::MakeColumn(
        // clang-format off
        2.0f, 0.0f, 0.0f, -.015f,
        0.0f, 2.0f, 0.0f, -.006f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 3.0f
        // clang-format on
    );
    Rect src = Rect::MakeLTRB(100.0f, 100.0f, 200.0f, 200.0f);
    // Exactly two of these should have a W<0
    EXPECT_VECTOR3_NEAR(matrix.TransformHomogenous(src.GetLeftTop()),
                        Vector3(200.0f, 200.0f, 0.9f));
    EXPECT_VECTOR3_NEAR(matrix.TransformHomogenous(src.GetRightTop()),
                        Vector3(400.0f, 200.0f, -0.6f));
    EXPECT_VECTOR3_NEAR(matrix.TransformHomogenous(src.GetLeftBottom()),
                        Vector3(200.0f, 400.0f, 0.3f));
    EXPECT_VECTOR3_NEAR(matrix.TransformHomogenous(src.GetRightBottom()),
                        Vector3(400.0f, 400.0f, -1.2f));

    Rect expect = Rect::MakeLTRB(222.2222f, 222.2222f, 5898373.f, 6553600.f);
    EXPECT_FALSE(src.TransformAndClipBounds(matrix).IsEmpty());
    EXPECT_RECT_NEAR(src.TransformAndClipBounds(matrix), expect);
  }

  {
    // This matrix should clip three corners.
    auto matrix = impeller::Matrix::MakeColumn(
        // clang-format off
        2.0f, 0.0f, 0.0f, -.02f,
        0.0f, 2.0f, 0.0f, -.006f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 3.0f
        // clang-format on
    );
    Rect src = Rect::MakeLTRB(100.0f, 100.0f, 200.0f, 200.0f);
    // Exactly three of these should have a W<0
    EXPECT_VECTOR3_NEAR(matrix.TransformHomogenous(src.GetLeftTop()),
                        Vector3(200.0f, 200.0f, 0.4f));
    EXPECT_VECTOR3_NEAR(matrix.TransformHomogenous(src.GetRightTop()),
                        Vector3(400.0f, 200.0f, -1.6f));
    EXPECT_VECTOR3_NEAR(matrix.TransformHomogenous(src.GetLeftBottom()),
                        Vector3(200.0f, 400.0f, -0.2f));
    EXPECT_VECTOR3_NEAR(matrix.TransformHomogenous(src.GetRightBottom()),
                        Vector3(400.0f, 400.0f, -2.2f));

    Rect expect = Rect::MakeLTRB(499.99988f, 499.99988f, 5898340.f, 4369400.f);
    EXPECT_FALSE(src.TransformAndClipBounds(matrix).IsEmpty());
    EXPECT_RECT_NEAR(src.TransformAndClipBounds(matrix), expect);
  }

  {
    // This matrix should clip all four corners.
    auto matrix = impeller::Matrix::MakeColumn(
        // clang-format off
        2.0f, 0.0f, 0.0f, -.025f,
        0.0f, 2.0f, 0.0f, -.006f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 3.0f
        // clang-format on
    );
    Rect src = Rect::MakeLTRB(100.0f, 100.0f, 200.0f, 200.0f);
    // All of these should have a W<0
    EXPECT_VECTOR3_NEAR(matrix.TransformHomogenous(src.GetLeftTop()),
                        Vector3(200.0f, 200.0f, -0.1f));
    EXPECT_VECTOR3_NEAR(matrix.TransformHomogenous(src.GetRightTop()),
                        Vector3(400.0f, 200.0f, -2.6f));
    EXPECT_VECTOR3_NEAR(matrix.TransformHomogenous(src.GetLeftBottom()),
                        Vector3(200.0f, 400.0f, -0.7f));
    EXPECT_VECTOR3_NEAR(matrix.TransformHomogenous(src.GetRightBottom()),
                        Vector3(400.0f, 400.0f, -3.2f));

    EXPECT_TRUE(src.TransformAndClipBounds(matrix).IsEmpty());
  }
}

}  // namespace testing
}  // namespace impeller
