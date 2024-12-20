// Copyright (c) 2009 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "insets.h"

#include "gtest/gtest.h"
#include "insets_f.h"
#include "rect.h"
#include "size.h"
#include "vector2d.h"

TEST(InsetsTest, InsetsDefault) {
  gfx::Insets insets;
  EXPECT_EQ(0, insets.top());
  EXPECT_EQ(0, insets.left());
  EXPECT_EQ(0, insets.bottom());
  EXPECT_EQ(0, insets.right());
  EXPECT_EQ(0, insets.width());
  EXPECT_EQ(0, insets.height());
  EXPECT_TRUE(insets.IsEmpty());
}

TEST(InsetsTest, Insets) {
  gfx::Insets insets(1, 2, 3, 4);
  EXPECT_EQ(1, insets.top());
  EXPECT_EQ(2, insets.left());
  EXPECT_EQ(3, insets.bottom());
  EXPECT_EQ(4, insets.right());
  EXPECT_EQ(6, insets.width());   // Left + right.
  EXPECT_EQ(4, insets.height());  // Top + bottom.
  EXPECT_FALSE(insets.IsEmpty());
}

TEST(InsetsTest, SetTop) {
  gfx::Insets insets(1);
  insets.set_top(2);
  EXPECT_EQ(gfx::Insets(2, 1, 1, 1), insets);
}

TEST(InsetsTest, SetBottom) {
  gfx::Insets insets(1);
  insets.set_bottom(2);
  EXPECT_EQ(gfx::Insets(1, 1, 2, 1), insets);
}

TEST(InsetsTest, SetLeft) {
  gfx::Insets insets(1);
  insets.set_left(2);
  EXPECT_EQ(gfx::Insets(1, 2, 1, 1), insets);
}

TEST(InsetsTest, SetRight) {
  gfx::Insets insets(1);
  insets.set_right(2);
  EXPECT_EQ(gfx::Insets(1, 1, 1, 2), insets);
}

TEST(InsetsTest, Set) {
  gfx::Insets insets;
  insets.Set(1, 2, 3, 4);
  EXPECT_EQ(1, insets.top());
  EXPECT_EQ(2, insets.left());
  EXPECT_EQ(3, insets.bottom());
  EXPECT_EQ(4, insets.right());
}

TEST(InsetsTest, Operators) {
  gfx::Insets insets;
  insets.Set(1, 2, 3, 4);
  insets += gfx::Insets(5, 6, 7, 8);
  EXPECT_EQ(6, insets.top());
  EXPECT_EQ(8, insets.left());
  EXPECT_EQ(10, insets.bottom());
  EXPECT_EQ(12, insets.right());

  insets -= gfx::Insets(-1, 0, 1, 2);
  EXPECT_EQ(7, insets.top());
  EXPECT_EQ(8, insets.left());
  EXPECT_EQ(9, insets.bottom());
  EXPECT_EQ(10, insets.right());

  insets = gfx::Insets(10, 10, 10, 10) + gfx::Insets(5, 5, 0, -20);
  EXPECT_EQ(15, insets.top());
  EXPECT_EQ(15, insets.left());
  EXPECT_EQ(10, insets.bottom());
  EXPECT_EQ(-10, insets.right());

  insets = gfx::Insets(10, 10, 10, 10) - gfx::Insets(5, 5, 0, -20);
  EXPECT_EQ(5, insets.top());
  EXPECT_EQ(5, insets.left());
  EXPECT_EQ(10, insets.bottom());
  EXPECT_EQ(30, insets.right());
}

TEST(InsetsFTest, Operators) {
  gfx::InsetsF insets;
  insets.Set(1.f, 2.5f, 3.3f, 4.1f);
  insets += gfx::InsetsF(5.8f, 6.7f, 7.6f, 8.5f);
  EXPECT_FLOAT_EQ(6.8f, insets.top());
  EXPECT_FLOAT_EQ(9.2f, insets.left());
  EXPECT_FLOAT_EQ(10.9f, insets.bottom());
  EXPECT_FLOAT_EQ(12.6f, insets.right());

  insets -= gfx::InsetsF(-1.f, 0, 1.1f, 2.2f);
  EXPECT_FLOAT_EQ(7.8f, insets.top());
  EXPECT_FLOAT_EQ(9.2f, insets.left());
  EXPECT_FLOAT_EQ(9.8f, insets.bottom());
  EXPECT_FLOAT_EQ(10.4f, insets.right());

  insets = gfx::InsetsF(10, 10.1f, 10.01f, 10.001f) +
           gfx::InsetsF(5.5f, 5.f, 0, -20.2f);
  EXPECT_FLOAT_EQ(15.5f, insets.top());
  EXPECT_FLOAT_EQ(15.1f, insets.left());
  EXPECT_FLOAT_EQ(10.01f, insets.bottom());
  EXPECT_FLOAT_EQ(-10.199f, insets.right());

  insets = gfx::InsetsF(10, 10.1f, 10.01f, 10.001f) -
           gfx::InsetsF(5.5f, 5.f, 0, -20.2f);
  EXPECT_FLOAT_EQ(4.5f, insets.top());
  EXPECT_FLOAT_EQ(5.1f, insets.left());
  EXPECT_FLOAT_EQ(10.01f, insets.bottom());
  EXPECT_FLOAT_EQ(30.201f, insets.right());
}

TEST(InsetsTest, Equality) {
  gfx::Insets insets1;
  insets1.Set(1, 2, 3, 4);
  gfx::Insets insets2;
  // Test operator== and operator!=.
  EXPECT_FALSE(insets1 == insets2);
  EXPECT_TRUE(insets1 != insets2);

  insets2.Set(1, 2, 3, 4);
  EXPECT_TRUE(insets1 == insets2);
  EXPECT_FALSE(insets1 != insets2);
}

TEST(InsetsTest, ToString) {
  gfx::Insets insets(1, 2, 3, 4);
  EXPECT_EQ("1,2,3,4", insets.ToString());
}

TEST(InsetsTest, Offset) {
  const gfx::Insets insets(1, 2, 3, 4);
  const gfx::Rect rect(5, 6, 7, 8);
  const gfx::Vector2d vector(9, 10);

  // Whether you inset then offset the rect, offset then inset the rect, or
  // offset the insets then apply to the rect, the outcome should be the same.
  gfx::Rect inset_first = rect;
  inset_first.Inset(insets);
  inset_first.Offset(vector);

  gfx::Rect offset_first = rect;
  offset_first.Offset(vector);
  offset_first.Inset(insets);

  gfx::Rect inset_by_offset = rect;
  inset_by_offset.Inset(insets.Offset(vector));

  EXPECT_EQ(inset_first, offset_first);
  EXPECT_EQ(inset_by_offset, inset_first);
}

TEST(InsetsTest, Scale) {
  gfx::Insets test(10, 5);
  test = test.Scale(2.f, 3.f);
  EXPECT_EQ(gfx::Insets(30, 10), test);

  test = gfx::Insets(7, 3);
  test = test.Scale(-2.f, -3.f);
  EXPECT_EQ(gfx::Insets(-21, -6), test);
}

TEST(InsetsTest, IntegerOverflow) {
  constexpr int int_min = std::numeric_limits<int>::min();
  constexpr int int_max = std::numeric_limits<int>::max();

  gfx::Insets width_height_test(int_max);
  EXPECT_EQ(int_max, width_height_test.width());
  EXPECT_EQ(int_max, width_height_test.height());

  gfx::Insets plus_test(int_max);
  plus_test += gfx::Insets(int_max);
  EXPECT_EQ(gfx::Insets(int_max), plus_test);

  gfx::Insets negation_test = -gfx::Insets(int_min);
  EXPECT_EQ(gfx::Insets(int_max), negation_test);

  gfx::Insets scale_test(int_max);
  scale_test = scale_test.Scale(2.f, 2.f);
  EXPECT_EQ(gfx::Insets(int_max), scale_test);
}

TEST(InsetsTest, IntegerUnderflow) {
  constexpr int int_min = std::numeric_limits<int>::min();
  constexpr int int_max = std::numeric_limits<int>::max();

  gfx::Insets width_height_test = gfx::Insets(int_min);
  EXPECT_EQ(int_min, width_height_test.width());
  EXPECT_EQ(int_min, width_height_test.height());

  gfx::Insets minus_test(int_min);
  minus_test -= gfx::Insets(int_max);
  EXPECT_EQ(gfx::Insets(int_min), minus_test);

  gfx::Insets scale_test = gfx::Insets(int_min);
  scale_test = scale_test.Scale(2.f, 2.f);
  EXPECT_EQ(gfx::Insets(int_min), scale_test);
}

TEST(InsetsTest, IntegerOverflowSetVariants) {
  constexpr int int_max = std::numeric_limits<int>::max();

  gfx::Insets set_test(20);
  set_test.set_top(int_max);
  EXPECT_EQ(int_max, set_test.top());
  EXPECT_EQ(0, set_test.bottom());

  set_test.set_left(int_max);
  EXPECT_EQ(int_max, set_test.left());
  EXPECT_EQ(0, set_test.right());

  set_test = gfx::Insets(30);
  set_test.set_bottom(int_max);
  EXPECT_EQ(int_max - 30, set_test.bottom());
  EXPECT_EQ(30, set_test.top());

  set_test.set_right(int_max);
  EXPECT_EQ(int_max - 30, set_test.right());
  EXPECT_EQ(30, set_test.left());
}

TEST(InsetsTest, IntegerUnderflowSetVariants) {
  constexpr int int_min = std::numeric_limits<int>::min();

  gfx::Insets set_test(-20);
  set_test.set_top(int_min);
  EXPECT_EQ(int_min, set_test.top());
  EXPECT_EQ(0, set_test.bottom());

  set_test.set_left(int_min);
  EXPECT_EQ(int_min, set_test.left());
  EXPECT_EQ(0, set_test.right());

  set_test = gfx::Insets(-30);
  set_test.set_bottom(int_min);
  EXPECT_EQ(int_min + 30, set_test.bottom());
  EXPECT_EQ(-30, set_test.top());

  set_test.set_right(int_min);
  EXPECT_EQ(int_min + 30, set_test.right());
  EXPECT_EQ(-30, set_test.left());
}

TEST(InsetsTest, IntegerOverflowSet) {
  constexpr int int_max = std::numeric_limits<int>::max();

  gfx::Insets set_all_test;
  set_all_test.Set(10, 20, int_max, int_max);
  EXPECT_EQ(gfx::Insets(10, 20, int_max - 10, int_max - 20), set_all_test);
}

TEST(InsetsTest, IntegerOverflowOffset) {
  constexpr int int_max = std::numeric_limits<int>::max();

  const gfx::Vector2d max_vector(int_max, int_max);
  gfx::Insets insets(1, 2, 3, 4);
  gfx::Insets offset_test = insets.Offset(max_vector);
  EXPECT_EQ(gfx::Insets(int_max, int_max, 3 - int_max, 4 - int_max),
            offset_test);
}

TEST(InsetsTest, IntegerUnderflowOffset) {
  constexpr int int_min = std::numeric_limits<int>::min();

  const gfx::Vector2d min_vector(int_min, int_min);
  gfx::Insets insets(-10);
  gfx::Insets offset_test = insets.Offset(min_vector);
  EXPECT_EQ(gfx::Insets(int_min, int_min, -10 - int_min, -10 - int_min),
            offset_test);
}

TEST(InsetsTest, Size) {
  gfx::Insets insets(1, 2, 3, 4);
  EXPECT_EQ(gfx::Size(6, 4), insets.size());
}
