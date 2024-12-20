// Copyright (c) 2012 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "size.h"
#include "gtest/gtest.h"
#include "size_conversions.h"
#include "size_f.h"

namespace gfx {

namespace {

static constexpr float kTrivial = 8.f * std::numeric_limits<float>::epsilon();

int TestSizeF(const SizeF& s) {
  return s.width();
}

}  // namespace

TEST(SizeTest, ToSizeF) {
  // Check that explicit conversion from integer to float compiles.
  Size a(10, 20);
  float width = TestSizeF(gfx::SizeF(a));
  EXPECT_EQ(width, a.width());

  SizeF b(10, 20);

  EXPECT_EQ(b, gfx::SizeF(a));
}

TEST(SizeTest, ToFlooredSize) {
  EXPECT_EQ(Size(0, 0), ToFlooredSize(SizeF(0, 0)));
  EXPECT_EQ(Size(0, 0), ToFlooredSize(SizeF(0.0001f, 0.0001f)));
  EXPECT_EQ(Size(0, 0), ToFlooredSize(SizeF(0.4999f, 0.4999f)));
  EXPECT_EQ(Size(0, 0), ToFlooredSize(SizeF(0.5f, 0.5f)));
  EXPECT_EQ(Size(0, 0), ToFlooredSize(SizeF(0.9999f, 0.9999f)));

  EXPECT_EQ(Size(10, 10), ToFlooredSize(SizeF(10, 10)));
  EXPECT_EQ(Size(10, 10), ToFlooredSize(SizeF(10.0001f, 10.0001f)));
  EXPECT_EQ(Size(10, 10), ToFlooredSize(SizeF(10.4999f, 10.4999f)));
  EXPECT_EQ(Size(10, 10), ToFlooredSize(SizeF(10.5f, 10.5f)));
  EXPECT_EQ(Size(10, 10), ToFlooredSize(SizeF(10.9999f, 10.9999f)));
}

TEST(SizeTest, ToCeiledSize) {
  EXPECT_EQ(Size(0, 0), ToCeiledSize(SizeF(0, 0)));
  EXPECT_EQ(Size(1, 1), ToCeiledSize(SizeF(0.0001f, 0.0001f)));
  EXPECT_EQ(Size(1, 1), ToCeiledSize(SizeF(0.4999f, 0.4999f)));
  EXPECT_EQ(Size(1, 1), ToCeiledSize(SizeF(0.5f, 0.5f)));
  EXPECT_EQ(Size(1, 1), ToCeiledSize(SizeF(0.9999f, 0.9999f)));

  EXPECT_EQ(Size(10, 10), ToCeiledSize(SizeF(10, 10)));
  EXPECT_EQ(Size(11, 11), ToCeiledSize(SizeF(10.0001f, 10.0001f)));
  EXPECT_EQ(Size(11, 11), ToCeiledSize(SizeF(10.4999f, 10.4999f)));
  EXPECT_EQ(Size(11, 11), ToCeiledSize(SizeF(10.5f, 10.5f)));
  EXPECT_EQ(Size(11, 11), ToCeiledSize(SizeF(10.9999f, 10.9999f)));
}

TEST(SizeTest, ToRoundedSize) {
  EXPECT_EQ(Size(0, 0), ToRoundedSize(SizeF(0, 0)));
  EXPECT_EQ(Size(0, 0), ToRoundedSize(SizeF(0.0001f, 0.0001f)));
  EXPECT_EQ(Size(0, 0), ToRoundedSize(SizeF(0.4999f, 0.4999f)));
  EXPECT_EQ(Size(1, 1), ToRoundedSize(SizeF(0.5f, 0.5f)));
  EXPECT_EQ(Size(1, 1), ToRoundedSize(SizeF(0.9999f, 0.9999f)));

  EXPECT_EQ(Size(10, 10), ToRoundedSize(SizeF(10, 10)));
  EXPECT_EQ(Size(10, 10), ToRoundedSize(SizeF(10.0001f, 10.0001f)));
  EXPECT_EQ(Size(10, 10), ToRoundedSize(SizeF(10.4999f, 10.4999f)));
  EXPECT_EQ(Size(11, 11), ToRoundedSize(SizeF(10.5f, 10.5f)));
  EXPECT_EQ(Size(11, 11), ToRoundedSize(SizeF(10.9999f, 10.9999f)));
}

TEST(SizeTest, ClampSize) {
  Size a;

  a = Size(3, 5);
  EXPECT_EQ(Size(3, 5).ToString(), a.ToString());
  a.SetToMax(Size(2, 4));
  EXPECT_EQ(Size(3, 5).ToString(), a.ToString());
  a.SetToMax(Size(3, 5));
  EXPECT_EQ(Size(3, 5).ToString(), a.ToString());
  a.SetToMax(Size(4, 2));
  EXPECT_EQ(Size(4, 5).ToString(), a.ToString());
  a.SetToMax(Size(8, 10));
  EXPECT_EQ(Size(8, 10).ToString(), a.ToString());

  a.SetToMin(Size(9, 11));
  EXPECT_EQ(Size(8, 10).ToString(), a.ToString());
  a.SetToMin(Size(8, 10));
  EXPECT_EQ(Size(8, 10).ToString(), a.ToString());
  a.SetToMin(Size(11, 9));
  EXPECT_EQ(Size(8, 9).ToString(), a.ToString());
  a.SetToMin(Size(7, 11));
  EXPECT_EQ(Size(7, 9).ToString(), a.ToString());
  a.SetToMin(Size(3, 5));
  EXPECT_EQ(Size(3, 5).ToString(), a.ToString());
}

TEST(SizeTest, ClampSizeF) {
  SizeF a;

  a = SizeF(3.5f, 5.5f);
  EXPECT_EQ(SizeF(3.5f, 5.5f).ToString(), a.ToString());
  a.SetToMax(SizeF(2.5f, 4.5f));
  EXPECT_EQ(SizeF(3.5f, 5.5f).ToString(), a.ToString());
  a.SetToMax(SizeF(3.5f, 5.5f));
  EXPECT_EQ(SizeF(3.5f, 5.5f).ToString(), a.ToString());
  a.SetToMax(SizeF(4.5f, 2.5f));
  EXPECT_EQ(SizeF(4.5f, 5.5f).ToString(), a.ToString());
  a.SetToMax(SizeF(8.5f, 10.5f));
  EXPECT_EQ(SizeF(8.5f, 10.5f).ToString(), a.ToString());

  a.SetToMin(SizeF(9.5f, 11.5f));
  EXPECT_EQ(SizeF(8.5f, 10.5f).ToString(), a.ToString());
  a.SetToMin(SizeF(8.5f, 10.5f));
  EXPECT_EQ(SizeF(8.5f, 10.5f).ToString(), a.ToString());
  a.SetToMin(SizeF(11.5f, 9.5f));
  EXPECT_EQ(SizeF(8.5f, 9.5f).ToString(), a.ToString());
  a.SetToMin(SizeF(7.5f, 11.5f));
  EXPECT_EQ(SizeF(7.5f, 9.5f).ToString(), a.ToString());
  a.SetToMin(SizeF(3.5f, 5.5f));
  EXPECT_EQ(SizeF(3.5f, 5.5f).ToString(), a.ToString());
}

TEST(SizeTest, Enlarge) {
  Size test(3, 4);
  test.Enlarge(5, -8);
  EXPECT_EQ(test, Size(8, -4));
}

TEST(SizeTest, IntegerOverflow) {
  int int_max = std::numeric_limits<int>::max();
  int int_min = std::numeric_limits<int>::min();

  Size max_size(int_max, int_max);
  Size min_size(int_min, int_min);
  Size test;

  test = Size();
  test.Enlarge(int_max, int_max);
  EXPECT_EQ(test, max_size);

  test = Size();
  test.Enlarge(int_min, int_min);
  EXPECT_EQ(test, min_size);

  test = Size(10, 20);
  test.Enlarge(int_max, int_max);
  EXPECT_EQ(test, max_size);

  test = Size(-10, -20);
  test.Enlarge(int_min, int_min);
  EXPECT_EQ(test, min_size);
}

// This checks that we set IsEmpty appropriately.
TEST(SizeTest, TrivialDimensionTests) {
  const float clearly_trivial = kTrivial / 2.f;
  const float massize_dimension = 4e13f;

  // First, using the constructor.
  EXPECT_TRUE(SizeF(clearly_trivial, 1.f).IsEmpty());
  EXPECT_TRUE(SizeF(.01f, clearly_trivial).IsEmpty());
  EXPECT_TRUE(SizeF(0.f, 0.f).IsEmpty());
  EXPECT_FALSE(SizeF(.01f, .01f).IsEmpty());

  // Then use the setter.
  SizeF test(2.f, 1.f);
  EXPECT_FALSE(test.IsEmpty());

  test.SetSize(clearly_trivial, 1.f);
  EXPECT_TRUE(test.IsEmpty());

  test.SetSize(.01f, clearly_trivial);
  EXPECT_TRUE(test.IsEmpty());

  test.SetSize(0.f, 0.f);
  EXPECT_TRUE(test.IsEmpty());

  test.SetSize(.01f, .01f);
  EXPECT_FALSE(test.IsEmpty());

  // Now just one dimension at a time.
  test.set_width(clearly_trivial);
  EXPECT_TRUE(test.IsEmpty());

  test.set_width(massize_dimension);
  test.set_height(clearly_trivial);
  EXPECT_TRUE(test.IsEmpty());

  test.set_width(clearly_trivial);
  test.set_height(massize_dimension);
  EXPECT_TRUE(test.IsEmpty());

  test.set_width(2.f);
  EXPECT_FALSE(test.IsEmpty());
}

// These are the ramifications of the decision to keep the recorded size
// at zero for trivial sizes.
TEST(SizeTest, ClampsToZero) {
  const float clearly_trivial = kTrivial / 2.f;
  const float nearly_trivial = kTrivial * 1.5f;

  SizeF test(clearly_trivial, 1.f);

  EXPECT_FLOAT_EQ(0.f, test.width());
  EXPECT_FLOAT_EQ(1.f, test.height());

  test.SetSize(.01f, clearly_trivial);

  EXPECT_FLOAT_EQ(.01f, test.width());
  EXPECT_FLOAT_EQ(0.f, test.height());

  test.SetSize(nearly_trivial, nearly_trivial);

  EXPECT_FLOAT_EQ(nearly_trivial, test.width());
  EXPECT_FLOAT_EQ(nearly_trivial, test.height());

  test.Scale(0.5f);

  EXPECT_FLOAT_EQ(0.f, test.width());
  EXPECT_FLOAT_EQ(0.f, test.height());

  test.SetSize(0.f, 0.f);
  test.Enlarge(clearly_trivial, clearly_trivial);
  test.Enlarge(clearly_trivial, clearly_trivial);
  test.Enlarge(clearly_trivial, clearly_trivial);

  EXPECT_EQ(SizeF(0.f, 0.f), test);
}

// These make sure the constructor and setter have the same effect on the
// boundary case. This claims to know the boundary, but not which way it goes.
TEST(SizeTest, ConsistentClamping) {
  SizeF resized;

  resized.SetSize(kTrivial, 0.f);
  EXPECT_EQ(SizeF(kTrivial, 0.f), resized);

  resized.SetSize(0.f, kTrivial);
  EXPECT_EQ(SizeF(0.f, kTrivial), resized);
}

// Let's make sure we don't unexpectedly grow the struct by adding constants.
// Also, if some platform packs floats inefficiently, it would be worth noting.
TEST(SizeTest, StaysSmall) {
  EXPECT_EQ(2 * sizeof(float), sizeof(SizeF));
}

TEST(SizeTest, OperatorAddSub) {
  Size lhs(100, 20);
  Size rhs(50, 10);

  lhs += rhs;
  EXPECT_EQ(Size(150, 30), lhs);

  lhs = Size(100, 20);
  EXPECT_EQ(Size(150, 30), lhs + rhs);

  lhs = Size(100, 20);
  lhs -= rhs;
  EXPECT_EQ(Size(50, 10), lhs);

  lhs = Size(100, 20);
  EXPECT_EQ(Size(50, 10), lhs - rhs);
}

TEST(SizeTest, OperatorAddOverflow) {
  int int_max = std::numeric_limits<int>::max();

  Size lhs(int_max, int_max);
  Size rhs(int_max, int_max);
  EXPECT_EQ(Size(int_max, int_max), lhs + rhs);
}

TEST(SizeTest, OperatorSubClampAtZero) {
  Size lhs(10, 10);
  Size rhs(100, 100);
  EXPECT_EQ(Size(0, 0), lhs - rhs);

  lhs = Size(10, 10);
  rhs = Size(100, 100);
  lhs -= rhs;
  EXPECT_EQ(Size(0, 0), lhs);
}

TEST(SizeTest, OperatorCompare) {
  Size lhs(100, 20);
  Size rhs(50, 10);

  EXPECT_TRUE(lhs != rhs);
  EXPECT_FALSE(lhs == rhs);

  rhs = Size(100, 20);
  EXPECT_TRUE(lhs == rhs);
  EXPECT_FALSE(lhs != rhs);
}

}  // namespace gfx
