// Copyright (c) 2011 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <sstream>

#include "gtest/gtest.h"
#include "range.h"

namespace {

template <typename T>
class RangeTest : public testing::Test {};

typedef testing::Types<gfx::Range> RangeTypes;
TYPED_TEST_SUITE(RangeTest, RangeTypes);

template <typename T>
void TestContainsAndIntersects(const T& r1, const T& r2, const T& r3) {
  EXPECT_TRUE(r1.Intersects(r1));
  EXPECT_TRUE(r1.Contains(r1));
  EXPECT_EQ(T(10, 12), r1.Intersect(r1));

  EXPECT_FALSE(r1.Intersects(r2));
  EXPECT_FALSE(r1.Contains(r2));
  EXPECT_TRUE(r1.Intersect(r2).is_empty());
  EXPECT_FALSE(r2.Intersects(r1));
  EXPECT_FALSE(r2.Contains(r1));
  EXPECT_TRUE(r2.Intersect(r1).is_empty());

  EXPECT_TRUE(r1.Intersects(r3));
  EXPECT_TRUE(r3.Intersects(r1));
  EXPECT_TRUE(r3.Contains(r1));
  EXPECT_FALSE(r1.Contains(r3));
  EXPECT_EQ(T(10, 12), r1.Intersect(r3));
  EXPECT_EQ(T(10, 12), r3.Intersect(r1));

  EXPECT_TRUE(r2.Intersects(r3));
  EXPECT_TRUE(r3.Intersects(r2));
  EXPECT_FALSE(r3.Contains(r2));
  EXPECT_FALSE(r2.Contains(r3));
  EXPECT_EQ(T(5, 8), r2.Intersect(r3));
  EXPECT_EQ(T(5, 8), r3.Intersect(r2));
}

}  // namespace

TYPED_TEST(RangeTest, EmptyInit) {
  TypeParam r;
  EXPECT_EQ(0U, r.start());
  EXPECT_EQ(0U, r.end());
  EXPECT_EQ(0U, r.length());
  EXPECT_FALSE(r.is_reversed());
  EXPECT_TRUE(r.is_empty());
  EXPECT_TRUE(r.IsValid());
  EXPECT_EQ(0U, r.GetMin());
  EXPECT_EQ(0U, r.GetMax());
}

TYPED_TEST(RangeTest, StartEndInit) {
  TypeParam r(10, 15);
  EXPECT_EQ(10U, r.start());
  EXPECT_EQ(15U, r.end());
  EXPECT_EQ(5U, r.length());
  EXPECT_FALSE(r.is_reversed());
  EXPECT_FALSE(r.is_empty());
  EXPECT_TRUE(r.IsValid());
  EXPECT_EQ(10U, r.GetMin());
  EXPECT_EQ(15U, r.GetMax());
}

TYPED_TEST(RangeTest, StartEndReversedInit) {
  TypeParam r(10, 5);
  EXPECT_EQ(10U, r.start());
  EXPECT_EQ(5U, r.end());
  EXPECT_EQ(5U, r.length());
  EXPECT_TRUE(r.is_reversed());
  EXPECT_FALSE(r.is_empty());
  EXPECT_TRUE(r.IsValid());
  EXPECT_EQ(5U, r.GetMin());
  EXPECT_EQ(10U, r.GetMax());
}

TYPED_TEST(RangeTest, PositionInit) {
  TypeParam r(12);
  EXPECT_EQ(12U, r.start());
  EXPECT_EQ(12U, r.end());
  EXPECT_EQ(0U, r.length());
  EXPECT_FALSE(r.is_reversed());
  EXPECT_TRUE(r.is_empty());
  EXPECT_TRUE(r.IsValid());
  EXPECT_EQ(12U, r.GetMin());
  EXPECT_EQ(12U, r.GetMax());
}

TYPED_TEST(RangeTest, InvalidRange) {
  TypeParam r(TypeParam::InvalidRange());
  EXPECT_EQ(0U, r.length());
  EXPECT_EQ(r.start(), r.end());
  EXPECT_EQ(r.GetMax(), r.GetMin());
  EXPECT_FALSE(r.is_reversed());
  EXPECT_TRUE(r.is_empty());
  EXPECT_FALSE(r.IsValid());
  EXPECT_EQ(r, TypeParam::InvalidRange());
  EXPECT_TRUE(r.EqualsIgnoringDirection(TypeParam::InvalidRange()));
}

TYPED_TEST(RangeTest, Equality) {
  TypeParam r1(10, 4);
  TypeParam r2(10, 4);
  TypeParam r3(10, 2);
  EXPECT_EQ(r1, r2);
  EXPECT_NE(r1, r3);
  EXPECT_NE(r2, r3);

  TypeParam r4(11, 4);
  EXPECT_NE(r1, r4);
  EXPECT_NE(r2, r4);
  EXPECT_NE(r3, r4);

  TypeParam r5(12, 5);
  EXPECT_NE(r1, r5);
  EXPECT_NE(r2, r5);
  EXPECT_NE(r3, r5);
}

TYPED_TEST(RangeTest, EqualsIgnoringDirection) {
  TypeParam r1(10, 5);
  TypeParam r2(5, 10);
  EXPECT_TRUE(r1.EqualsIgnoringDirection(r2));
}

TYPED_TEST(RangeTest, SetStart) {
  TypeParam r(10, 20);
  EXPECT_EQ(10U, r.start());
  EXPECT_EQ(10U, r.length());

  r.set_start(42);
  EXPECT_EQ(42U, r.start());
  EXPECT_EQ(20U, r.end());
  EXPECT_EQ(22U, r.length());
  EXPECT_TRUE(r.is_reversed());
}

TYPED_TEST(RangeTest, SetEnd) {
  TypeParam r(10, 13);
  EXPECT_EQ(10U, r.start());
  EXPECT_EQ(3U, r.length());

  r.set_end(20);
  EXPECT_EQ(10U, r.start());
  EXPECT_EQ(20U, r.end());
  EXPECT_EQ(10U, r.length());
}

TYPED_TEST(RangeTest, SetStartAndEnd) {
  TypeParam r;
  r.set_end(5);
  r.set_start(1);
  EXPECT_EQ(1U, r.start());
  EXPECT_EQ(5U, r.end());
  EXPECT_EQ(4U, r.length());
  EXPECT_EQ(1U, r.GetMin());
  EXPECT_EQ(5U, r.GetMax());
}

TYPED_TEST(RangeTest, ReversedRange) {
  TypeParam r(10, 5);
  EXPECT_EQ(10U, r.start());
  EXPECT_EQ(5U, r.end());
  EXPECT_EQ(5U, r.length());
  EXPECT_TRUE(r.is_reversed());
  EXPECT_TRUE(r.IsValid());
  EXPECT_EQ(5U, r.GetMin());
  EXPECT_EQ(10U, r.GetMax());
}

TYPED_TEST(RangeTest, SetReversedRange) {
  TypeParam r(10, 20);
  r.set_start(25);
  EXPECT_EQ(25U, r.start());
  EXPECT_EQ(20U, r.end());
  EXPECT_EQ(5U, r.length());
  EXPECT_TRUE(r.is_reversed());
  EXPECT_TRUE(r.IsValid());

  r.set_end(21);
  EXPECT_EQ(25U, r.start());
  EXPECT_EQ(21U, r.end());
  EXPECT_EQ(4U, r.length());
  EXPECT_TRUE(r.IsValid());
  EXPECT_EQ(21U, r.GetMin());
  EXPECT_EQ(25U, r.GetMax());
}

TYPED_TEST(RangeTest, ContainAndIntersect) {
  {
    SCOPED_TRACE("contain and intersect");
    TypeParam r1(10, 12);
    TypeParam r2(1, 8);
    TypeParam r3(5, 12);
    TestContainsAndIntersects(r1, r2, r3);
  }
  {
    SCOPED_TRACE("contain and intersect: reversed");
    TypeParam r1(12, 10);
    TypeParam r2(8, 1);
    TypeParam r3(12, 5);
    TestContainsAndIntersects(r1, r2, r3);
  }
  // Invalid rect tests
  TypeParam r1(10, 12);
  TypeParam r2(8, 1);
  TypeParam invalid = r1.Intersect(r2);
  EXPECT_FALSE(invalid.IsValid());
  EXPECT_FALSE(invalid.Contains(invalid));
  EXPECT_FALSE(invalid.Contains(r1));
  EXPECT_FALSE(invalid.Intersects(invalid));
  EXPECT_FALSE(invalid.Intersects(r1));
  EXPECT_FALSE(r1.Contains(invalid));
  EXPECT_FALSE(r1.Intersects(invalid));
}

TEST(RangeTest, RangeOperations) {
  constexpr gfx::Range invalid_range = gfx::Range::InvalidRange();
  constexpr gfx::Range ranges[] = {{0, 0}, {0, 1}, {0, 2}, {1, 0}, {1, 1},
                                   {1, 2}, {2, 0}, {2, 1}, {2, 2}};

  // Ensures valid behavior over same range.
  for (const auto& range : ranges) {
    SCOPED_TRACE(range.ToString());
    // A range should contain itself.
    EXPECT_TRUE(range.Contains(range));
    // A ranges should intersect with itself.
    EXPECT_TRUE(range.Intersects(range));
  }

  // Ensures valid behavior with an invalid range.
  for (const auto& range : ranges) {
    SCOPED_TRACE(range.ToString());
    EXPECT_FALSE(invalid_range.Contains(range));
    EXPECT_FALSE(invalid_range.Intersects(range));
    EXPECT_FALSE(range.Contains(invalid_range));
    EXPECT_FALSE(range.Intersects(invalid_range));
  }
  EXPECT_FALSE(invalid_range.Contains(invalid_range));
  EXPECT_FALSE(invalid_range.Intersects(invalid_range));

  // Ensures consistent operations between Contains(...) and Intersects(...).
  for (const auto& range1 : ranges) {
    for (const auto& range2 : ranges) {
      SCOPED_TRACE(testing::Message()
                   << "range1=" << range1 << " range2=" << range2);
      if (range1.Contains(range2)) {
        EXPECT_TRUE(range1.Intersects(range2));
        EXPECT_EQ(range2.Contains(range1),
                  range1.EqualsIgnoringDirection(range2));
      }
      EXPECT_EQ(range2.Intersects(range1), range1.Intersects(range2));

      EXPECT_EQ(range1.Intersect(range2) != invalid_range,
                range1.Intersects(range2));
    }
  }

  // Ranges should behave the same way no matter the direction.
  for (const auto& range1 : ranges) {
    for (const auto& range2 : ranges) {
      SCOPED_TRACE(testing::Message()
                   << "range1=" << range1 << " range2=" << range2);
      EXPECT_EQ(range1.Contains(range2),
                range1.Contains(gfx::Range(range2.GetMax(), range2.GetMin())));
      EXPECT_EQ(
          range1.Intersects(range2),
          range1.Intersects(gfx::Range(range2.GetMax(), range2.GetMin())));
    }
  }
}

TEST(RangeTest, ContainsAndIntersects) {
  constexpr gfx::Range r1(0, 0);
  constexpr gfx::Range r2(0, 1);
  constexpr gfx::Range r3(1, 2);
  constexpr gfx::Range r4(1, 0);
  constexpr gfx::Range r5(2, 1);
  constexpr gfx::Range r6(0, 2);
  constexpr gfx::Range r7(2, 0);
  constexpr gfx::Range r8(1, 1);

  // Ensures Contains(...) handle the open range.
  EXPECT_TRUE(r2.Contains(r1));
  EXPECT_TRUE(r4.Contains(r1));
  EXPECT_TRUE(r3.Contains(r5));
  EXPECT_TRUE(r5.Contains(r3));

  // Ensures larger ranges contains smaller ranges.
  EXPECT_TRUE(r6.Contains(r1));
  EXPECT_TRUE(r6.Contains(r2));
  EXPECT_TRUE(r6.Contains(r3));
  EXPECT_TRUE(r6.Contains(r4));
  EXPECT_TRUE(r6.Contains(r5));

  EXPECT_TRUE(r7.Contains(r1));
  EXPECT_TRUE(r7.Contains(r2));
  EXPECT_TRUE(r7.Contains(r3));
  EXPECT_TRUE(r7.Contains(r4));
  EXPECT_TRUE(r7.Contains(r5));

  // Ensures Intersects(...) handle the open range.
  EXPECT_TRUE(r2.Intersects(r1));
  EXPECT_TRUE(r4.Intersects(r1));

  // Ensures larger ranges intersects smaller ranges.
  EXPECT_TRUE(r6.Intersects(r1));
  EXPECT_TRUE(r6.Intersects(r2));
  EXPECT_TRUE(r6.Intersects(r3));
  EXPECT_TRUE(r6.Intersects(r4));
  EXPECT_TRUE(r6.Intersects(r5));

  EXPECT_TRUE(r7.Intersects(r1));
  EXPECT_TRUE(r7.Intersects(r2));
  EXPECT_TRUE(r7.Intersects(r3));
  EXPECT_TRUE(r7.Intersects(r4));
  EXPECT_TRUE(r7.Intersects(r5));

  // Ensures adjacent ranges don't overlap.
  EXPECT_FALSE(r2.Intersects(r3));
  EXPECT_FALSE(r5.Intersects(r4));

  // Ensures empty ranges are handled correctly.
  EXPECT_FALSE(r1.Contains(r8));
  EXPECT_FALSE(r2.Contains(r8));
  EXPECT_TRUE(r3.Contains(r8));
  EXPECT_TRUE(r8.Contains(r8));

  EXPECT_FALSE(r1.Intersects(r8));
  EXPECT_FALSE(r2.Intersects(r8));
  EXPECT_TRUE(r3.Intersects(r8));
  EXPECT_TRUE(r8.Intersects(r8));
}

TEST(RangeTest, Intersect) {
  EXPECT_EQ(gfx::Range(0, 1).Intersect({0, 1}), gfx::Range(0, 1));
  EXPECT_EQ(gfx::Range(0, 1).Intersect({0, 0}), gfx::Range(0, 0));
  EXPECT_EQ(gfx::Range(0, 0).Intersect({1, 0}), gfx::Range(0, 0));
  EXPECT_EQ(gfx::Range(0, 4).Intersect({2, 3}), gfx::Range(2, 3));
  EXPECT_EQ(gfx::Range(0, 4).Intersect({2, 7}), gfx::Range(2, 4));
  EXPECT_EQ(gfx::Range(0, 4).Intersect({3, 4}), gfx::Range(3, 4));

  EXPECT_EQ(gfx::Range(0, 1).Intersect({1, 1}), gfx::Range::InvalidRange());
  EXPECT_EQ(gfx::Range(1, 1).Intersect({1, 0}), gfx::Range::InvalidRange());
  EXPECT_EQ(gfx::Range(0, 1).Intersect({1, 2}), gfx::Range::InvalidRange());
  EXPECT_EQ(gfx::Range(0, 1).Intersect({2, 1}), gfx::Range::InvalidRange());
  EXPECT_EQ(gfx::Range(2, 1).Intersect({1, 0}), gfx::Range::InvalidRange());
}

TEST(RangeTest, IsBoundedBy) {
  constexpr gfx::Range r1(0, 0);
  constexpr gfx::Range r2(0, 1);
  EXPECT_TRUE(r1.IsBoundedBy(r1));
  EXPECT_FALSE(r2.IsBoundedBy(r1));

  constexpr gfx::Range r3(0, 2);
  constexpr gfx::Range r4(2, 2);
  EXPECT_TRUE(r4.IsBoundedBy(r3));
  EXPECT_FALSE(r3.IsBoundedBy(r4));
}

TEST(RangeTest, ToString) {
  gfx::Range range(4, 7);
  EXPECT_EQ("{4,7}", range.ToString());

  range = gfx::Range::InvalidRange();
  std::ostringstream expected;
  expected << "{" << range.start() << "," << range.end() << "}";
  EXPECT_EQ(expected.str(), range.ToString());
}
