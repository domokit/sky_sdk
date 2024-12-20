// Copyright (c) 2013 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <cstddef>
#include <limits>

#include "ax_build/build_config.h"
#include "gfx/test/gfx_util.h"
#include "gtest/gtest.h"
#include "rect.h"
#include "rect_conversions.h"

#if defined(OS_WIN)
#include <windows.h>
#endif

namespace gfx {

template <typename T, size_t N>
constexpr size_t size(const T (&array)[N]) noexcept {
  return N;
}

TEST(RectTest, Contains) {
  static const struct ContainsCase {
    int rect_x;
    int rect_y;
    int rect_width;
    int rect_height;
    int point_x;
    int point_y;
    bool contained;
  } contains_cases[] = {
    {0, 0, 10, 10, 0, 0, true},
    {0, 0, 10, 10, 5, 5, true},
    {0, 0, 10, 10, 9, 9, true},
    {0, 0, 10, 10, 5, 10, false},
    {0, 0, 10, 10, 10, 5, false},
    {0, 0, 10, 10, -1, -1, false},
    {0, 0, 10, 10, 50, 50, false},
#if defined(NDEBUG) && !defined(DCHECK_ALWAYS_ON)
    {0, 0, -10, -10, 0, 0, false},
#endif
  };
  for (size_t i = 0; i < size(contains_cases); ++i) {
    const ContainsCase& value = contains_cases[i];
    Rect rect(value.rect_x, value.rect_y, value.rect_width, value.rect_height);
    EXPECT_EQ(value.contained, rect.Contains(value.point_x, value.point_y));
  }
}

TEST(RectTest, Intersects) {
  static const struct {
    int x1;  // rect 1
    int y1;
    int w1;
    int h1;
    int x2;  // rect 2
    int y2;
    int w2;
    int h2;
    bool intersects;
  } tests[] = {{0, 0, 0, 0, 0, 0, 0, 0, false},
               {0, 0, 0, 0, -10, -10, 20, 20, false},
               {-10, 0, 0, 20, 0, -10, 20, 0, false},
               {0, 0, 10, 10, 0, 0, 10, 10, true},
               {0, 0, 10, 10, 10, 10, 10, 10, false},
               {10, 10, 10, 10, 0, 0, 10, 10, false},
               {10, 10, 10, 10, 5, 5, 10, 10, true},
               {10, 10, 10, 10, 15, 15, 10, 10, true},
               {10, 10, 10, 10, 20, 15, 10, 10, false},
               {10, 10, 10, 10, 21, 15, 10, 10, false}};
  for (size_t i = 0; i < size(tests); ++i) {
    Rect r1(tests[i].x1, tests[i].y1, tests[i].w1, tests[i].h1);
    Rect r2(tests[i].x2, tests[i].y2, tests[i].w2, tests[i].h2);
    EXPECT_EQ(tests[i].intersects, r1.Intersects(r2));
    EXPECT_EQ(tests[i].intersects, r2.Intersects(r1));
  }
}

TEST(RectTest, Intersect) {
  static const struct {
    int x1;  // rect 1
    int y1;
    int w1;
    int h1;
    int x2;  // rect 2
    int y2;
    int w2;
    int h2;
    int x3;  // rect 3: the union of rects 1 and 2
    int y3;
    int w3;
    int h3;
  } tests[] = {{0, 0, 0, 0,  // zeros
                0, 0, 0, 0, 0, 0, 0, 0},
               {0, 0, 4, 4,  // equal
                0, 0, 4, 4, 0, 0, 4, 4},
               {0, 0, 4, 4,  // neighboring
                4, 4, 4, 4, 0, 0, 0, 0},
               {0, 0, 4, 4,  // overlapping corners
                2, 2, 4, 4, 2, 2, 2, 2},
               {0, 0, 4, 4,  // T junction
                3, 1, 4, 2, 3, 1, 1, 2},
               {3, 0, 2, 2,  // gap
                0, 0, 2, 2, 0, 0, 0, 0}};
  for (size_t i = 0; i < size(tests); ++i) {
    Rect r1(tests[i].x1, tests[i].y1, tests[i].w1, tests[i].h1);
    Rect r2(tests[i].x2, tests[i].y2, tests[i].w2, tests[i].h2);
    Rect r3(tests[i].x3, tests[i].y3, tests[i].w3, tests[i].h3);
    Rect ir = IntersectRects(r1, r2);
    EXPECT_EQ(r3.x(), ir.x());
    EXPECT_EQ(r3.y(), ir.y());
    EXPECT_EQ(r3.width(), ir.width());
    EXPECT_EQ(r3.height(), ir.height());
  }
}

TEST(RectTest, Union) {
  static const struct Test {
    int x1;  // rect 1
    int y1;
    int w1;
    int h1;
    int x2;  // rect 2
    int y2;
    int w2;
    int h2;
    int x3;  // rect 3: the union of rects 1 and 2
    int y3;
    int w3;
    int h3;
  } tests[] = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
               {0, 0, 4, 4, 0, 0, 4, 4, 0, 0, 4, 4},
               {0, 0, 4, 4, 4, 4, 4, 4, 0, 0, 8, 8},
               {0, 0, 4, 4, 0, 5, 4, 4, 0, 0, 4, 9},
               {0, 0, 2, 2, 3, 3, 2, 2, 0, 0, 5, 5},
               {3, 3, 2, 2,  // reverse r1 and r2 from previous test
                0, 0, 2, 2, 0, 0, 5, 5},
               {0, 0, 0, 0,  // union with empty rect
                2, 2, 2, 2, 2, 2, 2, 2}};
  for (size_t i = 0; i < size(tests); ++i) {
    Rect r1(tests[i].x1, tests[i].y1, tests[i].w1, tests[i].h1);
    Rect r2(tests[i].x2, tests[i].y2, tests[i].w2, tests[i].h2);
    Rect r3(tests[i].x3, tests[i].y3, tests[i].w3, tests[i].h3);
    Rect u = UnionRects(r1, r2);
    EXPECT_EQ(r3.x(), u.x());
    EXPECT_EQ(r3.y(), u.y());
    EXPECT_EQ(r3.width(), u.width());
    EXPECT_EQ(r3.height(), u.height());
  }
}

TEST(RectTest, Equals) {
  ASSERT_TRUE(Rect(0, 0, 0, 0) == Rect(0, 0, 0, 0));
  ASSERT_TRUE(Rect(1, 2, 3, 4) == Rect(1, 2, 3, 4));
  ASSERT_FALSE(Rect(0, 0, 0, 0) == Rect(0, 0, 0, 1));
  ASSERT_FALSE(Rect(0, 0, 0, 0) == Rect(0, 0, 1, 0));
  ASSERT_FALSE(Rect(0, 0, 0, 0) == Rect(0, 1, 0, 0));
  ASSERT_FALSE(Rect(0, 0, 0, 0) == Rect(1, 0, 0, 0));
}

TEST(RectTest, AdjustToFit) {
  static const struct Test {
    int x1;  // source
    int y1;
    int w1;
    int h1;
    int x2;  // target
    int y2;
    int w2;
    int h2;
    int x3;  // rect 3: results of invoking AdjustToFit
    int y3;
    int w3;
    int h3;
  } tests[] = {{0, 0, 2, 2, 0, 0, 2, 2, 0, 0, 2, 2},
               {2, 2, 3, 3, 0, 0, 4, 4, 1, 1, 3, 3},
               {-1, -1, 5, 5, 0, 0, 4, 4, 0, 0, 4, 4},
               {2, 2, 4, 4, 0, 0, 3, 3, 0, 0, 3, 3},
               {2, 2, 1, 1, 0, 0, 3, 3, 2, 2, 1, 1}};
  for (size_t i = 0; i < size(tests); ++i) {
    Rect r1(tests[i].x1, tests[i].y1, tests[i].w1, tests[i].h1);
    Rect r2(tests[i].x2, tests[i].y2, tests[i].w2, tests[i].h2);
    Rect r3(tests[i].x3, tests[i].y3, tests[i].w3, tests[i].h3);
    Rect u = r1;
    u.AdjustToFit(r2);
    EXPECT_EQ(r3.x(), u.x());
    EXPECT_EQ(r3.y(), u.y());
    EXPECT_EQ(r3.width(), u.width());
    EXPECT_EQ(r3.height(), u.height());
  }
}

TEST(RectTest, Subtract) {
  Rect result;

  // Matching
  result = Rect(10, 10, 20, 20);
  result.Subtract(Rect(10, 10, 20, 20));
  EXPECT_EQ(Rect(0, 0, 0, 0), result);

  // Contains
  result = Rect(10, 10, 20, 20);
  result.Subtract(Rect(5, 5, 30, 30));
  EXPECT_EQ(Rect(0, 0, 0, 0), result);

  // No intersection
  result = Rect(10, 10, 20, 20);
  result.Subtract(Rect(30, 30, 30, 30));
  EXPECT_EQ(Rect(10, 10, 20, 20), result);

  // Not a complete intersection in either direction
  result = Rect(10, 10, 20, 20);
  result.Subtract(Rect(15, 15, 20, 20));
  EXPECT_EQ(Rect(10, 10, 20, 20), result);

  // Complete intersection in the x-direction, top edge is fully covered.
  result = Rect(10, 10, 20, 20);
  result.Subtract(Rect(10, 15, 20, 20));
  EXPECT_EQ(Rect(10, 10, 20, 5), result);

  // Complete intersection in the x-direction, top edge is fully covered.
  result = Rect(10, 10, 20, 20);
  result.Subtract(Rect(5, 15, 30, 20));
  EXPECT_EQ(Rect(10, 10, 20, 5), result);

  // Complete intersection in the x-direction, bottom edge is fully covered.
  result = Rect(10, 10, 20, 20);
  result.Subtract(Rect(5, 5, 30, 20));
  EXPECT_EQ(Rect(10, 25, 20, 5), result);

  // Complete intersection in the x-direction, none of the edges is fully
  // covered.
  result = Rect(10, 10, 20, 20);
  result.Subtract(Rect(5, 15, 30, 1));
  EXPECT_EQ(Rect(10, 10, 20, 20), result);

  // Complete intersection in the y-direction, left edge is fully covered.
  result = Rect(10, 10, 20, 20);
  result.Subtract(Rect(10, 10, 10, 30));
  EXPECT_EQ(Rect(20, 10, 10, 20), result);

  // Complete intersection in the y-direction, left edge is fully covered.
  result = Rect(10, 10, 20, 20);
  result.Subtract(Rect(5, 5, 20, 30));
  EXPECT_EQ(Rect(25, 10, 5, 20), result);

  // Complete intersection in the y-direction, right edge is fully covered.
  result = Rect(10, 10, 20, 20);
  result.Subtract(Rect(20, 5, 20, 30));
  EXPECT_EQ(Rect(10, 10, 10, 20), result);

  // Complete intersection in the y-direction, none of the edges is fully
  // covered.
  result = Rect(10, 10, 20, 20);
  result.Subtract(Rect(15, 5, 1, 30));
  EXPECT_EQ(Rect(10, 10, 20, 20), result);
}

TEST(RectTest, IsEmpty) {
  EXPECT_TRUE(Rect(0, 0, 0, 0).IsEmpty());
  EXPECT_TRUE(Rect(0, 0, 0, 0).size().IsEmpty());
  EXPECT_TRUE(Rect(0, 0, 10, 0).IsEmpty());
  EXPECT_TRUE(Rect(0, 0, 10, 0).size().IsEmpty());
  EXPECT_TRUE(Rect(0, 0, 0, 10).IsEmpty());
  EXPECT_TRUE(Rect(0, 0, 0, 10).size().IsEmpty());
  EXPECT_FALSE(Rect(0, 0, 10, 10).IsEmpty());
  EXPECT_FALSE(Rect(0, 0, 10, 10).size().IsEmpty());
}

TEST(RectTest, SplitVertically) {
  Rect left_half, right_half;

  // Splitting when origin is (0, 0).
  Rect(0, 0, 20, 20).SplitVertically(&left_half, &right_half);
  EXPECT_TRUE(left_half == Rect(0, 0, 10, 20));
  EXPECT_TRUE(right_half == Rect(10, 0, 10, 20));

  // Splitting when origin is arbitrary.
  Rect(10, 10, 20, 10).SplitVertically(&left_half, &right_half);
  EXPECT_TRUE(left_half == Rect(10, 10, 10, 10));
  EXPECT_TRUE(right_half == Rect(20, 10, 10, 10));

  // Splitting a rectangle of zero width.
  Rect(10, 10, 0, 10).SplitVertically(&left_half, &right_half);
  EXPECT_TRUE(left_half == Rect(10, 10, 0, 10));
  EXPECT_TRUE(right_half == Rect(10, 10, 0, 10));

  // Splitting a rectangle of odd width.
  Rect(10, 10, 5, 10).SplitVertically(&left_half, &right_half);
  EXPECT_TRUE(left_half == Rect(10, 10, 2, 10));
  EXPECT_TRUE(right_half == Rect(12, 10, 3, 10));
}

TEST(RectTest, CenterPoint) {
  Point center;

  // When origin is (0, 0).
  center = Rect(0, 0, 20, 20).CenterPoint();
  EXPECT_TRUE(center == Point(10, 10));

  // When origin is even.
  center = Rect(10, 10, 20, 20).CenterPoint();
  EXPECT_TRUE(center == Point(20, 20));

  // When origin is odd.
  center = Rect(11, 11, 20, 20).CenterPoint();
  EXPECT_TRUE(center == Point(21, 21));

  // When 0 width or height.
  center = Rect(10, 10, 0, 20).CenterPoint();
  EXPECT_TRUE(center == Point(10, 20));
  center = Rect(10, 10, 20, 0).CenterPoint();
  EXPECT_TRUE(center == Point(20, 10));

  // When an odd size.
  center = Rect(10, 10, 21, 21).CenterPoint();
  EXPECT_TRUE(center == Point(20, 20));

  // When an odd size and position.
  center = Rect(11, 11, 21, 21).CenterPoint();
  EXPECT_TRUE(center == Point(21, 21));
}

TEST(RectTest, CenterPointF) {
  PointF center;

  // When origin is (0, 0).
  center = RectF(0, 0, 20, 20).CenterPoint();
  EXPECT_TRUE(center == PointF(10, 10));

  // When origin is even.
  center = RectF(10, 10, 20, 20).CenterPoint();
  EXPECT_TRUE(center == PointF(20, 20));

  // When origin is odd.
  center = RectF(11, 11, 20, 20).CenterPoint();
  EXPECT_TRUE(center == PointF(21, 21));

  // When 0 width or height.
  center = RectF(10, 10, 0, 20).CenterPoint();
  EXPECT_TRUE(center == PointF(10, 20));
  center = RectF(10, 10, 20, 0).CenterPoint();
  EXPECT_TRUE(center == PointF(20, 10));

  // When an odd size.
  center = RectF(10, 10, 21, 21).CenterPoint();
  EXPECT_TRUE(center == PointF(20.5f, 20.5f));

  // When an odd size and position.
  center = RectF(11, 11, 21, 21).CenterPoint();
  EXPECT_TRUE(center == PointF(21.5f, 21.5f));
}

TEST(RectTest, SharesEdgeWith) {
  Rect r(2, 3, 4, 5);

  // Must be non-overlapping
  EXPECT_FALSE(r.SharesEdgeWith(r));

  Rect just_above(2, 1, 4, 2);
  Rect just_below(2, 8, 4, 2);
  Rect just_left(0, 3, 2, 5);
  Rect just_right(6, 3, 2, 5);

  EXPECT_TRUE(r.SharesEdgeWith(just_above));
  EXPECT_TRUE(r.SharesEdgeWith(just_below));
  EXPECT_TRUE(r.SharesEdgeWith(just_left));
  EXPECT_TRUE(r.SharesEdgeWith(just_right));

  // Wrong placement
  Rect same_height_no_edge(0, 0, 1, 5);
  Rect same_width_no_edge(0, 0, 4, 1);

  EXPECT_FALSE(r.SharesEdgeWith(same_height_no_edge));
  EXPECT_FALSE(r.SharesEdgeWith(same_width_no_edge));

  Rect just_above_no_edge(2, 1, 5, 2);  // too wide
  Rect just_below_no_edge(2, 8, 3, 2);  // too narrow
  Rect just_left_no_edge(0, 3, 2, 6);   // too tall
  Rect just_right_no_edge(6, 3, 2, 4);  // too short

  EXPECT_FALSE(r.SharesEdgeWith(just_above_no_edge));
  EXPECT_FALSE(r.SharesEdgeWith(just_below_no_edge));
  EXPECT_FALSE(r.SharesEdgeWith(just_left_no_edge));
  EXPECT_FALSE(r.SharesEdgeWith(just_right_no_edge));
}

// Similar to EXPECT_FLOAT_EQ, but lets NaN equal NaN
#define EXPECT_FLOAT_AND_NAN_EQ(a, b) \
  {                                   \
    if (a == a || b == b) {           \
      EXPECT_FLOAT_EQ(a, b);          \
    }                                 \
  }

TEST(RectTest, ScaleRect) {
  static const struct Test {
    int x1;  // source
    int y1;
    int w1;
    int h1;
    float scale;
    float x2;  // target
    float y2;
    float w2;
    float h2;
  } tests[] = {
      {3, 3, 3, 3, 1.5f, 4.5f, 4.5f, 4.5f, 4.5f},
      {3, 3, 3, 3, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
      {3, 3, 3, 3, std::numeric_limits<float>::quiet_NaN(),
       std::numeric_limits<float>::quiet_NaN(),
       std::numeric_limits<float>::quiet_NaN(),
       std::numeric_limits<float>::quiet_NaN(),
       std::numeric_limits<float>::quiet_NaN()},
      {3, 3, 3, 3, std::numeric_limits<float>::max(),
       std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
       std::numeric_limits<float>::max(), std::numeric_limits<float>::max()}};

  for (size_t i = 0; i < size(tests); ++i) {
    RectF r1(tests[i].x1, tests[i].y1, tests[i].w1, tests[i].h1);
    RectF r2(tests[i].x2, tests[i].y2, tests[i].w2, tests[i].h2);

    RectF scaled = ScaleRect(r1, tests[i].scale);
    EXPECT_FLOAT_AND_NAN_EQ(r2.x(), scaled.x());
    EXPECT_FLOAT_AND_NAN_EQ(r2.y(), scaled.y());
    EXPECT_FLOAT_AND_NAN_EQ(r2.width(), scaled.width());
    EXPECT_FLOAT_AND_NAN_EQ(r2.height(), scaled.height());
  }
}

TEST(RectTest, ToEnclosedRect) {
  static const int max_int = std::numeric_limits<int>::max();
  static const int min_int = std::numeric_limits<int>::min();
  static const float max_float = std::numeric_limits<float>::max();
  static const float max_int_f = static_cast<float>(max_int);
  static const float min_int_f = static_cast<float>(min_int);

  static const struct Test {
    struct {
      float x;
      float y;
      float width;
      float height;
    } in;
    struct {
      int x;
      int y;
      int width;
      int height;
    } expected;
  } tests[] = {
      {{0.0f, 0.0f, 0.0f, 0.0f}, {0, 0, 0, 0}},
      {{-1.5f, -1.5f, 3.0f, 3.0f}, {-1, -1, 2, 2}},
      {{-1.5f, -1.5f, 3.5f, 3.5f}, {-1, -1, 3, 3}},
      {{max_float, max_float, 2.0f, 2.0f}, {max_int, max_int, 0, 0}},
      {{0.0f, 0.0f, max_float, max_float}, {0, 0, max_int, max_int}},
      {{20000.5f, 20000.5f, 0.5f, 0.5f}, {20001, 20001, 0, 0}},
      {{max_int_f, max_int_f, max_int_f, max_int_f}, {max_int, max_int, 0, 0}},
      {{1.9999f, 2.0002f, 5.9998f, 6.0001f}, {2, 3, 5, 5}},
      {{1.9999f, 2.0001f, 6.0002f, 5.9998f}, {2, 3, 6, 4}},
      {{1.9998f, 2.0002f, 6.0001f, 5.9999f}, {2, 3, 5, 5}}};

  for (size_t i = 0; i < size(tests); ++i) {
    RectF source(tests[i].in.x, tests[i].in.y, tests[i].in.width,
                 tests[i].in.height);
    Rect enclosed = ToEnclosedRect(source);

    EXPECT_EQ(tests[i].expected.x, enclosed.x());
    EXPECT_EQ(tests[i].expected.y, enclosed.y());
    EXPECT_EQ(tests[i].expected.width, enclosed.width());
    EXPECT_EQ(tests[i].expected.height, enclosed.height());
  }

  {
    RectF source(min_int_f, min_int_f, max_int_f * 3.f, max_int_f * 3.f);
    Rect enclosed = ToEnclosedRect(source);

    // That rect can't be represented, but it should be big.
    EXPECT_EQ(max_int, enclosed.width());
    EXPECT_EQ(max_int, enclosed.height());
    // It should include some axis near the global origin.
    EXPECT_GT(1, enclosed.x());
    EXPECT_GT(1, enclosed.y());
    // And it should not cause computation issues for itself.
    EXPECT_LT(0, enclosed.right());
    EXPECT_LT(0, enclosed.bottom());
  }
}

TEST(RectTest, ToEnclosingRect) {
  static const int max_int = std::numeric_limits<int>::max();
  static const int min_int = std::numeric_limits<int>::min();
  static const float max_float = std::numeric_limits<float>::max();
  static const float epsilon_float = std::numeric_limits<float>::epsilon();
  static const float max_int_f = static_cast<float>(max_int);
  static const float min_int_f = static_cast<float>(min_int);
  static const struct Test {
    struct {
      float x;
      float y;
      float width;
      float height;
    } in;
    struct {
      int x;
      int y;
      int width;
      int height;
    } expected;
  } tests[] = {
      {{0.0f, 0.0f, 0.0f, 0.0f}, {0, 0, 0, 0}},
      {{5.5f, 5.5f, 0.0f, 0.0f}, {5, 5, 0, 0}},
      {{3.5f, 2.5f, epsilon_float, -0.0f}, {3, 2, 0, 0}},
      {{3.5f, 2.5f, 0.f, 0.001f}, {3, 2, 0, 1}},
      {{-1.5f, -1.5f, 3.0f, 3.0f}, {-2, -2, 4, 4}},
      {{-1.5f, -1.5f, 3.5f, 3.5f}, {-2, -2, 4, 4}},
      {{max_float, max_float, 2.0f, 2.0f}, {max_int, max_int, 0, 0}},
      {{0.0f, 0.0f, max_float, max_float}, {0, 0, max_int, max_int}},
      {{20000.5f, 20000.5f, 0.5f, 0.5f}, {20000, 20000, 1, 1}},
      {{max_int_f, max_int_f, max_int_f, max_int_f}, {max_int, max_int, 0, 0}},
      {{-0.5f, -0.5f, 22777712.f, 1.f}, {-1, -1, 22777713, 2}},
      {{1.9999f, 2.0002f, 5.9998f, 6.0001f}, {1, 2, 7, 7}},
      {{1.9999f, 2.0001f, 6.0002f, 5.9998f}, {1, 2, 8, 6}},
      {{1.9998f, 2.0002f, 6.0001f, 5.9999f}, {1, 2, 7, 7}}};

  for (size_t i = 0; i < size(tests); ++i) {
    RectF source(tests[i].in.x, tests[i].in.y, tests[i].in.width,
                 tests[i].in.height);

    Rect enclosing = ToEnclosingRect(source);
    EXPECT_EQ(tests[i].expected.x, enclosing.x());
    EXPECT_EQ(tests[i].expected.y, enclosing.y());
    EXPECT_EQ(tests[i].expected.width, enclosing.width());
    EXPECT_EQ(tests[i].expected.height, enclosing.height());
  }

  {
    RectF source(min_int_f, min_int_f, max_int_f * 3.f, max_int_f * 3.f);
    Rect enclosing = ToEnclosingRect(source);

    // That rect can't be represented, but it should be big.
    EXPECT_EQ(max_int, enclosing.width());
    EXPECT_EQ(max_int, enclosing.height());
    // It should include some axis near the global origin.
    EXPECT_GT(1, enclosing.x());
    EXPECT_GT(1, enclosing.y());
    // And it should cause computation issues for itself.
    EXPECT_LT(0, enclosing.right());
    EXPECT_LT(0, enclosing.bottom());
  }
}

TEST(RectTest, ToEnclosingRectIgnoringError) {
  static const int max_int = std::numeric_limits<int>::max();
  static const float max_float = std::numeric_limits<float>::max();
  static const float epsilon_float = std::numeric_limits<float>::epsilon();
  static const float max_int_f = static_cast<float>(max_int);
  static const float error = 0.001f;
  static const struct Test {
    struct {
      float x;
      float y;
      float width;
      float height;
    } in;
    struct {
      int x;
      int y;
      int width;
      int height;
    } expected;
  } tests[] = {
      {{0.0f, 0.0f, 0.0f, 0.0f}, {0, 0, 0, 0}},
      {{5.5f, 5.5f, 0.0f, 0.0f}, {5, 5, 0, 0}},
      {{3.5f, 2.5f, epsilon_float, -0.0f}, {3, 2, 0, 0}},
      {{3.5f, 2.5f, 0.f, 0.001f}, {3, 2, 0, 1}},
      {{-1.5f, -1.5f, 3.0f, 3.0f}, {-2, -2, 4, 4}},
      {{-1.5f, -1.5f, 3.5f, 3.5f}, {-2, -2, 4, 4}},
      {{max_float, max_float, 2.0f, 2.0f}, {max_int, max_int, 0, 0}},
      {{0.0f, 0.0f, max_float, max_float}, {0, 0, max_int, max_int}},
      {{20000.5f, 20000.5f, 0.5f, 0.5f}, {20000, 20000, 1, 1}},
      {{max_int_f, max_int_f, max_int_f, max_int_f}, {max_int, max_int, 0, 0}},
      {{-0.5f, -0.5f, 22777712.f, 1.f}, {-1, -1, 22777713, 2}},
      {{1.9999f, 2.0002f, 5.9998f, 6.0001f}, {2, 2, 6, 6}},
      {{1.9999f, 2.0001f, 6.0002f, 5.9998f}, {2, 2, 6, 6}},
      {{1.9998f, 2.0002f, 6.0001f, 5.9999f}, {2, 2, 6, 6}}};

  for (size_t i = 0; i < size(tests); ++i) {
    RectF source(tests[i].in.x, tests[i].in.y, tests[i].in.width,
                 tests[i].in.height);

    Rect enclosing = ToEnclosingRectIgnoringError(source, error);
    EXPECT_EQ(tests[i].expected.x, enclosing.x());
    EXPECT_EQ(tests[i].expected.y, enclosing.y());
    EXPECT_EQ(tests[i].expected.width, enclosing.width());
    EXPECT_EQ(tests[i].expected.height, enclosing.height());
  }
}

TEST(RectTest, ToNearestRect) {
  Rect rect;
  EXPECT_EQ(rect, ToNearestRect(RectF(rect)));

  rect = Rect(-1, -1, 3, 3);
  EXPECT_EQ(rect, ToNearestRect(RectF(rect)));

  RectF rectf(-1.00001f, -0.999999f, 3.0000001f, 2.999999f);
  EXPECT_EQ(rect, ToNearestRect(rectf));
}

TEST(RectTest, ToFlooredRect) {
  static const struct Test {
    float x1;  // source
    float y1;
    float w1;
    float h1;
    int x2;  // target
    int y2;
    int w2;
    int h2;
  } tests[] = {
      {0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 0, 0},
      {-1.5f, -1.5f, 3.0f, 3.0f, -2, -2, 3, 3},
      {-1.5f, -1.5f, 3.5f, 3.5f, -2, -2, 3, 3},
      {20000.5f, 20000.5f, 0.5f, 0.5f, 20000, 20000, 0, 0},
  };

  for (size_t i = 0; i < size(tests); ++i) {
    RectF r1(tests[i].x1, tests[i].y1, tests[i].w1, tests[i].h1);
    Rect r2(tests[i].x2, tests[i].y2, tests[i].w2, tests[i].h2);

    Rect floored = ToFlooredRectDeprecated(r1);
    EXPECT_FLOAT_EQ(r2.x(), floored.x());
    EXPECT_FLOAT_EQ(r2.y(), floored.y());
    EXPECT_FLOAT_EQ(r2.width(), floored.width());
    EXPECT_FLOAT_EQ(r2.height(), floored.height());
  }
}

TEST(RectTest, ScaleToEnclosedRect) {
  static const struct Test {
    Rect input_rect;
    float input_scale;
    Rect expected_rect;
  } tests[] = {{
                   Rect(),
                   5.f,
                   Rect(),
               },
               {
                   Rect(1, 1, 1, 1),
                   5.f,
                   Rect(5, 5, 5, 5),
               },
               {
                   Rect(-1, -1, 0, 0),
                   5.f,
                   Rect(-5, -5, 0, 0),
               },
               {
                   Rect(1, -1, 0, 1),
                   5.f,
                   Rect(5, -5, 0, 5),
               },
               {
                   Rect(-1, 1, 1, 0),
                   5.f,
                   Rect(-5, 5, 5, 0),
               },
               {
                   Rect(1, 2, 3, 4),
                   1.5f,
                   Rect(2, 3, 4, 6),
               },
               {
                   Rect(-1, -2, 0, 0),
                   1.5f,
                   Rect(-1, -3, 0, 0),
               }};

  for (size_t i = 0; i < size(tests); ++i) {
    Rect result =
        ScaleToEnclosedRect(tests[i].input_rect, tests[i].input_scale);
    EXPECT_EQ(tests[i].expected_rect, result);
  }
}

TEST(RectTest, ScaleToEnclosingRect) {
  static const struct Test {
    Rect input_rect;
    float input_scale;
    Rect expected_rect;
  } tests[] = {{
                   Rect(),
                   5.f,
                   Rect(),
               },
               {
                   Rect(1, 1, 1, 1),
                   5.f,
                   Rect(5, 5, 5, 5),
               },
               {
                   Rect(-1, -1, 0, 0),
                   5.f,
                   Rect(-5, -5, 0, 0),
               },
               {
                   Rect(1, -1, 0, 1),
                   5.f,
                   Rect(5, -5, 0, 5),
               },
               {
                   Rect(-1, 1, 1, 0),
                   5.f,
                   Rect(-5, 5, 5, 0),
               },
               {
                   Rect(1, 2, 3, 4),
                   1.5f,
                   Rect(1, 3, 5, 6),
               },
               {
                   Rect(-1, -2, 0, 0),
                   1.5f,
                   Rect(-2, -3, 0, 0),
               }};

  for (size_t i = 0; i < size(tests); ++i) {
    Rect result =
        ScaleToEnclosingRect(tests[i].input_rect, tests[i].input_scale);
    EXPECT_EQ(tests[i].expected_rect, result);
    Rect result_safe =
        ScaleToEnclosingRectSafe(tests[i].input_rect, tests[i].input_scale);
    EXPECT_EQ(tests[i].expected_rect, result_safe);
  }
}

#if defined(OS_WIN)
TEST(RectTest, ConstructAndAssign) {
  const RECT rect_1 = {0, 0, 10, 10};
  const RECT rect_2 = {0, 0, -10, -10};
  Rect test1(rect_1);
  Rect test2(rect_2);
}
#endif

TEST(RectTest, ToRectF) {
  // Check that explicit conversion from integer to float compiles.
  Rect a(10, 20, 30, 40);
  RectF b(10, 20, 30, 40);

  RectF c = RectF(a);
  EXPECT_EQ(b, c);
}

TEST(RectTest, BoundingRect) {
  struct {
    Point a;
    Point b;
    Rect expected;
  } int_tests[] = {
      // If point B dominates A, then A should be the origin.
      {Point(4, 6), Point(4, 6), Rect(4, 6, 0, 0)},
      {Point(4, 6), Point(8, 6), Rect(4, 6, 4, 0)},
      {Point(4, 6), Point(4, 9), Rect(4, 6, 0, 3)},
      {Point(4, 6), Point(8, 9), Rect(4, 6, 4, 3)},
      // If point A dominates B, then B should be the origin.
      {Point(4, 6), Point(4, 6), Rect(4, 6, 0, 0)},
      {Point(8, 6), Point(4, 6), Rect(4, 6, 4, 0)},
      {Point(4, 9), Point(4, 6), Rect(4, 6, 0, 3)},
      {Point(8, 9), Point(4, 6), Rect(4, 6, 4, 3)},
      // If neither point dominates, then the origin is a combination of the
      // two.
      {Point(4, 6), Point(6, 4), Rect(4, 4, 2, 2)},
      {Point(-4, -6), Point(-6, -4), Rect(-6, -6, 2, 2)},
      {Point(-4, 6), Point(6, -4), Rect(-4, -4, 10, 10)},
  };

  for (size_t i = 0; i < size(int_tests); ++i) {
    Rect actual = BoundingRect(int_tests[i].a, int_tests[i].b);
    EXPECT_EQ(int_tests[i].expected, actual);
  }

  struct {
    PointF a;
    PointF b;
    RectF expected;
  } float_tests[] = {
      // If point B dominates A, then A should be the origin.
      {PointF(4.2f, 6.8f), PointF(4.2f, 6.8f), RectF(4.2f, 6.8f, 0, 0)},
      {PointF(4.2f, 6.8f), PointF(8.5f, 6.8f), RectF(4.2f, 6.8f, 4.3f, 0)},
      {PointF(4.2f, 6.8f), PointF(4.2f, 9.3f), RectF(4.2f, 6.8f, 0, 2.5f)},
      {PointF(4.2f, 6.8f), PointF(8.5f, 9.3f), RectF(4.2f, 6.8f, 4.3f, 2.5f)},
      // If point A dominates B, then B should be the origin.
      {PointF(4.2f, 6.8f), PointF(4.2f, 6.8f), RectF(4.2f, 6.8f, 0, 0)},
      {PointF(8.5f, 6.8f), PointF(4.2f, 6.8f), RectF(4.2f, 6.8f, 4.3f, 0)},
      {PointF(4.2f, 9.3f), PointF(4.2f, 6.8f), RectF(4.2f, 6.8f, 0, 2.5f)},
      {PointF(8.5f, 9.3f), PointF(4.2f, 6.8f), RectF(4.2f, 6.8f, 4.3f, 2.5f)},
      // If neither point dominates, then the origin is a combination of the
      // two.
      {PointF(4.2f, 6.8f), PointF(6.8f, 4.2f), RectF(4.2f, 4.2f, 2.6f, 2.6f)},
      {PointF(-4.2f, -6.8f), PointF(-6.8f, -4.2f),
       RectF(-6.8f, -6.8f, 2.6f, 2.6f)},
      {PointF(-4.2f, 6.8f), PointF(6.8f, -4.2f),
       RectF(-4.2f, -4.2f, 11.0f, 11.0f)}};

  for (size_t i = 0; i < size(float_tests); ++i) {
    RectF actual = BoundingRect(float_tests[i].a, float_tests[i].b);
    EXPECT_RECTF_EQ(float_tests[i].expected, actual);
  }
}

TEST(RectTest, IsExpressibleAsRect) {
  EXPECT_TRUE(RectF().IsExpressibleAsRect());

  float min = static_cast<float>(std::numeric_limits<int>::min());
  float max = static_cast<float>(std::numeric_limits<int>::max());
  float infinity = std::numeric_limits<float>::infinity();

  EXPECT_TRUE(
      RectF(min + 200, min + 200, max - 200, max - 200).IsExpressibleAsRect());
  EXPECT_FALSE(
      RectF(min - 200, min + 200, max + 200, max + 200).IsExpressibleAsRect());
  EXPECT_FALSE(
      RectF(min + 200, min - 200, max + 200, max + 200).IsExpressibleAsRect());
  EXPECT_FALSE(
      RectF(min + 200, min + 200, max + 200, max - 200).IsExpressibleAsRect());
  EXPECT_FALSE(
      RectF(min + 200, min + 200, max - 200, max + 200).IsExpressibleAsRect());

  EXPECT_TRUE(RectF(0, 0, max - 200, max - 200).IsExpressibleAsRect());
  EXPECT_FALSE(RectF(200, 0, max + 200, max - 200).IsExpressibleAsRect());
  EXPECT_FALSE(RectF(0, 200, max - 200, max + 200).IsExpressibleAsRect());
  EXPECT_FALSE(RectF(0, 0, max + 200, max - 200).IsExpressibleAsRect());
  EXPECT_FALSE(RectF(0, 0, max - 200, max + 200).IsExpressibleAsRect());

  EXPECT_FALSE(RectF(infinity, 0, 1, 1).IsExpressibleAsRect());
  EXPECT_FALSE(RectF(0, infinity, 1, 1).IsExpressibleAsRect());
  EXPECT_FALSE(RectF(0, 0, infinity, 1).IsExpressibleAsRect());
  EXPECT_FALSE(RectF(0, 0, 1, infinity).IsExpressibleAsRect());
}

TEST(RectTest, Offset) {
  Rect i(1, 2, 3, 4);

  EXPECT_EQ(Rect(2, 1, 3, 4), (i + Vector2d(1, -1)));
  EXPECT_EQ(Rect(2, 1, 3, 4), (Vector2d(1, -1) + i));
  i += Vector2d(1, -1);
  EXPECT_EQ(Rect(2, 1, 3, 4), i);
  EXPECT_EQ(Rect(1, 2, 3, 4), (i - Vector2d(1, -1)));
  i -= Vector2d(1, -1);
  EXPECT_EQ(Rect(1, 2, 3, 4), i);

  RectF f(1.1f, 2.2f, 3.3f, 4.4f);
  EXPECT_EQ(RectF(2.2f, 1.1f, 3.3f, 4.4f), (f + Vector2dF(1.1f, -1.1f)));
  EXPECT_EQ(RectF(2.2f, 1.1f, 3.3f, 4.4f), (Vector2dF(1.1f, -1.1f) + f));
  f += Vector2dF(1.1f, -1.1f);
  EXPECT_EQ(RectF(2.2f, 1.1f, 3.3f, 4.4f), f);
  EXPECT_EQ(RectF(1.1f, 2.2f, 3.3f, 4.4f), (f - Vector2dF(1.1f, -1.1f)));
  f -= Vector2dF(1.1f, -1.1f);
  EXPECT_EQ(RectF(1.1f, 2.2f, 3.3f, 4.4f), f);
}

TEST(RectTest, Corners) {
  Rect i(1, 2, 3, 4);
  RectF f(1.1f, 2.1f, 3.1f, 4.1f);

  EXPECT_EQ(Point(1, 2), i.origin());
  EXPECT_EQ(Point(4, 2), i.top_right());
  EXPECT_EQ(Point(1, 6), i.bottom_left());
  EXPECT_EQ(Point(4, 6), i.bottom_right());

  EXPECT_EQ(PointF(1.1f, 2.1f), f.origin());
  EXPECT_EQ(PointF(4.2f, 2.1f), f.top_right());
  EXPECT_EQ(PointF(1.1f, 6.2f), f.bottom_left());
  EXPECT_EQ(PointF(4.2f, 6.2f), f.bottom_right());
}

TEST(RectTest, Centers) {
  Rect i(10, 20, 30, 40);
  EXPECT_EQ(Point(10, 40), i.left_center());
  EXPECT_EQ(Point(25, 20), i.top_center());
  EXPECT_EQ(Point(40, 40), i.right_center());
  EXPECT_EQ(Point(25, 60), i.bottom_center());

  RectF f(10.1f, 20.2f, 30.3f, 40.4f);
  EXPECT_EQ(PointF(10.1f, 40.4f), f.left_center());
  EXPECT_EQ(PointF(25.25f, 20.2f), f.top_center());
  EXPECT_EQ(PointF(40.4f, 40.4f), f.right_center());
  EXPECT_EQ(25.25f, f.bottom_center().x());
  EXPECT_NEAR(60.6f, f.bottom_center().y(), 0.001f);
}

TEST(RectTest, Transpose) {
  Rect i(10, 20, 30, 40);
  i.Transpose();
  EXPECT_EQ(Rect(20, 10, 40, 30), i);

  RectF f(10.1f, 20.2f, 30.3f, 40.4f);
  f.Transpose();
  EXPECT_EQ(RectF(20.2f, 10.1f, 40.4f, 30.3f), f);
}

TEST(RectTest, ManhattanDistanceToPoint) {
  Rect i(1, 2, 3, 4);
  EXPECT_EQ(0, i.ManhattanDistanceToPoint(Point(1, 2)));
  EXPECT_EQ(0, i.ManhattanDistanceToPoint(Point(4, 6)));
  EXPECT_EQ(0, i.ManhattanDistanceToPoint(Point(2, 4)));
  EXPECT_EQ(3, i.ManhattanDistanceToPoint(Point(0, 0)));
  EXPECT_EQ(2, i.ManhattanDistanceToPoint(Point(2, 0)));
  EXPECT_EQ(3, i.ManhattanDistanceToPoint(Point(5, 0)));
  EXPECT_EQ(1, i.ManhattanDistanceToPoint(Point(5, 4)));
  EXPECT_EQ(3, i.ManhattanDistanceToPoint(Point(5, 8)));
  EXPECT_EQ(2, i.ManhattanDistanceToPoint(Point(3, 8)));
  EXPECT_EQ(2, i.ManhattanDistanceToPoint(Point(0, 7)));
  EXPECT_EQ(1, i.ManhattanDistanceToPoint(Point(0, 3)));

  RectF f(1.1f, 2.1f, 3.1f, 4.1f);
  EXPECT_FLOAT_EQ(0.f, f.ManhattanDistanceToPoint(PointF(1.1f, 2.1f)));
  EXPECT_FLOAT_EQ(0.f, f.ManhattanDistanceToPoint(PointF(4.2f, 6.f)));
  EXPECT_FLOAT_EQ(0.f, f.ManhattanDistanceToPoint(PointF(2.f, 4.f)));
  EXPECT_FLOAT_EQ(3.2f, f.ManhattanDistanceToPoint(PointF(0.f, 0.f)));
  EXPECT_FLOAT_EQ(2.1f, f.ManhattanDistanceToPoint(PointF(2.f, 0.f)));
  EXPECT_FLOAT_EQ(2.9f, f.ManhattanDistanceToPoint(PointF(5.f, 0.f)));
  EXPECT_FLOAT_EQ(.8f, f.ManhattanDistanceToPoint(PointF(5.f, 4.f)));
  EXPECT_FLOAT_EQ(2.6f, f.ManhattanDistanceToPoint(PointF(5.f, 8.f)));
  EXPECT_FLOAT_EQ(1.8f, f.ManhattanDistanceToPoint(PointF(3.f, 8.f)));
  EXPECT_FLOAT_EQ(1.9f, f.ManhattanDistanceToPoint(PointF(0.f, 7.f)));
  EXPECT_FLOAT_EQ(1.1f, f.ManhattanDistanceToPoint(PointF(0.f, 3.f)));
}

TEST(RectTest, ManhattanInternalDistance) {
  Rect i(0, 0, 400, 400);
  EXPECT_EQ(0, i.ManhattanInternalDistance(gfx::Rect(-1, 0, 2, 1)));
  EXPECT_EQ(1, i.ManhattanInternalDistance(gfx::Rect(400, 0, 1, 400)));
  EXPECT_EQ(2, i.ManhattanInternalDistance(gfx::Rect(-100, -100, 100, 100)));
  EXPECT_EQ(2, i.ManhattanInternalDistance(gfx::Rect(-101, 100, 100, 100)));
  EXPECT_EQ(4, i.ManhattanInternalDistance(gfx::Rect(-101, -101, 100, 100)));
  EXPECT_EQ(435, i.ManhattanInternalDistance(gfx::Rect(630, 603, 100, 100)));

  RectF f(0.0f, 0.0f, 400.0f, 400.0f);
  static const float kEpsilon = std::numeric_limits<float>::epsilon();

  EXPECT_FLOAT_EQ(
      0.0f, f.ManhattanInternalDistance(gfx::RectF(-1.0f, 0.0f, 2.0f, 1.0f)));
  EXPECT_FLOAT_EQ(kEpsilon, f.ManhattanInternalDistance(
                                gfx::RectF(400.0f, 0.0f, 1.0f, 400.0f)));
  EXPECT_FLOAT_EQ(2.0f * kEpsilon, f.ManhattanInternalDistance(gfx::RectF(
                                       -100.0f, -100.0f, 100.0f, 100.0f)));
  EXPECT_FLOAT_EQ(1.0f + kEpsilon, f.ManhattanInternalDistance(gfx::RectF(
                                       -101.0f, 100.0f, 100.0f, 100.0f)));
  EXPECT_FLOAT_EQ(2.0f + 2.0f * kEpsilon,
                  f.ManhattanInternalDistance(
                      gfx::RectF(-101.0f, -101.0f, 100.0f, 100.0f)));
  EXPECT_FLOAT_EQ(
      433.0f + 2.0f * kEpsilon,
      f.ManhattanInternalDistance(gfx::RectF(630.0f, 603.0f, 100.0f, 100.0f)));

  EXPECT_FLOAT_EQ(
      0.0f, f.ManhattanInternalDistance(gfx::RectF(-1.0f, 0.0f, 1.1f, 1.0f)));
  EXPECT_FLOAT_EQ(0.1f + kEpsilon, f.ManhattanInternalDistance(
                                       gfx::RectF(-1.5f, 0.0f, 1.4f, 1.0f)));
  EXPECT_FLOAT_EQ(kEpsilon, f.ManhattanInternalDistance(
                                gfx::RectF(-1.5f, 0.0f, 1.5f, 1.0f)));
}

TEST(RectTest, IntegerOverflow) {
  int limit = std::numeric_limits<int>::max();
  int min_limit = std::numeric_limits<int>::min();
  int expected = 10;
  int large_number = limit - expected;

  Rect height_overflow(0, large_number, 100, 100);
  EXPECT_EQ(large_number, height_overflow.y());
  EXPECT_EQ(expected, height_overflow.height());

  Rect width_overflow(large_number, 0, 100, 100);
  EXPECT_EQ(large_number, width_overflow.x());
  EXPECT_EQ(expected, width_overflow.width());

  Rect size_height_overflow(Point(0, large_number), Size(100, 100));
  EXPECT_EQ(large_number, size_height_overflow.y());
  EXPECT_EQ(expected, size_height_overflow.height());

  Rect size_width_overflow(Point(large_number, 0), Size(100, 100));
  EXPECT_EQ(large_number, size_width_overflow.x());
  EXPECT_EQ(expected, size_width_overflow.width());

  Rect set_height_overflow(0, large_number, 100, 5);
  EXPECT_EQ(5, set_height_overflow.height());
  set_height_overflow.set_height(100);
  EXPECT_EQ(expected, set_height_overflow.height());

  Rect set_y_overflow(100, 100, 100, 100);
  EXPECT_EQ(100, set_y_overflow.height());
  set_y_overflow.set_y(large_number);
  EXPECT_EQ(expected, set_y_overflow.height());

  Rect set_width_overflow(large_number, 0, 5, 100);
  EXPECT_EQ(5, set_width_overflow.width());
  set_width_overflow.set_width(100);
  EXPECT_EQ(expected, set_width_overflow.width());

  Rect set_x_overflow(100, 100, 100, 100);
  EXPECT_EQ(100, set_x_overflow.width());
  set_x_overflow.set_x(large_number);
  EXPECT_EQ(expected, set_x_overflow.width());

  Point large_offset(large_number, large_number);
  Size size(100, 100);
  Size expected_size(10, 10);

  Rect set_origin_overflow(100, 100, 100, 100);
  EXPECT_EQ(size, set_origin_overflow.size());
  set_origin_overflow.set_origin(large_offset);
  EXPECT_EQ(large_offset, set_origin_overflow.origin());
  EXPECT_EQ(expected_size, set_origin_overflow.size());

  Rect set_size_overflow(large_number, large_number, 5, 5);
  EXPECT_EQ(Size(5, 5), set_size_overflow.size());
  set_size_overflow.set_size(size);
  EXPECT_EQ(large_offset, set_size_overflow.origin());
  EXPECT_EQ(expected_size, set_size_overflow.size());

  Rect set_rect_overflow;
  set_rect_overflow.SetRect(large_number, large_number, 100, 100);
  EXPECT_EQ(large_offset, set_rect_overflow.origin());
  EXPECT_EQ(expected_size, set_rect_overflow.size());

  // Insetting an empty rect, but the total inset (left + right) could overflow.
  Rect inset_overflow;
  inset_overflow.Inset(large_number, large_number, 100, 100);
  EXPECT_EQ(large_offset, inset_overflow.origin());
  EXPECT_EQ(gfx::Size(), inset_overflow.size());

  // Insetting where the total inset (width - left - right) could overflow.
  // Also, this insetting by the min limit in all directions cannot
  // represent width() without overflow, so that will also clamp.
  Rect inset_overflow2;
  inset_overflow2.Inset(min_limit, min_limit, min_limit, min_limit);
  EXPECT_EQ(inset_overflow2, gfx::Rect(min_limit, min_limit, limit, limit));

  // Insetting where the width shouldn't change, but if the insets operations
  // clamped in the wrong order, e.g. ((width - left) - right) vs (width - (left
  // + right)) then this will not work properly.  This is the proper order,
  // as if left + right overflows, the width cannot be decreased by more than
  // max int anyway.  Additionally, if left + right underflows, it cannot be
  // increased by more then max int.
  Rect inset_overflow3(0, 0, limit, limit);
  inset_overflow3.Inset(-100, -100, 100, 100);
  EXPECT_EQ(inset_overflow3, gfx::Rect(-100, -100, limit, limit));

  Rect inset_overflow4(-1000, -1000, limit, limit);
  inset_overflow4.Inset(100, 100, -100, -100);
  EXPECT_EQ(inset_overflow4, gfx::Rect(-900, -900, limit, limit));

  Rect offset_overflow(0, 0, 100, 100);
  offset_overflow.Offset(large_number, large_number);
  EXPECT_EQ(large_offset, offset_overflow.origin());
  EXPECT_EQ(expected_size, offset_overflow.size());

  Rect operator_overflow(0, 0, 100, 100);
  operator_overflow += Vector2d(large_number, large_number);
  EXPECT_EQ(large_offset, operator_overflow.origin());
  EXPECT_EQ(expected_size, operator_overflow.size());

  Rect origin_maxint(limit, limit, limit, limit);
  EXPECT_EQ(origin_maxint, Rect(gfx::Point(limit, limit), gfx::Size()));

  // Expect a rect at the origin and a rect whose right/bottom is maxint
  // create a rect that extends from 0..maxint in both extents.
  {
    Rect origin_small(0, 0, 100, 100);
    Rect big_clamped(50, 50, limit, limit);
    EXPECT_EQ(big_clamped.right(), limit);

    Rect unioned = UnionRects(origin_small, big_clamped);
    Rect rect_limit(0, 0, limit, limit);
    EXPECT_EQ(unioned, rect_limit);
  }

  // Expect a rect that would overflow width (but not right) to be clamped
  // and to have maxint extents after unioning.
  {
    Rect small(-500, -400, 100, 100);
    Rect big(-400, -500, limit, limit);
    // Technically, this should be limit + 100 width, but will clamp to maxint.
    EXPECT_EQ(UnionRects(small, big), Rect(-500, -500, limit, limit));
  }

  // Expect a rect that would overflow right *and* width to be clamped.
  {
    Rect clamped(500, 500, limit, limit);
    Rect positive_origin(100, 100, 500, 500);

    // Ideally, this should be (100, 100, limit + 400, limit + 400).
    // However, width overflows and would be clamped to limit, but right
    // overflows too and so will be clamped to limit - 100.
    Rect expected(100, 100, limit - 100, limit - 100);
    EXPECT_EQ(UnionRects(clamped, positive_origin), expected);
  }

  // Unioning a left=minint rect with a right=maxint rect.
  // We can't represent both ends of the spectrum in the same rect.
  // Make sure we keep the most useful area.
  {
    int part_limit = min_limit / 3;
    Rect left_minint(min_limit, min_limit, 1, 1);
    Rect right_maxint(limit - 1, limit - 1, limit, limit);
    Rect expected(part_limit, part_limit, 2 * part_limit, 2 * part_limit);
    Rect result = UnionRects(left_minint, right_maxint);

    // The result should be maximally big.
    EXPECT_EQ(limit, result.height());
    EXPECT_EQ(limit, result.width());

    // The result should include the area near the origin.
    EXPECT_GT(-part_limit, result.x());
    EXPECT_LT(part_limit, result.right());
    EXPECT_GT(-part_limit, result.y());
    EXPECT_LT(part_limit, result.bottom());

    // More succinctly, but harder to read in the results.
    EXPECT_TRUE(UnionRects(left_minint, right_maxint).Contains(expected));
  }
}

TEST(RectTest, ScaleToEnclosingRectSafe) {
  const int max_int = std::numeric_limits<int>::max();
  const int min_int = std::numeric_limits<int>::min();
  const float max_float = std::numeric_limits<float>::max();

  Rect xy_underflow(-100000, -123456, 10, 20);
  EXPECT_EQ(ScaleToEnclosingRectSafe(xy_underflow, 100000, 100000),
            Rect(min_int, min_int, 1000000, 2000000));

  // A location overflow means that width/right and bottom/top also
  // overflow so need to be clamped.
  Rect xy_overflow(100000, 123456, 10, 20);
  EXPECT_EQ(ScaleToEnclosingRectSafe(xy_overflow, 100000, 100000),
            Rect(max_int, max_int, 0, 0));

  // In practice all rects are clamped to 0 width / 0 height so
  // negative sizes don't matter, but try this for the sake of testing.
  Rect size_underflow(-1, -2, 100000, 100000);
  EXPECT_EQ(ScaleToEnclosingRectSafe(size_underflow, -100000, -100000),
            Rect(100000, 200000, 0, 0));

  Rect size_overflow(-1, -2, 123456, 234567);
  EXPECT_EQ(ScaleToEnclosingRectSafe(size_overflow, 100000, 100000),
            Rect(-100000, -200000, max_int, max_int));
  // Verify width/right gets clamped properly too if x/y positive.
  Rect size_overflow2(1, 2, 123456, 234567);
  EXPECT_EQ(ScaleToEnclosingRectSafe(size_overflow2, 100000, 100000),
            Rect(100000, 200000, max_int - 100000, max_int - 200000));

  Rect max_rect(max_int, max_int, max_int, max_int);
  EXPECT_EQ(ScaleToEnclosingRectSafe(max_rect, max_float, max_float),
            Rect(max_int, max_int, 0, 0));

  Rect min_rect(min_int, min_int, max_int, max_int);
  // Min rect can't be scaled up any further in any dimension.
  EXPECT_EQ(ScaleToEnclosingRectSafe(min_rect, 2, 3.5), min_rect);
  EXPECT_EQ(ScaleToEnclosingRectSafe(min_rect, max_float, max_float), min_rect);
  // Min rect scaled by min is an empty rect at (max, max)
  EXPECT_EQ(ScaleToEnclosingRectSafe(min_rect, min_int, min_int), max_rect);
}

}  // namespace gfx
