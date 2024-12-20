// Copyright (c) 2012 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <cmath>
#include <cstddef>
#include <limits>

#include "gtest/gtest.h"
#include "vector2d.h"
#include "vector2d_f.h"

namespace gfx {

template <typename T, size_t N>
constexpr size_t size(const T (&array)[N]) noexcept {
  return N;
}

TEST(Vector2dTest, ConversionToFloat) {
  Vector2d i(3, 4);
  Vector2dF f = i;
  EXPECT_EQ(i, f);
}

TEST(Vector2dTest, IsZero) {
  Vector2d int_zero(0, 0);
  Vector2d int_nonzero(2, -2);
  Vector2dF float_zero(0, 0);
  Vector2dF float_nonzero(0.1f, -0.1f);

  EXPECT_TRUE(int_zero.IsZero());
  EXPECT_FALSE(int_nonzero.IsZero());
  EXPECT_TRUE(float_zero.IsZero());
  EXPECT_FALSE(float_nonzero.IsZero());
}

TEST(Vector2dTest, Add) {
  Vector2d i1(3, 5);
  Vector2d i2(4, -1);

  const struct {
    Vector2d expected;
    Vector2d actual;
  } int_tests[] = {{Vector2d(3, 5), i1 + Vector2d()},
                   {Vector2d(3 + 4, 5 - 1), i1 + i2},
                   {Vector2d(3 - 4, 5 + 1), i1 - i2}};

  for (size_t i = 0; i < size(int_tests); ++i)
    EXPECT_EQ(int_tests[i].expected.ToString(), int_tests[i].actual.ToString());

  Vector2dF f1(3.1f, 5.1f);
  Vector2dF f2(4.3f, -1.3f);

  const struct {
    Vector2dF expected;
    Vector2dF actual;
  } float_tests[] = {{Vector2dF(3.1F, 5.1F), f1 + Vector2d()},
                     {Vector2dF(3.1F, 5.1F), f1 + Vector2dF()},
                     {Vector2dF(3.1f + 4.3f, 5.1f - 1.3f), f1 + f2},
                     {Vector2dF(3.1f - 4.3f, 5.1f + 1.3f), f1 - f2}};

  for (size_t i = 0; i < size(float_tests); ++i)
    EXPECT_EQ(float_tests[i].expected.ToString(),
              float_tests[i].actual.ToString());
}

TEST(Vector2dTest, Negative) {
  const struct {
    Vector2d expected;
    Vector2d actual;
  } int_tests[] = {{Vector2d(0, 0), -Vector2d(0, 0)},
                   {Vector2d(-3, -3), -Vector2d(3, 3)},
                   {Vector2d(3, 3), -Vector2d(-3, -3)},
                   {Vector2d(-3, 3), -Vector2d(3, -3)},
                   {Vector2d(3, -3), -Vector2d(-3, 3)}};

  for (size_t i = 0; i < size(int_tests); ++i)
    EXPECT_EQ(int_tests[i].expected.ToString(), int_tests[i].actual.ToString());

  const struct {
    Vector2dF expected;
    Vector2dF actual;
  } float_tests[] = {{Vector2dF(0, 0), -Vector2d(0, 0)},
                     {Vector2dF(-0.3f, -0.3f), -Vector2dF(0.3f, 0.3f)},
                     {Vector2dF(0.3f, 0.3f), -Vector2dF(-0.3f, -0.3f)},
                     {Vector2dF(-0.3f, 0.3f), -Vector2dF(0.3f, -0.3f)},
                     {Vector2dF(0.3f, -0.3f), -Vector2dF(-0.3f, 0.3f)}};

  for (size_t i = 0; i < size(float_tests); ++i)
    EXPECT_EQ(float_tests[i].expected.ToString(),
              float_tests[i].actual.ToString());
}

TEST(Vector2dTest, Scale) {
  float double_values[][4] = {
      {4.5f, 1.2f, 3.3f, 5.6f},  {4.5f, -1.2f, 3.3f, 5.6f},
      {4.5f, 1.2f, 3.3f, -5.6f}, {4.5f, 1.2f, -3.3f, -5.6f},
      {-4.5f, 1.2f, 3.3f, 5.6f}, {-4.5f, 1.2f, 0, 5.6f},
      {-4.5f, 1.2f, 3.3f, 0},    {4.5f, 0, 3.3f, 5.6f},
      {0, 1.2f, 3.3f, 5.6f}};

  for (size_t i = 0; i < size(double_values); ++i) {
    Vector2dF v(double_values[i][0], double_values[i][1]);
    v.Scale(double_values[i][2], double_values[i][3]);
    EXPECT_EQ(v.x(), double_values[i][0] * double_values[i][2]);
    EXPECT_EQ(v.y(), double_values[i][1] * double_values[i][3]);

    Vector2dF v2 =
        ScaleVector2d(gfx::Vector2dF(double_values[i][0], double_values[i][1]),
                      double_values[i][2], double_values[i][3]);
    EXPECT_EQ(double_values[i][0] * double_values[i][2], v2.x());
    EXPECT_EQ(double_values[i][1] * double_values[i][3], v2.y());
  }

  float single_values[][3] = {
      {4.5f, 1.2f, 3.3f},  {4.5f, -1.2f, 3.3f}, {4.5f, 1.2f, 3.3f},
      {4.5f, 1.2f, -3.3f}, {-4.5f, 1.2f, 3.3f}, {-4.5f, 1.2f, 0},
      {-4.5f, 1.2f, 3.3f}, {4.5f, 0, 3.3f},     {0, 1.2f, 3.3f}};

  for (size_t i = 0; i < size(single_values); ++i) {
    Vector2dF v(single_values[i][0], single_values[i][1]);
    v.Scale(single_values[i][2]);
    EXPECT_EQ(v.x(), single_values[i][0] * single_values[i][2]);
    EXPECT_EQ(v.y(), single_values[i][1] * single_values[i][2]);

    Vector2dF v2 =
        ScaleVector2d(gfx::Vector2dF(double_values[i][0], double_values[i][1]),
                      double_values[i][2]);
    EXPECT_EQ(single_values[i][0] * single_values[i][2], v2.x());
    EXPECT_EQ(single_values[i][1] * single_values[i][2], v2.y());
  }
}

TEST(Vector2dTest, Length) {
  int int_values[][2] = {
      {0, 0}, {10, 20}, {20, 10}, {-10, -20}, {-20, 10}, {10, -20},
  };

  for (size_t i = 0; i < size(int_values); ++i) {
    int v0 = int_values[i][0];
    int v1 = int_values[i][1];
    double length_squared =
        static_cast<double>(v0) * v0 + static_cast<double>(v1) * v1;
    double length = std::sqrt(length_squared);
    Vector2d vector(v0, v1);
    EXPECT_EQ(static_cast<float>(length_squared), vector.LengthSquared());
    EXPECT_EQ(static_cast<float>(length), vector.Length());
  }

  float float_values[][2] = {
      {0, 0},
      {10.5f, 20.5f},
      {20.5f, 10.5f},
      {-10.5f, -20.5f},
      {-20.5f, 10.5f},
      {10.5f, -20.5f},
      // A large vector that fails if the Length function doesn't use
      // double precision internally.
      {1236278317862780234892374893213178027.12122348904204230f,
       335890352589839028212313231225425134332.38123f},
  };

  for (size_t i = 0; i < size(float_values); ++i) {
    double v0 = float_values[i][0];
    double v1 = float_values[i][1];
    double length_squared =
        static_cast<double>(v0) * v0 + static_cast<double>(v1) * v1;
    double length = std::sqrt(length_squared);
    Vector2dF vector(v0, v1);
    EXPECT_DOUBLE_EQ(length_squared, vector.LengthSquared());
    EXPECT_FLOAT_EQ(static_cast<float>(length), vector.Length());
  }
}

TEST(Vector2dTest, ClampVector2d) {
  Vector2d a;

  a = Vector2d(3, 5);
  EXPECT_EQ(Vector2d(3, 5).ToString(), a.ToString());
  a.SetToMax(Vector2d(2, 4));
  EXPECT_EQ(Vector2d(3, 5).ToString(), a.ToString());
  a.SetToMax(Vector2d(3, 5));
  EXPECT_EQ(Vector2d(3, 5).ToString(), a.ToString());
  a.SetToMax(Vector2d(4, 2));
  EXPECT_EQ(Vector2d(4, 5).ToString(), a.ToString());
  a.SetToMax(Vector2d(8, 10));
  EXPECT_EQ(Vector2d(8, 10).ToString(), a.ToString());

  a.SetToMin(Vector2d(9, 11));
  EXPECT_EQ(Vector2d(8, 10).ToString(), a.ToString());
  a.SetToMin(Vector2d(8, 10));
  EXPECT_EQ(Vector2d(8, 10).ToString(), a.ToString());
  a.SetToMin(Vector2d(11, 9));
  EXPECT_EQ(Vector2d(8, 9).ToString(), a.ToString());
  a.SetToMin(Vector2d(7, 11));
  EXPECT_EQ(Vector2d(7, 9).ToString(), a.ToString());
  a.SetToMin(Vector2d(3, 5));
  EXPECT_EQ(Vector2d(3, 5).ToString(), a.ToString());
}

TEST(Vector2dTest, ClampVector2dF) {
  Vector2dF a;

  a = Vector2dF(3.5f, 5.5f);
  EXPECT_EQ(Vector2dF(3.5f, 5.5f).ToString(), a.ToString());
  a.SetToMax(Vector2dF(2.5f, 4.5f));
  EXPECT_EQ(Vector2dF(3.5f, 5.5f).ToString(), a.ToString());
  a.SetToMax(Vector2dF(3.5f, 5.5f));
  EXPECT_EQ(Vector2dF(3.5f, 5.5f).ToString(), a.ToString());
  a.SetToMax(Vector2dF(4.5f, 2.5f));
  EXPECT_EQ(Vector2dF(4.5f, 5.5f).ToString(), a.ToString());
  a.SetToMax(Vector2dF(8.5f, 10.5f));
  EXPECT_EQ(Vector2dF(8.5f, 10.5f).ToString(), a.ToString());

  a.SetToMin(Vector2dF(9.5f, 11.5f));
  EXPECT_EQ(Vector2dF(8.5f, 10.5f).ToString(), a.ToString());
  a.SetToMin(Vector2dF(8.5f, 10.5f));
  EXPECT_EQ(Vector2dF(8.5f, 10.5f).ToString(), a.ToString());
  a.SetToMin(Vector2dF(11.5f, 9.5f));
  EXPECT_EQ(Vector2dF(8.5f, 9.5f).ToString(), a.ToString());
  a.SetToMin(Vector2dF(7.5f, 11.5f));
  EXPECT_EQ(Vector2dF(7.5f, 9.5f).ToString(), a.ToString());
  a.SetToMin(Vector2dF(3.5f, 5.5f));
  EXPECT_EQ(Vector2dF(3.5f, 5.5f).ToString(), a.ToString());
}

TEST(Vector2dTest, IntegerOverflow) {
  int int_max = std::numeric_limits<int>::max();
  int int_min = std::numeric_limits<int>::min();

  Vector2d max_vector(int_max, int_max);
  Vector2d min_vector(int_min, int_min);
  Vector2d test;

  test = Vector2d();
  test += Vector2d(int_max, int_max);
  EXPECT_EQ(test, max_vector);

  test = Vector2d();
  test += Vector2d(int_min, int_min);
  EXPECT_EQ(test, min_vector);

  test = Vector2d(10, 20);
  test += Vector2d(int_max, int_max);
  EXPECT_EQ(test, max_vector);

  test = Vector2d(-10, -20);
  test += Vector2d(int_min, int_min);
  EXPECT_EQ(test, min_vector);

  test = Vector2d();
  test -= Vector2d(int_max, int_max);
  EXPECT_EQ(test, Vector2d(-int_max, -int_max));

  test = Vector2d();
  test -= Vector2d(int_min, int_min);
  EXPECT_EQ(test, max_vector);

  test = Vector2d(10, 20);
  test -= Vector2d(int_min, int_min);
  EXPECT_EQ(test, max_vector);

  test = Vector2d(-10, -20);
  test -= Vector2d(int_max, int_max);
  EXPECT_EQ(test, min_vector);
}

}  // namespace gfx
