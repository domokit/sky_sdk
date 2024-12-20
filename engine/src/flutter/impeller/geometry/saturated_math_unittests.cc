// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "gtest/gtest.h"

#include "impeller/geometry/saturated_math.h"

namespace impeller {
namespace testing {

TEST(SaturatedMath, ExplicitAddOfSignedInts) {
  {
    EXPECT_EQ(saturated::Add<int8_t>(0x79, 5), int8_t(0x7E));
    EXPECT_EQ(saturated::Add<int8_t>(0x7A, 5), int8_t(0x7F));
    EXPECT_EQ(saturated::Add<int8_t>(0x7B, 5), int8_t(0x7F));
  }
  {
    EXPECT_EQ(saturated::Add<int8_t>(0x86, -5), int8_t(0x81));
    EXPECT_EQ(saturated::Add<int8_t>(0x85, -5), int8_t(0x80));
    EXPECT_EQ(saturated::Add<int8_t>(0x84, -5), int8_t(0x80));
  }
  {
    EXPECT_EQ(saturated::Add<int16_t>(0x7FF9, 5), int16_t(0x7FFE));
    EXPECT_EQ(saturated::Add<int16_t>(0x7FFA, 5), int16_t(0x7FFF));
    EXPECT_EQ(saturated::Add<int16_t>(0x7FFB, 5), int16_t(0x7FFF));
  }
  {
    EXPECT_EQ(saturated::Add<int16_t>(0x8006, -5), int16_t(0x8001));
    EXPECT_EQ(saturated::Add<int16_t>(0x8005, -5), int16_t(0x8000));
    EXPECT_EQ(saturated::Add<int16_t>(0x8004, -5), int16_t(0x8000));
  }
  {
    EXPECT_EQ(saturated::Add<int32_t>(0x7FFFFFF9, 5), int32_t(0x7FFFFFFE));
    EXPECT_EQ(saturated::Add<int32_t>(0x7FFFFFFA, 5), int32_t(0x7FFFFFFF));
    EXPECT_EQ(saturated::Add<int32_t>(0x7FFFFFFB, 5), int32_t(0x7FFFFFFF));
  }
  {
    EXPECT_EQ(saturated::Add<int32_t>(0x80000006, -5), int32_t(0x80000001));
    EXPECT_EQ(saturated::Add<int32_t>(0x80000005, -5), int32_t(0x80000000));
    EXPECT_EQ(saturated::Add<int32_t>(0x80000004, -5), int32_t(0x80000000));
  }
  {
    EXPECT_EQ(saturated::Add<int64_t>(0x7FFFFFFFFFFFFFF9, 5),
              int64_t(0x7FFFFFFFFFFFFFFE));
    EXPECT_EQ(saturated::Add<int64_t>(0x7FFFFFFFFFFFFFFA, 5),
              int64_t(0x7FFFFFFFFFFFFFFF));
    EXPECT_EQ(saturated::Add<int64_t>(0x7FFFFFFFFFFFFFFB, 5),
              int64_t(0x7FFFFFFFFFFFFFFF));
  }
  {
    EXPECT_EQ(saturated::Add<int64_t>(0x8000000000000006, -5),
              int64_t(0x8000000000000001));
    EXPECT_EQ(saturated::Add<int64_t>(0x8000000000000005, -5),
              int64_t(0x8000000000000000));
    EXPECT_EQ(saturated::Add<int64_t>(0x8000000000000004, -5),
              int64_t(0x8000000000000000));
  }
}

TEST(SaturatedMath, ImplicitAddOfSignedInts) {
  {
    int8_t a = 0x79;
    int8_t b = 5;
    EXPECT_EQ(saturated::Add(a, b), int8_t(0x7E));
    a = 0x7A;
    EXPECT_EQ(saturated::Add(a, b), int8_t(0x7F));
    a = 0x7B;
    EXPECT_EQ(saturated::Add(a, b), int8_t(0x7F));
  }
  {
    int8_t a = 0x86;
    int8_t b = -5;
    EXPECT_EQ(saturated::Add(a, b), int8_t(0x81));
    a = 0x85;
    EXPECT_EQ(saturated::Add(a, b), int8_t(0x80));
    a = 0x84;
    EXPECT_EQ(saturated::Add(a, b), int8_t(0x80));
  }
  {
    int16_t a = 0x7FF9;
    int16_t b = 5;
    EXPECT_EQ(saturated::Add(a, b), int16_t(0x7FFE));
    a = 0x7FFA;
    EXPECT_EQ(saturated::Add(a, b), int16_t(0x7FFF));
    a = 0x7FFB;
    EXPECT_EQ(saturated::Add(a, b), int16_t(0x7FFF));
  }
  {
    int16_t a = 0x8006;
    int16_t b = -5;
    EXPECT_EQ(saturated::Add(a, b), int16_t(0x8001));
    a = 0x8005;
    EXPECT_EQ(saturated::Add(a, b), int16_t(0x8000));
    a = 0x8004;
    EXPECT_EQ(saturated::Add(a, b), int16_t(0x8000));
  }
  {
    int32_t a = 0x7FFFFFF9;
    int32_t b = 5;
    EXPECT_EQ(saturated::Add(a, b), int32_t(0x7FFFFFFE));
    a = 0x7FFFFFFA;
    EXPECT_EQ(saturated::Add(a, b), int32_t(0x7FFFFFFF));
    a = 0x7FFFFFFB;
    EXPECT_EQ(saturated::Add(a, b), int32_t(0x7FFFFFFF));
  }
  {
    int32_t a = 0x80000006;
    int32_t b = -5;
    EXPECT_EQ(saturated::Add(a, b), int32_t(0x80000001));
    a = 0x80000005;
    EXPECT_EQ(saturated::Add(a, b), int32_t(0x80000000));
    a = 0x80000004;
    EXPECT_EQ(saturated::Add(a, b), int32_t(0x80000000));
  }
  {
    int64_t a = 0x7FFFFFFFFFFFFFF9;
    int64_t b = 5;
    EXPECT_EQ(saturated::Add(a, b), int64_t(0x7FFFFFFFFFFFFFFE));
    a = 0x7FFFFFFFFFFFFFFA;
    EXPECT_EQ(saturated::Add(a, b), int64_t(0x7FFFFFFFFFFFFFFF));
    a = 0x7FFFFFFFFFFFFFFB;
    EXPECT_EQ(saturated::Add(a, b), int64_t(0x7FFFFFFFFFFFFFFF));
  }
  {
    int64_t a = 0x8000000000000006;
    int64_t b = -5;
    EXPECT_EQ(saturated::Add(a, b), int64_t(0x8000000000000001));
    a = 0x8000000000000005;
    EXPECT_EQ(saturated::Add(a, b), int64_t(0x8000000000000000));
    a = 0x8000000000000004;
    EXPECT_EQ(saturated::Add(a, b), int64_t(0x8000000000000000));
  }
}

TEST(SaturatedMath, ExplicitAddOfFloatingPoint) {
  {
    const float inf = std::numeric_limits<float>::infinity();
    const float max = std::numeric_limits<float>::max();
    const float big = max * 0.5f;

    EXPECT_EQ(saturated::Add<float>(big, big), max);
    EXPECT_EQ(saturated::Add<float>(max, big), inf);
    EXPECT_EQ(saturated::Add<float>(big, max), inf);
    EXPECT_EQ(saturated::Add<float>(max, max), inf);
    EXPECT_EQ(saturated::Add<float>(max, inf), inf);
    EXPECT_EQ(saturated::Add<float>(inf, max), inf);
    EXPECT_EQ(saturated::Add<float>(inf, inf), inf);

    EXPECT_EQ(saturated::Add<float>(-big, -big), -max);
    EXPECT_EQ(saturated::Add<float>(-max, -big), -inf);
    EXPECT_EQ(saturated::Add<float>(-big, -max), -inf);
    EXPECT_EQ(saturated::Add<float>(-max, -max), -inf);
    EXPECT_EQ(saturated::Add<float>(-max, -inf), -inf);
    EXPECT_EQ(saturated::Add<float>(-inf, -max), -inf);
    EXPECT_EQ(saturated::Add<float>(-inf, -inf), -inf);

    EXPECT_EQ(saturated::Add<float>(big, -big), 0.0f);
    EXPECT_EQ(saturated::Add<float>(max, -big), big);
    EXPECT_EQ(saturated::Add<float>(big, -max), -big);
    EXPECT_EQ(saturated::Add<float>(max, -max), 0.0f);
    EXPECT_EQ(saturated::Add<float>(max, -inf), -inf);
    EXPECT_EQ(saturated::Add<float>(inf, -max), inf);
    EXPECT_TRUE(std::isnan(saturated::Add<float>(inf, -inf)));

    EXPECT_EQ(saturated::Add<float>(-big, big), 0.0f);
    EXPECT_EQ(saturated::Add<float>(-max, big), -big);
    EXPECT_EQ(saturated::Add<float>(-big, max), big);
    EXPECT_EQ(saturated::Add<float>(-max, max), 0.0f);
    EXPECT_EQ(saturated::Add<float>(-max, inf), inf);
    EXPECT_EQ(saturated::Add<float>(-inf, max), -inf);
    EXPECT_TRUE(std::isnan(saturated::Add<float>(-inf, inf)));
  }
  {
    const double inf = std::numeric_limits<double>::infinity();
    const double max = std::numeric_limits<double>::max();
    const double big = max * 0.5f;

    EXPECT_EQ(saturated::Add<double>(big, big), max);
    EXPECT_EQ(saturated::Add<double>(max, big), inf);
    EXPECT_EQ(saturated::Add<double>(big, max), inf);
    EXPECT_EQ(saturated::Add<double>(max, max), inf);
    EXPECT_EQ(saturated::Add<double>(max, inf), inf);
    EXPECT_EQ(saturated::Add<double>(inf, max), inf);
    EXPECT_EQ(saturated::Add<double>(inf, inf), inf);

    EXPECT_EQ(saturated::Add<double>(-big, -big), -max);
    EXPECT_EQ(saturated::Add<double>(-max, -big), -inf);
    EXPECT_EQ(saturated::Add<double>(-big, -max), -inf);
    EXPECT_EQ(saturated::Add<double>(-max, -max), -inf);
    EXPECT_EQ(saturated::Add<double>(-max, -inf), -inf);
    EXPECT_EQ(saturated::Add<double>(-inf, -max), -inf);
    EXPECT_EQ(saturated::Add<double>(-inf, -inf), -inf);

    EXPECT_EQ(saturated::Add<double>(big, -big), 0.0f);
    EXPECT_EQ(saturated::Add<double>(max, -big), big);
    EXPECT_EQ(saturated::Add<double>(big, -max), -big);
    EXPECT_EQ(saturated::Add<double>(max, -max), 0.0f);
    EXPECT_EQ(saturated::Add<double>(max, -inf), -inf);
    EXPECT_EQ(saturated::Add<double>(inf, -max), inf);
    EXPECT_TRUE(std::isnan(saturated::Add<double>(inf, -inf)));

    EXPECT_EQ(saturated::Add<double>(-big, big), 0.0f);
    EXPECT_EQ(saturated::Add<double>(-max, big), -big);
    EXPECT_EQ(saturated::Add<double>(-big, max), big);
    EXPECT_EQ(saturated::Add<double>(-max, max), 0.0f);
    EXPECT_EQ(saturated::Add<double>(-max, inf), inf);
    EXPECT_EQ(saturated::Add<double>(-inf, max), -inf);
    EXPECT_TRUE(std::isnan(saturated::Add<double>(-inf, inf)));
  }
  {
    const Scalar inf = std::numeric_limits<Scalar>::infinity();
    const Scalar max = std::numeric_limits<Scalar>::max();
    const Scalar big = max * 0.5f;

    EXPECT_EQ(saturated::Add<Scalar>(big, big), max);
    EXPECT_EQ(saturated::Add<Scalar>(max, big), inf);
    EXPECT_EQ(saturated::Add<Scalar>(big, max), inf);
    EXPECT_EQ(saturated::Add<Scalar>(max, max), inf);
    EXPECT_EQ(saturated::Add<Scalar>(max, inf), inf);
    EXPECT_EQ(saturated::Add<Scalar>(inf, max), inf);
    EXPECT_EQ(saturated::Add<Scalar>(inf, inf), inf);

    EXPECT_EQ(saturated::Add<Scalar>(-big, -big), -max);
    EXPECT_EQ(saturated::Add<Scalar>(-max, -big), -inf);
    EXPECT_EQ(saturated::Add<Scalar>(-big, -max), -inf);
    EXPECT_EQ(saturated::Add<Scalar>(-max, -max), -inf);
    EXPECT_EQ(saturated::Add<Scalar>(-max, -inf), -inf);
    EXPECT_EQ(saturated::Add<Scalar>(-inf, -max), -inf);
    EXPECT_EQ(saturated::Add<Scalar>(-inf, -inf), -inf);

    EXPECT_EQ(saturated::Add<Scalar>(big, -big), 0.0f);
    EXPECT_EQ(saturated::Add<Scalar>(max, -big), big);
    EXPECT_EQ(saturated::Add<Scalar>(big, -max), -big);
    EXPECT_EQ(saturated::Add<Scalar>(max, -max), 0.0f);
    EXPECT_EQ(saturated::Add<Scalar>(max, -inf), -inf);
    EXPECT_EQ(saturated::Add<Scalar>(inf, -max), inf);
    EXPECT_TRUE(std::isnan(saturated::Add<Scalar>(inf, -inf)));

    EXPECT_EQ(saturated::Add<Scalar>(-big, big), 0.0f);
    EXPECT_EQ(saturated::Add<Scalar>(-max, big), -big);
    EXPECT_EQ(saturated::Add<Scalar>(-big, max), big);
    EXPECT_EQ(saturated::Add<Scalar>(-max, max), 0.0f);
    EXPECT_EQ(saturated::Add<Scalar>(-max, inf), inf);
    EXPECT_EQ(saturated::Add<Scalar>(-inf, max), -inf);
    EXPECT_TRUE(std::isnan(saturated::Add<Scalar>(-inf, inf)));
  }
}

TEST(SaturatedMath, ImplicitAddOfFloatingPoint) {
  {
    const float inf = std::numeric_limits<float>::infinity();
    const float max = std::numeric_limits<float>::max();
    const float big = max * 0.5f;

    EXPECT_EQ(saturated::Add(big, big), max);
    EXPECT_EQ(saturated::Add(max, big), inf);
    EXPECT_EQ(saturated::Add(big, max), inf);
    EXPECT_EQ(saturated::Add(max, max), inf);
    EXPECT_EQ(saturated::Add(max, inf), inf);
    EXPECT_EQ(saturated::Add(inf, max), inf);
    EXPECT_EQ(saturated::Add(inf, inf), inf);

    EXPECT_EQ(saturated::Add(-big, -big), -max);
    EXPECT_EQ(saturated::Add(-max, -big), -inf);
    EXPECT_EQ(saturated::Add(-big, -max), -inf);
    EXPECT_EQ(saturated::Add(-max, -max), -inf);
    EXPECT_EQ(saturated::Add(-max, -inf), -inf);
    EXPECT_EQ(saturated::Add(-inf, -max), -inf);
    EXPECT_EQ(saturated::Add(-inf, -inf), -inf);

    EXPECT_EQ(saturated::Add(big, -big), 0.0f);
    EXPECT_EQ(saturated::Add(max, -big), big);
    EXPECT_EQ(saturated::Add(big, -max), -big);
    EXPECT_EQ(saturated::Add(max, -max), 0.0f);
    EXPECT_EQ(saturated::Add(max, -inf), -inf);
    EXPECT_EQ(saturated::Add(inf, -max), inf);
    EXPECT_TRUE(std::isnan(saturated::Add(inf, -inf)));

    EXPECT_EQ(saturated::Add(-big, big), 0.0f);
    EXPECT_EQ(saturated::Add(-max, big), -big);
    EXPECT_EQ(saturated::Add(-big, max), big);
    EXPECT_EQ(saturated::Add(-max, max), 0.0f);
    EXPECT_EQ(saturated::Add(-max, inf), inf);
    EXPECT_EQ(saturated::Add(-inf, max), -inf);
    EXPECT_TRUE(std::isnan(saturated::Add(-inf, inf)));
  }
  {
    const double inf = std::numeric_limits<double>::infinity();
    const double max = std::numeric_limits<double>::max();
    const double big = max * 0.5f;

    EXPECT_EQ(saturated::Add(big, big), max);
    EXPECT_EQ(saturated::Add(max, big), inf);
    EXPECT_EQ(saturated::Add(big, max), inf);
    EXPECT_EQ(saturated::Add(max, max), inf);
    EXPECT_EQ(saturated::Add(max, inf), inf);
    EXPECT_EQ(saturated::Add(inf, max), inf);
    EXPECT_EQ(saturated::Add(inf, inf), inf);

    EXPECT_EQ(saturated::Add(-big, -big), -max);
    EXPECT_EQ(saturated::Add(-max, -big), -inf);
    EXPECT_EQ(saturated::Add(-big, -max), -inf);
    EXPECT_EQ(saturated::Add(-max, -max), -inf);
    EXPECT_EQ(saturated::Add(-max, -inf), -inf);
    EXPECT_EQ(saturated::Add(-inf, -max), -inf);
    EXPECT_EQ(saturated::Add(-inf, -inf), -inf);

    EXPECT_EQ(saturated::Add(big, -big), 0.0f);
    EXPECT_EQ(saturated::Add(max, -big), big);
    EXPECT_EQ(saturated::Add(big, -max), -big);
    EXPECT_EQ(saturated::Add(max, -max), 0.0f);
    EXPECT_EQ(saturated::Add(max, -inf), -inf);
    EXPECT_EQ(saturated::Add(inf, -max), inf);
    EXPECT_TRUE(std::isnan(saturated::Add(inf, -inf)));

    EXPECT_EQ(saturated::Add(-big, big), 0.0f);
    EXPECT_EQ(saturated::Add(-max, big), -big);
    EXPECT_EQ(saturated::Add(-big, max), big);
    EXPECT_EQ(saturated::Add(-max, max), 0.0f);
    EXPECT_EQ(saturated::Add(-max, inf), inf);
    EXPECT_EQ(saturated::Add(-inf, max), -inf);
    EXPECT_TRUE(std::isnan(saturated::Add(-inf, inf)));
  }
  {
    const Scalar inf = std::numeric_limits<Scalar>::infinity();
    const Scalar max = std::numeric_limits<Scalar>::max();
    const Scalar big = max * 0.5f;

    EXPECT_EQ(saturated::Add(big, big), max);
    EXPECT_EQ(saturated::Add(max, big), inf);
    EXPECT_EQ(saturated::Add(big, max), inf);
    EXPECT_EQ(saturated::Add(max, max), inf);
    EXPECT_EQ(saturated::Add(max, inf), inf);
    EXPECT_EQ(saturated::Add(inf, max), inf);
    EXPECT_EQ(saturated::Add(inf, inf), inf);

    EXPECT_EQ(saturated::Add(-big, -big), -max);
    EXPECT_EQ(saturated::Add(-max, -big), -inf);
    EXPECT_EQ(saturated::Add(-big, -max), -inf);
    EXPECT_EQ(saturated::Add(-max, -max), -inf);
    EXPECT_EQ(saturated::Add(-max, -inf), -inf);
    EXPECT_EQ(saturated::Add(-inf, -max), -inf);
    EXPECT_EQ(saturated::Add(-inf, -inf), -inf);

    EXPECT_EQ(saturated::Add(big, -big), 0.0f);
    EXPECT_EQ(saturated::Add(max, -big), big);
    EXPECT_EQ(saturated::Add(big, -max), -big);
    EXPECT_EQ(saturated::Add(max, -max), 0.0f);
    EXPECT_EQ(saturated::Add(max, -inf), -inf);
    EXPECT_EQ(saturated::Add(inf, -max), inf);
    EXPECT_TRUE(std::isnan(saturated::Add(inf, -inf)));

    EXPECT_EQ(saturated::Add(-big, big), 0.0f);
    EXPECT_EQ(saturated::Add(-max, big), -big);
    EXPECT_EQ(saturated::Add(-big, max), big);
    EXPECT_EQ(saturated::Add(-max, max), 0.0f);
    EXPECT_EQ(saturated::Add(-max, inf), inf);
    EXPECT_EQ(saturated::Add(-inf, max), -inf);
    EXPECT_TRUE(std::isnan(saturated::Add(-inf, inf)));
  }
}

TEST(SaturatedMath, ExplicitSubOfSignedInts) {
  {
    EXPECT_EQ(saturated::Sub<int8_t>(0x79, -5), int8_t(0x7E));
    EXPECT_EQ(saturated::Sub<int8_t>(0x7A, -5), int8_t(0x7F));
    EXPECT_EQ(saturated::Sub<int8_t>(0x7B, -5), int8_t(0x7F));
  }
  {
    EXPECT_EQ(saturated::Sub<int8_t>(0x86, 5), int8_t(0x81));
    EXPECT_EQ(saturated::Sub<int8_t>(0x85, 5), int8_t(0x80));
    EXPECT_EQ(saturated::Sub<int8_t>(0x84, 5), int8_t(0x80));
  }
  {
    EXPECT_EQ(saturated::Sub<int16_t>(0x7FF9, -5), int16_t(0x7FFE));
    EXPECT_EQ(saturated::Sub<int16_t>(0x7FFA, -5), int16_t(0x7FFF));
    EXPECT_EQ(saturated::Sub<int16_t>(0x7FFB, -5), int16_t(0x7FFF));
  }
  {
    EXPECT_EQ(saturated::Sub<int16_t>(0x8006, 5), int16_t(0x8001));
    EXPECT_EQ(saturated::Sub<int16_t>(0x8005, 5), int16_t(0x8000));
    EXPECT_EQ(saturated::Sub<int16_t>(0x8004, 5), int16_t(0x8000));
  }
  {
    EXPECT_EQ(saturated::Sub<int32_t>(0x7FFFFFF9, -5), int32_t(0x7FFFFFFE));
    EXPECT_EQ(saturated::Sub<int32_t>(0x7FFFFFFA, -5), int32_t(0x7FFFFFFF));
    EXPECT_EQ(saturated::Sub<int32_t>(0x7FFFFFFB, -5), int32_t(0x7FFFFFFF));
  }
  {
    EXPECT_EQ(saturated::Sub<int32_t>(0x80000006, 5), int32_t(0x80000001));
    EXPECT_EQ(saturated::Sub<int32_t>(0x80000005, 5), int32_t(0x80000000));
    EXPECT_EQ(saturated::Sub<int32_t>(0x80000004, 5), int32_t(0x80000000));
  }
  {
    EXPECT_EQ(saturated::Sub<int64_t>(0x7FFFFFFFFFFFFFF9, -5),
              int64_t(0x7FFFFFFFFFFFFFFE));
    EXPECT_EQ(saturated::Sub<int64_t>(0x7FFFFFFFFFFFFFFA, -5),
              int64_t(0x7FFFFFFFFFFFFFFF));
    EXPECT_EQ(saturated::Sub<int64_t>(0x7FFFFFFFFFFFFFFB, -5),
              int64_t(0x7FFFFFFFFFFFFFFF));
  }
  {
    EXPECT_EQ(saturated::Sub<int64_t>(0x8000000000000006, 5),
              int64_t(0x8000000000000001));
    EXPECT_EQ(saturated::Sub<int64_t>(0x8000000000000005, 5),
              int64_t(0x8000000000000000));
    EXPECT_EQ(saturated::Sub<int64_t>(0x8000000000000004, 5),
              int64_t(0x8000000000000000));
  }
}

TEST(SaturatedMath, ImplicitSubOfSignedInts) {
  {
    int8_t a = 0x79;
    int8_t b = -5;
    EXPECT_EQ(saturated::Sub(a, b), int8_t(0x7E));
    a = 0x7A;
    EXPECT_EQ(saturated::Sub(a, b), int8_t(0x7F));
    a = 0x7B;
    EXPECT_EQ(saturated::Sub(a, b), int8_t(0x7F));
  }
  {
    int8_t a = 0x86;
    int8_t b = 5;
    EXPECT_EQ(saturated::Sub(a, b), int8_t(0x81));
    a = 0x85;
    EXPECT_EQ(saturated::Sub(a, b), int8_t(0x80));
    a = 0x84;
    EXPECT_EQ(saturated::Sub(a, b), int8_t(0x80));
  }
  {
    int16_t a = 0x7FF9;
    int16_t b = -5;
    EXPECT_EQ(saturated::Sub(a, b), int16_t(0x7FFE));
    a = 0x7FFA;
    EXPECT_EQ(saturated::Sub(a, b), int16_t(0x7FFF));
    a = 0x7FFB;
    EXPECT_EQ(saturated::Sub(a, b), int16_t(0x7FFF));
  }
  {
    int16_t a = 0x8006;
    int16_t b = 5;
    EXPECT_EQ(saturated::Sub(a, b), int16_t(0x8001));
    a = 0x8005;
    EXPECT_EQ(saturated::Sub(a, b), int16_t(0x8000));
    a = 0x8004;
    EXPECT_EQ(saturated::Sub(a, b), int16_t(0x8000));
  }
  {
    int32_t a = 0x7FFFFFF9;
    int32_t b = -5;
    EXPECT_EQ(saturated::Sub(a, b), int32_t(0x7FFFFFFE));
    a = 0x7FFFFFFA;
    EXPECT_EQ(saturated::Sub(a, b), int32_t(0x7FFFFFFF));
    a = 0x7FFFFFFB;
    EXPECT_EQ(saturated::Sub(a, b), int32_t(0x7FFFFFFF));
  }
  {
    int32_t a = 0x80000006;
    int32_t b = 5;
    EXPECT_EQ(saturated::Sub(a, b), int32_t(0x80000001));
    a = 0x80000005;
    EXPECT_EQ(saturated::Sub(a, b), int32_t(0x80000000));
    a = 0x80000004;
    EXPECT_EQ(saturated::Sub(a, b), int32_t(0x80000000));
  }
  {
    int64_t a = 0x7FFFFFFFFFFFFFF9;
    int64_t b = -5;
    EXPECT_EQ(saturated::Sub(a, b), int64_t(0x7FFFFFFFFFFFFFFE));
    a = 0x7FFFFFFFFFFFFFFA;
    EXPECT_EQ(saturated::Sub(a, b), int64_t(0x7FFFFFFFFFFFFFFF));
    a = 0x7FFFFFFFFFFFFFFB;
    EXPECT_EQ(saturated::Sub(a, b), int64_t(0x7FFFFFFFFFFFFFFF));
  }
  {
    int64_t a = 0x8000000000000006;
    int64_t b = 5;
    EXPECT_EQ(saturated::Sub(a, b), int64_t(0x8000000000000001));
    a = 0x8000000000000005;
    EXPECT_EQ(saturated::Sub(a, b), int64_t(0x8000000000000000));
    a = 0x8000000000000004;
    EXPECT_EQ(saturated::Sub(a, b), int64_t(0x8000000000000000));
  }
}

TEST(SaturatedMath, ExplicitSubOfFloatingPoint) {
  {
    const float inf = std::numeric_limits<float>::infinity();
    const float max = std::numeric_limits<float>::max();
    const float big = max * 0.5f;

    EXPECT_EQ(saturated::Sub<float>(big, big), 0.0f);
    EXPECT_EQ(saturated::Sub<float>(max, big), big);
    EXPECT_EQ(saturated::Sub<float>(big, max), -big);
    EXPECT_EQ(saturated::Sub<float>(max, max), 0.0f);
    EXPECT_EQ(saturated::Sub<float>(max, inf), -inf);
    EXPECT_EQ(saturated::Sub<float>(inf, max), inf);
    EXPECT_TRUE(std::isnan(saturated::Sub<float>(inf, inf)));

    EXPECT_EQ(saturated::Sub<float>(-big, -big), 0.0f);
    EXPECT_EQ(saturated::Sub<float>(-max, -big), -big);
    EXPECT_EQ(saturated::Sub<float>(-big, -max), big);
    EXPECT_EQ(saturated::Sub<float>(-max, -max), 0.0f);
    EXPECT_EQ(saturated::Sub<float>(-max, -inf), inf);
    EXPECT_EQ(saturated::Sub<float>(-inf, -max), -inf);
    EXPECT_TRUE(std::isnan(saturated::Sub<float>(-inf, -inf)));

    EXPECT_EQ(saturated::Sub<float>(big, -big), max);
    EXPECT_EQ(saturated::Sub<float>(max, -big), inf);
    EXPECT_EQ(saturated::Sub<float>(big, -max), inf);
    EXPECT_EQ(saturated::Sub<float>(max, -max), inf);
    EXPECT_EQ(saturated::Sub<float>(max, -inf), inf);
    EXPECT_EQ(saturated::Sub<float>(inf, -max), inf);
    EXPECT_EQ(saturated::Sub<float>(inf, -inf), inf);

    EXPECT_EQ(saturated::Sub<float>(-big, big), -max);
    EXPECT_EQ(saturated::Sub<float>(-max, big), -inf);
    EXPECT_EQ(saturated::Sub<float>(-big, max), -inf);
    EXPECT_EQ(saturated::Sub<float>(-max, max), -inf);
    EXPECT_EQ(saturated::Sub<float>(-max, inf), -inf);
    EXPECT_EQ(saturated::Sub<float>(-inf, max), -inf);
    EXPECT_EQ(saturated::Sub<float>(-inf, inf), -inf);
  }
  {
    const double inf = std::numeric_limits<double>::infinity();
    const double max = std::numeric_limits<double>::max();
    const double big = max * 0.5f;

    EXPECT_EQ(saturated::Sub<double>(big, big), 0.0f);
    EXPECT_EQ(saturated::Sub<double>(max, big), big);
    EXPECT_EQ(saturated::Sub<double>(big, max), -big);
    EXPECT_EQ(saturated::Sub<double>(max, max), 0.0f);
    EXPECT_EQ(saturated::Sub<double>(max, inf), -inf);
    EXPECT_EQ(saturated::Sub<double>(inf, max), inf);
    EXPECT_TRUE(std::isnan(saturated::Sub<double>(inf, inf)));

    EXPECT_EQ(saturated::Sub<double>(-big, -big), 0.0f);
    EXPECT_EQ(saturated::Sub<double>(-max, -big), -big);
    EXPECT_EQ(saturated::Sub<double>(-big, -max), big);
    EXPECT_EQ(saturated::Sub<double>(-max, -max), 0.0f);
    EXPECT_EQ(saturated::Sub<double>(-max, -inf), inf);
    EXPECT_EQ(saturated::Sub<double>(-inf, -max), -inf);
    EXPECT_TRUE(std::isnan(saturated::Sub<double>(-inf, -inf)));

    EXPECT_EQ(saturated::Sub<double>(big, -big), max);
    EXPECT_EQ(saturated::Sub<double>(max, -big), inf);
    EXPECT_EQ(saturated::Sub<double>(big, -max), inf);
    EXPECT_EQ(saturated::Sub<double>(max, -max), inf);
    EXPECT_EQ(saturated::Sub<double>(max, -inf), inf);
    EXPECT_EQ(saturated::Sub<double>(inf, -max), inf);
    EXPECT_EQ(saturated::Sub<double>(inf, -inf), inf);

    EXPECT_EQ(saturated::Sub<double>(-big, big), -max);
    EXPECT_EQ(saturated::Sub<double>(-max, big), -inf);
    EXPECT_EQ(saturated::Sub<double>(-big, max), -inf);
    EXPECT_EQ(saturated::Sub<double>(-max, max), -inf);
    EXPECT_EQ(saturated::Sub<double>(-max, inf), -inf);
    EXPECT_EQ(saturated::Sub<double>(-inf, max), -inf);
    EXPECT_EQ(saturated::Sub<double>(-inf, inf), -inf);
  }
  {
    const Scalar inf = std::numeric_limits<Scalar>::infinity();
    const Scalar max = std::numeric_limits<Scalar>::max();
    const Scalar big = max * 0.5f;

    EXPECT_EQ(saturated::Sub<Scalar>(big, big), 0.0f);
    EXPECT_EQ(saturated::Sub<Scalar>(max, big), big);
    EXPECT_EQ(saturated::Sub<Scalar>(big, max), -big);
    EXPECT_EQ(saturated::Sub<Scalar>(max, max), 0.0f);
    EXPECT_EQ(saturated::Sub<Scalar>(max, inf), -inf);
    EXPECT_EQ(saturated::Sub<Scalar>(inf, max), inf);
    EXPECT_TRUE(std::isnan(saturated::Sub<Scalar>(inf, inf)));

    EXPECT_EQ(saturated::Sub<Scalar>(-big, -big), 0.0f);
    EXPECT_EQ(saturated::Sub<Scalar>(-max, -big), -big);
    EXPECT_EQ(saturated::Sub<Scalar>(-big, -max), big);
    EXPECT_EQ(saturated::Sub<Scalar>(-max, -max), 0.0f);
    EXPECT_EQ(saturated::Sub<Scalar>(-max, -inf), inf);
    EXPECT_EQ(saturated::Sub<Scalar>(-inf, -max), -inf);
    EXPECT_TRUE(std::isnan(saturated::Sub<Scalar>(-inf, -inf)));

    EXPECT_EQ(saturated::Sub<Scalar>(big, -big), max);
    EXPECT_EQ(saturated::Sub<Scalar>(max, -big), inf);
    EXPECT_EQ(saturated::Sub<Scalar>(big, -max), inf);
    EXPECT_EQ(saturated::Sub<Scalar>(max, -max), inf);
    EXPECT_EQ(saturated::Sub<Scalar>(max, -inf), inf);
    EXPECT_EQ(saturated::Sub<Scalar>(inf, -max), inf);
    EXPECT_EQ(saturated::Sub<Scalar>(inf, -inf), inf);

    EXPECT_EQ(saturated::Sub<Scalar>(-big, big), -max);
    EXPECT_EQ(saturated::Sub<Scalar>(-max, big), -inf);
    EXPECT_EQ(saturated::Sub<Scalar>(-big, max), -inf);
    EXPECT_EQ(saturated::Sub<Scalar>(-max, max), -inf);
    EXPECT_EQ(saturated::Sub<Scalar>(-max, inf), -inf);
    EXPECT_EQ(saturated::Sub<Scalar>(-inf, max), -inf);
    EXPECT_EQ(saturated::Sub<Scalar>(-inf, inf), -inf);
  }
}

TEST(SaturatedMath, ImplicitSubOfFloatingPoint) {
  {
    const float inf = std::numeric_limits<float>::infinity();
    const float max = std::numeric_limits<float>::max();
    const float big = max * 0.5f;

    EXPECT_EQ(saturated::Sub(big, big), 0.0f);
    EXPECT_EQ(saturated::Sub(max, big), big);
    EXPECT_EQ(saturated::Sub(big, max), -big);
    EXPECT_EQ(saturated::Sub(max, max), 0.0f);
    EXPECT_EQ(saturated::Sub(max, inf), -inf);
    EXPECT_EQ(saturated::Sub(inf, max), inf);
    EXPECT_TRUE(std::isnan(saturated::Sub(inf, inf)));

    EXPECT_EQ(saturated::Sub(-big, -big), 0.0f);
    EXPECT_EQ(saturated::Sub(-max, -big), -big);
    EXPECT_EQ(saturated::Sub(-big, -max), big);
    EXPECT_EQ(saturated::Sub(-max, -max), 0.0f);
    EXPECT_EQ(saturated::Sub(-max, -inf), inf);
    EXPECT_EQ(saturated::Sub(-inf, -max), -inf);
    EXPECT_TRUE(std::isnan(saturated::Sub(-inf, -inf)));

    EXPECT_EQ(saturated::Sub(big, -big), max);
    EXPECT_EQ(saturated::Sub(max, -big), inf);
    EXPECT_EQ(saturated::Sub(big, -max), inf);
    EXPECT_EQ(saturated::Sub(max, -max), inf);
    EXPECT_EQ(saturated::Sub(max, -inf), inf);
    EXPECT_EQ(saturated::Sub(inf, -max), inf);
    EXPECT_EQ(saturated::Sub(inf, -inf), inf);

    EXPECT_EQ(saturated::Sub(-big, big), -max);
    EXPECT_EQ(saturated::Sub(-max, big), -inf);
    EXPECT_EQ(saturated::Sub(-big, max), -inf);
    EXPECT_EQ(saturated::Sub(-max, max), -inf);
    EXPECT_EQ(saturated::Sub(-max, inf), -inf);
    EXPECT_EQ(saturated::Sub(-inf, max), -inf);
    EXPECT_EQ(saturated::Sub(-inf, inf), -inf);
  }
  {
    const double inf = std::numeric_limits<double>::infinity();
    const double max = std::numeric_limits<double>::max();
    const double big = max * 0.5f;

    EXPECT_EQ(saturated::Sub(big, big), 0.0f);
    EXPECT_EQ(saturated::Sub(max, big), big);
    EXPECT_EQ(saturated::Sub(big, max), -big);
    EXPECT_EQ(saturated::Sub(max, max), 0.0f);
    EXPECT_EQ(saturated::Sub(max, inf), -inf);
    EXPECT_EQ(saturated::Sub(inf, max), inf);
    EXPECT_TRUE(std::isnan(saturated::Sub(inf, inf)));

    EXPECT_EQ(saturated::Sub(-big, -big), 0.0f);
    EXPECT_EQ(saturated::Sub(-max, -big), -big);
    EXPECT_EQ(saturated::Sub(-big, -max), big);
    EXPECT_EQ(saturated::Sub(-max, -max), 0.0f);
    EXPECT_EQ(saturated::Sub(-max, -inf), inf);
    EXPECT_EQ(saturated::Sub(-inf, -max), -inf);
    EXPECT_TRUE(std::isnan(saturated::Sub(-inf, -inf)));

    EXPECT_EQ(saturated::Sub(big, -big), max);
    EXPECT_EQ(saturated::Sub(max, -big), inf);
    EXPECT_EQ(saturated::Sub(big, -max), inf);
    EXPECT_EQ(saturated::Sub(max, -max), inf);
    EXPECT_EQ(saturated::Sub(max, -inf), inf);
    EXPECT_EQ(saturated::Sub(inf, -max), inf);
    EXPECT_EQ(saturated::Sub(inf, -inf), inf);

    EXPECT_EQ(saturated::Sub(-big, big), -max);
    EXPECT_EQ(saturated::Sub(-max, big), -inf);
    EXPECT_EQ(saturated::Sub(-big, max), -inf);
    EXPECT_EQ(saturated::Sub(-max, max), -inf);
    EXPECT_EQ(saturated::Sub(-max, inf), -inf);
    EXPECT_EQ(saturated::Sub(-inf, max), -inf);
    EXPECT_EQ(saturated::Sub(-inf, inf), -inf);
  }
  {
    const Scalar inf = std::numeric_limits<Scalar>::infinity();
    const Scalar max = std::numeric_limits<Scalar>::max();
    const Scalar big = max * 0.5f;

    EXPECT_EQ(saturated::Sub(big, big), 0.0f);
    EXPECT_EQ(saturated::Sub(max, big), big);
    EXPECT_EQ(saturated::Sub(big, max), -big);
    EXPECT_EQ(saturated::Sub(max, max), 0.0f);
    EXPECT_EQ(saturated::Sub(max, inf), -inf);
    EXPECT_EQ(saturated::Sub(inf, max), inf);
    EXPECT_TRUE(std::isnan(saturated::Sub(inf, inf)));

    EXPECT_EQ(saturated::Sub(-big, -big), 0.0f);
    EXPECT_EQ(saturated::Sub(-max, -big), -big);
    EXPECT_EQ(saturated::Sub(-big, -max), big);
    EXPECT_EQ(saturated::Sub(-max, -max), 0.0f);
    EXPECT_EQ(saturated::Sub(-max, -inf), inf);
    EXPECT_EQ(saturated::Sub(-inf, -max), -inf);
    EXPECT_TRUE(std::isnan(saturated::Sub(-inf, -inf)));

    EXPECT_EQ(saturated::Sub(big, -big), max);
    EXPECT_EQ(saturated::Sub(max, -big), inf);
    EXPECT_EQ(saturated::Sub(big, -max), inf);
    EXPECT_EQ(saturated::Sub(max, -max), inf);
    EXPECT_EQ(saturated::Sub(max, -inf), inf);
    EXPECT_EQ(saturated::Sub(inf, -max), inf);
    EXPECT_EQ(saturated::Sub(inf, -inf), inf);

    EXPECT_EQ(saturated::Sub(-big, big), -max);
    EXPECT_EQ(saturated::Sub(-max, big), -inf);
    EXPECT_EQ(saturated::Sub(-big, max), -inf);
    EXPECT_EQ(saturated::Sub(-max, max), -inf);
    EXPECT_EQ(saturated::Sub(-max, inf), -inf);
    EXPECT_EQ(saturated::Sub(-inf, max), -inf);
    EXPECT_EQ(saturated::Sub(-inf, inf), -inf);
  }
}

TEST(SaturatedMath, ExplicitAverageScalarOfSignedInts) {
  // For each type try:
  //
  // - near the limits, averaging to 0
  // - at the limits, averaging to 0 or 0.5 depending on precision
  // - both large enough for the sum to overflow
  // - both negative enough for the sum to underflow
  {
    EXPECT_EQ(saturated::AverageScalar<int8_t>(0x81, 0x7F), -0.0f);
    EXPECT_EQ(saturated::AverageScalar<int8_t>(0x80, 0x7F), -0.5f);
    EXPECT_EQ(saturated::AverageScalar<int8_t>(0x70, 0x75), 114.5f);
    EXPECT_EQ(saturated::AverageScalar<int8_t>(0x85, 0x8A), -120.5f);
  }
  {
    EXPECT_EQ(saturated::AverageScalar<int16_t>(0x8001, 0x7FFF), -0.0f);
    EXPECT_EQ(saturated::AverageScalar<int16_t>(0x8000, 0x7FFF), -0.5f);
    EXPECT_EQ(saturated::AverageScalar<int16_t>(0x7000, 0x7005), 28674.5f);
    EXPECT_EQ(saturated::AverageScalar<int16_t>(0x8005, 0x800A), -32760.5f);
  }
  {
    EXPECT_EQ(saturated::AverageScalar<int32_t>(0x80000001, 0x7FFFFFFF), -0.0f);
    EXPECT_EQ(saturated::AverageScalar<int32_t>(0x80000000, 0x7FFFFFFF), -0.5f);
    EXPECT_EQ(saturated::AverageScalar<int32_t>(0x70000000, 0x70000005),
              1879048195.5f);
    EXPECT_EQ(saturated::AverageScalar<int32_t>(0x80000005, 0x8000000A),
              -2147483655.5f);
  }
  {
    EXPECT_EQ(saturated::AverageScalar<int64_t>(0x8000000000000001,
                                                0x7FFFFFFFFFFFFFFF),
              0.0f);
    // 64-bit integers overflow the ability of a Scalar (float) to
    // represent discrete integers and so the two numbers we are
    // averaging here will look like the same number with different
    // signs and the answer will be "0"
    EXPECT_EQ(saturated::AverageScalar<int64_t>(0x8000000000000000,
                                                0x7FFFFFFFFFFFFFFF),
              0.0f);
    EXPECT_NEAR(saturated::AverageScalar<int64_t>(0x7000000000000000,
                                                  0x7000000000000005),
                8.07045053e+18, 1e18);
    EXPECT_NEAR(saturated::AverageScalar<int64_t>(0x8000000000000005,
                                                  0x800000000000000A),
                -9.223372e+18, 1e18);
  }
}

TEST(SaturatedMath, ImplicitAverageScalarOfSignedInts) {
  // For each type try:
  //
  // - near the limits, averaging to 0
  // - at the limits, averaging to 0 or 0.5 depending on precision
  // - both large enough for the sum to overflow
  // - both negative enough for the sum to underflow
  {
    int8_t a = 0x81;
    int8_t b = 0x7f;
    EXPECT_EQ(saturated::AverageScalar(a, b), -0.0f);
    a = 0x80;
    EXPECT_EQ(saturated::AverageScalar(a, b), -0.5f);
    a = 0x70;
    b = 0x75;
    EXPECT_EQ(saturated::AverageScalar(a, b), 114.5f);
    a = 0x85;
    b = 0x8A;
    EXPECT_EQ(saturated::AverageScalar(a, b), -120.5f);
  }
  {
    int16_t a = 0x8001;
    int16_t b = 0x7FFF;
    EXPECT_EQ(saturated::AverageScalar(a, b), -0.0f);
    a = 0x8000;
    EXPECT_EQ(saturated::AverageScalar(a, b), -0.5f);
    a = 0x7000;
    b = 0x7005;
    EXPECT_EQ(saturated::AverageScalar(a, b), 28674.5f);
    a = 0x8005;
    b = 0x800A;
    EXPECT_EQ(saturated::AverageScalar(a, b), -32760.5f);
  }
  {
    int32_t a = 0x80000001;
    int32_t b = 0x7FFFFFFF;
    EXPECT_EQ(saturated::AverageScalar(a, b), -0.0f);
    a = 0x80000000;
    EXPECT_EQ(saturated::AverageScalar(a, b), -0.5f);
    a = 0x70000000;
    b = 0x70000005;
    EXPECT_EQ(saturated::AverageScalar(a, b), 1879048195.5f);
    a = 0x80000005;
    b = 0x8000000A;
    EXPECT_EQ(saturated::AverageScalar(a, b), -2147483655.5f);
  }
  {
    int64_t a = 0x8000000000000001;
    int64_t b = 0x7FFFFFFFFFFFFFFF;
    EXPECT_EQ(saturated::AverageScalar(a, b), 0.0f);
    // 64-bit integers overflow the ability of a Scalar (float) to
    // represent discrete integers and so the two numbers we are
    // averaging here will look like the same number with different
    // signs and the answer will be "0"
    a = 0x8000000000000000;
    EXPECT_EQ(saturated::AverageScalar<int64_t>(a, b), 0.0f);
    a = 0x7000000000000000;
    b = 0x7000000000000005;
    EXPECT_NEAR(saturated::AverageScalar<int64_t>(a, b), 8.0704505e+18, 1e18);
    a = 0x8000000000000005;
    b = 0x800000000000000A;
    EXPECT_NEAR(saturated::AverageScalar<int64_t>(a, b), -9.223372e+18, 1e18);
  }
}

TEST(SaturatedMath, ExplicitAverageScalarOfFloatingPoint) {
  const Scalar s_inf = std::numeric_limits<Scalar>::infinity();
  const Scalar s_max = std::numeric_limits<Scalar>::max();
  const Scalar s_big = s_max * 0.5f;

  {
    const float inf = std::numeric_limits<Scalar>::infinity();
    const float max = std::numeric_limits<float>::max();
    const float big = max * 0.5f;

    EXPECT_EQ(saturated::AverageScalar<float>(big, big), s_big);
    EXPECT_EQ(saturated::AverageScalar<float>(max, max), s_max);
    EXPECT_EQ(saturated::AverageScalar<float>(big, -big), 0.0f);
    EXPECT_EQ(saturated::AverageScalar<float>(max, -max), 0.0f);
    EXPECT_EQ(saturated::AverageScalar<float>(-big, big), 0.0f);
    EXPECT_EQ(saturated::AverageScalar<float>(-max, max), 0.0f);
    EXPECT_EQ(saturated::AverageScalar<float>(-big, -big), -s_big);
    EXPECT_EQ(saturated::AverageScalar<float>(-max, -max), -s_max);

    EXPECT_EQ(saturated::AverageScalar<float>(inf, inf), s_inf);
    EXPECT_EQ(saturated::AverageScalar<float>(-inf, -inf), -s_inf);
    EXPECT_TRUE(std::isnan(saturated::AverageScalar<float>(-inf, inf)));
    EXPECT_TRUE(std::isnan(saturated::AverageScalar<float>(inf, -inf)));
  }
  {
    const double inf = std::numeric_limits<Scalar>::infinity();
    const double max = std::numeric_limits<double>::max();
    const double big = max * 0.5;

    // Most of the averages below using the double constants will
    // overflow the Scalar return value and result in infinity,
    // so we also test with some Scalar constants (promoted to double)
    // to verify that they don't overflow in the double template
    EXPECT_EQ(saturated::AverageScalar<double>(s_big, s_big), s_big);
    EXPECT_EQ(saturated::AverageScalar<double>(s_max, s_max), s_max);
    EXPECT_EQ(saturated::AverageScalar<double>(-s_big, -s_big), -s_big);
    EXPECT_EQ(saturated::AverageScalar<double>(-s_max, -s_max), -s_max);

    // And now testing continues with the double constants which
    // mostly overflow
    EXPECT_EQ(saturated::AverageScalar<double>(big, big), s_inf);
    EXPECT_EQ(saturated::AverageScalar<double>(max, max), s_inf);
    EXPECT_EQ(saturated::AverageScalar<double>(big, -big), 0.0f);
    EXPECT_EQ(saturated::AverageScalar<double>(max, -max), 0.0f);
    EXPECT_EQ(saturated::AverageScalar<double>(-big, big), 0.0f);
    EXPECT_EQ(saturated::AverageScalar<double>(-max, max), 0.0f);
    EXPECT_EQ(saturated::AverageScalar<double>(-big, -big), -s_inf);
    EXPECT_EQ(saturated::AverageScalar<double>(-max, -max), -s_inf);

    EXPECT_EQ(saturated::AverageScalar<double>(inf, inf), s_inf);
    EXPECT_EQ(saturated::AverageScalar<double>(-inf, -inf), -s_inf);
    EXPECT_TRUE(std::isnan(saturated::AverageScalar<double>(-inf, inf)));
    EXPECT_TRUE(std::isnan(saturated::AverageScalar<double>(inf, -inf)));
  }
  {
    const Scalar inf = std::numeric_limits<Scalar>::infinity();
    const Scalar max = std::numeric_limits<Scalar>::max();
    const Scalar big = max * 0.5f;

    EXPECT_EQ(saturated::AverageScalar<Scalar>(big, big), s_big);
    EXPECT_EQ(saturated::AverageScalar<Scalar>(max, max), s_max);
    EXPECT_EQ(saturated::AverageScalar<Scalar>(big, -big), 0.0f);
    EXPECT_EQ(saturated::AverageScalar<Scalar>(max, -max), 0.0f);
    EXPECT_EQ(saturated::AverageScalar<Scalar>(-big, big), 0.0f);
    EXPECT_EQ(saturated::AverageScalar<Scalar>(-max, max), 0.0f);
    EXPECT_EQ(saturated::AverageScalar<Scalar>(-big, -big), -s_big);
    EXPECT_EQ(saturated::AverageScalar<Scalar>(-max, -max), -s_max);

    EXPECT_EQ(saturated::AverageScalar<Scalar>(inf, inf), s_inf);
    EXPECT_EQ(saturated::AverageScalar<Scalar>(-inf, -inf), -s_inf);
    EXPECT_TRUE(std::isnan(saturated::AverageScalar<Scalar>(-inf, s_inf)));
    EXPECT_TRUE(std::isnan(saturated::AverageScalar<Scalar>(inf, -s_inf)));
  }
}

TEST(SaturatedMath, ImplicitAverageScalarOfFloatingPoint) {
  // All return values are Scalar regardless of the operand types
  // so these constants are used as the expected answers.
  const Scalar s_inf = std::numeric_limits<Scalar>::infinity();
  const Scalar s_max = std::numeric_limits<Scalar>::max();
  const Scalar s_big = s_max * 0.5f;

  {
    const float inf = std::numeric_limits<float>::infinity();
    const float max = std::numeric_limits<float>::max();
    const float big = max * 0.5f;

    EXPECT_EQ(saturated::AverageScalar(big, big), s_big);
    EXPECT_EQ(saturated::AverageScalar(max, max), s_max);
    EXPECT_EQ(saturated::AverageScalar(big, -big), 0.0f);
    EXPECT_EQ(saturated::AverageScalar(max, -max), 0.0f);
    EXPECT_EQ(saturated::AverageScalar(-big, big), 0.0f);
    EXPECT_EQ(saturated::AverageScalar(-max, max), 0.0f);
    EXPECT_EQ(saturated::AverageScalar(-big, -big), -s_big);
    EXPECT_EQ(saturated::AverageScalar(-max, -max), -s_max);

    EXPECT_EQ(saturated::AverageScalar(inf, inf), s_inf);
    EXPECT_EQ(saturated::AverageScalar(-inf, -inf), -s_inf);
    EXPECT_TRUE(std::isnan(saturated::AverageScalar(-inf, inf)));
    EXPECT_TRUE(std::isnan(saturated::AverageScalar(inf, -inf)));
  }
  {
    const double inf = std::numeric_limits<double>::infinity();
    const double max = std::numeric_limits<double>::max();
    const double big = max * 0.5;

    // The s_constants converted to double. We should get finite results
    // from finding the averages of these values, but we'll get a lot of
    // overflow to infinity when testing the large double constants.
    const double d_s_max = s_max;
    const double d_s_big = s_big;
    EXPECT_EQ(saturated::AverageScalar(d_s_big, d_s_big), s_big);
    EXPECT_EQ(saturated::AverageScalar(d_s_max, d_s_max), s_max);
    EXPECT_EQ(saturated::AverageScalar(-d_s_big, -d_s_big), -s_big);
    EXPECT_EQ(saturated::AverageScalar(-d_s_max, -d_s_max), -s_max);

    // And now testing continues with the double constants which
    // mostly overflow
    EXPECT_EQ(saturated::AverageScalar(big, big), s_inf);
    EXPECT_EQ(saturated::AverageScalar(max, max), s_inf);
    EXPECT_EQ(saturated::AverageScalar(big, -big), 0.0f);
    EXPECT_EQ(saturated::AverageScalar(max, -max), 0.0f);
    EXPECT_EQ(saturated::AverageScalar(-big, big), 0.0f);
    EXPECT_EQ(saturated::AverageScalar(-max, max), 0.0f);
    EXPECT_EQ(saturated::AverageScalar(-big, -big), -s_inf);
    EXPECT_EQ(saturated::AverageScalar(-max, -max), -s_inf);

    EXPECT_EQ(saturated::AverageScalar(inf, inf), s_inf);
    EXPECT_EQ(saturated::AverageScalar(-inf, -inf), -s_inf);
    EXPECT_TRUE(std::isnan(saturated::AverageScalar(-inf, inf)));
    EXPECT_TRUE(std::isnan(saturated::AverageScalar(inf, -inf)));
  }
  {
    const Scalar inf = std::numeric_limits<Scalar>::infinity();
    const Scalar max = std::numeric_limits<Scalar>::max();
    const Scalar big = max * 0.5f;

    EXPECT_EQ(saturated::AverageScalar(big, big), s_big);
    EXPECT_EQ(saturated::AverageScalar(max, max), s_max);
    EXPECT_EQ(saturated::AverageScalar(big, -big), 0.0f);
    EXPECT_EQ(saturated::AverageScalar(max, -max), 0.0f);
    EXPECT_EQ(saturated::AverageScalar(-big, big), 0.0f);
    EXPECT_EQ(saturated::AverageScalar(-max, max), 0.0f);
    EXPECT_EQ(saturated::AverageScalar(-big, -big), -s_big);
    EXPECT_EQ(saturated::AverageScalar(-max, -max), -s_max);

    EXPECT_EQ(saturated::AverageScalar(inf, inf), s_inf);
    EXPECT_EQ(saturated::AverageScalar(-inf, -inf), -s_inf);
    EXPECT_TRUE(std::isnan(saturated::AverageScalar(-inf, s_inf)));
    EXPECT_TRUE(std::isnan(saturated::AverageScalar(inf, -s_inf)));
  }
}

TEST(SaturatedMath, CastingFiniteDoubleToFloatStaysFinite) {
  const double d_max = std::numeric_limits<double>::max();
  const float f_max = std::numeric_limits<float>::max();

  {
    const float result = saturated::Cast<double, float>(d_max);
    EXPECT_EQ(result, f_max);
  }

  {
    const float result = saturated::Cast<double, float>(-d_max);
    EXPECT_EQ(result, -f_max);
  }
}

TEST(SaturatedMath, CastingInfiniteDoubleToFloatStaysInfinite) {
  const double d_inf = std::numeric_limits<double>::infinity();
  const float f_max = std::numeric_limits<float>::infinity();

  {
    const float result = saturated::Cast<double, float>(d_inf);
    EXPECT_EQ(result, f_max);
  }

  {
    const float result = saturated::Cast<double, float>(-d_inf);
    EXPECT_EQ(result, -f_max);
  }
}

TEST(SaturatedMath, CastingNaNDoubleToFloatStaysNaN) {
  const double d_nan = std::numeric_limits<double>::quiet_NaN();

  {
    const float result = saturated::Cast<double, float>(d_nan);
    EXPECT_TRUE(std::isnan(result));
  }

  {
    const float result = saturated::Cast<double, float>(-d_nan);
    EXPECT_TRUE(std::isnan(result));
  }
}

TEST(SaturatedMath, CastingLargeScalarToSignedIntProducesLimit) {
  // larger than even any [u]int64_t;
  const Scalar large = 1e20f;

  {
    const auto result = saturated::Cast<Scalar, int8_t>(large);
    EXPECT_EQ(result, int8_t(0x7F));
  }
  {
    const auto result = saturated::Cast<Scalar, int8_t>(-large);
    EXPECT_EQ(result, int8_t(0x80));
  }

  {
    const auto result = saturated::Cast<Scalar, int16_t>(large);
    EXPECT_EQ(result, int16_t(0x7FFF));
  }
  {
    const auto result = saturated::Cast<Scalar, int16_t>(-large);
    EXPECT_EQ(result, int16_t(0x8000));
  }

  {
    const auto result = saturated::Cast<Scalar, int32_t>(large);
    EXPECT_EQ(result, int32_t(0x7FFFFFFF));
  }
  {
    const auto result = saturated::Cast<Scalar, int32_t>(-large);
    EXPECT_EQ(result, int32_t(0x80000000));
  }

  {
    const auto result = saturated::Cast<Scalar, int64_t>(large);
    EXPECT_EQ(result, int64_t(0x7FFFFFFFFFFFFFFF));
  }
  {
    const auto result = saturated::Cast<Scalar, int64_t>(-large);
    EXPECT_EQ(result, int64_t(0x8000000000000000));
  }
}

TEST(SaturatedMath, CastingInfiniteScalarToSignedIntProducesLimit) {
  // larger than even any [u]int64_t;
  const Scalar inf = std::numeric_limits<Scalar>::infinity();

  {
    const auto result = saturated::Cast<Scalar, int8_t>(inf);
    EXPECT_EQ(result, int8_t(0x7F));
  }
  {
    const auto result = saturated::Cast<Scalar, int8_t>(-inf);
    EXPECT_EQ(result, int8_t(0x80));
  }

  {
    const auto result = saturated::Cast<Scalar, int16_t>(inf);
    EXPECT_EQ(result, int16_t(0x7FFF));
  }
  {
    const auto result = saturated::Cast<Scalar, int16_t>(-inf);
    EXPECT_EQ(result, int16_t(0x8000));
  }

  {
    const auto result = saturated::Cast<Scalar, int32_t>(inf);
    EXPECT_EQ(result, int32_t(0x7FFFFFFF));
  }
  {
    const auto result = saturated::Cast<Scalar, int32_t>(-inf);
    EXPECT_EQ(result, int32_t(0x80000000));
  }

  {
    const auto result = saturated::Cast<Scalar, int64_t>(inf);
    EXPECT_EQ(result, int64_t(0x7FFFFFFFFFFFFFFF));
  }
  {
    const auto result = saturated::Cast<Scalar, int64_t>(-inf);
    EXPECT_EQ(result, int64_t(0x8000000000000000));
  }
}

TEST(SaturatedMath, CastingNaNScalarToSignedIntProducesZero) {
  // larger than even any [u]int64_t;
  const Scalar nan = std::numeric_limits<Scalar>::quiet_NaN();

  {
    const auto result = saturated::Cast<Scalar, int8_t>(nan);
    EXPECT_EQ(result, int8_t(0));
  }

  {
    const auto result = saturated::Cast<Scalar, int16_t>(nan);
    EXPECT_EQ(result, int16_t(0));
  }

  {
    const auto result = saturated::Cast<Scalar, int32_t>(nan);
    EXPECT_EQ(result, int32_t(0));
  }

  {
    const auto result = saturated::Cast<Scalar, int64_t>(nan);
    EXPECT_EQ(result, int64_t(0));
  }
}

}  // namespace testing
}  // namespace impeller
