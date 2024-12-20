// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "gtest/gtest.h"
#include "impeller/geometry/geometry_asserts.h"

#include <limits>
#include <map>
#include <sstream>
#include <type_traits>

#include "flutter/fml/build_config.h"
#include "flutter/testing/testing.h"
#include "impeller/geometry/color.h"
#include "impeller/geometry/constants.h"
#include "impeller/geometry/gradient.h"
#include "impeller/geometry/half.h"
#include "impeller/geometry/point.h"
#include "impeller/geometry/rect.h"
#include "impeller/geometry/scalar.h"
#include "impeller/geometry/separated_vector.h"
#include "impeller/geometry/size.h"

// TODO(zanderso): https://github.com/flutter/flutter/issues/127701
// NOLINTBEGIN(bugprone-unchecked-optional-access)

namespace impeller {
namespace testing {

TEST(GeometryTest, ScalarNearlyEqual) {
  ASSERT_FALSE(ScalarNearlyEqual(0.0021f, 0.001f));
  ASSERT_TRUE(ScalarNearlyEqual(0.0019f, 0.001f));
  ASSERT_TRUE(ScalarNearlyEqual(0.002f, 0.001f, 0.0011f));
  ASSERT_FALSE(ScalarNearlyEqual(0.002f, 0.001f, 0.0009f));
  ASSERT_TRUE(ScalarNearlyEqual(
      1.0f, 1.0f + std::numeric_limits<float>::epsilon() * 4));
}

TEST(GeometryTest, MakeColumn) {
  auto matrix = Matrix::MakeColumn(1, 2, 3, 4,     //
                                   5, 6, 7, 8,     //
                                   9, 10, 11, 12,  //
                                   13, 14, 15, 16);

  auto expect = Matrix{1,  2,  3,  4,   //
                       5,  6,  7,  8,   //
                       9,  10, 11, 12,  //
                       13, 14, 15, 16};

  ASSERT_TRUE(matrix == expect);
}

TEST(GeometryTest, MakeRow) {
  auto matrix = Matrix::MakeRow(1, 2, 3, 4,     //
                                5, 6, 7, 8,     //
                                9, 10, 11, 12,  //
                                13, 14, 15, 16);

  auto expect = Matrix{1, 5, 9,  13,  //
                       2, 6, 10, 14,  //
                       3, 7, 11, 15,  //
                       4, 8, 12, 16};

  ASSERT_TRUE(matrix == expect);
}

TEST(GeometryTest, RotationMatrix) {
  auto rotation = Matrix::MakeRotationZ(Radians{kPiOver4});
  // clang-format off
  auto expect = Matrix{k1OverSqrt2,  k1OverSqrt2, 0, 0,
                       -k1OverSqrt2, k1OverSqrt2, 0, 0,
                       0,            0,           1, 0,
                       0,            0,           0, 1};
  // clang-format on
  ASSERT_MATRIX_NEAR(rotation, expect);
}

TEST(GeometryTest, InvertMultMatrix) {
  {
    auto rotation = Matrix::MakeRotationZ(Radians{kPiOver4});
    auto invert = rotation.Invert();
    // clang-format off
    auto expect = Matrix{k1OverSqrt2, -k1OverSqrt2, 0, 0,
                         k1OverSqrt2, k1OverSqrt2,  0, 0,
                         0,           0,            1, 0,
                         0,           0,            0, 1};
    // clang-format on
    ASSERT_MATRIX_NEAR(invert, expect);
  }
  {
    auto scale = Matrix::MakeScale(Vector2{2, 4});
    auto invert = scale.Invert();
    auto expect = Matrix{0.5, 0,    0, 0,  //
                         0,   0.25, 0, 0,  //
                         0,   0,    1, 0,  //
                         0,   0,    0, 1};
    ASSERT_MATRIX_NEAR(invert, expect);
  }
}

TEST(GeometryTest, MatrixBasis) {
  auto matrix = Matrix{1,  2,  3,  4,   //
                       5,  6,  7,  8,   //
                       9,  10, 11, 12,  //
                       13, 14, 15, 16};
  auto basis = matrix.Basis();
  auto expect = Matrix{1, 2,  3,  0,  //
                       5, 6,  7,  0,  //
                       9, 10, 11, 0,  //
                       0, 0,  0,  1};
  ASSERT_MATRIX_NEAR(basis, expect);
}

TEST(GeometryTest, MutliplicationMatrix) {
  auto rotation = Matrix::MakeRotationZ(Radians{kPiOver4});
  auto invert = rotation.Invert();
  ASSERT_MATRIX_NEAR(rotation * invert, Matrix{});
}

TEST(GeometryTest, DeterminantTest) {
  auto matrix = Matrix{3, 4, 14, 155, 2, 1, 3, 4, 2, 3, 2, 1, 1, 2, 4, 2};
  ASSERT_EQ(matrix.GetDeterminant(), -1889);
}

TEST(GeometryTest, InvertMatrix) {
  auto inverted = Matrix{10,  -9,  -12, 8,   //
                         7,   -12, 11,  22,  //
                         -10, 10,  3,   6,   //
                         -2,  22,  2,   1}
                      .Invert();

  auto result = Matrix{
      438.0 / 85123.0,   1751.0 / 85123.0, -7783.0 / 85123.0, 4672.0 / 85123.0,
      393.0 / 85123.0,   -178.0 / 85123.0, -570.0 / 85123.0,  4192 / 85123.0,
      -5230.0 / 85123.0, 2802.0 / 85123.0, -3461.0 / 85123.0, 962.0 / 85123.0,
      2690.0 / 85123.0,  1814.0 / 85123.0, 3896.0 / 85123.0,  319.0 / 85123.0};

  ASSERT_MATRIX_NEAR(inverted, result);
}

TEST(GeometryTest, TestDecomposition) {
  auto rotated = Matrix::MakeRotationZ(Radians{kPiOver4});

  auto result = rotated.Decompose();

  ASSERT_TRUE(result.has_value());

  MatrixDecomposition res = result.value();

  auto quaternion = Quaternion{{0.0, 0.0, 1.0}, kPiOver4};
  ASSERT_QUATERNION_NEAR(res.rotation, quaternion);
}

TEST(GeometryTest, TestDecomposition2) {
  auto rotated = Matrix::MakeRotationZ(Radians{kPiOver4});
  auto scaled = Matrix::MakeScale({2.0, 3.0, 1.0});
  auto translated = Matrix::MakeTranslation({-200, 750, 20});

  auto result = (translated * rotated * scaled).Decompose();

  ASSERT_TRUE(result.has_value());

  MatrixDecomposition res = result.value();

  auto quaternion = Quaternion{{0.0, 0.0, 1.0}, kPiOver4};

  ASSERT_QUATERNION_NEAR(res.rotation, quaternion);

  ASSERT_FLOAT_EQ(res.translation.x, -200);
  ASSERT_FLOAT_EQ(res.translation.y, 750);
  ASSERT_FLOAT_EQ(res.translation.z, 20);

  ASSERT_FLOAT_EQ(res.scale.x, 2);
  ASSERT_FLOAT_EQ(res.scale.y, 3);
  ASSERT_FLOAT_EQ(res.scale.z, 1);
}

TEST(GeometryTest, TestRecomposition) {
  /*
   *  Decomposition.
   */
  auto rotated = Matrix::MakeRotationZ(Radians{kPiOver4});

  auto result = rotated.Decompose();

  ASSERT_TRUE(result.has_value());

  MatrixDecomposition res = result.value();

  auto quaternion = Quaternion{{0.0, 0.0, 1.0}, kPiOver4};

  ASSERT_QUATERNION_NEAR(res.rotation, quaternion);

  /*
   *  Recomposition.
   */
  ASSERT_MATRIX_NEAR(rotated, Matrix{res});
}

TEST(GeometryTest, TestRecomposition2) {
  auto matrix = Matrix::MakeTranslation({100, 100, 100}) *
                Matrix::MakeRotationZ(Radians{kPiOver4}) *
                Matrix::MakeScale({2.0, 2.0, 2.0});

  auto result = matrix.Decompose();

  ASSERT_TRUE(result.has_value());

  ASSERT_MATRIX_NEAR(matrix, Matrix{result.value()});
}

TEST(GeometryTest, MatrixVectorMultiplication) {
  {
    auto matrix = Matrix::MakeTranslation({100, 100, 100}) *
                  Matrix::MakeRotationZ(Radians{kPiOver2}) *
                  Matrix::MakeScale({2.0, 2.0, 2.0});
    auto vector = Vector4(10, 20, 30, 2);

    Vector4 result = matrix * vector;
    auto expected = Vector4(160, 220, 260, 2);
    ASSERT_VECTOR4_NEAR(result, expected);
  }

  {
    auto matrix = Matrix::MakeTranslation({100, 100, 100}) *
                  Matrix::MakeRotationZ(Radians{kPiOver2}) *
                  Matrix::MakeScale({2.0, 2.0, 2.0});
    auto vector = Vector3(10, 20, 30);

    Vector3 result = matrix * vector;
    auto expected = Vector3(60, 120, 160);
    ASSERT_VECTOR3_NEAR(result, expected);
  }

  {
    auto matrix = Matrix::MakeTranslation({100, 100, 100}) *
                  Matrix::MakeRotationZ(Radians{kPiOver2}) *
                  Matrix::MakeScale({2.0, 2.0, 2.0});
    auto vector = Point(10, 20);

    Point result = matrix * vector;
    auto expected = Point(60, 120);
    ASSERT_POINT_NEAR(result, expected);
  }

  // Matrix Vector ops should respect perspective transforms.
  {
    auto matrix = Matrix::MakePerspective(Radians(kPiOver2), 1, 1, 100);
    auto vector = Vector3(3, 3, -3);

    Vector3 result = matrix * vector;
    auto expected = Vector3(-1, -1, 1.3468);
    ASSERT_VECTOR3_NEAR(result, expected);
  }

  {
    auto matrix = Matrix::MakePerspective(Radians(kPiOver2), 1, 1, 100) *
                  Matrix::MakeTranslation(Vector3(0, 0, -3));
    auto point = Point(3, 3);

    Point result = matrix * point;
    auto expected = Point(-1, -1);
    ASSERT_POINT_NEAR(result, expected);
  }

  // Resolves to 0 on perspective singularity.
  {
    auto matrix = Matrix::MakePerspective(Radians(kPiOver2), 1, 1, 100);
    auto point = Point(3, 3);

    Point result = matrix * point;
    auto expected = Point(0, 0);
    ASSERT_POINT_NEAR(result, expected);
  }
}

TEST(GeometryTest, MatrixMakeRotationFromQuaternion) {
  {
    auto matrix = Matrix::MakeRotation(Quaternion({1, 0, 0}, kPiOver2));
    auto expected = Matrix::MakeRotationX(Radians(kPiOver2));
    ASSERT_MATRIX_NEAR(matrix, expected);
  }

  {
    auto matrix = Matrix::MakeRotation(Quaternion({0, 1, 0}, kPiOver2));
    auto expected = Matrix::MakeRotationY(Radians(kPiOver2));
    ASSERT_MATRIX_NEAR(matrix, expected);
  }

  {
    auto matrix = Matrix::MakeRotation(Quaternion({0, 0, 1}, kPiOver2));
    auto expected = Matrix::MakeRotationZ(Radians(kPiOver2));
    ASSERT_MATRIX_NEAR(matrix, expected);
  }
}

TEST(GeometryTest, MatrixTransformDirection) {
  {
    auto matrix = Matrix::MakeTranslation({100, 100, 100}) *
                  Matrix::MakeRotationZ(Radians{kPiOver2}) *
                  Matrix::MakeScale({2.0, 2.0, 2.0});
    auto vector = Vector4(10, 20, 30, 2);

    Vector4 result = matrix.TransformDirection(vector);
    auto expected = Vector4(-40, 20, 60, 2);
    ASSERT_VECTOR4_NEAR(result, expected);
  }

  {
    auto matrix = Matrix::MakeTranslation({100, 100, 100}) *
                  Matrix::MakeRotationZ(Radians{kPiOver2}) *
                  Matrix::MakeScale({2.0, 2.0, 2.0});
    auto vector = Vector3(10, 20, 30);

    Vector3 result = matrix.TransformDirection(vector);
    auto expected = Vector3(-40, 20, 60);
    ASSERT_VECTOR3_NEAR(result, expected);
  }

  {
    auto matrix = Matrix::MakeTranslation({0, -0.4, 100}) *
                  Matrix::MakeRotationZ(Radians{kPiOver2}) *
                  Matrix::MakeScale({2.0, 2.0, 2.0});
    auto vector = Point(10, 20);

    Point result = matrix.TransformDirection(vector);
    auto expected = Point(-40, 20);
    ASSERT_POINT_NEAR(result, expected);
  }
}

TEST(GeometryTest, MatrixGetMaxBasisLengthXY) {
  {
    auto m = Matrix::MakeScale({3, 1, 1});
    ASSERT_EQ(m.GetMaxBasisLengthXY(), 3);

    m = m * Matrix::MakeSkew(0, 4);
    ASSERT_EQ(m.GetMaxBasisLengthXY(), 5);
  }

  {
    auto m = Matrix::MakeScale({-3, 4, 7});
    ASSERT_EQ(m.GetMaxBasisLengthXY(), 4);
  }

  {
    // clang-format off
    auto m = Matrix::MakeColumn(
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        4.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f
    );
    // clang-format on
    ASSERT_EQ(m.GetMaxBasisLengthXY(), 1.0f);
  }
}

TEST(GeometryTest, MatrixMakeOrthographic) {
  {
    auto m = Matrix::MakeOrthographic(Size(100, 200));
    auto expect = Matrix{
        0.02, 0,     0,   0,  //
        0,    -0.01, 0,   0,  //
        0,    0,     0,   0,  //
        -1,   1,     0.5, 1,  //
    };
    ASSERT_MATRIX_NEAR(m, expect);
  }

  {
    auto m = Matrix::MakeOrthographic(Size(400, 100));
    auto expect = Matrix{
        0.005, 0,     0,   0,  //
        0,     -0.02, 0,   0,  //
        0,     0,     0,   0,  //
        -1,    1,     0.5, 1,  //
    };
    ASSERT_MATRIX_NEAR(m, expect);
  }
}

TEST(GeometryTest, MatrixMakePerspective) {
  {
    auto m = Matrix::MakePerspective(Degrees(60), Size(100, 200), 1, 10);
    auto expect = Matrix{
        3.4641, 0,       0,        0,  //
        0,      1.73205, 0,        0,  //
        0,      0,       1.11111,  1,  //
        0,      0,       -1.11111, 0,  //
    };
    ASSERT_MATRIX_NEAR(m, expect);
  }

  {
    auto m = Matrix::MakePerspective(Radians(1), 2, 10, 20);
    auto expect = Matrix{
        0.915244, 0,       0,   0,  //
        0,        1.83049, 0,   0,  //
        0,        0,       2,   1,  //
        0,        0,       -20, 0,  //
    };
    ASSERT_MATRIX_NEAR(m, expect);
  }
}

TEST(GeometryTest, MatrixGetBasisVectors) {
  {
    auto m = Matrix();
    Vector3 x = m.GetBasisX();
    Vector3 y = m.GetBasisY();
    Vector3 z = m.GetBasisZ();
    ASSERT_VECTOR3_NEAR(x, Vector3(1, 0, 0));
    ASSERT_VECTOR3_NEAR(y, Vector3(0, 1, 0));
    ASSERT_VECTOR3_NEAR(z, Vector3(0, 0, 1));
  }

  {
    auto m = Matrix::MakeRotationZ(Radians{kPiOver2}) *
             Matrix::MakeRotationX(Radians{kPiOver2}) *
             Matrix::MakeScale(Vector3(2, 3, 4));
    Vector3 x = m.GetBasisX();
    Vector3 y = m.GetBasisY();
    Vector3 z = m.GetBasisZ();
    ASSERT_VECTOR3_NEAR(x, Vector3(0, 2, 0));
    ASSERT_VECTOR3_NEAR(y, Vector3(0, 0, 3));
    ASSERT_VECTOR3_NEAR(z, Vector3(4, 0, 0));
  }
}

TEST(GeometryTest, MatrixGetDirectionScale) {
  {
    auto m = Matrix();
    Scalar result = m.GetDirectionScale(Vector3{1, 0, 0});
    ASSERT_FLOAT_EQ(result, 1);
  }

  {
    auto m = Matrix::MakeRotationX(Degrees{10}) *
             Matrix::MakeRotationY(Degrees{83}) *
             Matrix::MakeRotationZ(Degrees{172});
    Scalar result = m.GetDirectionScale(Vector3{0, 1, 0});
    ASSERT_FLOAT_EQ(result, 1);
  }

  {
    auto m = Matrix::MakeRotationZ(Radians{kPiOver2}) *
             Matrix::MakeScale(Vector3(3, 4, 5));
    Scalar result = m.GetDirectionScale(Vector3{2, 0, 0});
    ASSERT_FLOAT_EQ(result, 8);
  }
}

TEST(GeometryTest, MatrixTranslationScaleOnly) {
  {
    auto m = Matrix();
    bool result = m.IsTranslationScaleOnly();
    ASSERT_TRUE(result);
  }

  {
    auto m = Matrix::MakeScale(Vector3(2, 3, 4));
    bool result = m.IsTranslationScaleOnly();
    ASSERT_TRUE(result);
  }

  {
    auto m = Matrix::MakeTranslation(Vector3(2, 3, 4));
    bool result = m.IsTranslationScaleOnly();
    ASSERT_TRUE(result);
  }

  {
    auto m = Matrix::MakeRotationZ(Degrees(10));
    bool result = m.IsTranslationScaleOnly();
    ASSERT_FALSE(result);
  }
}

TEST(GeometryTest, MatrixLookAt) {
  {
    auto m = Matrix::MakeLookAt(Vector3(0, 0, -1), Vector3(0, 0, 1),
                                Vector3(0, 1, 0));
    auto expected = Matrix{
        1, 0, 0, 0,  //
        0, 1, 0, 0,  //
        0, 0, 1, 0,  //
        0, 0, 1, 1,  //
    };
    ASSERT_MATRIX_NEAR(m, expected);
  }

  // Sideways tilt.
  {
    auto m = Matrix::MakeLookAt(Vector3(0, 0, -1), Vector3(0, 0, 1),
                                Vector3(1, 1, 0).Normalize());

    // clang-format off
    auto expected = Matrix{
        k1OverSqrt2, k1OverSqrt2, 0, 0,
       -k1OverSqrt2, k1OverSqrt2, 0, 0,
        0,           0,           1, 0,
        0,           0,           1, 1,
    };
    // clang-format on
    ASSERT_MATRIX_NEAR(m, expected);
  }

  // Half way between +x and -y, yaw 90
  {
    auto m =
        Matrix::MakeLookAt(Vector3(), Vector3(10, -10, 0), Vector3(0, 0, -1));

    // clang-format off
    auto expected = Matrix{
       -k1OverSqrt2,  0,  k1OverSqrt2, 0,
       -k1OverSqrt2,  0, -k1OverSqrt2, 0,
        0,           -1,  0,           0,
        0,            0,  0,           1,
    };
    // clang-format on
    ASSERT_MATRIX_NEAR(m, expected);
  }
}

TEST(GeometryTest, QuaternionLerp) {
  auto q1 = Quaternion{{0.0, 0.0, 1.0}, 0.0};
  auto q2 = Quaternion{{0.0, 0.0, 1.0}, kPiOver4};

  auto q3 = q1.Slerp(q2, 0.5);

  auto expected = Quaternion{{0.0, 0.0, 1.0}, kPiOver4 / 2.0};

  ASSERT_QUATERNION_NEAR(q3, expected);
}

TEST(GeometryTest, QuaternionVectorMultiply) {
  {
    Quaternion q({0, 0, 1}, 0);
    Vector3 v(0, 1, 0);

    Vector3 result = q * v;
    Vector3 expected(0, 1, 0);

    ASSERT_VECTOR3_NEAR(result, expected);
  }

  {
    Quaternion q({0, 0, 1}, k2Pi);
    Vector3 v(1, 0, 0);

    Vector3 result = q * v;
    Vector3 expected(1, 0, 0);

    ASSERT_VECTOR3_NEAR(result, expected);
  }

  {
    Quaternion q({0, 0, 1}, kPiOver4);
    Vector3 v(0, 1, 0);

    Vector3 result = q * v;
    Vector3 expected(-k1OverSqrt2, k1OverSqrt2, 0);

    ASSERT_VECTOR3_NEAR(result, expected);
  }

  {
    Quaternion q(Vector3(1, 0, 1).Normalize(), kPi);
    Vector3 v(0, 0, -1);

    Vector3 result = q * v;
    Vector3 expected(-1, 0, 0);

    ASSERT_VECTOR3_NEAR(result, expected);
  }
}

TEST(GeometryTest, CanGenerateMipCounts) {
  ASSERT_EQ((Size{128, 128}.MipCount()), 7u);
  ASSERT_EQ((Size{128, 256}.MipCount()), 8u);
  ASSERT_EQ((Size{128, 130}.MipCount()), 8u);
  ASSERT_EQ((Size{128, 257}.MipCount()), 9u);
  ASSERT_EQ((Size{257, 128}.MipCount()), 9u);
  ASSERT_EQ((Size{128, 0}.MipCount()), 1u);
  ASSERT_EQ((Size{128, -25}.MipCount()), 1u);
  ASSERT_EQ((Size{-128, 25}.MipCount()), 1u);
  ASSERT_EQ((Size{1, 1}.MipCount()), 1u);
  ASSERT_EQ((Size{0, 0}.MipCount()), 1u);
}

TEST(GeometryTest, CanConvertTTypesExplicitly) {
  {
    Point p1(1.0, 2.0);
    IPoint p2 = static_cast<IPoint>(p1);
    ASSERT_EQ(p2.x, 1u);
    ASSERT_EQ(p2.y, 2u);
  }

  {
    Size s1(1.0, 2.0);
    ISize s2 = static_cast<ISize>(s1);
    ASSERT_EQ(s2.width, 1u);
    ASSERT_EQ(s2.height, 2u);
  }

  {
    Size s1(1.0, 2.0);
    Point p1 = static_cast<Point>(s1);
    ASSERT_EQ(p1.x, 1u);
    ASSERT_EQ(p1.y, 2u);
  }
}

TEST(GeometryTest, CanPerformAlgebraicPointOps) {
  {
    IPoint p1(1, 2);
    IPoint p2 = p1 + IPoint(1, 2);
    ASSERT_EQ(p2.x, 2u);
    ASSERT_EQ(p2.y, 4u);
  }

  {
    IPoint p1(3, 6);
    IPoint p2 = p1 - IPoint(1, 2);
    ASSERT_EQ(p2.x, 2u);
    ASSERT_EQ(p2.y, 4u);
  }

  {
    IPoint p1(1, 2);
    IPoint p2 = p1 * IPoint(2, 3);
    ASSERT_EQ(p2.x, 2u);
    ASSERT_EQ(p2.y, 6u);
  }

  {
    IPoint p1(2, 6);
    IPoint p2 = p1 / IPoint(2, 3);
    ASSERT_EQ(p2.x, 1u);
    ASSERT_EQ(p2.y, 2u);
  }
}

TEST(GeometryTest, CanPerformAlgebraicPointOpsWithArithmeticTypes) {
  // LHS
  {
    IPoint p1(1, 2);
    IPoint p2 = p1 * 2.0f;
    ASSERT_EQ(p2.x, 2u);
    ASSERT_EQ(p2.y, 4u);
  }

  {
    IPoint p1(2, 6);
    IPoint p2 = p1 / 2.0f;
    ASSERT_EQ(p2.x, 1u);
    ASSERT_EQ(p2.y, 3u);
  }

  // RHS
  {
    IPoint p1(1, 2);
    IPoint p2 = 2.0f * p1;
    ASSERT_EQ(p2.x, 2u);
    ASSERT_EQ(p2.y, 4u);
  }

  {
    IPoint p1(2, 6);
    IPoint p2 = 12.0f / p1;
    ASSERT_EQ(p2.x, 6u);
    ASSERT_EQ(p2.y, 2u);
  }
}

TEST(GeometryTest, PointIntegerCoercesToFloat) {
  // Integer on LHS, float on RHS
  {
    IPoint p1(1, 2);
    Point p2 = p1 + Point(1, 2);
    ASSERT_FLOAT_EQ(p2.x, 2u);
    ASSERT_FLOAT_EQ(p2.y, 4u);
  }

  {
    IPoint p1(3, 6);
    Point p2 = p1 - Point(1, 2);
    ASSERT_FLOAT_EQ(p2.x, 2u);
    ASSERT_FLOAT_EQ(p2.y, 4u);
  }

  {
    IPoint p1(1, 2);
    Point p2 = p1 * Point(2, 3);
    ASSERT_FLOAT_EQ(p2.x, 2u);
    ASSERT_FLOAT_EQ(p2.y, 6u);
  }

  {
    IPoint p1(2, 6);
    Point p2 = p1 / Point(2, 3);
    ASSERT_FLOAT_EQ(p2.x, 1u);
    ASSERT_FLOAT_EQ(p2.y, 2u);
  }

  // Float on LHS, integer on RHS
  {
    Point p1(1, 2);
    Point p2 = p1 + IPoint(1, 2);
    ASSERT_FLOAT_EQ(p2.x, 2u);
    ASSERT_FLOAT_EQ(p2.y, 4u);
  }

  {
    Point p1(3, 6);
    Point p2 = p1 - IPoint(1, 2);
    ASSERT_FLOAT_EQ(p2.x, 2u);
    ASSERT_FLOAT_EQ(p2.y, 4u);
  }

  {
    Point p1(1, 2);
    Point p2 = p1 * IPoint(2, 3);
    ASSERT_FLOAT_EQ(p2.x, 2u);
    ASSERT_FLOAT_EQ(p2.y, 6u);
  }

  {
    Point p1(2, 6);
    Point p2 = p1 / IPoint(2, 3);
    ASSERT_FLOAT_EQ(p2.x, 1u);
    ASSERT_FLOAT_EQ(p2.y, 2u);
  }
}

TEST(GeometryTest, SizeCoercesToPoint) {
  // Point on LHS, Size on RHS
  {
    IPoint p1(1, 2);
    IPoint p2 = p1 + ISize(1, 2);
    ASSERT_EQ(p2.x, 2u);
    ASSERT_EQ(p2.y, 4u);
  }

  {
    IPoint p1(3, 6);
    IPoint p2 = p1 - ISize(1, 2);
    ASSERT_EQ(p2.x, 2u);
    ASSERT_EQ(p2.y, 4u);
  }

  {
    IPoint p1(1, 2);
    IPoint p2 = p1 * ISize(2, 3);
    ASSERT_EQ(p2.x, 2u);
    ASSERT_EQ(p2.y, 6u);
  }

  {
    IPoint p1(2, 6);
    IPoint p2 = p1 / ISize(2, 3);
    ASSERT_EQ(p2.x, 1u);
    ASSERT_EQ(p2.y, 2u);
  }

  // Size on LHS, Point on RHS
  {
    ISize p1(1, 2);
    IPoint p2 = p1 + IPoint(1, 2);
    ASSERT_EQ(p2.x, 2u);
    ASSERT_EQ(p2.y, 4u);
  }

  {
    ISize p1(3, 6);
    IPoint p2 = p1 - IPoint(1, 2);
    ASSERT_EQ(p2.x, 2u);
    ASSERT_EQ(p2.y, 4u);
  }

  {
    ISize p1(1, 2);
    IPoint p2 = p1 * IPoint(2, 3);
    ASSERT_EQ(p2.x, 2u);
    ASSERT_EQ(p2.y, 6u);
  }

  {
    ISize p1(2, 6);
    IPoint p2 = p1 / IPoint(2, 3);
    ASSERT_EQ(p2.x, 1u);
    ASSERT_EQ(p2.y, 2u);
  }
}

TEST(GeometryTest, CanUsePointAssignmentOperators) {
  // Point on RHS
  {
    IPoint p(1, 2);
    p += IPoint(1, 2);
    ASSERT_EQ(p.x, 2u);
    ASSERT_EQ(p.y, 4u);
  }

  {
    IPoint p(3, 6);
    p -= IPoint(1, 2);
    ASSERT_EQ(p.x, 2u);
    ASSERT_EQ(p.y, 4u);
  }

  {
    IPoint p(1, 2);
    p *= IPoint(2, 3);
    ASSERT_EQ(p.x, 2u);
    ASSERT_EQ(p.y, 6u);
  }

  {
    IPoint p(2, 6);
    p /= IPoint(2, 3);
    ASSERT_EQ(p.x, 1u);
    ASSERT_EQ(p.y, 2u);
  }

  // Size on RHS
  {
    IPoint p(1, 2);
    p += ISize(1, 2);
    ASSERT_EQ(p.x, 2u);
    ASSERT_EQ(p.y, 4u);
  }

  {
    IPoint p(3, 6);
    p -= ISize(1, 2);
    ASSERT_EQ(p.x, 2u);
    ASSERT_EQ(p.y, 4u);
  }

  {
    IPoint p(1, 2);
    p *= ISize(2, 3);
    ASSERT_EQ(p.x, 2u);
    ASSERT_EQ(p.y, 6u);
  }

  {
    IPoint p(2, 6);
    p /= ISize(2, 3);
    ASSERT_EQ(p.x, 1u);
    ASSERT_EQ(p.y, 2u);
  }

  // Arithmetic type on RHS
  {
    IPoint p(1, 2);
    p *= 3;
    ASSERT_EQ(p.x, 3u);
    ASSERT_EQ(p.y, 6u);
  }

  {
    IPoint p(3, 6);
    p /= 3;
    ASSERT_EQ(p.x, 1u);
    ASSERT_EQ(p.y, 2u);
  }
}

TEST(GeometryTest, PointDotProduct) {
  {
    Point p(1, 0);
    Scalar s = p.Dot(Point(-1, 0));
    ASSERT_FLOAT_EQ(s, -1);
  }

  {
    Point p(0, -1);
    Scalar s = p.Dot(Point(-1, 0));
    ASSERT_FLOAT_EQ(s, 0);
  }

  {
    Point p(1, 2);
    Scalar s = p.Dot(Point(3, -4));
    ASSERT_FLOAT_EQ(s, -5);
  }
}

TEST(GeometryTest, PointCrossProduct) {
  {
    Point p(1, 0);
    Scalar s = p.Cross(Point(-1, 0));
    ASSERT_FLOAT_EQ(s, 0);
  }

  {
    Point p(0, -1);
    Scalar s = p.Cross(Point(-1, 0));
    ASSERT_FLOAT_EQ(s, -1);
  }

  {
    Point p(1, 2);
    Scalar s = p.Cross(Point(3, -4));
    ASSERT_FLOAT_EQ(s, -10);
  }
}

TEST(GeometryTest, PointReflect) {
  {
    Point axis = Point(0, 1);
    Point a(2, 3);
    auto reflected = a.Reflect(axis);
    auto expected = Point(2, -3);
    ASSERT_POINT_NEAR(reflected, expected);
  }

  {
    Point axis = Point(1, 1).Normalize();
    Point a(1, 0);
    auto reflected = a.Reflect(axis);
    auto expected = Point(0, -1);
    ASSERT_POINT_NEAR(reflected, expected);
  }

  {
    Point axis = Point(1, 1).Normalize();
    Point a(-1, -1);
    auto reflected = a.Reflect(axis);
    ASSERT_POINT_NEAR(reflected, -a);
  }
}

TEST(GeometryTest, PointAbs) {
  Point a(-1, -2);
  auto a_abs = a.Abs();
  auto expected = Point(1, 2);
  ASSERT_POINT_NEAR(a_abs, expected);
}

TEST(GeometryTest, PointRotate) {
  {
    Point a(1, 0);
    auto rotated = a.Rotate(Radians{kPiOver2});
    auto expected = Point(0, 1);
    ASSERT_POINT_NEAR(rotated, expected);
  }

  {
    Point a(1, 0);
    auto rotated = a.Rotate(Radians{-kPiOver2});
    auto expected = Point(0, -1);
    ASSERT_POINT_NEAR(rotated, expected);
  }

  {
    Point a(1, 0);
    auto rotated = a.Rotate(Radians{kPi});
    auto expected = Point(-1, 0);
    ASSERT_POINT_NEAR(rotated, expected);
  }

  {
    Point a(1, 0);
    auto rotated = a.Rotate(Radians{kPi * 1.5});
    auto expected = Point(0, -1);
    ASSERT_POINT_NEAR(rotated, expected);
  }
}

TEST(GeometryTest, PointAngleTo) {
  // Negative result in the CCW (with up = -Y) direction.
  {
    Point a(1, 1);
    Point b(1, -1);
    Radians actual = a.AngleTo(b);
    Radians expected = Radians{-kPi / 2};
    ASSERT_FLOAT_EQ(actual.radians, expected.radians);
  }

  // Check the other direction to ensure the result is signed correctly.
  {
    Point a(1, -1);
    Point b(1, 1);
    Radians actual = a.AngleTo(b);
    Radians expected = Radians{kPi / 2};
    ASSERT_FLOAT_EQ(actual.radians, expected.radians);
  }

  // Differences in magnitude should have no impact on the result.
  {
    Point a(100, -100);
    Point b(0.01, 0.01);
    Radians actual = a.AngleTo(b);
    Radians expected = Radians{kPi / 2};
    ASSERT_FLOAT_EQ(actual.radians, expected.radians);
  }
}

TEST(GeometryTest, PointMin) {
  Point p(1, 2);
  Point result = p.Min({0, 10});
  Point expected(0, 2);
  ASSERT_POINT_NEAR(result, expected);
}

TEST(GeometryTest, Vector4IsFinite) {
  {
    Vector4 v;
    ASSERT_TRUE(v.IsFinite());
    v.x = std::numeric_limits<Scalar>::infinity();
    ASSERT_FALSE(v.IsFinite());
    v.x = -std::numeric_limits<Scalar>::infinity();
    ASSERT_FALSE(v.IsFinite());
    v.x = -std::numeric_limits<Scalar>::quiet_NaN();
    ASSERT_FALSE(v.IsFinite());
  }

  {
    Vector4 v;
    ASSERT_TRUE(v.IsFinite());
    v.y = std::numeric_limits<Scalar>::infinity();
    ASSERT_FALSE(v.IsFinite());
    v.y = -std::numeric_limits<Scalar>::infinity();
    ASSERT_FALSE(v.IsFinite());
    v.y = -std::numeric_limits<Scalar>::quiet_NaN();
    ASSERT_FALSE(v.IsFinite());
  }

  {
    Vector4 v;
    ASSERT_TRUE(v.IsFinite());
    v.z = std::numeric_limits<Scalar>::infinity();
    ASSERT_FALSE(v.IsFinite());
    v.z = -std::numeric_limits<Scalar>::infinity();
    ASSERT_FALSE(v.IsFinite());
    v.z = -std::numeric_limits<Scalar>::quiet_NaN();
    ASSERT_FALSE(v.IsFinite());
  }

  {
    Vector4 v;
    ASSERT_TRUE(v.IsFinite());
    v.w = std::numeric_limits<Scalar>::infinity();
    ASSERT_FALSE(v.IsFinite());
    v.w = -std::numeric_limits<Scalar>::infinity();
    ASSERT_FALSE(v.IsFinite());
    v.w = -std::numeric_limits<Scalar>::quiet_NaN();
    ASSERT_FALSE(v.IsFinite());
  }
}

TEST(GeometryTest, Vector3Min) {
  Vector3 p(1, 2, 3);
  Vector3 result = p.Min({0, 10, 2});
  Vector3 expected(0, 2, 2);
  ASSERT_VECTOR3_NEAR(result, expected);
}

TEST(GeometryTest, Vector4Min) {
  Vector4 p(1, 2, 3, 4);
  Vector4 result = p.Min({0, 10, 2, 1});
  Vector4 expected(0, 2, 2, 1);
  ASSERT_VECTOR4_NEAR(result, expected);
}

TEST(GeometryTest, PointMax) {
  Point p(1, 2);
  Point result = p.Max({0, 10});
  Point expected(1, 10);
  ASSERT_POINT_NEAR(result, expected);
}

TEST(GeometryTest, Vector3Max) {
  Vector3 p(1, 2, 3);
  Vector3 result = p.Max({0, 10, 2});
  Vector3 expected(1, 10, 3);
  ASSERT_VECTOR3_NEAR(result, expected);
}

TEST(GeometryTest, Vector4Max) {
  Vector4 p(1, 2, 3, 4);
  Vector4 result = p.Max({0, 10, 2, 1});
  Vector4 expected(1, 10, 3, 4);
  ASSERT_VECTOR4_NEAR(result, expected);
}

TEST(GeometryTest, PointFloor) {
  Point p(1.5, 2.3);
  Point result = p.Floor();
  Point expected(1, 2);
  ASSERT_POINT_NEAR(result, expected);
}

TEST(GeometryTest, Vector3Floor) {
  Vector3 p(1.5, 2.3, 3.9);
  Vector3 result = p.Floor();
  Vector3 expected(1, 2, 3);
  ASSERT_VECTOR3_NEAR(result, expected);
}

TEST(GeometryTest, Vector4Floor) {
  Vector4 p(1.5, 2.3, 3.9, 4.0);
  Vector4 result = p.Floor();
  Vector4 expected(1, 2, 3, 4);
  ASSERT_VECTOR4_NEAR(result, expected);
}

TEST(GeometryTest, PointCeil) {
  Point p(1.5, 2.3);
  Point result = p.Ceil();
  Point expected(2, 3);
  ASSERT_POINT_NEAR(result, expected);
}

TEST(GeometryTest, Vector3Ceil) {
  Vector3 p(1.5, 2.3, 3.9);
  Vector3 result = p.Ceil();
  Vector3 expected(2, 3, 4);
  ASSERT_VECTOR3_NEAR(result, expected);
}

TEST(GeometryTest, Vector4Ceil) {
  Vector4 p(1.5, 2.3, 3.9, 4.0);
  Vector4 result = p.Ceil();
  Vector4 expected(2, 3, 4, 4);
  ASSERT_VECTOR4_NEAR(result, expected);
}

TEST(GeometryTest, PointRound) {
  Point p(1.5, 2.3);
  Point result = p.Round();
  Point expected(2, 2);
  ASSERT_POINT_NEAR(result, expected);
}

TEST(GeometryTest, Vector3Round) {
  Vector3 p(1.5, 2.3, 3.9);
  Vector3 result = p.Round();
  Vector3 expected(2, 2, 4);
  ASSERT_VECTOR3_NEAR(result, expected);
}

TEST(GeometryTest, Vector4Round) {
  Vector4 p(1.5, 2.3, 3.9, 4.0);
  Vector4 result = p.Round();
  Vector4 expected(2, 2, 4, 4);
  ASSERT_VECTOR4_NEAR(result, expected);
}

TEST(GeometryTest, PointLerp) {
  Point p(1, 2);
  Point result = p.Lerp({5, 10}, 0.75);
  Point expected(4, 8);
  ASSERT_POINT_NEAR(result, expected);
}

TEST(GeometryTest, Vector3Lerp) {
  Vector3 p(1, 2, 3);
  Vector3 result = p.Lerp({5, 10, 15}, 0.75);
  Vector3 expected(4, 8, 12);
  ASSERT_VECTOR3_NEAR(result, expected);
}

TEST(GeometryTest, Vector4Lerp) {
  Vector4 p(1, 2, 3, 4);
  Vector4 result = p.Lerp({5, 10, 15, 20}, 0.75);
  Vector4 expected(4, 8, 12, 16);
  ASSERT_VECTOR4_NEAR(result, expected);
}

TEST(GeometryTest, SeparatedVector2NormalizesWithConstructor) {
  SeparatedVector2 v(Vector2(10, 0));
  ASSERT_POINT_NEAR(v.direction, Vector2(1, 0));
  ASSERT_NEAR(v.magnitude, 10, kEhCloseEnough);
}

TEST(GeometryTest, SeparatedVector2GetVector) {
  SeparatedVector2 v(Vector2(10, 0));
  ASSERT_POINT_NEAR(v.GetVector(), Vector2(10, 0));
}

TEST(GeometryTest, SeparatedVector2GetAlignment) {
  // Parallel
  {
    SeparatedVector2 v(Vector2(10, 0));
    Scalar actual = v.GetAlignment(SeparatedVector2(Vector2(5, 0)));
    ASSERT_NEAR(actual, 1, kEhCloseEnough);
  }

  // Perpendicular
  {
    SeparatedVector2 v(Vector2(10, 0));
    Scalar actual = v.GetAlignment(SeparatedVector2(Vector2(0, 5)));
    ASSERT_NEAR(actual, 0, kEhCloseEnough);
  }

  // Opposite parallel
  {
    SeparatedVector2 v(Vector2(0, 10));
    Scalar actual = v.GetAlignment(SeparatedVector2(Vector2(0, -5)));
    ASSERT_NEAR(actual, -1, kEhCloseEnough);
  }
}

TEST(GeometryTest, SeparatedVector2AngleTo) {
  {
    SeparatedVector2 v(Vector2(10, 0));
    Radians actual = v.AngleTo(SeparatedVector2(Vector2(5, 0)));
    Radians expected = Radians{0};
    ASSERT_NEAR(actual.radians, expected.radians, kEhCloseEnough);
  }

  {
    SeparatedVector2 v(Vector2(10, 0));
    Radians actual = v.AngleTo(SeparatedVector2(Vector2(0, -5)));
    Radians expected = Radians{-kPi / 2};
    ASSERT_NEAR(actual.radians, expected.radians, kEhCloseEnough);
  }
}

TEST(GeometryTest, CanUseVector3AssignmentOperators) {
  {
    Vector3 p(1, 2, 4);
    p += Vector3(1, 2, 4);
    ASSERT_EQ(p.x, 2u);
    ASSERT_EQ(p.y, 4u);
    ASSERT_EQ(p.z, 8u);
  }

  {
    Vector3 p(3, 6, 8);
    p -= Vector3(1, 2, 3);
    ASSERT_EQ(p.x, 2u);
    ASSERT_EQ(p.y, 4u);
    ASSERT_EQ(p.z, 5u);
  }

  {
    Vector3 p(1, 2, 3);
    p *= Vector3(2, 3, 4);
    ASSERT_EQ(p.x, 2u);
    ASSERT_EQ(p.y, 6u);
    ASSERT_EQ(p.z, 12u);
  }

  {
    Vector3 p(1, 2, 3);
    p *= 2;
    ASSERT_EQ(p.x, 2u);
    ASSERT_EQ(p.y, 4u);
    ASSERT_EQ(p.z, 6u);
  }

  {
    Vector3 p(2, 6, 12);
    p /= Vector3(2, 3, 4);
    ASSERT_EQ(p.x, 1u);
    ASSERT_EQ(p.y, 2u);
    ASSERT_EQ(p.z, 3u);
  }

  {
    Vector3 p(2, 6, 12);
    p /= 2;
    ASSERT_EQ(p.x, 1u);
    ASSERT_EQ(p.y, 3u);
    ASSERT_EQ(p.z, 6u);
  }
}

TEST(GeometryTest, CanPerformAlgebraicVector3Ops) {
  {
    Vector3 p1(1, 2, 3);
    Vector3 p2 = p1 + Vector3(1, 2, 3);
    ASSERT_EQ(p2.x, 2u);
    ASSERT_EQ(p2.y, 4u);
    ASSERT_EQ(p2.z, 6u);
  }

  {
    Vector3 p1(3, 6, 9);
    Vector3 p2 = p1 - Vector3(1, 2, 3);
    ASSERT_EQ(p2.x, 2u);
    ASSERT_EQ(p2.y, 4u);
    ASSERT_EQ(p2.z, 6u);
  }

  {
    Vector3 p1(1, 2, 3);
    Vector3 p2 = p1 * Vector3(2, 3, 4);
    ASSERT_EQ(p2.x, 2u);
    ASSERT_EQ(p2.y, 6u);
    ASSERT_EQ(p2.z, 12u);
  }

  {
    Vector3 p1(2, 6, 12);
    Vector3 p2 = p1 / Vector3(2, 3, 4);
    ASSERT_EQ(p2.x, 1u);
    ASSERT_EQ(p2.y, 2u);
    ASSERT_EQ(p2.z, 3u);
  }
}

TEST(GeometryTest, CanPerformAlgebraicVector3OpsWithArithmeticTypes) {
  // LHS
  {
    Vector3 p1(1, 2, 3);
    Vector3 p2 = p1 + 2.0f;
    ASSERT_EQ(p2.x, 3);
    ASSERT_EQ(p2.y, 4);
    ASSERT_EQ(p2.z, 5);
  }

  {
    Vector3 p1(1, 2, 3);
    Vector3 p2 = p1 - 2.0f;
    ASSERT_EQ(p2.x, -1);
    ASSERT_EQ(p2.y, 0);
    ASSERT_EQ(p2.z, 1);
  }

  {
    Vector3 p1(1, 2, 3);
    Vector3 p2 = p1 * 2.0f;
    ASSERT_EQ(p2.x, 2);
    ASSERT_EQ(p2.y, 4);
    ASSERT_EQ(p2.z, 6);
  }

  {
    Vector3 p1(2, 6, 12);
    Vector3 p2 = p1 / 2.0f;
    ASSERT_EQ(p2.x, 1);
    ASSERT_EQ(p2.y, 3);
    ASSERT_EQ(p2.z, 6);
  }

  // RHS
  {
    Vector3 p1(1, 2, 3);
    Vector3 p2 = 2.0f + p1;
    ASSERT_EQ(p2.x, 3);
    ASSERT_EQ(p2.y, 4);
    ASSERT_EQ(p2.z, 5);
  }

  {
    Vector3 p1(1, 2, 3);
    Vector3 p2 = 2.0f - p1;
    ASSERT_EQ(p2.x, 1);
    ASSERT_EQ(p2.y, 0);
    ASSERT_EQ(p2.z, -1);
  }

  {
    Vector3 p1(1, 2, 3);
    Vector3 p2 = 2.0f * p1;
    ASSERT_EQ(p2.x, 2);
    ASSERT_EQ(p2.y, 4);
    ASSERT_EQ(p2.z, 6);
  }

  {
    Vector3 p1(2, 6, 12);
    Vector3 p2 = 12.0f / p1;
    ASSERT_EQ(p2.x, 6);
    ASSERT_EQ(p2.y, 2);
    ASSERT_EQ(p2.z, 1);
  }
}

TEST(GeometryTest, ColorPremultiply) {
  {
    Color a(1.0, 0.5, 0.2, 0.5);
    Color premultiplied = a.Premultiply();
    Color expected = Color(0.5, 0.25, 0.1, 0.5);
    ASSERT_COLOR_NEAR(premultiplied, expected);
  }

  {
    Color a(0.5, 0.25, 0.1, 0.5);
    Color unpremultiplied = a.Unpremultiply();
    Color expected = Color(1.0, 0.5, 0.2, 0.5);
    ASSERT_COLOR_NEAR(unpremultiplied, expected);
  }

  {
    Color a(0.5, 0.25, 0.1, 0.0);
    Color unpremultiplied = a.Unpremultiply();
    Color expected = Color(0.0, 0.0, 0.0, 0.0);
    ASSERT_COLOR_NEAR(unpremultiplied, expected);
  }
}

TEST(GeometryTest, ColorR8G8B8A8) {
  {
    Color a(1.0, 0.5, 0.2, 0.5);
    std::array<uint8_t, 4> expected = {255, 128, 51, 128};
    ASSERT_ARRAY_4_NEAR(a.ToR8G8B8A8(), expected);
  }

  {
    Color a(0.0, 0.0, 0.0, 0.0);
    std::array<uint8_t, 4> expected = {0, 0, 0, 0};
    ASSERT_ARRAY_4_NEAR(a.ToR8G8B8A8(), expected);
  }

  {
    Color a(1.0, 1.0, 1.0, 1.0);
    std::array<uint8_t, 4> expected = {255, 255, 255, 255};
    ASSERT_ARRAY_4_NEAR(a.ToR8G8B8A8(), expected);
  }
}

TEST(GeometryTest, ColorLerp) {
  {
    Color a(0.0, 0.0, 0.0, 0.0);
    Color b(1.0, 1.0, 1.0, 1.0);

    ASSERT_COLOR_NEAR(Color::Lerp(a, b, 0.5), Color(0.5, 0.5, 0.5, 0.5));
    ASSERT_COLOR_NEAR(Color::Lerp(a, b, 0.0), a);
    ASSERT_COLOR_NEAR(Color::Lerp(a, b, 1.0), b);
    ASSERT_COLOR_NEAR(Color::Lerp(a, b, 0.2), Color(0.2, 0.2, 0.2, 0.2));
  }

  {
    Color a(0.2, 0.4, 1.0, 0.5);
    Color b(0.4, 1.0, 0.2, 0.3);

    ASSERT_COLOR_NEAR(Color::Lerp(a, b, 0.5), Color(0.3, 0.7, 0.6, 0.4));
    ASSERT_COLOR_NEAR(Color::Lerp(a, b, 0.0), a);
    ASSERT_COLOR_NEAR(Color::Lerp(a, b, 1.0), b);
    ASSERT_COLOR_NEAR(Color::Lerp(a, b, 0.2), Color(0.24, 0.52, 0.84, 0.46));
  }
}

TEST(GeometryTest, ColorClamp01) {
  {
    Color result = Color(0.5, 0.5, 0.5, 0.5).Clamp01();
    Color expected = Color(0.5, 0.5, 0.5, 0.5);
    ASSERT_COLOR_NEAR(result, expected);
  }

  {
    Color result = Color(-1, -1, -1, -1).Clamp01();
    Color expected = Color(0, 0, 0, 0);
    ASSERT_COLOR_NEAR(result, expected);
  }

  {
    Color result = Color(2, 2, 2, 2).Clamp01();
    Color expected = Color(1, 1, 1, 1);
    ASSERT_COLOR_NEAR(result, expected);
  }
}

TEST(GeometryTest, ColorMakeRGBA8) {
  {
    Color a = Color::MakeRGBA8(0, 0, 0, 0);
    Color b = Color::BlackTransparent();
    ASSERT_COLOR_NEAR(a, b);
  }

  {
    Color a = Color::MakeRGBA8(255, 255, 255, 255);
    Color b = Color::White();
    ASSERT_COLOR_NEAR(a, b);
  }

  {
    Color a = Color::MakeRGBA8(63, 127, 191, 127);
    Color b(0.247059, 0.498039, 0.74902, 0.498039);
    ASSERT_COLOR_NEAR(a, b);
  }
}

TEST(GeometryTest, ColorApplyColorMatrix) {
  {
    ColorMatrix color_matrix = {
        1, 1, 1, 1, 1,  //
        1, 1, 1, 1, 1,  //
        1, 1, 1, 1, 1,  //
        1, 1, 1, 1, 1,  //
    };
    auto result = Color::White().ApplyColorMatrix(color_matrix);
    auto expected = Color(1, 1, 1, 1);
    ASSERT_COLOR_NEAR(result, expected);
  }

  {
    ColorMatrix color_matrix = {
        0.1, 0,   0,   0,   0.01,  //
        0,   0.2, 0,   0,   0.02,  //
        0,   0,   0.3, 0,   0.03,  //
        0,   0,   0,   0.4, 0.04,  //
    };
    auto result = Color::White().ApplyColorMatrix(color_matrix);
    auto expected = Color(0.11, 0.22, 0.33, 0.44);
    ASSERT_COLOR_NEAR(result, expected);
  }
}

TEST(GeometryTest, ColorLinearToSRGB) {
  {
    auto result = Color::White().LinearToSRGB();
    auto expected = Color(1, 1, 1, 1);
    ASSERT_COLOR_NEAR(result, expected);
  }

  {
    auto result = Color::BlackTransparent().LinearToSRGB();
    auto expected = Color(0, 0, 0, 0);
    ASSERT_COLOR_NEAR(result, expected);
  }

  {
    auto result = Color(0.2, 0.4, 0.6, 0.8).LinearToSRGB();
    auto expected = Color(0.484529, 0.665185, 0.797738, 0.8);
    ASSERT_COLOR_NEAR(result, expected);
  }
}

TEST(GeometryTest, ColorSRGBToLinear) {
  {
    auto result = Color::White().SRGBToLinear();
    auto expected = Color(1, 1, 1, 1);
    ASSERT_COLOR_NEAR(result, expected);
  }

  {
    auto result = Color::BlackTransparent().SRGBToLinear();
    auto expected = Color(0, 0, 0, 0);
    ASSERT_COLOR_NEAR(result, expected);
  }

  {
    auto result = Color(0.2, 0.4, 0.6, 0.8).SRGBToLinear();
    auto expected = Color(0.0331048, 0.132868, 0.318547, 0.8);
    ASSERT_COLOR_NEAR(result, expected);
  }
}

struct ColorBlendTestData {
  static constexpr Color kDestinationColor =
      Color::CornflowerBlue().WithAlpha(0.75);
  static constexpr Color kSourceColors[] = {Color::White().WithAlpha(0.75),
                                            Color::LimeGreen().WithAlpha(0.75),
                                            Color::Black().WithAlpha(0.75)};

  static const std::map<BlendMode, Color>
      kExpectedResults[sizeof(kSourceColors)];
};

// THIS RESULT TABLE IS GENERATED!
//
// Uncomment the `GenerateColorBlendResults` test below to print a new table
// after making changes to `Color::Blend`.
const std::map<BlendMode, Color> ColorBlendTestData::kExpectedResults[sizeof(
    ColorBlendTestData::kSourceColors)] = {
    {
        {BlendMode::kClear, {0, 0, 0, 0}},
        {BlendMode::kSource, {1, 1, 1, 0.75}},
        {BlendMode::kDestination, {0.392157, 0.584314, 0.929412, 0.75}},
        {BlendMode::kSourceOver, {0.878431, 0.916863, 0.985882, 0.9375}},
        {BlendMode::kDestinationOver, {0.513726, 0.667451, 0.943529, 0.9375}},
        {BlendMode::kSourceIn, {1, 1, 1, 0.5625}},
        {BlendMode::kDestinationIn, {0.392157, 0.584314, 0.929412, 0.5625}},
        {BlendMode::kSourceOut, {1, 1, 1, 0.1875}},
        {BlendMode::kDestinationOut, {0.392157, 0.584314, 0.929412, 0.1875}},
        {BlendMode::kSourceATop, {0.848039, 0.896078, 0.982353, 0.75}},
        {BlendMode::kDestinationATop, {0.544118, 0.688235, 0.947059, 0.75}},
        {BlendMode::kXor, {0.696078, 0.792157, 0.964706, 0.375}},
        {BlendMode::kPlus, {1, 1, 1, 1}},
        {BlendMode::kModulate, {0.392157, 0.584314, 0.929412, 0.5625}},
        {BlendMode::kScreen, {0.878431, 0.916863, 0.985882, 0.9375}},
        {BlendMode::kOverlay, {0.74902, 0.916863, 0.985882, 0.9375}},
        {BlendMode::kDarken, {0.513726, 0.667451, 0.943529, 0.9375}},
        {BlendMode::kLighten, {0.878431, 0.916863, 0.985882, 0.9375}},
        {BlendMode::kColorDodge, {0.878431, 0.916863, 0.985882, 0.9375}},
        {BlendMode::kColorBurn, {0.513725, 0.667451, 0.943529, 0.9375}},
        {BlendMode::kHardLight, {0.878431, 0.916863, 0.985882, 0.9375}},
        {BlendMode::kSoftLight, {0.654166, 0.775505, 0.964318, 0.9375}},
        {BlendMode::kDifference, {0.643137, 0.566275, 0.428235, 0.9375}},
        {BlendMode::kExclusion, {0.643137, 0.566275, 0.428235, 0.9375}},
        {BlendMode::kMultiply, {0.513726, 0.667451, 0.943529, 0.9375}},
        {BlendMode::kHue, {0.617208, 0.655639, 0.724659, 0.9375}},
        {BlendMode::kSaturation, {0.617208, 0.655639, 0.724659, 0.9375}},
        {BlendMode::kColor, {0.617208, 0.655639, 0.724659, 0.9375}},
        {BlendMode::kLuminosity, {0.878431, 0.916863, 0.985882, 0.9375}},
    },
    {
        {BlendMode::kClear, {0, 0, 0, 0}},
        {BlendMode::kSource, {0.196078, 0.803922, 0.196078, 0.75}},
        {BlendMode::kDestination, {0.392157, 0.584314, 0.929412, 0.75}},
        {BlendMode::kSourceOver, {0.235294, 0.76, 0.342745, 0.9375}},
        {BlendMode::kDestinationOver, {0.352941, 0.628235, 0.782745, 0.9375}},
        {BlendMode::kSourceIn, {0.196078, 0.803922, 0.196078, 0.5625}},
        {BlendMode::kDestinationIn, {0.392157, 0.584314, 0.929412, 0.5625}},
        {BlendMode::kSourceOut, {0.196078, 0.803922, 0.196078, 0.1875}},
        {BlendMode::kDestinationOut, {0.392157, 0.584314, 0.929412, 0.1875}},
        {BlendMode::kSourceATop, {0.245098, 0.74902, 0.379412, 0.75}},
        {BlendMode::kDestinationATop, {0.343137, 0.639216, 0.746078, 0.75}},
        {BlendMode::kXor, {0.294118, 0.694118, 0.562745, 0.375}},
        {BlendMode::kPlus, {0.441176, 1, 0.844118, 1}},
        {BlendMode::kModulate, {0.0768935, 0.469742, 0.182238, 0.5625}},
        {BlendMode::kScreen, {0.424452, 0.828743, 0.79105, 0.9375}},
        {BlendMode::kOverlay, {0.209919, 0.779839, 0.757001, 0.9375}},
        {BlendMode::kDarken, {0.235294, 0.628235, 0.342745, 0.9375}},
        {BlendMode::kLighten, {0.352941, 0.76, 0.782745, 0.9375}},
        {BlendMode::kColorDodge, {0.41033, 0.877647, 0.825098, 0.9375}},
        {BlendMode::kColorBurn, {0.117647, 0.567403, 0.609098, 0.9375}},
        {BlendMode::kHardLight, {0.209919, 0.779839, 0.443783, 0.9375}},
        {BlendMode::kSoftLight, {0.266006, 0.693915, 0.758818, 0.9375}},
        {BlendMode::kDifference, {0.235294, 0.409412, 0.665098, 0.9375}},
        {BlendMode::kExclusion, {0.378316, 0.546897, 0.681707, 0.9375}},
        {BlendMode::kMultiply, {0.163783, 0.559493, 0.334441, 0.9375}},
        {BlendMode::kHue, {0.266235, 0.748588, 0.373686, 0.9375}},
        {BlendMode::kSaturation, {0.339345, 0.629787, 0.811502, 0.9375}},
        {BlendMode::kColor, {0.241247, 0.765953, 0.348698, 0.9375}},
        {BlendMode::kLuminosity, {0.346988, 0.622282, 0.776792, 0.9375}},
    },
    {
        {BlendMode::kClear, {0, 0, 0, 0}},
        {BlendMode::kSource, {0, 0, 0, 0.75}},
        {BlendMode::kDestination, {0.392157, 0.584314, 0.929412, 0.75}},
        {BlendMode::kSourceOver, {0.0784314, 0.116863, 0.185882, 0.9375}},
        {BlendMode::kDestinationOver, {0.313726, 0.467451, 0.743529, 0.9375}},
        {BlendMode::kSourceIn, {0, 0, 0, 0.5625}},
        {BlendMode::kDestinationIn, {0.392157, 0.584314, 0.929412, 0.5625}},
        {BlendMode::kSourceOut, {0, 0, 0, 0.1875}},
        {BlendMode::kDestinationOut, {0.392157, 0.584314, 0.929412, 0.1875}},
        {BlendMode::kSourceATop, {0.0980392, 0.146078, 0.232353, 0.75}},
        {BlendMode::kDestinationATop, {0.294118, 0.438235, 0.697059, 0.75}},
        {BlendMode::kXor, {0.196078, 0.292157, 0.464706, 0.375}},
        {BlendMode::kPlus, {0.294118, 0.438235, 0.697059, 1}},
        {BlendMode::kModulate, {0, 0, 0, 0.5625}},
        {BlendMode::kScreen, {0.313726, 0.467451, 0.743529, 0.9375}},
        {BlendMode::kOverlay, {0.0784314, 0.218039, 0.701176, 0.9375}},
        {BlendMode::kDarken, {0.0784314, 0.116863, 0.185882, 0.9375}},
        {BlendMode::kLighten, {0.313726, 0.467451, 0.743529, 0.9375}},
        {BlendMode::kColorDodge, {0.313726, 0.467451, 0.743529, 0.9375}},
        {BlendMode::kColorBurn, {0.0784314, 0.116863, 0.185882, 0.9375}},
        {BlendMode::kHardLight, {0.0784314, 0.116863, 0.185882, 0.9375}},
        {BlendMode::kSoftLight, {0.170704, 0.321716, 0.704166, 0.9375}},
        {BlendMode::kDifference, {0.313726, 0.467451, 0.743529, 0.9375}},
        {BlendMode::kExclusion, {0.313726, 0.467451, 0.743529, 0.9375}},
        {BlendMode::kMultiply, {0.0784314, 0.116863, 0.185882, 0.9375}},
        {BlendMode::kHue, {0.417208, 0.455639, 0.524659, 0.9375}},
        {BlendMode::kSaturation, {0.417208, 0.455639, 0.524659, 0.9375}},
        {BlendMode::kColor, {0.417208, 0.455639, 0.524659, 0.9375}},
        {BlendMode::kLuminosity, {0.0784314, 0.116863, 0.185882, 0.9375}},
    },
};

/// To print a new ColorBlendTestData::kExpectedResults table, uncomment this
/// test and run with:
/// --gtest_filter="GeometryTest.GenerateColorBlendResults"
/*
TEST(GeometryTest, GenerateColorBlendResults) {
  auto& o = std::cout;
  using BlendT = std::underlying_type_t<BlendMode>;
  o << "{";
  for (const auto& source : ColorBlendTestData::kSourceColors) {
    o << "{";
    for (BlendT blend_i = 0;
         blend_i < static_cast<BlendT>(BlendMode::kLast) + 1; blend_i++) {
      auto blend = static_cast<BlendMode>(blend_i);
      Color c = ColorBlendTestData::kDestinationColor.Blend(source, blend);
      o << "{ BlendMode::k" << BlendModeToString(blend) << ", ";
      o << "{" << c.red << "," << c.green << "," << c.blue << "," << c.alpha
        << "}";
      o << "}," << std::endl;
    }
    o << "},";
  }
  o << "};" << std::endl;
}
*/

#define _BLEND_MODE_RESULT_CHECK(blend_mode)                \
  expected = ColorBlendTestData::kExpectedResults[source_i] \
                 .find(BlendMode::k##blend_mode)            \
                 ->second;                                  \
  EXPECT_COLOR_NEAR(dst.Blend(src, BlendMode::k##blend_mode), expected);

TEST(GeometryTest, ColorBlendReturnsExpectedResults) {
  Color dst = ColorBlendTestData::kDestinationColor;
  for (size_t source_i = 0;
       source_i < sizeof(ColorBlendTestData::kSourceColors) / sizeof(Color);
       source_i++) {
    Color src = ColorBlendTestData::kSourceColors[source_i];

    Color expected;
    IMPELLER_FOR_EACH_BLEND_MODE(_BLEND_MODE_RESULT_CHECK)
  }
}

#define _BLEND_MODE_NAME_CHECK(blend_mode) \
  case BlendMode::k##blend_mode:           \
    ASSERT_STREQ(result, #blend_mode);     \
    break;

TEST(GeometryTest, BlendModeToString) {
  using BlendT = std::underlying_type_t<BlendMode>;
  for (BlendT i = 0; i <= static_cast<BlendT>(BlendMode::kLast); i++) {
    auto mode = static_cast<BlendMode>(i);
    auto result = BlendModeToString(mode);
    switch (mode) { IMPELLER_FOR_EACH_BLEND_MODE(_BLEND_MODE_NAME_CHECK) }
  }
}

TEST(GeometryTest, CanConvertBetweenDegressAndRadians) {
  {
    auto deg = Degrees{90.0};
    Radians rad = deg;
    ASSERT_FLOAT_EQ(rad.radians, kPiOver2);
  }
}

TEST(GeometryTest, MatrixPrinting) {
  {
    std::stringstream stream;
    Matrix m;
    stream << m;
    ASSERT_EQ(stream.str(), R"((
       1.000000,       0.000000,       0.000000,       0.000000,
       0.000000,       1.000000,       0.000000,       0.000000,
       0.000000,       0.000000,       1.000000,       0.000000,
       0.000000,       0.000000,       0.000000,       1.000000,
))");
  }

  {
    std::stringstream stream;
    Matrix m = Matrix::MakeTranslation(Vector3(10, 20, 30));
    stream << m;

    ASSERT_EQ(stream.str(), R"((
       1.000000,       0.000000,       0.000000,      10.000000,
       0.000000,       1.000000,       0.000000,      20.000000,
       0.000000,       0.000000,       1.000000,      30.000000,
       0.000000,       0.000000,       0.000000,       1.000000,
))");
  }
}

TEST(GeometryTest, PointPrinting) {
  {
    std::stringstream stream;
    Point m;
    stream << m;
    ASSERT_EQ(stream.str(), "(0, 0)");
  }

  {
    std::stringstream stream;
    Point m(13, 37);
    stream << m;
    ASSERT_EQ(stream.str(), "(13, 37)");
  }
}

TEST(GeometryTest, Vector3Printing) {
  {
    std::stringstream stream;
    Vector3 m;
    stream << m;
    ASSERT_EQ(stream.str(), "(0, 0, 0)");
  }

  {
    std::stringstream stream;
    Vector3 m(1, 2, 3);
    stream << m;
    ASSERT_EQ(stream.str(), "(1, 2, 3)");
  }
}

TEST(GeometryTest, Vector4Printing) {
  {
    std::stringstream stream;
    Vector4 m;
    stream << m;
    ASSERT_EQ(stream.str(), "(0, 0, 0, 1)");
  }

  {
    std::stringstream stream;
    Vector4 m(1, 2, 3, 4);
    stream << m;
    ASSERT_EQ(stream.str(), "(1, 2, 3, 4)");
  }
}

TEST(GeometryTest, ColorPrinting) {
  {
    std::stringstream stream;
    Color m;
    stream << m;
    ASSERT_EQ(stream.str(), "(0, 0, 0, 0)");
  }

  {
    std::stringstream stream;
    Color m(1, 2, 3, 4);
    stream << m;
    ASSERT_EQ(stream.str(), "(1, 2, 3, 4)");
  }
}

TEST(GeometryTest, ToIColor) {
  ASSERT_EQ(Color::ToIColor(Color(0, 0, 0, 0)), 0u);
  ASSERT_EQ(Color::ToIColor(Color(1.0, 1.0, 1.0, 1.0)), 0xFFFFFFFF);
  ASSERT_EQ(Color::ToIColor(Color(0.5, 0.5, 1.0, 1.0)), 0xFF8080FF);
}

TEST(GeometryTest, Gradient) {
  {
    // Simple 2 color gradient produces color buffer containing exactly those
    // values.
    std::vector<Color> colors = {Color::Red(), Color::Blue()};
    std::vector<Scalar> stops = {0.0, 1.0};

    auto gradient = CreateGradientBuffer(colors, stops);

    ASSERT_COLOR_BUFFER_NEAR(gradient.color_bytes, colors);
    ASSERT_EQ(gradient.texture_size, 2u);
  }

  {
    // Gradient with duplicate stops does not create an empty texture.
    std::vector<Color> colors = {Color::Red(), Color::Yellow(), Color::Black(),
                                 Color::Blue()};
    std::vector<Scalar> stops = {0.0, 0.25, 0.25, 1.0};

    auto gradient = CreateGradientBuffer(colors, stops);
    ASSERT_EQ(gradient.texture_size, 5u);
  }

  {
    // Simple N color gradient produces color buffer containing exactly those
    // values.
    std::vector<Color> colors = {Color::Red(), Color::Blue(), Color::Green(),
                                 Color::White()};
    std::vector<Scalar> stops = {0.0, 0.33, 0.66, 1.0};

    auto gradient = CreateGradientBuffer(colors, stops);

    ASSERT_COLOR_BUFFER_NEAR(gradient.color_bytes, colors);
    ASSERT_EQ(gradient.texture_size, 4u);
  }

  {
    // Gradient with color stops will lerp and scale buffer.
    std::vector<Color> colors = {Color::Red(), Color::Blue(), Color::Green()};
    std::vector<Scalar> stops = {0.0, 0.25, 1.0};

    auto gradient = CreateGradientBuffer(colors, stops);

    std::vector<Color> lerped_colors = {
        Color::Red(),
        Color::Blue(),
        Color::Lerp(Color::Blue(), Color::Green(), 0.3333),
        Color::Lerp(Color::Blue(), Color::Green(), 0.6666),
        Color::Green(),
    };
    ASSERT_COLOR_BUFFER_NEAR(gradient.color_bytes, lerped_colors);
    ASSERT_EQ(gradient.texture_size, 5u);
  }

  {
    // Gradient size is capped at 1024.
    std::vector<Color> colors = {};
    std::vector<Scalar> stops = {};
    for (auto i = 0u; i < 1025; i++) {
      colors.push_back(Color::Blue());
      stops.push_back(i / 1025.0);
    }

    auto gradient = CreateGradientBuffer(colors, stops);

    ASSERT_EQ(gradient.texture_size, 1024u);
    ASSERT_EQ(gradient.color_bytes.size(), 1024u * 4);
  }
}

TEST(GeometryTest, HalfConversions) {
#if defined(FML_OS_MACOSX) || defined(FML_OS_IOS) || \
    defined(FML_OS_IOS_SIMULATOR)
  ASSERT_EQ(ScalarToHalf(0.0), 0.0f16);
  ASSERT_EQ(ScalarToHalf(0.05), 0.05f16);
  ASSERT_EQ(ScalarToHalf(2.43), 2.43f16);
  ASSERT_EQ(ScalarToHalf(-1.45), -1.45f16);

  // 65504 is the largest possible half.
  ASSERT_EQ(ScalarToHalf(65504.0f), 65504.0f16);
  ASSERT_EQ(ScalarToHalf(65504.0f + 1), 65504.0f16);

  // Colors
  ASSERT_EQ(HalfVector4(Color::Red()),
            HalfVector4(1.0f16, 0.0f16, 0.0f16, 1.0f16));
  ASSERT_EQ(HalfVector4(Color::Green()),
            HalfVector4(0.0f16, 1.0f16, 0.0f16, 1.0f16));
  ASSERT_EQ(HalfVector4(Color::Blue()),
            HalfVector4(0.0f16, 0.0f16, 1.0f16, 1.0f16));
  ASSERT_EQ(HalfVector4(Color::Black().WithAlpha(0)),
            HalfVector4(0.0f16, 0.0f16, 0.0f16, 0.0f16));

  ASSERT_EQ(HalfVector3(Vector3(4.0, 6.0, -1.0)),
            HalfVector3(4.0f16, 6.0f16, -1.0f16));
  ASSERT_EQ(HalfVector2(Vector2(4.0, 6.0)), HalfVector2(4.0f16, 6.0f16));

  ASSERT_EQ(Half(0.5f), Half(0.5f16));
  ASSERT_EQ(Half(0.5), Half(0.5f16));
  ASSERT_EQ(Half(5), Half(5.0f16));
#else
  GTEST_SKIP() << "Half-precision floats (IEEE 754) are not portable and "
                  "only used on Apple platforms.";
#endif  // FML_OS_MACOSX || FML_OS_IOS || FML_OS_IOS_SIMULATOR
}

}  // namespace testing
}  // namespace impeller

// NOLINTEND(bugprone-unchecked-optional-access)
