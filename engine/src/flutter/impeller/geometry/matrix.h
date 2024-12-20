// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_GEOMETRY_MATRIX_H_
#define FLUTTER_IMPELLER_GEOMETRY_MATRIX_H_

#include <cmath>
#include <iomanip>
#include <limits>
#include <optional>
#include <ostream>
#include <utility>

#include "impeller/geometry/matrix_decomposition.h"
#include "impeller/geometry/point.h"
#include "impeller/geometry/quaternion.h"
#include "impeller/geometry/scalar.h"
#include "impeller/geometry/shear.h"
#include "impeller/geometry/size.h"
#include "impeller/geometry/vector.h"

namespace impeller {

//------------------------------------------------------------------------------
/// @brief      A 4x4 matrix using column-major storage.
///
///             Utility methods that need to make assumptions about normalized
///             device coordinates must use the following convention:
///               * Left-handed coordinate system. Positive rotation is
///                 clockwise about axis of rotation.
///               * Lower left corner is -1.0f, -1.0.
///               * Upper right corner is  1.0f,  1.0.
///               * Visible z-space is from 0.0 to 1.0.
///                 * This is NOT the same as OpenGL! Be careful.
///               * NDC origin is at (0.0f, 0.0f, 0.5f).
struct Matrix {
  union {
    Scalar m[16];
    Scalar e[4][4];
    Vector4 vec[4];
  };

  //----------------------------------------------------------------------------
  /// Constructs a default identity matrix.
  ///
  constexpr Matrix()
      // clang-format off
      : vec{ Vector4(1.0f,  0.0f,  0.0f,  0.0f),
             Vector4(0.0f,  1.0f,  0.0f,  0.0f),
             Vector4(0.0f,  0.0f,  1.0f,  0.0f),
             Vector4(0.0f,  0.0f,  0.0f,  1.0f)} {}
  // clang-format on

  // clang-format off
  constexpr Matrix(Scalar m0,  Scalar m1,  Scalar m2,  Scalar m3,
                   Scalar m4,  Scalar m5,  Scalar m6,  Scalar m7,
                   Scalar m8,  Scalar m9,  Scalar m10, Scalar m11,
                   Scalar m12, Scalar m13, Scalar m14, Scalar m15)
      : vec{Vector4(m0,  m1,  m2,  m3),
            Vector4(m4,  m5,  m6,  m7),
            Vector4(m8,  m9,  m10, m11),
            Vector4(m12, m13, m14, m15)} {}
  // clang-format on

  explicit Matrix(const MatrixDecomposition& decomposition);

  // clang-format off
  static constexpr Matrix MakeColumn(
                   Scalar m0,  Scalar m1,  Scalar m2,  Scalar m3,
                   Scalar m4,  Scalar m5,  Scalar m6,  Scalar m7,
                   Scalar m8,  Scalar m9,  Scalar m10, Scalar m11,
                   Scalar m12, Scalar m13, Scalar m14, Scalar m15){
    return Matrix(m0,  m1,  m2,  m3,
                  m4,  m5,  m6,  m7,
                  m8,  m9,  m10, m11,
                  m12, m13, m14, m15);

  }
  // clang-format on

  // clang-format off
  static constexpr Matrix MakeRow(
                   Scalar m0,  Scalar m1,  Scalar m2,  Scalar m3,
                   Scalar m4,  Scalar m5,  Scalar m6,  Scalar m7,
                   Scalar m8,  Scalar m9,  Scalar m10, Scalar m11,
                   Scalar m12, Scalar m13, Scalar m14, Scalar m15){
    return Matrix(m0,  m4,  m8,   m12,
                  m1,  m5,  m9,   m13,
                  m2,  m6,  m10,  m14,
                  m3,  m7,  m11,  m15);
  }
  // clang-format on

  static constexpr Matrix MakeTranslation(const Vector3& t) {
    // clang-format off
    return Matrix(1.0f, 0.0f, 0.0f, 0.0f,
                  0.0f, 1.0f, 0.0f, 0.0f,
                  0.0f, 0.0f, 1.0f, 0.0f,
                  t.x, t.y, t.z, 1.0f);
    // clang-format on
  }

  static constexpr Matrix MakeScale(const Vector3& s) {
    // clang-format off
    return Matrix(s.x, 0.0f, 0.0f, 0.0f,
                  0.0f, s.y, 0.0f, 0.0f,
                  0.0f, 0.0f, s.z, 0.0f,
                  0.0f, 0.0f, 0.0f, 1.0f);
    // clang-format on
  }

  static constexpr Matrix MakeTranslateScale(const Vector3& s,
                                             const Vector3& t) {
    // clang-format off
    return Matrix(s.x, 0.0f, 0.0f, 0.0f,
                  0.0f, s.y, 0.0f, 0.0f,
                  0.0f, 0.0f, s.z, 0.0f,
                  t.x , t.y, t.z, 1.0f);
    // clang-format on
  }

  static constexpr Matrix MakeScale(const Vector2& s) {
    return MakeScale(Vector3(s.x, s.y, 1.0f));
  }

  static constexpr Matrix MakeSkew(Scalar sx, Scalar sy) {
    // clang-format off
    return Matrix(1.0f, sy , 0.0f, 0.0f,
                  sx , 1.0f, 0.0f, 0.0f,
                  0.0f, 0.0f, 1.0f, 0.0f,
                  0.0f, 0.0f, 0.0f, 1.0f);
    // clang-format on
  }

  static Matrix MakeRotation(Quaternion q) {
    // clang-format off
    return Matrix(
      1.0f - 2.0f * q.y * q.y  - 2.0f * q.z * q.z,
      2.0f * q.x  * q.y + 2.0f * q.z  * q.w,
      2.0f * q.x  * q.z - 2.0f * q.y  * q.w,
      0.0f,

      2.0f * q.x  * q.y - 2.0f * q.z  * q.w,
      1.0f - 2.0f * q.x * q.x  - 2.0f * q.z * q.z,
      2.0f * q.y  * q.z + 2.0f * q.x  * q.w,
      0.0f,

      2.0f * q.x  * q.z + 2.0f * q.y * q.w,
      2.0f * q.y  * q.z - 2.0f * q.x * q.w,
      1.0f - 2.0f * q.x * q.x  - 2.0f * q.y * q.y,
      0.0f,

      0.0f,
      0.0f,
      0.0f,
      1.0f);
    // clang-format on
  }

  static Matrix MakeRotation(Radians radians, const Vector4& r) {
    const Vector4 v = r.Normalize();

    const Vector2 cos_sin = CosSin(radians);
    const Scalar cosine = cos_sin.x;
    const Scalar cosp = 1.0f - cosine;
    const Scalar sine = cos_sin.y;

    // clang-format off
    return Matrix(
      cosine + cosp * v.x * v.x,
      cosp * v.x * v.y + v.z * sine,
      cosp * v.x * v.z - v.y * sine,
      0.0f,

      cosp * v.x * v.y - v.z * sine,
      cosine + cosp * v.y * v.y,
      cosp * v.y * v.z + v.x * sine,
      0.0f,

      cosp * v.x * v.z + v.y * sine,
      cosp * v.y * v.z - v.x * sine,
      cosine + cosp * v.z * v.z,
      0.0f,

      0.0f,
      0.0f,
      0.0f,
      1.0f);
    // clang-format on
  }

  static Matrix MakeRotationX(Radians r) {
    const Vector2 cos_sin = CosSin(r);
    const Scalar cosine = cos_sin.x;
    const Scalar sine = cos_sin.y;

    // clang-format off
    return Matrix(
      1.0f,  0.0f,    0.0f,    0.0f,
      0.0f,  cosine,  sine,    0.0f,
      0.0f, -sine,    cosine,  0.0f,
      0.0f,  0.0f,    0.0f,    1.0f
    );
    // clang-format on
  }

  static Matrix MakeRotationY(Radians r) {
    const Vector2 cos_sin = CosSin(r);
    const Scalar cosine = cos_sin.x;
    const Scalar sine = cos_sin.y;

    // clang-format off
    return Matrix(
      cosine,  0.0f, -sine,    0.0f,
      0.0f,    1.0f,  0.0f,    0.0f,
      sine,    0.0f,  cosine,  0.0f,
      0.0f,    0.0f,  0.0f,    1.0f
    );
    // clang-format on
  }

  static Matrix MakeRotationZ(Radians r) {
    const Vector2 cos_sin = CosSin(r);
    const Scalar cosine = cos_sin.x;
    const Scalar sine = cos_sin.y;

    // clang-format off
    return Matrix (
      cosine, sine,   0.0f, 0.0f,
      -sine,  cosine, 0.0f, 0.0f,
      0.0f,    0.0f,    1.0f, 0.0f,
      0.0f,    0.0f,    0.0f, 1.0
    );
    // clang-format on
  }

  /// The Matrix without its `w` components (without translation).
  constexpr Matrix Basis() const {
    // clang-format off
    return Matrix(
      m[0], m[1], m[2],  0.0f,
      m[4], m[5], m[6],  0.0f,
      m[8], m[9], m[10], 0.0f,
      0.0f,  0.0f,  0.0f,   1.0
    );
    // clang-format on
  }

  constexpr Matrix Translate(const Vector3& t) const {
    // clang-format off
    return Matrix(m[0], m[1], m[2], m[3],
                  m[4], m[5], m[6], m[7],
                  m[8], m[9], m[10], m[11],
                  m[0] * t.x + m[4] * t.y + m[8]  * t.z + m[12],
                  m[1] * t.x + m[5] * t.y + m[9]  * t.z + m[13],
                  m[2] * t.x + m[6] * t.y + m[10] * t.z + m[14],
                  m[3] * t.x + m[7] * t.y + m[11] * t.z + m[15]);
    // clang-format on
  }

  constexpr Matrix Scale(const Vector3& s) const {
    // clang-format off
    return Matrix(m[0] * s.x, m[1] * s.x, m[2]  * s.x, m[3]  * s.x,
                  m[4] * s.y, m[5] * s.y, m[6]  * s.y, m[7]  * s.y,
                  m[8] * s.z, m[9] * s.z, m[10] * s.z, m[11] * s.z,
                  m[12]     , m[13]     , m[14]      , m[15]       );
    // clang-format on
  }

  constexpr Matrix Multiply(const Matrix& o) const {
    // clang-format off
    return Matrix(
        m[0] * o.m[0]  + m[4] * o.m[1]  + m[8]  * o.m[2]  + m[12] * o.m[3],
        m[1] * o.m[0]  + m[5] * o.m[1]  + m[9]  * o.m[2]  + m[13] * o.m[3],
        m[2] * o.m[0]  + m[6] * o.m[1]  + m[10] * o.m[2]  + m[14] * o.m[3],
        m[3] * o.m[0]  + m[7] * o.m[1]  + m[11] * o.m[2]  + m[15] * o.m[3],
        m[0] * o.m[4]  + m[4] * o.m[5]  + m[8]  * o.m[6]  + m[12] * o.m[7],
        m[1] * o.m[4]  + m[5] * o.m[5]  + m[9]  * o.m[6]  + m[13] * o.m[7],
        m[2] * o.m[4]  + m[6] * o.m[5]  + m[10] * o.m[6]  + m[14] * o.m[7],
        m[3] * o.m[4]  + m[7] * o.m[5]  + m[11] * o.m[6]  + m[15] * o.m[7],
        m[0] * o.m[8]  + m[4] * o.m[9]  + m[8]  * o.m[10] + m[12] * o.m[11],
        m[1] * o.m[8]  + m[5] * o.m[9]  + m[9]  * o.m[10] + m[13] * o.m[11],
        m[2] * o.m[8]  + m[6] * o.m[9]  + m[10] * o.m[10] + m[14] * o.m[11],
        m[3] * o.m[8]  + m[7] * o.m[9]  + m[11] * o.m[10] + m[15] * o.m[11],
        m[0] * o.m[12] + m[4] * o.m[13] + m[8]  * o.m[14] + m[12] * o.m[15],
        m[1] * o.m[12] + m[5] * o.m[13] + m[9]  * o.m[14] + m[13] * o.m[15],
        m[2] * o.m[12] + m[6] * o.m[13] + m[10] * o.m[14] + m[14] * o.m[15],
        m[3] * o.m[12] + m[7] * o.m[13] + m[11] * o.m[14] + m[15] * o.m[15]);
    // clang-format on
  }

  constexpr Matrix Transpose() const {
    // clang-format off
    return {
        m[0], m[4], m[8],  m[12],
        m[1], m[5], m[9],  m[13],
        m[2], m[6], m[10], m[14],
        m[3], m[7], m[11], m[15],
    };
    // clang-format on
  }

  Matrix Invert() const;

  Scalar GetDeterminant() const;

  bool IsInvertible() const { return GetDeterminant() != 0; }

  constexpr Scalar GetMaxBasisLengthXY() const {
    // The full basis computation requires computing the squared scaling factor
    // for translate/scale only matrices. This substantially limits the range of
    // precision for small and large scales. Instead, check for the common cases
    // and directly return the max scaling factor.
    if (e[0][1] == 0 && e[1][0] == 0) {
      return std::max(std::abs(e[0][0]), std::abs(e[1][1]));
    }
    return std::sqrt(std::max(e[0][0] * e[0][0] + e[0][1] * e[0][1],
                              e[1][0] * e[1][0] + e[1][1] * e[1][1]));
  }

  constexpr Vector3 GetBasisX() const { return Vector3(m[0], m[1], m[2]); }

  constexpr Vector3 GetBasisY() const { return Vector3(m[4], m[5], m[6]); }

  constexpr Vector3 GetBasisZ() const { return Vector3(m[8], m[9], m[10]); }

  constexpr Vector3 GetScale() const {
    return Vector3(GetBasisX().GetLength(), GetBasisY().GetLength(),
                   GetBasisZ().GetLength());
  }

  constexpr Scalar GetDirectionScale(Vector3 direction) const {
    return 1.0f / (this->Basis().Invert() * direction.Normalize()).GetLength() *
           direction.GetLength();
  }

  constexpr bool IsFinite() const {
    return vec[0].IsFinite() && vec[1].IsFinite() && vec[2].IsFinite() &&
           vec[3].IsFinite();
  }

  constexpr bool IsAffine() const {
    return (m[2] == 0 && m[3] == 0 && m[6] == 0 && m[7] == 0 && m[8] == 0 &&
            m[9] == 0 && m[10] == 1 && m[11] == 0 && m[14] == 0 && m[15] == 1);
  }

  constexpr bool HasPerspective2D() const {
    return m[3] != 0 || m[7] != 0 || m[15] != 1;
  }

  constexpr bool HasPerspective() const {
    return m[3] != 0 || m[7] != 0 || m[11] != 0 || m[15] != 1;
  }

  constexpr bool HasTranslation() const { return m[12] != 0 || m[13] != 0; }

  constexpr bool IsAligned2D(Scalar tolerance = 0) const {
    if (HasPerspective2D()) {
      return false;
    }
    if (ScalarNearlyZero(m[1], tolerance) &&
        ScalarNearlyZero(m[4], tolerance)) {
      return true;
    }
    if (ScalarNearlyZero(m[0], tolerance) &&
        ScalarNearlyZero(m[5], tolerance)) {
      return true;
    }
    return false;
  }

  constexpr bool IsAligned(Scalar tolerance = 0) const {
    if (HasPerspective()) {
      return false;
    }
    int v[] = {!ScalarNearlyZero(m[0], tolerance),  //
               !ScalarNearlyZero(m[1], tolerance),  //
               !ScalarNearlyZero(m[2], tolerance),  //
               !ScalarNearlyZero(m[4], tolerance),  //
               !ScalarNearlyZero(m[5], tolerance),  //
               !ScalarNearlyZero(m[6], tolerance),  //
               !ScalarNearlyZero(m[8], tolerance),  //
               !ScalarNearlyZero(m[9], tolerance),  //
               !ScalarNearlyZero(m[10], tolerance)};
    // Check if all three basis vectors are aligned to an axis.
    if (v[0] + v[1] + v[2] != 1 ||  //
        v[3] + v[4] + v[5] != 1 ||  //
        v[6] + v[7] + v[8] != 1) {
      return false;
    }
    // Ensure that none of the basis vectors overlap.
    if (v[0] + v[3] + v[6] != 1 ||  //
        v[1] + v[4] + v[7] != 1 ||  //
        v[2] + v[5] + v[8] != 1) {
      return false;
    }
    return true;
  }

  constexpr bool IsIdentity() const {
    return (
        // clang-format off
        m[0]  == 1.0f && m[1]  == 0.0f && m[2]  == 0.0f && m[3]  == 0.0f &&
        m[4]  == 0.0f && m[5]  == 1.0f && m[6]  == 0.0f && m[7]  == 0.0f &&
        m[8]  == 0.0f && m[9]  == 0.0f && m[10] == 1.0f && m[11] == 0.0f &&
        m[12] == 0.0f && m[13] == 0.0f && m[14] == 0.0f && m[15] == 1.0f
        // clang-format on
    );
  }

  /// @brief  Returns true if the matrix has no entries other than translation
  ///         components. Note that an identity matrix meets this criteria.
  constexpr bool IsTranslationOnly() const {
    return (
        // clang-format off
        m[0] == 1.0 && m[1]  == 0.0 && m[2]  == 0.0 && m[3]  == 0.0 &&
        m[4] == 0.0 && m[5]  == 1.0 && m[6]  == 0.0 && m[7]  == 0.0 &&
        m[8] == 0.0 && m[9]  == 0.0 && m[10] == 1.0 && m[11] == 0.0 &&
                                                       m[15] == 1.0
        // clang-format on
    );
  }

  /// @brief  Returns true if the matrix has a scale-only basis and is
  ///         non-projective. Note that an identity matrix meets this criteria.
  constexpr bool IsTranslationScaleOnly() const {
    return (
        // clang-format off
        m[0] != 0.0 && m[1]  == 0.0 && m[2]  == 0.0 && m[3]  == 0.0 &&
        m[4] == 0.0 && m[5]  != 0.0 && m[6]  == 0.0 && m[7]  == 0.0 &&
        m[8] == 0.0 && m[9]  == 0.0 && m[10] != 0.0 && m[11] == 0.0 &&
                                                       m[15] == 1.0
        // clang-format on
    );
  }

  std::optional<MatrixDecomposition> Decompose() const;

  bool Equals(const Matrix& matrix, Scalar epsilon = 1e-5f) const {
    const Scalar* a = m;
    const Scalar* b = matrix.m;
    return ScalarNearlyEqual(a[0], b[0], epsilon) &&
           ScalarNearlyEqual(a[1], b[1], epsilon) &&
           ScalarNearlyEqual(a[2], b[2], epsilon) &&
           ScalarNearlyEqual(a[3], b[3], epsilon) &&
           ScalarNearlyEqual(a[4], b[4], epsilon) &&
           ScalarNearlyEqual(a[5], b[5], epsilon) &&
           ScalarNearlyEqual(a[6], b[6], epsilon) &&
           ScalarNearlyEqual(a[7], b[7], epsilon) &&
           ScalarNearlyEqual(a[8], b[8], epsilon) &&
           ScalarNearlyEqual(a[9], b[9], epsilon) &&
           ScalarNearlyEqual(a[10], b[10], epsilon) &&
           ScalarNearlyEqual(a[11], b[11], epsilon) &&
           ScalarNearlyEqual(a[12], b[12], epsilon) &&
           ScalarNearlyEqual(a[13], b[13], epsilon) &&
           ScalarNearlyEqual(a[14], b[14], epsilon) &&
           ScalarNearlyEqual(a[15], b[15], epsilon);
  }

  constexpr bool operator==(const Matrix& m) const {
    // clang-format off
    return vec[0] == m.vec[0]
        && vec[1] == m.vec[1]
        && vec[2] == m.vec[2]
        && vec[3] == m.vec[3];
    // clang-format on
  }

  constexpr bool operator!=(const Matrix& m) const {
    // clang-format off
    return vec[0] != m.vec[0]
        || vec[1] != m.vec[1]
        || vec[2] != m.vec[2]
        || vec[3] != m.vec[3];
    // clang-format on
  }

  Matrix operator+(const Vector3& t) const { return Translate(t); }

  Matrix operator-(const Vector3& t) const { return Translate(-t); }

  Matrix operator*(const Matrix& m) const { return Multiply(m); }

  Matrix operator+(const Matrix& m) const;

  constexpr Vector4 operator*(const Vector4& v) const {
    return Vector4(v.x * m[0] + v.y * m[4] + v.z * m[8] + v.w * m[12],
                   v.x * m[1] + v.y * m[5] + v.z * m[9] + v.w * m[13],
                   v.x * m[2] + v.y * m[6] + v.z * m[10] + v.w * m[14],
                   v.x * m[3] + v.y * m[7] + v.z * m[11] + v.w * m[15]);
  }

  constexpr Vector3 operator*(const Vector3& v) const {
    Scalar w = v.x * m[3] + v.y * m[7] + v.z * m[11] + m[15];
    Vector3 result(v.x * m[0] + v.y * m[4] + v.z * m[8] + m[12],
                   v.x * m[1] + v.y * m[5] + v.z * m[9] + m[13],
                   v.x * m[2] + v.y * m[6] + v.z * m[10] + m[14]);

    // This is Skia's behavior, but it may be reasonable to allow UB for the w=0
    // case.
    if (w) {
      w = 1 / w;
    }
    return result * w;
  }

  constexpr Point operator*(const Point& v) const {
    Scalar w = v.x * m[3] + v.y * m[7] + m[15];
    Point result(v.x * m[0] + v.y * m[4] + m[12],
                 v.x * m[1] + v.y * m[5] + m[13]);

    // This is Skia's behavior, but it may be reasonable to allow UB for the w=0
    // case.
    if (w) {
      w = 1 / w;
    }
    return result * w;
  }

  constexpr Vector3 TransformHomogenous(const Point& v) const {
    return Vector3(v.x * m[0] + v.y * m[4] + m[12],
                   v.x * m[1] + v.y * m[5] + m[13],
                   v.x * m[3] + v.y * m[7] + m[15]);
  }

  constexpr Vector4 TransformDirection(const Vector4& v) const {
    return Vector4(v.x * m[0] + v.y * m[4] + v.z * m[8],
                   v.x * m[1] + v.y * m[5] + v.z * m[9],
                   v.x * m[2] + v.y * m[6] + v.z * m[10], v.w);
  }

  constexpr Vector3 TransformDirection(const Vector3& v) const {
    return Vector3(v.x * m[0] + v.y * m[4] + v.z * m[8],
                   v.x * m[1] + v.y * m[5] + v.z * m[9],
                   v.x * m[2] + v.y * m[6] + v.z * m[10]);
  }

  constexpr Vector2 TransformDirection(const Vector2& v) const {
    return Vector2(v.x * m[0] + v.y * m[4], v.x * m[1] + v.y * m[5]);
  }

  constexpr Quad Transform(const Quad& quad) const {
    return {
        *this * quad[0],
        *this * quad[1],
        *this * quad[2],
        *this * quad[3],
    };
  }

  template <class T>
  static constexpr Matrix MakeOrthographic(TSize<T> size) {
    // Per assumptions about NDC documented above.
    const auto scale =
        MakeScale({2.0f / static_cast<Scalar>(size.width),
                   -2.0f / static_cast<Scalar>(size.height), 0.0f});
    const auto translate = MakeTranslation({-1.0f, 1.0f, 0.5f});
    return translate * scale;
  }

  static constexpr Matrix MakePerspective(Radians fov_y,
                                          Scalar aspect_ratio,
                                          Scalar z_near,
                                          Scalar z_far) {
    Scalar height = std::tan(fov_y.radians * 0.5f);
    Scalar width = height * aspect_ratio;

    // clang-format off
    return {
      1.0f / width, 0.0f,           0.0f,                                 0.0f,
      0.0f,         1.0f / height,  0.0f,                                 0.0f,
      0.0f,         0.0f,           z_far / (z_far - z_near),             1.0f,
      0.0f,         0.0f,          -(z_far * z_near) / (z_far - z_near),  0.0f,
    };
    // clang-format on
  }

  template <class T>
  static constexpr Matrix MakePerspective(Radians fov_y,
                                          TSize<T> size,
                                          Scalar z_near,
                                          Scalar z_far) {
    return MakePerspective(fov_y, static_cast<Scalar>(size.width) / size.height,
                           z_near, z_far);
  }

  static constexpr Matrix MakeLookAt(Vector3 position,
                                     Vector3 target,
                                     Vector3 up) {
    Vector3 forward = (target - position).Normalize();
    Vector3 right = up.Cross(forward);
    up = forward.Cross(right);

    // clang-format off
    return {
       right.x,              up.x,              forward.x,             0.0f,
       right.y,              up.y,              forward.y,             0.0f,
       right.z,              up.z,              forward.z,             0.0f,
      -right.Dot(position), -up.Dot(position), -forward.Dot(position), 1.0f
    };
    // clang-format on
  }

 private:
  static constexpr Vector2 CosSin(Radians radians) {
    // The precision of a float around 1.0 is much lower than it is
    // around 0.0, so we end up with cases on quadrant rotations where
    // we get a +/-1.0 for one of the values and a non-zero value for
    // the other. This happens around quadrant rotations which makes it
    // especially common and results in unclean quadrant rotation
    // matrices which do not return true from |IsAligned[2D]| even
    // though that is exactly where you need them to exhibit that property.
    // It also injects small floating point mantissa errors into the
    // matrices whenever you concatenate them with a quadrant rotation.
    //
    // This issue is also exacerbated by the fact that, in radians, the
    // angles for quadrant rotations are irrational numbers. The measuring
    // error for representing 90 degree multiples is small enough that
    // either sin or cos will return a value near +/-1.0, but not small
    // enough that the other value will be a clean 0.0.
    //
    // Some geometry packages simply discard very small numbers from
    // sin/cos, but the following approach specifically targets just the
    // area around a quadrant rotation (where either the sin or cos are
    // measuring as +/-1.0) for symmetry of precision.

    Scalar sin = std::sin(radians.radians);
    if (std::abs(sin) == 1.0f) {
      // 90 or 270 degrees (mod 360)
      return {0.0f, sin};
    } else {
      Scalar cos = std::cos(radians.radians);
      if (std::abs(cos) == 1.0f) {
        // 0 or 180 degrees (mod 360)
        return {cos, 0.0f};
      }
      return {cos, sin};
    }
  }
};

static_assert(sizeof(struct Matrix) == sizeof(Scalar) * 16,
              "The matrix must be of consistent size.");

}  // namespace impeller

namespace std {
inline std::ostream& operator<<(std::ostream& out, const impeller::Matrix& m) {
  out << "(" << std::endl << std::fixed;
  for (size_t i = 0; i < 4u; i++) {
    for (size_t j = 0; j < 4u; j++) {
      out << std::setw(15) << m.e[j][i] << ",";
    }
    out << std::endl;
  }
  out << ")";
  return out;
}

}  // namespace std

#endif  // FLUTTER_IMPELLER_GEOMETRY_MATRIX_H_
