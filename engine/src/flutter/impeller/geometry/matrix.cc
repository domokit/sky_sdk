// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/geometry/matrix.h"

#include <climits>
#include <sstream>

namespace impeller {

Matrix::Matrix(const MatrixDecomposition& d) : Matrix() {
  /*
   *  Apply perspective.
   */
  for (int i = 0; i < 4; i++) {
    e[i][3] = d.perspective.e[i];
  }

  /*
   *  Apply translation.
   */
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      e[3][i] += d.translation.e[j] * e[j][i];
    }
  }

  /*
   *  Apply rotation.
   */

  Matrix rotation;

  const auto x = -d.rotation.x;
  const auto y = -d.rotation.y;
  const auto z = -d.rotation.z;
  const auto w = d.rotation.w;

  /*
   *  Construct a composite rotation matrix from the quaternion values.
   */

  rotation.e[0][0] = 1.0 - 2.0 * (y * y + z * z);
  rotation.e[0][1] = 2.0 * (x * y - z * w);
  rotation.e[0][2] = 2.0 * (x * z + y * w);
  rotation.e[1][0] = 2.0 * (x * y + z * w);
  rotation.e[1][1] = 1.0 - 2.0 * (x * x + z * z);
  rotation.e[1][2] = 2.0 * (y * z - x * w);
  rotation.e[2][0] = 2.0 * (x * z - y * w);
  rotation.e[2][1] = 2.0 * (y * z + x * w);
  rotation.e[2][2] = 1.0 - 2.0 * (x * x + y * y);

  *this = *this * rotation;

  /*
   *  Apply shear.
   */
  Matrix shear;

  if (d.shear.e[2] != 0) {
    shear.e[2][1] = d.shear.e[2];
    *this = *this * shear;
  }

  if (d.shear.e[1] != 0) {
    shear.e[2][1] = 0.0;
    shear.e[2][0] = d.shear.e[1];
    *this = *this * shear;
  }

  if (d.shear.e[0] != 0) {
    shear.e[2][0] = 0.0;
    shear.e[1][0] = d.shear.e[0];
    *this = *this * shear;
  }

  /*
   *  Apply scale.
   */
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      e[i][j] *= d.scale.e[i];
    }
  }
}

Matrix Matrix::operator+(const Matrix& o) const {
  return Matrix(
      m[0] + o.m[0], m[1] + o.m[1], m[2] + o.m[2], m[3] + o.m[3],         //
      m[4] + o.m[4], m[5] + o.m[5], m[6] + o.m[6], m[7] + o.m[7],         //
      m[8] + o.m[8], m[9] + o.m[9], m[10] + o.m[10], m[11] + o.m[11],     //
      m[12] + o.m[12], m[13] + o.m[13], m[14] + o.m[14], m[15] + o.m[15]  //
  );
}

Matrix Matrix::Invert() const {
  Matrix tmp{
      m[5] * m[10] * m[15] - m[5] * m[11] * m[14] - m[9] * m[6] * m[15] +
          m[9] * m[7] * m[14] + m[13] * m[6] * m[11] - m[13] * m[7] * m[10],

      -m[1] * m[10] * m[15] + m[1] * m[11] * m[14] + m[9] * m[2] * m[15] -
          m[9] * m[3] * m[14] - m[13] * m[2] * m[11] + m[13] * m[3] * m[10],

      m[1] * m[6] * m[15] - m[1] * m[7] * m[14] - m[5] * m[2] * m[15] +
          m[5] * m[3] * m[14] + m[13] * m[2] * m[7] - m[13] * m[3] * m[6],

      -m[1] * m[6] * m[11] + m[1] * m[7] * m[10] + m[5] * m[2] * m[11] -
          m[5] * m[3] * m[10] - m[9] * m[2] * m[7] + m[9] * m[3] * m[6],

      -m[4] * m[10] * m[15] + m[4] * m[11] * m[14] + m[8] * m[6] * m[15] -
          m[8] * m[7] * m[14] - m[12] * m[6] * m[11] + m[12] * m[7] * m[10],

      m[0] * m[10] * m[15] - m[0] * m[11] * m[14] - m[8] * m[2] * m[15] +
          m[8] * m[3] * m[14] + m[12] * m[2] * m[11] - m[12] * m[3] * m[10],

      -m[0] * m[6] * m[15] + m[0] * m[7] * m[14] + m[4] * m[2] * m[15] -
          m[4] * m[3] * m[14] - m[12] * m[2] * m[7] + m[12] * m[3] * m[6],

      m[0] * m[6] * m[11] - m[0] * m[7] * m[10] - m[4] * m[2] * m[11] +
          m[4] * m[3] * m[10] + m[8] * m[2] * m[7] - m[8] * m[3] * m[6],

      m[4] * m[9] * m[15] - m[4] * m[11] * m[13] - m[8] * m[5] * m[15] +
          m[8] * m[7] * m[13] + m[12] * m[5] * m[11] - m[12] * m[7] * m[9],

      -m[0] * m[9] * m[15] + m[0] * m[11] * m[13] + m[8] * m[1] * m[15] -
          m[8] * m[3] * m[13] - m[12] * m[1] * m[11] + m[12] * m[3] * m[9],

      m[0] * m[5] * m[15] - m[0] * m[7] * m[13] - m[4] * m[1] * m[15] +
          m[4] * m[3] * m[13] + m[12] * m[1] * m[7] - m[12] * m[3] * m[5],

      -m[0] * m[5] * m[11] + m[0] * m[7] * m[9] + m[4] * m[1] * m[11] -
          m[4] * m[3] * m[9] - m[8] * m[1] * m[7] + m[8] * m[3] * m[5],

      -m[4] * m[9] * m[14] + m[4] * m[10] * m[13] + m[8] * m[5] * m[14] -
          m[8] * m[6] * m[13] - m[12] * m[5] * m[10] + m[12] * m[6] * m[9],

      m[0] * m[9] * m[14] - m[0] * m[10] * m[13] - m[8] * m[1] * m[14] +
          m[8] * m[2] * m[13] + m[12] * m[1] * m[10] - m[12] * m[2] * m[9],

      -m[0] * m[5] * m[14] + m[0] * m[6] * m[13] + m[4] * m[1] * m[14] -
          m[4] * m[2] * m[13] - m[12] * m[1] * m[6] + m[12] * m[2] * m[5],

      m[0] * m[5] * m[10] - m[0] * m[6] * m[9] - m[4] * m[1] * m[10] +
          m[4] * m[2] * m[9] + m[8] * m[1] * m[6] - m[8] * m[2] * m[5]};

  Scalar det =
      m[0] * tmp.m[0] + m[1] * tmp.m[4] + m[2] * tmp.m[8] + m[3] * tmp.m[12];

  if (det == 0) {
    return {};
  }

  det = 1.0 / det;

  return {tmp.m[0] * det,  tmp.m[1] * det,  tmp.m[2] * det,  tmp.m[3] * det,
          tmp.m[4] * det,  tmp.m[5] * det,  tmp.m[6] * det,  tmp.m[7] * det,
          tmp.m[8] * det,  tmp.m[9] * det,  tmp.m[10] * det, tmp.m[11] * det,
          tmp.m[12] * det, tmp.m[13] * det, tmp.m[14] * det, tmp.m[15] * det};
}

Scalar Matrix::GetDeterminant() const {
  auto a00 = e[0][0];
  auto a01 = e[0][1];
  auto a02 = e[0][2];
  auto a03 = e[0][3];
  auto a10 = e[1][0];
  auto a11 = e[1][1];
  auto a12 = e[1][2];
  auto a13 = e[1][3];
  auto a20 = e[2][0];
  auto a21 = e[2][1];
  auto a22 = e[2][2];
  auto a23 = e[2][3];
  auto a30 = e[3][0];
  auto a31 = e[3][1];
  auto a32 = e[3][2];
  auto a33 = e[3][3];

  auto b00 = a00 * a11 - a01 * a10;
  auto b01 = a00 * a12 - a02 * a10;
  auto b02 = a00 * a13 - a03 * a10;
  auto b03 = a01 * a12 - a02 * a11;
  auto b04 = a01 * a13 - a03 * a11;
  auto b05 = a02 * a13 - a03 * a12;
  auto b06 = a20 * a31 - a21 * a30;
  auto b07 = a20 * a32 - a22 * a30;
  auto b08 = a20 * a33 - a23 * a30;
  auto b09 = a21 * a32 - a22 * a31;
  auto b10 = a21 * a33 - a23 * a31;
  auto b11 = a22 * a33 - a23 * a32;

  return b00 * b11 - b01 * b10 + b02 * b09 + b03 * b08 - b04 * b07 + b05 * b06;
}

/*
 *  Adapted for Impeller from Graphics Gems:
 *  http://www.realtimerendering.com/resources/GraphicsGems/gemsii/unmatrix.c
 */
std::optional<MatrixDecomposition> Matrix::Decompose() const {
  /*
   *  Normalize the matrix.
   */
  Matrix self = *this;

  if (self.e[3][3] == 0) {
    return std::nullopt;
  }

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      self.e[i][j] /= self.e[3][3];
    }
  }

  /*
   *  `perspectiveMatrix` is used to solve for perspective, but it also provides
   *  an easy way to test for singularity of the upper 3x3 component.
   */
  Matrix perpectiveMatrix = self;
  for (int i = 0; i < 3; i++) {
    perpectiveMatrix.e[i][3] = 0;
  }

  perpectiveMatrix.e[3][3] = 1;

  if (!perpectiveMatrix.IsInvertible()) {
    return std::nullopt;
  }

  MatrixDecomposition result;

  /*
   *  ==========================================================================
   *  First, isolate perspective.
   *  ==========================================================================
   */
  if (self.e[0][3] != 0.0 || self.e[1][3] != 0.0 || self.e[2][3] != 0.0) {
    /*
     *  prhs is the right hand side of the equation.
     */
    const Vector4 rightHandSide(self.e[0][3],  //
                                self.e[1][3],  //
                                self.e[2][3],  //
                                self.e[3][3]);

    /*
     *  Solve the equation by inverting `perspectiveMatrix` and multiplying
     *  prhs by the inverse.
     */

    result.perspective = perpectiveMatrix.Invert().Transpose() * rightHandSide;

    /*
     *  Clear the perspective partition.
     */
    self.e[0][3] = self.e[1][3] = self.e[2][3] = 0;
    self.e[3][3] = 1;
  }

  /*
   *  ==========================================================================
   *  Next, the translation.
   *  ==========================================================================
   */
  result.translation = {self.e[3][0], self.e[3][1], self.e[3][2]};
  self.e[3][0] = self.e[3][1] = self.e[3][2] = 0.0;

  /*
   *  ==========================================================================
   *  Next, the scale and shear.
   *  ==========================================================================
   */
  Vector3 row[3];
  for (int i = 0; i < 3; i++) {
    row[i].x = self.e[i][0];
    row[i].y = self.e[i][1];
    row[i].z = self.e[i][2];
  }

  /*
   *  Compute X scale factor and normalize first row.
   */
  result.scale.x = row[0].GetLength();
  row[0] = row[0].Normalize();

  /*
   *  Compute XY shear factor and make 2nd row orthogonal to 1st.
   */
  result.shear.xy = row[0].Dot(row[1]);
  row[1] = Vector3::Combine(row[1], 1.0, row[0], -result.shear.xy);

  /*
   *  Compute Y scale and normalize 2nd row.
   */
  result.scale.y = row[1].GetLength();
  row[1] = row[1].Normalize();
  result.shear.xy /= result.scale.y;

  /*
   *  Compute XZ and YZ shears, orthogonalize 3rd row.
   */
  result.shear.xz = row[0].Dot(row[2]);
  row[2] = Vector3::Combine(row[2], 1.0, row[0], -result.shear.xz);
  result.shear.yz = row[1].Dot(row[2]);
  row[2] = Vector3::Combine(row[2], 1.0, row[1], -result.shear.yz);

  /*
   *  Next, get Z scale and normalize 3rd row.
   */
  result.scale.z = row[2].GetLength();
  row[2] = row[2].Normalize();

  result.shear.xz /= result.scale.z;
  result.shear.yz /= result.scale.z;

  /*
   *  At this point, the matrix (in rows[]) is orthonormal.
   *  Check for a coordinate system flip.  If the determinant
   *  is -1, then negate the matrix and the scaling factors.
   */
  if (row[0].Dot(row[1].Cross(row[2])) < 0) {
    result.scale.x *= -1;
    result.scale.y *= -1;
    result.scale.z *= -1;

    for (int i = 0; i < 3; i++) {
      row[i].x *= -1;
      row[i].y *= -1;
      row[i].z *= -1;
    }
  }

  /*
   *  ==========================================================================
   *  Finally, get the rotations out.
   *  ==========================================================================
   */
  result.rotation.x =
      0.5 * sqrt(fmax(1.0 + row[0].x - row[1].y - row[2].z, 0.0));
  result.rotation.y =
      0.5 * sqrt(fmax(1.0 - row[0].x + row[1].y - row[2].z, 0.0));
  result.rotation.z =
      0.5 * sqrt(fmax(1.0 - row[0].x - row[1].y + row[2].z, 0.0));
  result.rotation.w =
      0.5 * sqrt(fmax(1.0 + row[0].x + row[1].y + row[2].z, 0.0));

  if (row[2].y > row[1].z) {
    result.rotation.x = -result.rotation.x;
  }
  if (row[0].z > row[2].x) {
    result.rotation.y = -result.rotation.y;
  }
  if (row[1].x > row[0].y) {
    result.rotation.z = -result.rotation.z;
  }

  return result;
}

uint64_t MatrixDecomposition::GetComponentsMask() const {
  uint64_t mask = 0;

  Quaternion noRotation(0.0, 0.0, 0.0, 1.0);
  if (rotation != noRotation) {
    mask = mask | static_cast<uint64_t>(Component::kRotation);
  }

  Vector4 defaultPerspective(0.0, 0.0, 0.0, 1.0);
  if (perspective != defaultPerspective) {
    mask = mask | static_cast<uint64_t>(Component::kPerspective);
  }

  Shear noShear(0.0, 0.0, 0.0);
  if (shear != noShear) {
    mask = mask | static_cast<uint64_t>(Component::kShear);
  }

  Vector3 defaultScale(1.0, 1.0, 1.0);
  if (scale != defaultScale) {
    mask = mask | static_cast<uint64_t>(Component::kScale);
  }

  Vector3 defaultTranslation(0.0, 0.0, 0.0);
  if (translation != defaultTranslation) {
    mask = mask | static_cast<uint64_t>(Component::kTranslation);
  }

  return mask;
}

}  // namespace impeller
