// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;
import 'dart:typed_data';

import 'package:ui/ui.dart' as ui;

import 'util.dart';

class Matrix4 {
  /// Constructs a new mat4.
  factory Matrix4(
    double arg0,
    double arg1,
    double arg2,
    double arg3,
    double arg4,
    double arg5,
    double arg6,
    double arg7,
    double arg8,
    double arg9,
    double arg10,
    double arg11,
    double arg12,
    double arg13,
    double arg14,
    double arg15,
  ) =>
      Matrix4.zero()..setValues(
        arg0,
        arg1,
        arg2,
        arg3,
        arg4,
        arg5,
        arg6,
        arg7,
        arg8,
        arg9,
        arg10,
        arg11,
        arg12,
        arg13,
        arg14,
        arg15,
      );

  /// Zero matrix.
  Matrix4.zero() : _m4storage = Float32List(16);

  /// Identity matrix.
  Matrix4.identity() : _m4storage = Float32List(16) {
    _m4storage[15] = 1.0;
    _m4storage[0] = 1.0;
    _m4storage[5] = 1.0;
    _m4storage[10] = 1.0;
  }

  /// Copies values from [other].
  factory Matrix4.copy(Matrix4 other) => Matrix4.zero()..setFrom(other);

  /// Constructs a matrix that is the inverse of [other].
  factory Matrix4.inverted(Matrix4 other) {
    final Matrix4 r = Matrix4.zero();
    final double determinant = r.copyInverse(other);
    if (determinant == 0.0) {
      throw ArgumentError.value(other, 'other', 'Matrix cannot be inverted');
    }
    return r;
  }

  /// Rotation of [radians_] around X.
  factory Matrix4.rotationX(double radians) =>
      Matrix4.zero()
        .._m4storage[15] = 1.0
        ..setRotationX(radians);

  /// Rotation of [radians_] around Y.
  factory Matrix4.rotationY(double radians) =>
      Matrix4.zero()
        .._m4storage[15] = 1.0
        ..setRotationY(radians);

  /// Rotation of [radians_] around Z.
  factory Matrix4.rotationZ(double radians) =>
      Matrix4.zero()
        .._m4storage[15] = 1.0
        ..setRotationZ(radians);

  /// Translation matrix.
  factory Matrix4.translation(Vector3 translation) =>
      Matrix4.identity()..setTranslation(translation);

  /// Translation matrix.
  factory Matrix4.translationValues(double x, double y, double z) =>
      Matrix4.identity()..setTranslationRaw(x, y, z);

  /// Scale matrix.
  factory Matrix4.diagonal3Values(double x, double y, double z) =>
      Matrix4.zero()
        .._m4storage[15] = 1.0
        .._m4storage[10] = z
        .._m4storage[5] = y
        .._m4storage[0] = x;

  /// Constructs Matrix4 with given [Float32List] as [storage].
  Matrix4.fromFloat32List(this._m4storage);

  /// Constructs Matrix4 with a [storage] that views given [buffer] starting at
  /// [offset]. [offset] has to be multiple of [Float32List.bytesPerElement].
  Matrix4.fromBuffer(ByteBuffer buffer, int offset)
    : _m4storage = Float32List.view(buffer, offset, 16);

  final Float32List _m4storage;

  /// The components of the matrix.
  Float32List get storage => _m4storage;

  /// Returns a matrix that is the inverse of [other] if [other] is invertible,
  /// otherwise `null`.
  static Matrix4? tryInvert(Matrix4 other) {
    final Matrix4 r = Matrix4.zero();
    final double determinant = r.copyInverse(other);
    if (determinant == 0.0) {
      return null;
    }
    return r;
  }

  /// Return index in storage for [row], [col] value.
  int index(int row, int col) => (col * 4) + row;

  /// Value at [row], [col].
  double entry(int row, int col) {
    assert((row >= 0) && (row < dimension));
    assert((col >= 0) && (col < dimension));

    return _m4storage[index(row, col)];
  }

  /// Set value at [row], [col] to be [v].
  void setEntry(int row, int col, double v) {
    assert((row >= 0) && (row < dimension));
    assert((col >= 0) && (col < dimension));

    _m4storage[index(row, col)] = v;
  }

  /// Sets the matrix with specified values.
  void setValues(
    double arg0,
    double arg1,
    double arg2,
    double arg3,
    double arg4,
    double arg5,
    double arg6,
    double arg7,
    double arg8,
    double arg9,
    double arg10,
    double arg11,
    double arg12,
    double arg13,
    double arg14,
    double arg15,
  ) {
    _m4storage[15] = arg15;
    _m4storage[14] = arg14;
    _m4storage[13] = arg13;
    _m4storage[12] = arg12;
    _m4storage[11] = arg11;
    _m4storage[10] = arg10;
    _m4storage[9] = arg9;
    _m4storage[8] = arg8;
    _m4storage[7] = arg7;
    _m4storage[6] = arg6;
    _m4storage[5] = arg5;
    _m4storage[4] = arg4;
    _m4storage[3] = arg3;
    _m4storage[2] = arg2;
    _m4storage[1] = arg1;
    _m4storage[0] = arg0;
  }

  /// Sets the entire matrix to the matrix in [arg].
  void setFrom(Matrix4 arg) {
    final Float32List argStorage = arg._m4storage;
    _m4storage[15] = argStorage[15];
    _m4storage[14] = argStorage[14];
    _m4storage[13] = argStorage[13];
    _m4storage[12] = argStorage[12];
    _m4storage[11] = argStorage[11];
    _m4storage[10] = argStorage[10];
    _m4storage[9] = argStorage[9];
    _m4storage[8] = argStorage[8];
    _m4storage[7] = argStorage[7];
    _m4storage[6] = argStorage[6];
    _m4storage[5] = argStorage[5];
    _m4storage[4] = argStorage[4];
    _m4storage[3] = argStorage[3];
    _m4storage[2] = argStorage[2];
    _m4storage[1] = argStorage[1];
    _m4storage[0] = argStorage[0];
  }

  /// Dimension of the matrix.
  int get dimension => 4;

  /// Access the element of the matrix at the index [i].
  double operator [](int i) => _m4storage[i];

  /// Set the element of the matrix at the index [i].
  void operator []=(int i, double v) {
    _m4storage[i] = v;
  }

  /// Clone matrix.
  Matrix4 clone() => Matrix4.copy(this);

  /// Copy into [arg].
  Matrix4 copyInto(Matrix4 arg) {
    final Float32List argStorage = arg._m4storage;
    // Start reading from the last element to eliminate range checks
    // in subsequent reads.
    argStorage[15] = _m4storage[15];
    argStorage[0] = _m4storage[0];
    argStorage[1] = _m4storage[1];
    argStorage[2] = _m4storage[2];
    argStorage[3] = _m4storage[3];
    argStorage[4] = _m4storage[4];
    argStorage[5] = _m4storage[5];
    argStorage[6] = _m4storage[6];
    argStorage[7] = _m4storage[7];
    argStorage[8] = _m4storage[8];
    argStorage[9] = _m4storage[9];
    argStorage[10] = _m4storage[10];
    argStorage[11] = _m4storage[11];
    argStorage[12] = _m4storage[12];
    argStorage[13] = _m4storage[13];
    argStorage[14] = _m4storage[14];
    return arg;
  }

  /// Translate this matrix by x, y, and z.
  void translate(double x, [double y = 0.0, double z = 0.0]) {
    const double tw = 1.0;
    final double t1 =
        _m4storage[0] * x + _m4storage[4] * y + _m4storage[8] * z + _m4storage[12] * tw;
    final double t2 =
        _m4storage[1] * x + _m4storage[5] * y + _m4storage[9] * z + _m4storage[13] * tw;
    final double t3 =
        _m4storage[2] * x + _m4storage[6] * y + _m4storage[10] * z + _m4storage[14] * tw;
    final double t4 =
        _m4storage[3] * x + _m4storage[7] * y + _m4storage[11] * z + _m4storage[15] * tw;
    _m4storage[12] = t1;
    _m4storage[13] = t2;
    _m4storage[14] = t3;
    _m4storage[15] = t4;
  }

  /// Scale this matrix by a [Vector3], [Vector4], or x,y,z
  void scale(double x, [double? y, double? z]) {
    final double sx = x;
    final double sy = y ?? x;
    final double sz = z ?? x;
    const double sw = 1.0;
    _m4storage[15] *= sw;
    _m4storage[0] *= sx;
    _m4storage[1] *= sx;
    _m4storage[2] *= sx;
    _m4storage[3] *= sx;
    _m4storage[4] *= sy;
    _m4storage[5] *= sy;
    _m4storage[6] *= sy;
    _m4storage[7] *= sy;
    _m4storage[8] *= sz;
    _m4storage[9] *= sz;
    _m4storage[10] *= sz;
    _m4storage[11] *= sz;
    _m4storage[12] *= sw;
    _m4storage[13] *= sw;
    _m4storage[14] *= sw;
  }

  /// Create a copy of [this] scaled by a [Vector3], [Vector4] or [x],[y], and
  /// [z].
  Matrix4 scaled(double x, [double? y, double? z]) => clone()..scale(x, y, z);

  /// Zeros [this].
  void setZero() {
    _m4storage[15] = 0.0;
    _m4storage[0] = 0.0;
    _m4storage[1] = 0.0;
    _m4storage[2] = 0.0;
    _m4storage[3] = 0.0;
    _m4storage[4] = 0.0;
    _m4storage[5] = 0.0;
    _m4storage[6] = 0.0;
    _m4storage[7] = 0.0;
    _m4storage[8] = 0.0;
    _m4storage[9] = 0.0;
    _m4storage[10] = 0.0;
    _m4storage[11] = 0.0;
    _m4storage[12] = 0.0;
    _m4storage[13] = 0.0;
    _m4storage[14] = 0.0;
  }

  /// Makes [this] into the identity matrix.
  void setIdentity() {
    _m4storage[15] = 1.0;
    _m4storage[0] = 1.0;
    _m4storage[1] = 0.0;
    _m4storage[2] = 0.0;
    _m4storage[3] = 0.0;
    _m4storage[4] = 0.0;
    _m4storage[5] = 1.0;
    _m4storage[6] = 0.0;
    _m4storage[7] = 0.0;
    _m4storage[8] = 0.0;
    _m4storage[9] = 0.0;
    _m4storage[10] = 1.0;
    _m4storage[11] = 0.0;
    _m4storage[12] = 0.0;
    _m4storage[13] = 0.0;
    _m4storage[14] = 0.0;
  }

  /// Returns the tranpose of this.
  Matrix4 transposed() => clone()..transpose();

  void transpose() {
    double temp;
    temp = _m4storage[4];
    _m4storage[4] = _m4storage[1];
    _m4storage[1] = temp;
    temp = _m4storage[8];
    _m4storage[8] = _m4storage[2];
    _m4storage[2] = temp;
    temp = _m4storage[12];
    _m4storage[12] = _m4storage[3];
    _m4storage[3] = temp;
    temp = _m4storage[9];
    _m4storage[9] = _m4storage[6];
    _m4storage[6] = temp;
    temp = _m4storage[13];
    _m4storage[13] = _m4storage[7];
    _m4storage[7] = temp;
    temp = _m4storage[14];
    _m4storage[14] = _m4storage[11];
    _m4storage[11] = temp;
  }

  /// Returns the determinant of this matrix.
  double determinant() {
    final Float32List m = _m4storage;
    final double det2_01_01 = m[0] * m[5] - m[1] * m[4];
    final double det2_01_02 = m[0] * m[6] - m[2] * m[4];
    final double det2_01_03 = m[0] * m[7] - m[3] * m[4];
    final double det2_01_12 = m[1] * m[6] - m[2] * m[5];
    final double det2_01_13 = m[1] * m[7] - m[3] * m[5];
    final double det2_01_23 = m[2] * m[7] - m[3] * m[6];
    final double det3_201_012 = m[8] * det2_01_12 - m[9] * det2_01_02 + m[10] * det2_01_01;
    final double det3_201_013 = m[8] * det2_01_13 - m[9] * det2_01_03 + m[11] * det2_01_01;
    final double det3_201_023 = m[8] * det2_01_23 - m[10] * det2_01_03 + m[11] * det2_01_02;
    final double det3_201_123 = m[9] * det2_01_23 - m[10] * det2_01_13 + m[11] * det2_01_12;
    return -det3_201_123 * m[12] +
        det3_201_023 * m[13] -
        det3_201_013 * m[14] +
        det3_201_012 * m[15];
  }

  /// Transform [arg] of type [Vector3] using the perspective transformation
  /// defined by [this].
  Vector3 perspectiveTransform({required double x, required double y, required double z}) {
    final double transformedX =
        (_m4storage[0] * x) + (_m4storage[4] * y) + (_m4storage[8] * z) + _m4storage[12];
    final double transformedY =
        (_m4storage[1] * x) + (_m4storage[5] * y) + (_m4storage[9] * z) + _m4storage[13];
    final double transformedZ =
        (_m4storage[2] * x) + (_m4storage[6] * y) + (_m4storage[10] * z) + _m4storage[14];
    final double w =
        1.0 / ((_m4storage[3] * x) + (_m4storage[7] * y) + (_m4storage[11] * z) + _m4storage[15]);

    return (x: transformedX * w, y: transformedY * w, z: transformedZ * w);
  }

  bool isIdentity() =>
      _m4storage[0] == 1.0 && // col 1
      _m4storage[1] == 0.0 &&
      _m4storage[2] == 0.0 &&
      _m4storage[3] == 0.0 &&
      _m4storage[4] == 0.0 && // col 2
      _m4storage[5] == 1.0 &&
      _m4storage[6] == 0.0 &&
      _m4storage[7] == 0.0 &&
      _m4storage[8] == 0.0 && // col 3
      _m4storage[9] == 0.0 &&
      _m4storage[10] == 1.0 &&
      _m4storage[11] == 0.0 &&
      _m4storage[12] == 0.0 && // col 4
      _m4storage[13] == 0.0 &&
      _m4storage[14] == 0.0 &&
      _m4storage[15] == 1.0;

  /// Whether transform is identity or simple translation using m[12,13,14].
  ///
  /// We check for [15] first since that will eliminate bounds checks for rest.
  bool isIdentityOrTranslation() =>
      _m4storage[15] == 1.0 &&
      _m4storage[0] == 1.0 && // col 1
      _m4storage[1] == 0.0 &&
      _m4storage[2] == 0.0 &&
      _m4storage[3] == 0.0 &&
      _m4storage[4] == 0.0 && // col 2
      _m4storage[5] == 1.0 &&
      _m4storage[6] == 0.0 &&
      _m4storage[7] == 0.0 &&
      _m4storage[8] == 0.0 && // col 3
      _m4storage[9] == 0.0 &&
      _m4storage[10] == 1.0 &&
      _m4storage[11] == 0.0;

  /// Returns the translation vector from this homogeneous transformation matrix.
  Vector3 getTranslation() {
    return (x: _m4storage[12], y: _m4storage[13], z: _m4storage[14]);
  }

  void rotate(Vector3 axis, double angle) {
    final double len = axis.length;
    final double x = axis.x / len;
    final double y = axis.y / len;
    final double z = axis.z / len;
    final double c = math.cos(angle);
    final double s = math.sin(angle);
    final double C = 1.0 - c;
    final double m11 = x * x * C + c;
    final double m12 = x * y * C - z * s;
    final double m13 = x * z * C + y * s;
    final double m21 = y * x * C + z * s;
    final double m22 = y * y * C + c;
    final double m23 = y * z * C - x * s;
    final double m31 = z * x * C - y * s;
    final double m32 = z * y * C + x * s;
    final double m33 = z * z * C + c;
    final double t1 = _m4storage[0] * m11 + _m4storage[4] * m21 + _m4storage[8] * m31;
    final double t2 = _m4storage[1] * m11 + _m4storage[5] * m21 + _m4storage[9] * m31;
    final double t3 = _m4storage[2] * m11 + _m4storage[6] * m21 + _m4storage[10] * m31;
    final double t4 = _m4storage[3] * m11 + _m4storage[7] * m21 + _m4storage[11] * m31;
    final double t5 = _m4storage[0] * m12 + _m4storage[4] * m22 + _m4storage[8] * m32;
    final double t6 = _m4storage[1] * m12 + _m4storage[5] * m22 + _m4storage[9] * m32;
    final double t7 = _m4storage[2] * m12 + _m4storage[6] * m22 + _m4storage[10] * m32;
    final double t8 = _m4storage[3] * m12 + _m4storage[7] * m22 + _m4storage[11] * m32;
    final double t9 = _m4storage[0] * m13 + _m4storage[4] * m23 + _m4storage[8] * m33;
    final double t10 = _m4storage[1] * m13 + _m4storage[5] * m23 + _m4storage[9] * m33;
    final double t11 = _m4storage[2] * m13 + _m4storage[6] * m23 + _m4storage[10] * m33;
    final double t12 = _m4storage[3] * m13 + _m4storage[7] * m23 + _m4storage[11] * m33;
    _m4storage[0] = t1;
    _m4storage[1] = t2;
    _m4storage[2] = t3;
    _m4storage[3] = t4;
    _m4storage[4] = t5;
    _m4storage[5] = t6;
    _m4storage[6] = t7;
    _m4storage[7] = t8;
    _m4storage[8] = t9;
    _m4storage[9] = t10;
    _m4storage[10] = t11;
    _m4storage[11] = t12;
  }

  void rotateZ(double angle) {
    final double cosAngle = math.cos(angle);
    final double sinAngle = math.sin(angle);
    final double t1 = _m4storage[0] * cosAngle + _m4storage[4] * sinAngle;
    final double t2 = _m4storage[1] * cosAngle + _m4storage[5] * sinAngle;
    final double t3 = _m4storage[2] * cosAngle + _m4storage[6] * sinAngle;
    final double t4 = _m4storage[3] * cosAngle + _m4storage[7] * sinAngle;
    final double t5 = _m4storage[0] * -sinAngle + _m4storage[4] * cosAngle;
    final double t6 = _m4storage[1] * -sinAngle + _m4storage[5] * cosAngle;
    final double t7 = _m4storage[2] * -sinAngle + _m4storage[6] * cosAngle;
    final double t8 = _m4storage[3] * -sinAngle + _m4storage[7] * cosAngle;
    _m4storage[0] = t1;
    _m4storage[1] = t2;
    _m4storage[2] = t3;
    _m4storage[3] = t4;
    _m4storage[4] = t5;
    _m4storage[5] = t6;
    _m4storage[6] = t7;
    _m4storage[7] = t8;
  }

  /// Sets the translation vector in this homogeneous transformation matrix.
  void setTranslation(Vector3 t) {
    _m4storage[14] = t.z;
    _m4storage[13] = t.y;
    _m4storage[12] = t.x;
  }

  /// Sets the translation vector in this homogeneous transformation matrix.
  void setTranslationRaw(double x, double y, double z) {
    _m4storage[14] = z;
    _m4storage[13] = y;
    _m4storage[12] = x;
  }

  /// Transposes just the upper 3x3 rotation matrix.
  void transposeRotation() {
    double temp;
    temp = _m4storage[1];
    _m4storage[1] = _m4storage[4];
    _m4storage[4] = temp;
    temp = _m4storage[2];
    _m4storage[2] = _m4storage[8];
    _m4storage[8] = temp;
    temp = _m4storage[4];
    _m4storage[4] = _m4storage[1];
    _m4storage[1] = temp;
    temp = _m4storage[6];
    _m4storage[6] = _m4storage[9];
    _m4storage[9] = temp;
    temp = _m4storage[8];
    _m4storage[8] = _m4storage[2];
    _m4storage[2] = temp;
    temp = _m4storage[9];
    _m4storage[9] = _m4storage[6];
    _m4storage[6] = temp;
  }

  /// Invert [this].
  double invert() => copyInverse(this);

  /// Set this matrix to be the inverse of [arg]
  double copyInverse(Matrix4 arg) {
    final Float32List argStorage = arg._m4storage;
    final double a00 = argStorage[0];
    final double a01 = argStorage[1];
    final double a02 = argStorage[2];
    final double a03 = argStorage[3];
    final double a10 = argStorage[4];
    final double a11 = argStorage[5];
    final double a12 = argStorage[6];
    final double a13 = argStorage[7];
    final double a20 = argStorage[8];
    final double a21 = argStorage[9];
    final double a22 = argStorage[10];
    final double a23 = argStorage[11];
    final double a30 = argStorage[12];
    final double a31 = argStorage[13];
    final double a32 = argStorage[14];
    final double a33 = argStorage[15];
    final double b00 = a00 * a11 - a01 * a10;
    final double b01 = a00 * a12 - a02 * a10;
    final double b02 = a00 * a13 - a03 * a10;
    final double b03 = a01 * a12 - a02 * a11;
    final double b04 = a01 * a13 - a03 * a11;
    final double b05 = a02 * a13 - a03 * a12;
    final double b06 = a20 * a31 - a21 * a30;
    final double b07 = a20 * a32 - a22 * a30;
    final double b08 = a20 * a33 - a23 * a30;
    final double b09 = a21 * a32 - a22 * a31;
    final double b10 = a21 * a33 - a23 * a31;
    final double b11 = a22 * a33 - a23 * a32;
    final double det = b00 * b11 - b01 * b10 + b02 * b09 + b03 * b08 - b04 * b07 + b05 * b06;
    if (det == 0.0) {
      setFrom(arg);
      return 0.0;
    }
    final double invDet = 1.0 / det;
    _m4storage[0] = (a11 * b11 - a12 * b10 + a13 * b09) * invDet;
    _m4storage[1] = (-a01 * b11 + a02 * b10 - a03 * b09) * invDet;
    _m4storage[2] = (a31 * b05 - a32 * b04 + a33 * b03) * invDet;
    _m4storage[3] = (-a21 * b05 + a22 * b04 - a23 * b03) * invDet;
    _m4storage[4] = (-a10 * b11 + a12 * b08 - a13 * b07) * invDet;
    _m4storage[5] = (a00 * b11 - a02 * b08 + a03 * b07) * invDet;
    _m4storage[6] = (-a30 * b05 + a32 * b02 - a33 * b01) * invDet;
    _m4storage[7] = (a20 * b05 - a22 * b02 + a23 * b01) * invDet;
    _m4storage[8] = (a10 * b10 - a11 * b08 + a13 * b06) * invDet;
    _m4storage[9] = (-a00 * b10 + a01 * b08 - a03 * b06) * invDet;
    _m4storage[10] = (a30 * b04 - a31 * b02 + a33 * b00) * invDet;
    _m4storage[11] = (-a20 * b04 + a21 * b02 - a23 * b00) * invDet;
    _m4storage[12] = (-a10 * b09 + a11 * b07 - a12 * b06) * invDet;
    _m4storage[13] = (a00 * b09 - a01 * b07 + a02 * b06) * invDet;
    _m4storage[14] = (-a30 * b03 + a31 * b01 - a32 * b00) * invDet;
    _m4storage[15] = (a20 * b03 - a21 * b01 + a22 * b00) * invDet;
    return det;
  }

  double invertRotation() {
    final double det = determinant();
    if (det == 0.0) {
      return 0.0;
    }
    final double invDet = 1.0 / det;
    double ix;
    double iy;
    double iz;
    double jx;
    double jy;
    double jz;
    double kx;
    double ky;
    double kz;
    ix = invDet * (_m4storage[5] * _m4storage[10] - _m4storage[6] * _m4storage[9]);
    iy = invDet * (_m4storage[2] * _m4storage[9] - _m4storage[1] * _m4storage[10]);
    iz = invDet * (_m4storage[1] * _m4storage[6] - _m4storage[2] * _m4storage[5]);
    jx = invDet * (_m4storage[6] * _m4storage[8] - _m4storage[4] * _m4storage[10]);
    jy = invDet * (_m4storage[0] * _m4storage[10] - _m4storage[2] * _m4storage[8]);
    jz = invDet * (_m4storage[2] * _m4storage[4] - _m4storage[0] * _m4storage[6]);
    kx = invDet * (_m4storage[4] * _m4storage[9] - _m4storage[5] * _m4storage[8]);
    ky = invDet * (_m4storage[1] * _m4storage[8] - _m4storage[0] * _m4storage[9]);
    kz = invDet * (_m4storage[0] * _m4storage[5] - _m4storage[1] * _m4storage[4]);
    _m4storage[0] = ix;
    _m4storage[1] = iy;
    _m4storage[2] = iz;
    _m4storage[4] = jx;
    _m4storage[5] = jy;
    _m4storage[6] = jz;
    _m4storage[8] = kx;
    _m4storage[9] = ky;
    _m4storage[10] = kz;
    return det;
  }

  /// Sets the upper 3x3 to a rotation of [radians] around X
  void setRotationX(double radians) {
    final double c = math.cos(radians);
    final double s = math.sin(radians);
    _m4storage[0] = 1.0;
    _m4storage[1] = 0.0;
    _m4storage[2] = 0.0;
    _m4storage[4] = 0.0;
    _m4storage[5] = c;
    _m4storage[6] = s;
    _m4storage[8] = 0.0;
    _m4storage[9] = -s;
    _m4storage[10] = c;
    _m4storage[3] = 0.0;
    _m4storage[7] = 0.0;
    _m4storage[11] = 0.0;
  }

  /// Sets the upper 3x3 to a rotation of [radians] around Y
  void setRotationY(double radians) {
    final double c = math.cos(radians);
    final double s = math.sin(radians);
    _m4storage[0] = c;
    _m4storage[1] = 0.0;
    _m4storage[2] = -s;
    _m4storage[4] = 0.0;
    _m4storage[5] = 1.0;
    _m4storage[6] = 0.0;
    _m4storage[8] = s;
    _m4storage[9] = 0.0;
    _m4storage[10] = c;
    _m4storage[3] = 0.0;
    _m4storage[7] = 0.0;
    _m4storage[11] = 0.0;
  }

  /// Sets the upper 3x3 to a rotation of [radians] around Z
  void setRotationZ(double radians) {
    final double c = math.cos(radians);
    final double s = math.sin(radians);
    _m4storage[0] = c;
    _m4storage[1] = s;
    _m4storage[2] = 0.0;
    _m4storage[4] = -s;
    _m4storage[5] = c;
    _m4storage[6] = 0.0;
    _m4storage[8] = 0.0;
    _m4storage[9] = 0.0;
    _m4storage[10] = 1.0;
    _m4storage[3] = 0.0;
    _m4storage[7] = 0.0;
    _m4storage[11] = 0.0;
  }

  /// Multiply [this] by [arg].
  void multiply(Matrix4 arg) {
    final double m33 = _m4storage[15];
    final double m00 = _m4storage[0];
    final double m01 = _m4storage[4];
    final double m02 = _m4storage[8];
    final double m03 = _m4storage[12];
    final double m10 = _m4storage[1];
    final double m11 = _m4storage[5];
    final double m12 = _m4storage[9];
    final double m13 = _m4storage[13];
    final double m20 = _m4storage[2];
    final double m21 = _m4storage[6];
    final double m22 = _m4storage[10];
    final double m23 = _m4storage[14];
    final double m30 = _m4storage[3];
    final double m31 = _m4storage[7];
    final double m32 = _m4storage[11];
    final Float32List argStorage = arg._m4storage;
    final double n33 = argStorage[15];
    final double n00 = argStorage[0];
    final double n01 = argStorage[4];
    final double n02 = argStorage[8];
    final double n03 = argStorage[12];
    final double n10 = argStorage[1];
    final double n11 = argStorage[5];
    final double n12 = argStorage[9];
    final double n13 = argStorage[13];
    final double n20 = argStorage[2];
    final double n21 = argStorage[6];
    final double n22 = argStorage[10];
    final double n23 = argStorage[14];
    final double n30 = argStorage[3];
    final double n31 = argStorage[7];
    final double n32 = argStorage[11];
    _m4storage[0] = (m00 * n00) + (m01 * n10) + (m02 * n20) + (m03 * n30);
    _m4storage[4] = (m00 * n01) + (m01 * n11) + (m02 * n21) + (m03 * n31);
    _m4storage[8] = (m00 * n02) + (m01 * n12) + (m02 * n22) + (m03 * n32);
    _m4storage[12] = (m00 * n03) + (m01 * n13) + (m02 * n23) + (m03 * n33);
    _m4storage[1] = (m10 * n00) + (m11 * n10) + (m12 * n20) + (m13 * n30);
    _m4storage[5] = (m10 * n01) + (m11 * n11) + (m12 * n21) + (m13 * n31);
    _m4storage[9] = (m10 * n02) + (m11 * n12) + (m12 * n22) + (m13 * n32);
    _m4storage[13] = (m10 * n03) + (m11 * n13) + (m12 * n23) + (m13 * n33);
    _m4storage[2] = (m20 * n00) + (m21 * n10) + (m22 * n20) + (m23 * n30);
    _m4storage[6] = (m20 * n01) + (m21 * n11) + (m22 * n21) + (m23 * n31);
    _m4storage[10] = (m20 * n02) + (m21 * n12) + (m22 * n22) + (m23 * n32);
    _m4storage[14] = (m20 * n03) + (m21 * n13) + (m22 * n23) + (m23 * n33);
    _m4storage[3] = (m30 * n00) + (m31 * n10) + (m32 * n20) + (m33 * n30);
    _m4storage[7] = (m30 * n01) + (m31 * n11) + (m32 * n21) + (m33 * n31);
    _m4storage[11] = (m30 * n02) + (m31 * n12) + (m32 * n22) + (m33 * n32);
    _m4storage[15] = (m30 * n03) + (m31 * n13) + (m32 * n23) + (m33 * n33);
  }

  /// Multiply a copy of [this] with [arg].
  Matrix4 multiplied(Matrix4 arg) => clone()..multiply(arg);

  /// Multiply a transposed [this] with [arg].
  void transposeMultiply(Matrix4 arg) {
    final double m33 = _m4storage[15];
    final double m00 = _m4storage[0];
    final double m01 = _m4storage[1];
    final double m02 = _m4storage[2];
    final double m03 = _m4storage[3];
    final double m10 = _m4storage[4];
    final double m11 = _m4storage[5];
    final double m12 = _m4storage[6];
    final double m13 = _m4storage[7];
    final double m20 = _m4storage[8];
    final double m21 = _m4storage[9];
    final double m22 = _m4storage[10];
    final double m23 = _m4storage[11];
    final double m30 = _m4storage[12];
    final double m31 = _m4storage[13];
    final double m32 = _m4storage[14];

    final Float32List argStorage = arg._m4storage;
    _m4storage[0] =
        (m00 * argStorage[0]) +
        (m01 * argStorage[1]) +
        (m02 * argStorage[2]) +
        (m03 * argStorage[3]);
    _m4storage[4] =
        (m00 * argStorage[4]) +
        (m01 * argStorage[5]) +
        (m02 * argStorage[6]) +
        (m03 * argStorage[7]);
    _m4storage[8] =
        (m00 * argStorage[8]) +
        (m01 * argStorage[9]) +
        (m02 * argStorage[10]) +
        (m03 * argStorage[11]);
    _m4storage[12] =
        (m00 * argStorage[12]) +
        (m01 * argStorage[13]) +
        (m02 * argStorage[14]) +
        (m03 * argStorage[15]);
    _m4storage[1] =
        (m10 * argStorage[0]) +
        (m11 * argStorage[1]) +
        (m12 * argStorage[2]) +
        (m13 * argStorage[3]);
    _m4storage[5] =
        (m10 * argStorage[4]) +
        (m11 * argStorage[5]) +
        (m12 * argStorage[6]) +
        (m13 * argStorage[7]);
    _m4storage[9] =
        (m10 * argStorage[8]) +
        (m11 * argStorage[9]) +
        (m12 * argStorage[10]) +
        (m13 * argStorage[11]);
    _m4storage[13] =
        (m10 * argStorage[12]) +
        (m11 * argStorage[13]) +
        (m12 * argStorage[14]) +
        (m13 * argStorage[15]);
    _m4storage[2] =
        (m20 * argStorage[0]) +
        (m21 * argStorage[1]) +
        (m22 * argStorage[2]) +
        (m23 * argStorage[3]);
    _m4storage[6] =
        (m20 * argStorage[4]) +
        (m21 * argStorage[5]) +
        (m22 * argStorage[6]) +
        (m23 * argStorage[7]);
    _m4storage[10] =
        (m20 * argStorage[8]) +
        (m21 * argStorage[9]) +
        (m22 * argStorage[10]) +
        (m23 * argStorage[11]);
    _m4storage[14] =
        (m20 * argStorage[12]) +
        (m21 * argStorage[13]) +
        (m22 * argStorage[14]) +
        (m23 * argStorage[15]);
    _m4storage[3] =
        (m30 * argStorage[0]) +
        (m31 * argStorage[1]) +
        (m32 * argStorage[2]) +
        (m33 * argStorage[3]);
    _m4storage[7] =
        (m30 * argStorage[4]) +
        (m31 * argStorage[5]) +
        (m32 * argStorage[6]) +
        (m33 * argStorage[7]);
    _m4storage[11] =
        (m30 * argStorage[8]) +
        (m31 * argStorage[9]) +
        (m32 * argStorage[10]) +
        (m33 * argStorage[11]);
    _m4storage[15] =
        (m30 * argStorage[12]) +
        (m31 * argStorage[13]) +
        (m32 * argStorage[14]) +
        (m33 * argStorage[15]);
  }

  /// Multiply [this] with a transposed [arg].
  void multiplyTranspose(Matrix4 arg) {
    final double m00 = _m4storage[0];
    final double m01 = _m4storage[4];
    final double m02 = _m4storage[8];
    final double m03 = _m4storage[12];
    final double m10 = _m4storage[1];
    final double m11 = _m4storage[5];
    final double m12 = _m4storage[9];
    final double m13 = _m4storage[13];
    final double m20 = _m4storage[2];
    final double m21 = _m4storage[6];
    final double m22 = _m4storage[10];
    final double m23 = _m4storage[14];
    final double m30 = _m4storage[3];
    final double m31 = _m4storage[7];
    final double m32 = _m4storage[11];
    final double m33 = _m4storage[15];
    final Float32List argStorage = arg._m4storage;
    _m4storage[0] =
        (m00 * argStorage[0]) +
        (m01 * argStorage[4]) +
        (m02 * argStorage[8]) +
        (m03 * argStorage[12]);
    _m4storage[4] =
        (m00 * argStorage[1]) +
        (m01 * argStorage[5]) +
        (m02 * argStorage[9]) +
        (m03 * argStorage[13]);
    _m4storage[8] =
        (m00 * argStorage[2]) +
        (m01 * argStorage[6]) +
        (m02 * argStorage[10]) +
        (m03 * argStorage[14]);
    _m4storage[12] =
        (m00 * argStorage[3]) +
        (m01 * argStorage[7]) +
        (m02 * argStorage[11]) +
        (m03 * argStorage[15]);
    _m4storage[1] =
        (m10 * argStorage[0]) +
        (m11 * argStorage[4]) +
        (m12 * argStorage[8]) +
        (m13 * argStorage[12]);
    _m4storage[5] =
        (m10 * argStorage[1]) +
        (m11 * argStorage[5]) +
        (m12 * argStorage[9]) +
        (m13 * argStorage[13]);
    _m4storage[9] =
        (m10 * argStorage[2]) +
        (m11 * argStorage[6]) +
        (m12 * argStorage[10]) +
        (m13 * argStorage[14]);
    _m4storage[13] =
        (m10 * argStorage[3]) +
        (m11 * argStorage[7]) +
        (m12 * argStorage[11]) +
        (m13 * argStorage[15]);
    _m4storage[2] =
        (m20 * argStorage[0]) +
        (m21 * argStorage[4]) +
        (m22 * argStorage[8]) +
        (m23 * argStorage[12]);
    _m4storage[6] =
        (m20 * argStorage[1]) +
        (m21 * argStorage[5]) +
        (m22 * argStorage[9]) +
        (m23 * argStorage[13]);
    _m4storage[10] =
        (m20 * argStorage[2]) +
        (m21 * argStorage[6]) +
        (m22 * argStorage[10]) +
        (m23 * argStorage[14]);
    _m4storage[14] =
        (m20 * argStorage[3]) +
        (m21 * argStorage[7]) +
        (m22 * argStorage[11]) +
        (m23 * argStorage[15]);
    _m4storage[3] =
        (m30 * argStorage[0]) +
        (m31 * argStorage[4]) +
        (m32 * argStorage[8]) +
        (m33 * argStorage[12]);
    _m4storage[7] =
        (m30 * argStorage[1]) +
        (m31 * argStorage[5]) +
        (m32 * argStorage[9]) +
        (m33 * argStorage[13]);
    _m4storage[11] =
        (m30 * argStorage[2]) +
        (m31 * argStorage[6]) +
        (m32 * argStorage[10]) +
        (m33 * argStorage[14]);
    _m4storage[15] =
        (m30 * argStorage[3]) +
        (m31 * argStorage[7]) +
        (m32 * argStorage[11]) +
        (m33 * argStorage[15]);
  }

  /// Transforms a 3-component vector in-place.
  void transform3(Float32List vector) {
    final double x =
        (_m4storage[0] * vector[0]) +
        (_m4storage[4] * vector[1]) +
        (_m4storage[8] * vector[2]) +
        _m4storage[12];
    final double y =
        (_m4storage[1] * vector[0]) +
        (_m4storage[5] * vector[1]) +
        (_m4storage[9] * vector[2]) +
        _m4storage[13];
    final double z =
        (_m4storage[2] * vector[0]) +
        (_m4storage[6] * vector[1]) +
        (_m4storage[10] * vector[2]) +
        _m4storage[14];
    vector[0] = x;
    vector[1] = y;
    vector[2] = z;
  }

  /// Transforms a 2-component vector in-place.
  ///
  /// This transformation forgets the final Z component. If you need the
  /// Z component, see [transform3].
  void transform2(Float32List vector) {
    final double x = vector[0];
    final double y = vector[1];
    vector[0] = (_m4storage[0] * x) + (_m4storage[4] * y) + _m4storage[12];
    vector[1] = (_m4storage[1] * x) + (_m4storage[5] * y) + _m4storage[13];
  }

  /// Transforms the input rect and calculates the bounding box of the rect
  /// after the transform.
  ui.Rect transformRect(ui.Rect rect) => transformRectWithMatrix(this, rect);

  /// Copies [this] into [array] starting at [offset].
  void copyIntoArray(List<num> array, [int offset = 0]) {
    final int i = offset;
    array[i + 15] = _m4storage[15];
    array[i + 14] = _m4storage[14];
    array[i + 13] = _m4storage[13];
    array[i + 12] = _m4storage[12];
    array[i + 11] = _m4storage[11];
    array[i + 10] = _m4storage[10];
    array[i + 9] = _m4storage[9];
    array[i + 8] = _m4storage[8];
    array[i + 7] = _m4storage[7];
    array[i + 6] = _m4storage[6];
    array[i + 5] = _m4storage[5];
    array[i + 4] = _m4storage[4];
    array[i + 3] = _m4storage[3];
    array[i + 2] = _m4storage[2];
    array[i + 1] = _m4storage[1];
    array[i + 0] = _m4storage[0];
  }

  /// Copies elements from [array] into [this] starting at [offset].
  void copyFromArray(List<double> array, [int offset = 0]) {
    final int i = offset;
    _m4storage[15] = array[i + 15];
    _m4storage[14] = array[i + 14];
    _m4storage[13] = array[i + 13];
    _m4storage[12] = array[i + 12];
    _m4storage[11] = array[i + 11];
    _m4storage[10] = array[i + 10];
    _m4storage[9] = array[i + 9];
    _m4storage[8] = array[i + 8];
    _m4storage[7] = array[i + 7];
    _m4storage[6] = array[i + 6];
    _m4storage[5] = array[i + 5];
    _m4storage[4] = array[i + 4];
    _m4storage[3] = array[i + 3];
    _m4storage[2] = array[i + 2];
    _m4storage[1] = array[i + 1];
    _m4storage[0] = array[i + 0];
  }

  /// Converts this matrix to a [Float64List].
  ///
  /// Generally we try to stick with 32-bit floats inside the engine code
  /// because it's faster (see [toMatrix32]). However, this method is handy
  /// in tests that use the public `dart:ui` surface.
  ///
  /// This method is not optimized, but also is not meant to be fast, only
  /// convenient.
  Float64List toFloat64() {
    return Float64List.fromList(_m4storage);
  }

  @override
  String toString() {
    String result = super.toString();
    assert(() {
      String fmt(int index) {
        return storage[index].toStringAsFixed(2);
      }

      result =
          '[${fmt(0)}, ${fmt(4)}, ${fmt(8)}, ${fmt(12)}]\n'
          '[${fmt(1)}, ${fmt(5)}, ${fmt(9)}, ${fmt(13)}]\n'
          '[${fmt(2)}, ${fmt(6)}, ${fmt(10)}, ${fmt(14)}]\n'
          '[${fmt(3)}, ${fmt(7)}, ${fmt(11)}, ${fmt(15)}]';
      return true;
    }());
    return result;
  }
}

const Vector3 kUnitX = (x: 1.0, y: 0.0, z: 0.0);
const Vector3 kUnitY = (x: 0.0, y: 1.0, z: 0.0);
const Vector3 kUnitZ = (x: 0.0, y: 0.0, z: 1.0);

/// 3D column vector.
typedef Vector3 = ({double x, double y, double z});

extension Vector3Extension on Vector3 {
  /// Length.
  double get length => math.sqrt(length2);

  /// Length squared.
  double get length2 {
    return (x * x) + (y * y) + (z * z);
  }
}

/// Converts a matrix represented using [Float64List] to one represented using
/// [Float32List].
///
/// 32-bit precision is sufficient because Flutter Engine itself (as well as
/// Skia) use 32-bit precision under the hood anyway.
///
/// 32-bit matrices require 2x less memory and in V8 they are allocated on the
/// JavaScript heap, thus avoiding a malloc.
///
/// See also:
/// * https://bugs.chromium.org/p/v8/issues/detail?id=9199
/// * https://bugs.chromium.org/p/v8/issues/detail?id=2022
Float32List toMatrix32(Float64List matrix64) {
  final Float32List matrix32 = Float32List(16);
  matrix32[15] = matrix64[15];
  matrix32[14] = matrix64[14];
  matrix32[13] = matrix64[13];
  matrix32[12] = matrix64[12];
  matrix32[11] = matrix64[11];
  matrix32[10] = matrix64[10];
  matrix32[9] = matrix64[9];
  matrix32[8] = matrix64[8];
  matrix32[7] = matrix64[7];
  matrix32[6] = matrix64[6];
  matrix32[5] = matrix64[5];
  matrix32[4] = matrix64[4];
  matrix32[3] = matrix64[3];
  matrix32[2] = matrix64[2];
  matrix32[1] = matrix64[1];
  matrix32[0] = matrix64[0];
  return matrix32;
}

/// Converts a matrix represented using [Float32List] to one represented using
/// [Float64List].
///
/// 32-bit precision is sufficient because Flutter Engine itself (as well as
/// Skia) use 32-bit precision under the hood anyway.
///
/// 32-bit matrices require 2x less memory and in V8 they are allocated on the
/// JavaScript heap, thus avoiding a malloc.
///
/// See also:
/// * https://bugs.chromium.org/p/v8/issues/detail?id=9199
/// * https://bugs.chromium.org/p/v8/issues/detail?id=2022
Float64List toMatrix64(Float32List matrix32) {
  final Float64List matrix64 = Float64List(16);
  matrix64[15] = matrix32[15];
  matrix64[14] = matrix32[14];
  matrix64[13] = matrix32[13];
  matrix64[12] = matrix32[12];
  matrix64[11] = matrix32[11];
  matrix64[10] = matrix32[10];
  matrix64[9] = matrix32[9];
  matrix64[8] = matrix32[8];
  matrix64[7] = matrix32[7];
  matrix64[6] = matrix32[6];
  matrix64[5] = matrix32[5];
  matrix64[4] = matrix32[4];
  matrix64[3] = matrix32[3];
  matrix64[2] = matrix32[2];
  matrix64[1] = matrix32[1];
  matrix64[0] = matrix32[0];
  return matrix64;
}

// Stores matrix in a form that allows zero allocation transforms.
// TODO(yjbanov): re-evaluate the need for this class. It may be an
//                over-optimization. It is only used by `GradientLinear` in the
//                HTML renderer. However that class creates a whole new WebGL
//                context to render the gradient, then copies the resulting
//                bitmap back into the destination canvas. This is multiple
//                orders of magnitude more computation and data copying. Saving
//                an allocation of one point is unlikely to save anything, but
//                is guaranteed to add complexity (e.g. it's stateful).
class FastMatrix32 {
  FastMatrix32(this.matrix);

  final Float32List matrix;
  double transformedX = 0;
  double transformedY = 0;

  /// Transforms the point defined by [x] and [y] using the [matrix] and stores
  /// the results in [transformedX] and [transformedY].
  void transform(double x, double y) {
    transformedX = matrix[12] + (matrix[0] * x) + (matrix[4] * y);
    transformedY = matrix[13] + (matrix[1] * x) + (matrix[5] * y);
  }

  String debugToString() =>
      '${matrix[0].toStringAsFixed(3)}, ${matrix[4].toStringAsFixed(3)}, ${matrix[8].toStringAsFixed(3)}, ${matrix[12].toStringAsFixed(3)}\n'
      '${matrix[1].toStringAsFixed(3)}, ${matrix[5].toStringAsFixed(3)}, ${matrix[9].toStringAsFixed(3)}, ${matrix[13].toStringAsFixed(3)}\n'
      '${matrix[2].toStringAsFixed(3)}, ${matrix[6].toStringAsFixed(3)}, ${matrix[10].toStringAsFixed(3)}, ${matrix[14].toStringAsFixed(3)}\n'
      '${matrix[3].toStringAsFixed(3)}, ${matrix[7].toStringAsFixed(3)}, ${matrix[11].toStringAsFixed(3)}, ${matrix[15].toStringAsFixed(3)}\n';
}
