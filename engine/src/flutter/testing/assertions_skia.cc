// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/testing/assertions_skia.h"

namespace std {

std::ostream& operator<<(std::ostream& os, const SkClipOp& o) {
  switch (o) {
    case SkClipOp::kDifference:
      os << "ClipOpDifference";
      break;
    case SkClipOp::kIntersect:
      os << "ClipOpIntersect";
      break;
    default:
      os << "ClipOpUnknown" << static_cast<int>(o);
      break;
  }
  return os;
}

std::ostream& operator<<(std::ostream& os, const SkMatrix& m) {
  os << std::endl;
  os << "Scale X: " << m[SkMatrix::kMScaleX] << ", ";
  os << "Skew  X: " << m[SkMatrix::kMSkewX] << ", ";
  os << "Trans X: " << m[SkMatrix::kMTransX] << std::endl;
  os << "Skew  Y: " << m[SkMatrix::kMSkewY] << ", ";
  os << "Scale Y: " << m[SkMatrix::kMScaleY] << ", ";
  os << "Trans Y: " << m[SkMatrix::kMTransY] << std::endl;
  os << "Persp X: " << m[SkMatrix::kMPersp0] << ", ";
  os << "Persp Y: " << m[SkMatrix::kMPersp1] << ", ";
  os << "Persp Z: " << m[SkMatrix::kMPersp2];
  os << std::endl;
  return os;
}

std::ostream& operator<<(std::ostream& os, const SkM44& m) {
  os << m.rc(0, 0) << ", " << m.rc(0, 1) << ", " << m.rc(0, 2) << ", "
     << m.rc(0, 3) << std::endl;
  os << m.rc(1, 0) << ", " << m.rc(1, 1) << ", " << m.rc(1, 2) << ", "
     << m.rc(1, 3) << std::endl;
  os << m.rc(2, 0) << ", " << m.rc(2, 1) << ", " << m.rc(2, 2) << ", "
     << m.rc(2, 3) << std::endl;
  os << m.rc(3, 0) << ", " << m.rc(3, 1) << ", " << m.rc(3, 2) << ", "
     << m.rc(3, 3);
  return os;
}

std::ostream& operator<<(std::ostream& os, const SkVector3& v) {
  return os << v.x() << ", " << v.y() << ", " << v.z();
}

std::ostream& operator<<(std::ostream& os, const SkIRect& r) {
  return os << "LTRB: " << r.fLeft << ", " << r.fTop << ", " << r.fRight << ", "
            << r.fBottom;
}

std::ostream& operator<<(std::ostream& os, const SkRect& r) {
  return os << "LTRB: " << r.fLeft << ", " << r.fTop << ", " << r.fRight << ", "
            << r.fBottom;
}

std::ostream& operator<<(std::ostream& os, const SkRRect& r) {
  return os << "LTRB: " << r.rect().fLeft << ", " << r.rect().fTop << ", "
            << r.rect().fRight << ", " << r.rect().fBottom;
}

std::ostream& operator<<(std::ostream& os, const SkPath& r) {
  return os << "Valid: " << r.isValid()
            << ", FillType: " << static_cast<int>(r.getFillType())
            << ", Bounds: " << r.getBounds();
}

std::ostream& operator<<(std::ostream& os, const SkPoint& r) {
  return os << "XY: " << r.fX << ", " << r.fY;
}

std::ostream& operator<<(std::ostream& os, const SkISize& size) {
  return os << size.width() << ", " << size.height();
}

std::ostream& operator<<(std::ostream& os, const SkColor4f& r) {
  return os << r.fR << ", " << r.fG << ", " << r.fB << ", " << r.fA;
}

std::ostream& operator<<(std::ostream& os, const SkPaint& r) {
  return os << "Color: " << r.getColor4f() << ", Style: " << r.getStyle()
            << ", AA: " << r.isAntiAlias() << ", Shader: " << r.getShader();
}

std::ostream& operator<<(std::ostream& os, const SkSamplingOptions& s) {
  if (s.useCubic) {
    return os << "CubicResampler: " << s.cubic.B << ", " << s.cubic.C;
  } else {
    return os << "Filter: " << static_cast<int>(s.filter)
              << ", Mipmap: " << static_cast<int>(s.mipmap);
  }
}

}  // namespace std
