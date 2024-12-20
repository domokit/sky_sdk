// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_DISPLAY_LIST_UTILS_DL_MATRIX_CLIP_TRACKER_H_
#define FLUTTER_DISPLAY_LIST_UTILS_DL_MATRIX_CLIP_TRACKER_H_

#include <vector>

#include "flutter/display_list/dl_canvas.h"
#include "flutter/display_list/geometry/dl_geometry_types.h"
#include "flutter/fml/logging.h"

#include "third_party/skia/include/core/SkM44.h"
#include "third_party/skia/include/core/SkMatrix.h"
#include "third_party/skia/include/core/SkPath.h"
#include "third_party/skia/include/core/SkRRect.h"
#include "third_party/skia/include/core/SkRect.h"
#include "third_party/skia/include/core/SkScalar.h"

namespace flutter {

class DisplayListMatrixClipState {
 private:
  using ClipOp = DlCanvas::ClipOp;

 public:
  explicit DisplayListMatrixClipState(const DlRect& cull_rect,
                                      const DlMatrix& matrix = DlMatrix());
  explicit DisplayListMatrixClipState(const SkRect& cull_rect);
  DisplayListMatrixClipState(const SkRect& cull_rect, const SkMatrix& matrix);
  DisplayListMatrixClipState(const SkRect& cull_rect, const SkM44& matrix);
  DisplayListMatrixClipState(const DisplayListMatrixClipState& other) = default;

  static bool is_3x3(const SkM44& m44);

  // This method should almost never be used as it breaks the encapsulation
  // of the enclosing clips. However it is needed for practical purposes in
  // some rare cases - such as when a saveLayer is collecting rendering
  // operations prior to applying a filter on the entire layer bounds and
  // some of those operations fall outside the enclosing clip, but their
  // filtered content will spread out from where they were rendered on the
  // layer into the enclosing clipped area.
  // Omitting the |cull_rect| argument, or passing nullptr, will restore the
  // cull rect to the initial value it had when the tracker was constructed.
  void resetDeviceCullRect(const DlRect& cull_rect);
  void resetDeviceCullRect(const SkRect& cull_rect) {
    resetDeviceCullRect(ToDlRect(cull_rect));
  }
  void resetLocalCullRect(const DlRect& cull_rect);
  void resetLocalCullRect(const SkRect& cull_rect) {
    resetLocalCullRect(ToDlRect(cull_rect));
  }

  bool using_4x4_matrix() const { return !matrix_.IsAffine(); }
  bool is_matrix_invertable() const { return matrix_.IsInvertible(); }
  bool has_perspective() const { return matrix_.HasPerspective(); }

  const DlMatrix& matrix() const { return matrix_; }
  SkM44 matrix_4x4() const { return SkM44::ColMajor(matrix_.m); }
  SkMatrix matrix_3x3() const { return ToSkMatrix(matrix_); }

  SkRect local_cull_rect() const { return ToSkRect(GetLocalCullCoverage()); }
  DlRect GetLocalCullCoverage() const;
  SkRect device_cull_rect() const { return ToSkRect(cull_rect_); }
  DlRect GetDeviceCullCoverage() const { return cull_rect_; }

  bool rect_covers_cull(const DlRect& content) const;
  bool rect_covers_cull(const SkRect& content) const {
    return rect_covers_cull(ToDlRect(content));
  }
  bool oval_covers_cull(const DlRect& content_bounds) const;
  bool oval_covers_cull(const SkRect& content_bounds) const {
    return oval_covers_cull(ToDlRect(content_bounds));
  }
  bool rrect_covers_cull(const DlRoundRect& content) const;
  bool rrect_covers_cull(const SkRRect& content) const {
    return rrect_covers_cull(ToDlRoundRect(content));
  }

  bool content_culled(const DlRect& content_bounds) const;
  bool content_culled(const SkRect& content_bounds) const {
    return content_culled(ToDlRect(content_bounds));
  }
  bool is_cull_rect_empty() const { return cull_rect_.IsEmpty(); }

  void translate(SkScalar tx, SkScalar ty) {
    matrix_ = matrix_.Translate({tx, ty});
  }
  void scale(SkScalar sx, SkScalar sy) {
    matrix_ = matrix_.Scale({sx, sy, 1.0f});
  }
  void skew(SkScalar skx, SkScalar sky) {
    matrix_ = matrix_ * DlMatrix::MakeSkew(skx, sky);
  }
  void rotate(SkScalar degrees) {
    matrix_ = matrix_ * DlMatrix::MakeRotationZ(DlDegrees(degrees));
  }
  void transform(const DlMatrix& matrix) { matrix_ = matrix_ * matrix; }
  void transform(const SkM44& m44) { transform(ToDlMatrix(m44)); }
  void transform(const SkMatrix& matrix) { transform(ToDlMatrix(matrix)); }
  // clang-format off
  void transform2DAffine(
      SkScalar mxx, SkScalar mxy, SkScalar mxt,
      SkScalar myx, SkScalar myy, SkScalar myt) {
    matrix_ = matrix_ * DlMatrix::MakeColumn(
         mxx,  myx, 0.0f, 0.0f,
         mxy,  myy, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
         mxt,  myt, 0.0f, 1.0f
    );
  }
  void transformFullPerspective(
      SkScalar mxx, SkScalar mxy, SkScalar mxz, SkScalar mxt,
      SkScalar myx, SkScalar myy, SkScalar myz, SkScalar myt,
      SkScalar mzx, SkScalar mzy, SkScalar mzz, SkScalar mzt,
      SkScalar mwx, SkScalar mwy, SkScalar mwz, SkScalar mwt) {
    matrix_ = matrix_ * DlMatrix::MakeColumn(
        mxx, myx, mzx, mwx,
        mxy, myy, mzy, mwy,
        mxz, myz, mzz, mwz,
        mxt, myt, mzt, mwt
    );
  }
  // clang-format on
  void setTransform(const DlMatrix& matrix) { matrix_ = matrix; }
  void setTransform(const SkMatrix& matrix) { matrix_ = ToDlMatrix(matrix); }
  void setTransform(const SkM44& m44) { matrix_ = ToDlMatrix(m44); }
  void setIdentity() { matrix_ = DlMatrix(); }
  // If the matrix in |other_tracker| is invertible then transform this
  // tracker by the inverse of its matrix and return true. Otherwise,
  // return false and leave this tracker unmodified.
  bool inverseTransform(const DisplayListMatrixClipState& other_tracker);

  bool mapRect(DlRect* rect) const { return mapRect(*rect, rect); }
  bool mapRect(const DlRect& src, DlRect* mapped) const {
    *mapped = src.TransformAndClipBounds(matrix_);
    return matrix_.IsAligned2D();
  }
  bool mapRect(SkRect* rect) const { return mapRect(*rect, rect); }
  bool mapRect(const SkRect& src, SkRect* mapped) const {
    *mapped = ToSkRect(ToDlRect(src).TransformAndClipBounds(matrix_));
    return matrix_.IsAligned2D();
  }

  /// @brief  Maps the rect by the current matrix and then clips it against
  ///         the current cull rect, returning true if the result is non-empty.
  bool mapAndClipRect(SkRect* rect) const {
    return mapAndClipRect(*rect, rect);
  }
  bool mapAndClipRect(const SkRect& src, SkRect* mapped) const;
  bool mapAndClipRect(DlRect* rect) const {
    return mapAndClipRect(*rect, rect);
  }
  bool mapAndClipRect(const DlRect& src, DlRect* mapped) const;

  void clipRect(const DlRect& rect, ClipOp op, bool is_aa);
  void clipRect(const SkRect& rect, ClipOp op, bool is_aa) {
    clipRect(ToDlRect(rect), op, is_aa);
  }
  void clipOval(const DlRect& bounds, ClipOp op, bool is_aa);
  void clipOval(const SkRect& bounds, ClipOp op, bool is_aa) {
    clipRect(ToDlRect(bounds), op, is_aa);
  }
  void clipRRect(const DlRoundRect& rrect, ClipOp op, bool is_aa);
  void clipRRect(const SkRRect& rrect, ClipOp op, bool is_aa) {
    clipRRect(ToDlRoundRect(rrect), op, is_aa);
  }
  void clipPath(const SkPath& path, ClipOp op, bool is_aa) {
    clipPath(DlPath(path), op, is_aa);
  }
  void clipPath(const DlPath& path, ClipOp op, bool is_aa);

 private:
  DlRect cull_rect_;
  DlMatrix matrix_;

  bool getLocalCullCorners(DlPoint corners[4]) const;
  void adjustCullRect(const DlRect& clip, ClipOp op, bool is_aa);
};

}  // namespace flutter

#endif  // FLUTTER_DISPLAY_LIST_UTILS_DL_MATRIX_CLIP_TRACKER_H_
