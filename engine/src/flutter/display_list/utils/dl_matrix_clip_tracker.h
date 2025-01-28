// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_DISPLAY_LIST_UTILS_DL_MATRIX_CLIP_TRACKER_H_
#define FLUTTER_DISPLAY_LIST_UTILS_DL_MATRIX_CLIP_TRACKER_H_

#include <vector>

#include "flutter/display_list/dl_canvas.h"
#include "flutter/display_list/geometry/dl_geometry_types.h"
#include "flutter/fml/logging.h"

namespace flutter {

class DisplayListMatrixClipState {
 public:
  explicit DisplayListMatrixClipState(const DlRect& cull_rect,
                                      const DlMatrix& matrix = DlMatrix());
  DisplayListMatrixClipState(const DisplayListMatrixClipState& other) = default;

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
  void resetLocalCullRect(const DlRect& cull_rect);

  bool using_4x4_matrix() const { return !matrix_.IsAffine(); }
  bool is_matrix_invertable() const { return matrix_.IsInvertible(); }
  bool has_perspective() const { return matrix_.HasPerspective(); }

  const DlMatrix& matrix() const { return matrix_; }

  DlRect GetLocalCullCoverage() const;
  DlRect GetDeviceCullCoverage() const { return cull_rect_; }

  bool rect_covers_cull(const DlRect& content) const;
  bool oval_covers_cull(const DlRect& content_bounds) const;
  bool rrect_covers_cull(const DlRoundRect& content) const;

  bool content_culled(const DlRect& content_bounds) const;
  bool is_cull_rect_empty() const { return cull_rect_.IsEmpty(); }

  void translate(DlScalar tx, DlScalar ty) {
    matrix_ = matrix_.Translate({tx, ty});
  }
  void scale(DlScalar sx, DlScalar sy) {
    matrix_ = matrix_.Scale({sx, sy, 1.0f});
  }
  void skew(DlScalar skx, DlScalar sky) {
    matrix_ = matrix_ * DlMatrix::MakeSkew(skx, sky);
  }
  void rotate(DlRadians angle) {
    matrix_ = matrix_ * DlMatrix::MakeRotationZ(angle);
  }
  void transform(const DlMatrix& matrix) { matrix_ = matrix_ * matrix; }
  // clang-format off
  void transform2DAffine(
      DlScalar mxx, DlScalar mxy, DlScalar mxt,
      DlScalar myx, DlScalar myy, DlScalar myt) {
    matrix_ = matrix_ * DlMatrix::MakeColumn(
         mxx,  myx, 0.0f, 0.0f,
         mxy,  myy, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
         mxt,  myt, 0.0f, 1.0f
    );
  }
  void transformFullPerspective(
      DlScalar mxx, DlScalar mxy, DlScalar mxz, DlScalar mxt,
      DlScalar myx, DlScalar myy, DlScalar myz, DlScalar myt,
      DlScalar mzx, DlScalar mzy, DlScalar mzz, DlScalar mzt,
      DlScalar mwx, DlScalar mwy, DlScalar mwz, DlScalar mwt) {
    matrix_ = matrix_ * DlMatrix::MakeColumn(
        mxx, myx, mzx, mwx,
        mxy, myy, mzy, mwy,
        mxz, myz, mzz, mwz,
        mxt, myt, mzt, mwt
    );
  }
  // clang-format on
  void setTransform(const DlMatrix& matrix) { matrix_ = matrix; }
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

  /// @brief  Maps the rect by the current matrix and then clips it against
  ///         the current cull rect, returning true if the result is non-empty.
  bool mapAndClipRect(DlRect* rect) const {
    return mapAndClipRect(*rect, rect);
  }
  bool mapAndClipRect(const DlRect& src, DlRect* mapped) const;

  void clipRect(const DlRect& rect, DlClipOp op, bool is_aa);
  void clipOval(const DlRect& bounds, DlClipOp op, bool is_aa);
  void clipRRect(const DlRoundRect& rrect, DlClipOp op, bool is_aa);
  void clipPath(const DlPath& path, DlClipOp op, bool is_aa);

 private:
  DlRect cull_rect_;
  DlMatrix matrix_;

  bool getLocalCullCorners(DlPoint corners[4]) const;
  void adjustCullRect(const DlRect& clip, DlClipOp op, bool is_aa);
};

}  // namespace flutter

#endif  // FLUTTER_DISPLAY_LIST_UTILS_DL_MATRIX_CLIP_TRACKER_H_
