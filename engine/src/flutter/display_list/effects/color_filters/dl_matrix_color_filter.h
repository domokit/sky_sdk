// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_DISPLAY_LIST_EFFECTS_COLOR_FILTERS_DL_MATRIX_COLOR_FILTER_H_
#define FLUTTER_DISPLAY_LIST_EFFECTS_COLOR_FILTERS_DL_MATRIX_COLOR_FILTER_H_

#include "flutter/display_list/effects/dl_color_filter.h"

namespace flutter {

// The Matrix type of ColorFilter which runs every pixel drawn by
// the rendering operation [iR,iG,iB,iA] through a vector/matrix
// multiplication, as in:
//
//  [ oR ]   [ m[ 0] m[ 1] m[ 2] m[ 3] m[ 4] ]   [ iR ]
//  [ oG ]   [ m[ 5] m[ 6] m[ 7] m[ 8] m[ 9] ]   [ iG ]
//  [ oB ] = [ m[10] m[11] m[12] m[13] m[14] ] x [ iB ]
//  [ oA ]   [ m[15] m[16] m[17] m[18] m[19] ]   [ iA ]
//                                               [  1 ]
//
// The resulting color [oR,oG,oB,oA] is then clamped to the range of
// valid pixel components before storing in the output.
//
// The incoming and outgoing [iR,iG,iB,iA] and [oR,oG,oB,oA] are
// considered to be non-premultiplied. When working on premultiplied
// pixel data, the necessary pre<->non-pre conversions must be performed.
class DlMatrixColorFilter final : public DlColorFilter {
 public:
  explicit DlMatrixColorFilter(const float matrix[20]) {
    memcpy(matrix_, matrix, sizeof(matrix_));
  }
  DlMatrixColorFilter(const DlMatrixColorFilter& filter)
      : DlMatrixColorFilter(filter.matrix_) {}
  explicit DlMatrixColorFilter(const DlMatrixColorFilter* filter)
      : DlMatrixColorFilter(filter->matrix_) {}

  DlColorFilterType type() const override { return DlColorFilterType::kMatrix; }
  size_t size() const override { return sizeof(*this); }

  bool modifies_transparent_black() const override;
  bool can_commute_with_opacity() const override;

  std::shared_ptr<DlColorFilter> shared() const override {
    return std::make_shared<DlMatrixColorFilter>(this);
  }

  const DlMatrixColorFilter* asMatrix() const override { return this; }

  const float& operator[](int index) const { return matrix_[index]; }
  void get_matrix(float matrix[20]) const {
    memcpy(matrix, matrix_, sizeof(matrix_));
  }

 protected:
  bool equals_(const DlColorFilter& other) const override {
    FML_DCHECK(other.type() == DlColorFilterType::kMatrix);
    auto that = static_cast<DlMatrixColorFilter const*>(&other);
    return memcmp(matrix_, that->matrix_, sizeof(matrix_)) == 0;
  }

 private:
  static std::shared_ptr<const DlColorFilter> Make(const float matrix[20]);

  float matrix_[20];

  friend class DlColorFilter;
};

}  // namespace flutter

#endif  // FLUTTER_DISPLAY_LIST_EFFECTS_COLOR_FILTERS_DL_MATRIX_COLOR_FILTER_H_
