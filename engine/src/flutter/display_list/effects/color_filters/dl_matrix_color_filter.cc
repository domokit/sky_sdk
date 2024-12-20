// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/effects/color_filters/dl_matrix_color_filter.h"

namespace flutter {

std::shared_ptr<const DlColorFilter> DlMatrixColorFilter::Make(
    const float matrix[20]) {
  float product = 0;
  for (int i = 0; i < 20; i++) {
    product *= matrix[i];
  }
  // If any of the elements of the matrix are infinity or NaN, then
  // |product| will be NaN, otherwise 0.
  if (product == 0) {
    return std::make_shared<DlMatrixColorFilter>(matrix);
  }
  return nullptr;
}

bool DlMatrixColorFilter::modifies_transparent_black() const {
  // Values are considered in non-premultiplied form when the matrix is
  // applied, but we only care about this answer for whether it leaves
  // an incoming color with a transparent alpha as transparent on output.
  // Thus, we only need to consider the alpha part of the matrix equation,
  // which is the last row. Since the incoming alpha value is 0, the last
  // equation ends up becoming A' = matrix_[19]. Negative results will be
  // clamped to the range [0,1] so we only care about positive values.
  // Non-finite values are clamped to a zero alpha.
  return (std::isfinite(matrix_[19]) && matrix_[19] > 0);
}

bool DlMatrixColorFilter::can_commute_with_opacity() const {
  // We need to check if:
  //   filter(color) * opacity == filter(color * opacity).
  //
  // filter(RGBA) = R' = [ R*m[ 0] + G*m[ 1] + B*m[ 2] + A*m[ 3] + m[ 4] ]
  //                G' = [ R*m[ 5] + G*m[ 6] + B*m[ 7] + A*m[ 8] + m[ 9] ]
  //                B' = [ R*m[10] + G*m[11] + B*m[12] + A*m[13] + m[14] ]
  //                A' = [ R*m[15] + G*m[16] + B*m[17] + A*m[18] + m[19] ]
  //
  // Applying the opacity only affects the alpha value since the operations
  // are performed on non-premultiplied colors. (If the data is stored in
  // premultiplied form, though, there may be rounding errors due to
  // premul->unpremul->premul conversions.)

  // We test for the successful cases and return false if they fail so that
  // we fail and return false if any matrix values are NaN.

  // If any of the alpha column are non-zero then the prior alpha affects
  // the result color, so applying opacity before the filter will change
  // the incoming alpha and therefore the colors that are produced.
  if (!(matrix_[3] == 0 &&    // A does not affect R'
        matrix_[8] == 0 &&    // A does not affect G'
        matrix_[13] == 0)) {  // A does not affect B'
    return false;
  }

  // Similarly, if any of the alpha row are non-zero then the prior colors
  // affect the result alpha in a way that prevents opacity from commuting
  // through the filter operation.
  if (!(matrix_[15] == 0 &&   // R does not affect A'
        matrix_[16] == 0 &&   // G does not affect A'
        matrix_[17] == 0 &&   // B does not affect A'
        matrix_[19] == 0)) {  // A' is not offset by an absolute value
    return false;
  }

  return true;
}

}  // namespace flutter
