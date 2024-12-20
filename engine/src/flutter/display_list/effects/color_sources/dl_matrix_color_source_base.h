// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_DISPLAY_LIST_EFFECTS_COLOR_SOURCES_DL_MATRIX_COLOR_SOURCE_BASE_H_
#define FLUTTER_DISPLAY_LIST_EFFECTS_COLOR_SOURCES_DL_MATRIX_COLOR_SOURCE_BASE_H_

#include "flutter/display_list/effects/dl_color_source.h"

namespace flutter {

// Utility base class common to all DlColorSource implementations that
// hold an optional DlMatrix
class DlMatrixColorSourceBase : public DlColorSource {
 public:
  const DlMatrix& matrix() const { return matrix_; }
  const DlMatrix* matrix_ptr() const {
    return matrix_.IsIdentity() ? nullptr : &matrix_;
  }

 protected:
  explicit DlMatrixColorSourceBase(const DlMatrix* matrix)
      : matrix_(matrix ? *matrix : DlMatrix()) {}

 private:
  const DlMatrix matrix_;
};

}  // namespace flutter

#endif  // FLUTTER_DISPLAY_LIST_EFFECTS_COLOR_SOURCES_DL_MATRIX_COLOR_SOURCE_BASE_H_
