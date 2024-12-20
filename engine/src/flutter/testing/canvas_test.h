// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_TESTING_CANVAS_TEST_H_
#define FLUTTER_TESTING_CANVAS_TEST_H_

#include "flutter/fml/macros.h"
#include "gtest/gtest.h"
#include "third_party/skia/include/core/SkColorSpace.h"

namespace flutter::testing {

// This fixture allows creating tests that make use of a mock |SkCanvas|.
template <typename BaseT>
class CanvasTestBase : public BaseT {
 public:
  CanvasTestBase() = default;

  sk_sp<SkColorSpace> mock_color_space() { return color_space_; }

 private:
  sk_sp<SkColorSpace> color_space_ = SkColorSpace::MakeSRGB();

  FML_DISALLOW_COPY_AND_ASSIGN(CanvasTestBase);
};
using CanvasTest = CanvasTestBase<::testing::Test>;

}  // namespace flutter::testing

#endif  // FLUTTER_TESTING_CANVAS_TEST_H_
