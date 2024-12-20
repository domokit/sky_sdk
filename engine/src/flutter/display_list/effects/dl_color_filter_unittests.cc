// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/effects/dl_color_filter.h"

#include "flutter/display_list/effects/dl_color_filters.h"
#include "flutter/display_list/testing/dl_test_equality.h"

namespace flutter {
namespace testing {

static const float kMatrix[20] = {
    1,  2,  3,  4,  5,   //
    6,  7,  8,  9,  10,  //
    11, 12, 13, 14, 15,  //
    16, 17, 18, 19, 20,  //
};

TEST(DisplayListColorFilter, BlendConstructor) {
  DlBlendColorFilter filter(DlColor::kRed(), DlBlendMode::kDstATop);
}

TEST(DisplayListColorFilter, BlendShared) {
  DlBlendColorFilter filter(DlColor::kRed(), DlBlendMode::kDstATop);
  ASSERT_NE(filter.shared().get(), &filter);
  ASSERT_EQ(*filter.shared(), filter);
}

TEST(DisplayListColorFilter, BlendAsBlend) {
  DlBlendColorFilter filter(DlColor::kRed(), DlBlendMode::kDstATop);
  ASSERT_NE(filter.asBlend(), nullptr);
  ASSERT_EQ(filter.asBlend(), &filter);
}

TEST(DisplayListColorFilter, BlendContents) {
  DlBlendColorFilter filter(DlColor::kRed(), DlBlendMode::kDstATop);
  ASSERT_EQ(filter.color(), DlColor::kRed());
  ASSERT_EQ(filter.mode(), DlBlendMode::kDstATop);
}

TEST(DisplayListColorFilter, BlendEquals) {
  DlBlendColorFilter filter1(DlColor::kRed(), DlBlendMode::kDstATop);
  DlBlendColorFilter filter2(DlColor::kRed(), DlBlendMode::kDstATop);
  TestEquals(filter1, filter2);
}

TEST(DisplayListColorFilter, BlendNotEquals) {
  DlBlendColorFilter filter1(DlColor::kRed(), DlBlendMode::kDstATop);
  DlBlendColorFilter filter2(DlColor::kBlue(), DlBlendMode::kDstATop);
  DlBlendColorFilter filter3(DlColor::kRed(), DlBlendMode::kDstIn);
  TestNotEquals(filter1, filter2, "Color differs");
  TestNotEquals(filter1, filter3, "Blend mode differs");
}

TEST(DisplayListColorFilter, NopBlendShouldNotCrash) {
  DlBlendColorFilter filter(DlColor::kTransparent(), DlBlendMode::kSrcOver);
  ASSERT_FALSE(filter.modifies_transparent_black());
}

TEST(DisplayListColorFilter, MatrixConstructor) {
  DlMatrixColorFilter filter(kMatrix);
}

TEST(DisplayListColorFilter, MatrixShared) {
  DlMatrixColorFilter filter(kMatrix);
  ASSERT_NE(filter.shared().get(), &filter);
  ASSERT_EQ(*filter.shared(), filter);
}

TEST(DisplayListColorFilter, MatrixAsMatrix) {
  DlMatrixColorFilter filter(kMatrix);
  ASSERT_NE(filter.asMatrix(), nullptr);
  ASSERT_EQ(filter.asMatrix(), &filter);
}

TEST(DisplayListColorFilter, MatrixContents) {
  float matrix[20];
  memcpy(matrix, kMatrix, sizeof(matrix));
  DlMatrixColorFilter filter(matrix);

  // Test deref operator []
  for (int i = 0; i < 20; i++) {
    ASSERT_EQ(filter[i], matrix[i]);
  }

  // Test get_matrix
  float matrix2[20];
  filter.get_matrix(matrix2);
  for (int i = 0; i < 20; i++) {
    ASSERT_EQ(matrix2[i], matrix[i]);
  }

  // Test perturbing original array does not affect filter
  float original_value = matrix[4];
  matrix[4] += 101;
  ASSERT_EQ(filter[4], original_value);
}

TEST(DisplayListColorFilter, MatrixEquals) {
  DlMatrixColorFilter filter1(kMatrix);
  DlMatrixColorFilter filter2(kMatrix);
  TestEquals(filter1, filter2);
}

TEST(DisplayListColorFilter, MatrixNotEquals) {
  float matrix[20];
  memcpy(matrix, kMatrix, sizeof(matrix));
  DlMatrixColorFilter filter1(matrix);
  matrix[4] += 101;
  DlMatrixColorFilter filter2(matrix);
  TestNotEquals(filter1, filter2, "Matrix differs");
}

TEST(DisplayListColorFilter, NopMatrixShouldNotCrash) {
  float matrix[20] = {
      1, 0, 0, 0, 0,  //
      0, 1, 0, 0, 0,  //
      0, 0, 1, 0, 0,  //
      0, 0, 0, 1, 0,  //
  };
  DlMatrixColorFilter filter(matrix);
  ASSERT_FALSE(filter.modifies_transparent_black());
}

TEST(DisplayListColorFilter, SrgbToLinearConstructor) {
  DlSrgbToLinearGammaColorFilter filter;
}

TEST(DisplayListColorFilter, SrgbToLinearShared) {
  DlSrgbToLinearGammaColorFilter filter;
  ASSERT_NE(filter.shared().get(), &filter);
  ASSERT_EQ(*filter.shared(), filter);
}

TEST(DisplayListColorFilter, SrgbToLinearEquals) {
  DlSrgbToLinearGammaColorFilter filter1;
  DlSrgbToLinearGammaColorFilter filter2;
  TestEquals(filter1, filter2);
  TestEquals(filter1, *DlColorFilter::MakeSrgbToLinearGamma());
}

TEST(DisplayListColorFilter, LinearToSrgbConstructor) {
  DlLinearToSrgbGammaColorFilter filter;
}

TEST(DisplayListColorFilter, LinearToSrgbShared) {
  DlLinearToSrgbGammaColorFilter filter;
  ASSERT_NE(filter.shared().get(), &filter);
  ASSERT_EQ(*filter.shared(), filter);
}

TEST(DisplayListColorFilter, LinearToSrgbEquals) {
  DlLinearToSrgbGammaColorFilter filter1;
  DlLinearToSrgbGammaColorFilter filter2;
  TestEquals(filter1, filter2);
  TestEquals(filter1, *DlColorFilter::MakeLinearToSrgbGamma());
}

}  // namespace testing
}  // namespace flutter
