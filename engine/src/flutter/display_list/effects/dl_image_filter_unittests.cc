// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/dl_blend_mode.h"
#include "flutter/display_list/dl_builder.h"
#include "flutter/display_list/dl_color.h"
#include "flutter/display_list/dl_sampling_options.h"
#include "flutter/display_list/dl_tile_mode.h"
#include "flutter/display_list/effects/dl_color_filters.h"
#include "flutter/display_list/effects/dl_image_filters.h"
#include "flutter/display_list/testing/dl_test_equality.h"
#include "flutter/display_list/utils/dl_comparable.h"
#include "flutter/testing/display_list_testing.h"
#include "gtest/gtest.h"

#include "include/core/SkMatrix.h"
#include "include/core/SkRect.h"
#include "third_party/skia/include/core/SkBlendMode.h"
#include "third_party/skia/include/core/SkColorFilter.h"
#include "third_party/skia/include/core/SkSamplingOptions.h"
#include "third_party/skia/include/effects/SkImageFilters.h"

namespace flutter {
namespace testing {

// SkRect::contains treats the rect as a half-open interval which is
// appropriate for so many operations. Unfortunately, we are using
// it here to test containment of the corners of a transformed quad
// so the corners of the quad that are measured against the right
// and bottom edges are contained even if they are on the right or
// bottom edge. This method does the "all sides inclusive" version
// of SkRect::contains.
static bool containsInclusive(const DlRect rect, const DlPoint p) {
  // Test with a slight offset of 1E-9 to "forgive" IEEE bit-rounding
  // Ending up with bounds that are off by 1E-9 (these numbers are all
  // being tested in device space with this method) will be off by a
  // negligible amount of a pixel that wouldn't contribute to changing
  // the color of a pixel.
  return (p.x >= rect.GetLeft() - 1E-9 &&   //
          p.x <= rect.GetRight() + 1E-9 &&  //
          p.y >= rect.GetTop() - 1E-9 &&    //
          p.y <= rect.GetBottom() + 1E-9);
}

static bool containsInclusive(const DlRect rect, const DlQuad quad) {
  return (containsInclusive(rect, quad[0]) &&  //
          containsInclusive(rect, quad[1]) &&  //
          containsInclusive(rect, quad[2]) &&  //
          containsInclusive(rect, quad[3]));
}

static bool containsInclusive(const DlIRect rect, const DlQuad quad) {
  return containsInclusive(DlRect::Make(rect), quad);
}

static bool containsInclusive(const DlIRect rect, const DlRect bounds) {
  return (bounds.GetLeft() >= rect.GetLeft() - 1E-9 &&
          bounds.GetTop() >= rect.GetTop() - 1E-9 &&
          bounds.GetRight() <= rect.GetRight() + 1E-9 &&
          bounds.GetBottom() <= rect.GetBottom() + 1E-9);
}

// Used to verify that the expected output bounds and reverse-engineered
// "input bounds for output bounds" rectangles are included in the rectangle
// returned from the various bounds computation methods under the specified
// matrix.
static void TestBoundsWithMatrix(const DlImageFilter& filter,
                                 const DlMatrix& matrix,
                                 const DlRect& sourceBounds,
                                 const DlQuad& expectedLocalOutputQuad) {
  DlRect device_input_bounds = sourceBounds.TransformAndClipBounds(matrix);
  DlQuad expected_output_quad = matrix.Transform(expectedLocalOutputQuad);

  DlIRect device_filter_ibounds;
  ASSERT_EQ(filter.map_device_bounds(DlIRect::RoundOut(device_input_bounds),
                                     matrix, device_filter_ibounds),
            &device_filter_ibounds);
  EXPECT_TRUE(containsInclusive(device_filter_ibounds, expected_output_quad))
      << filter << std::endl
      << sourceBounds << ", {" << std::endl
      << "  " << expectedLocalOutputQuad[0] << ", " << std::endl
      << "  " << expectedLocalOutputQuad[1] << ", " << std::endl
      << "  " << expectedLocalOutputQuad[2] << ", " << std::endl
      << "  " << expectedLocalOutputQuad[3] << std::endl
      << "}, " << matrix << ", " << std::endl
      << device_filter_ibounds << std::endl
      << device_input_bounds << ", {" << std::endl
      << "  " << expected_output_quad[0] << ", " << std::endl
      << "  " << expected_output_quad[1] << ", " << std::endl
      << "  " << expected_output_quad[2] << ", " << std::endl
      << "  " << expected_output_quad[3] << std::endl
      << "}";

  DlIRect reverse_input_ibounds;
  ASSERT_EQ(filter.get_input_device_bounds(device_filter_ibounds, matrix,
                                           reverse_input_ibounds),
            &reverse_input_ibounds);
  EXPECT_TRUE(containsInclusive(reverse_input_ibounds, device_input_bounds))
      << filter << std::endl
      << matrix << ", " << std::endl
      << reverse_input_ibounds << ", " << std::endl
      << device_input_bounds;
}

static void TestInvalidBounds(const DlImageFilter& filter,
                              const DlMatrix& matrix,
                              const DlRect& localInputBounds) {
  DlIRect device_input_bounds =
      DlIRect::RoundOut(localInputBounds.TransformBounds(matrix));

  DlRect local_filter_bounds;
  ASSERT_EQ(filter.map_local_bounds(localInputBounds, local_filter_bounds),
            nullptr);
  ASSERT_EQ(local_filter_bounds, localInputBounds);

  DlIRect device_filter_ibounds;
  ASSERT_EQ(filter.map_device_bounds(device_input_bounds, matrix,
                                     device_filter_ibounds),
            nullptr);
  ASSERT_EQ(device_filter_ibounds, device_input_bounds);

  DlIRect reverse_input_ibounds;
  ASSERT_EQ(filter.get_input_device_bounds(device_input_bounds, matrix,
                                           reverse_input_ibounds),
            nullptr);
  ASSERT_EQ(reverse_input_ibounds, device_input_bounds);
}

// localInputBounds is a sample bounds for testing as input to the filter.
// localExpectOutputBounds is the theoretical output bounds for applying
// the filter to the localInputBounds.
// localExpectInputBounds is the theoretical input bounds required for the
// filter to cover the localExpectOutputBounds
// If either of the expected bounds are nullptr then the bounds methods will
// be assumed to be unable to perform their computations for the given
// image filter and will be returning null.
static void TestBounds(const DlImageFilter& filter,
                       const DlRect& sourceBounds,
                       const DlQuad& expectedLocalOutputQuad) {
  DlRect local_filter_bounds;
  ASSERT_EQ(filter.map_local_bounds(sourceBounds, local_filter_bounds),
            &local_filter_bounds);
  ASSERT_TRUE(containsInclusive(local_filter_bounds, expectedLocalOutputQuad));

  for (int i_scale = 1; i_scale <= 4; i_scale++) {
    DlScalar scale = i_scale;
    for (int skew_eighths = 0; skew_eighths < 7; skew_eighths++) {
      DlScalar skew = skew_eighths / 8.0f;
      for (int degrees = 0; degrees <= 360; degrees += 15) {
        DlMatrix matrix;
        matrix = matrix.Scale({scale, scale, 1});
        matrix = DlMatrix::MakeSkew(skew, skew) * matrix;
        matrix = DlMatrix::MakeRotationZ(DlDegrees(degrees)) * matrix;
        ASSERT_TRUE(matrix.IsInvertible()) << matrix;
        ASSERT_FALSE(matrix.HasPerspective2D()) << matrix;
        TestBoundsWithMatrix(filter, matrix, sourceBounds,
                             expectedLocalOutputQuad);
        matrix.m[3] = 0.001f;
        matrix.m[7] = 0.001f;
        ASSERT_TRUE(matrix.IsInvertible()) << matrix;
        ASSERT_TRUE(matrix.HasPerspective2D()) << matrix;
        TestBoundsWithMatrix(filter, matrix, sourceBounds,
                             expectedLocalOutputQuad);
      }
    }
  }
}

static void TestBounds(const DlImageFilter& filter,
                       const DlRect& sourceBounds,
                       const DlRect& expectedLocalOutputBounds) {
  DlQuad expected_local_output_quad = expectedLocalOutputBounds.GetPoints();
  ASSERT_EQ(expected_local_output_quad.size(), 4u);  // Only 0u when empty
  TestBounds(filter, sourceBounds, expected_local_output_quad);
}

TEST(DisplayListImageFilter, BlurConstructor) {
  DlBlurImageFilter filter(5.0, 6.0, DlTileMode::kMirror);
}

TEST(DisplayListImageFilter, BlurShared) {
  DlBlurImageFilter filter(5.0, 6.0, DlTileMode::kMirror);

  ASSERT_NE(filter.shared().get(), &filter);
  ASSERT_EQ(*filter.shared(), filter);
}

TEST(DisplayListImageFilter, BlurAsBlur) {
  DlBlurImageFilter filter(5.0, 6.0, DlTileMode::kMirror);

  ASSERT_NE(filter.asBlur(), nullptr);
  ASSERT_EQ(filter.asBlur(), &filter);
}

TEST(DisplayListImageFilter, BlurContents) {
  DlBlurImageFilter filter(5.0, 6.0, DlTileMode::kMirror);

  ASSERT_EQ(filter.sigma_x(), 5.0);
  ASSERT_EQ(filter.sigma_y(), 6.0);
  ASSERT_EQ(filter.tile_mode(), DlTileMode::kMirror);
}

TEST(DisplayListImageFilter, BlurEquals) {
  DlBlurImageFilter filter1(5.0, 6.0, DlTileMode::kMirror);
  DlBlurImageFilter filter2(5.0, 6.0, DlTileMode::kMirror);

  TestEquals(filter1, filter2);
}

TEST(DisplayListImageFilter, BlurWithLocalMatrixEquals) {
  DlBlurImageFilter filter1(5.0, 6.0, DlTileMode::kMirror);
  DlBlurImageFilter filter2(5.0, 6.0, DlTileMode::kMirror);

  DlMatrix local_matrix = DlMatrix::MakeTranslation({10, 10});
  TestEquals(*filter1.makeWithLocalMatrix(local_matrix),
             *filter2.makeWithLocalMatrix(local_matrix));
}

TEST(DisplayListImageFilter, BlurNotEquals) {
  DlBlurImageFilter filter1(5.0, 6.0, DlTileMode::kMirror);
  DlBlurImageFilter filter2(7.0, 6.0, DlTileMode::kMirror);
  DlBlurImageFilter filter3(5.0, 8.0, DlTileMode::kMirror);
  DlBlurImageFilter filter4(5.0, 6.0, DlTileMode::kRepeat);

  TestNotEquals(filter1, filter2, "Sigma X differs");
  TestNotEquals(filter1, filter3, "Sigma Y differs");
  TestNotEquals(filter1, filter4, "Tile Mode differs");
}

TEST(DisplayListImageFilter, BlurBounds) {
  DlBlurImageFilter filter = DlBlurImageFilter(5, 10, DlTileMode::kDecal);
  DlRect input_bounds = DlRect::MakeLTRB(20, 20, 80, 80);
  DlRect expected_output_bounds = input_bounds.Expand(15, 30);
  TestBounds(filter, input_bounds, expected_output_bounds);
}

TEST(DisplayListImageFilter, BlurZeroSigma) {
  std::shared_ptr<DlImageFilter> filter =
      DlImageFilter::MakeBlur(0, 0, DlTileMode::kMirror);
  ASSERT_EQ(filter, nullptr);
  filter = DlImageFilter::MakeBlur(3, SK_ScalarNaN, DlTileMode::kMirror);
  ASSERT_EQ(filter, nullptr);
  filter = DlImageFilter::MakeBlur(SK_ScalarNaN, 3, DlTileMode::kMirror);
  ASSERT_EQ(filter, nullptr);
  filter =
      DlImageFilter::MakeBlur(SK_ScalarNaN, SK_ScalarNaN, DlTileMode::kMirror);
  ASSERT_EQ(filter, nullptr);
  filter = DlImageFilter::MakeBlur(3, 0, DlTileMode::kMirror);
  ASSERT_NE(filter, nullptr);
  filter = DlImageFilter::MakeBlur(0, 3, DlTileMode::kMirror);
  ASSERT_NE(filter, nullptr);
}

TEST(DisplayListImageFilter, DilateConstructor) {
  DlDilateImageFilter filter(5.0, 6.0);
}

TEST(DisplayListImageFilter, DilateShared) {
  DlDilateImageFilter filter(5.0, 6.0);

  ASSERT_NE(filter.shared().get(), &filter);
  ASSERT_EQ(*filter.shared(), filter);
}

TEST(DisplayListImageFilter, DilateAsDilate) {
  DlDilateImageFilter filter(5.0, 6.0);

  ASSERT_NE(filter.asDilate(), nullptr);
  ASSERT_EQ(filter.asDilate(), &filter);
}

TEST(DisplayListImageFilter, DilateContents) {
  DlDilateImageFilter filter(5.0, 6.0);

  ASSERT_EQ(filter.radius_x(), 5.0);
  ASSERT_EQ(filter.radius_y(), 6.0);
}

TEST(DisplayListImageFilter, DilateEquals) {
  DlDilateImageFilter filter1(5.0, 6.0);
  DlDilateImageFilter filter2(5.0, 6.0);

  TestEquals(filter1, filter2);
}

TEST(DisplayListImageFilter, DilateWithLocalMatrixEquals) {
  DlDilateImageFilter filter1(5.0, 6.0);
  DlDilateImageFilter filter2(5.0, 6.0);

  DlMatrix local_matrix = DlMatrix::MakeTranslation({10, 10});
  TestEquals(*filter1.makeWithLocalMatrix(local_matrix),
             *filter2.makeWithLocalMatrix(local_matrix));
}

TEST(DisplayListImageFilter, DilateNotEquals) {
  DlDilateImageFilter filter1(5.0, 6.0);
  DlDilateImageFilter filter2(7.0, 6.0);
  DlDilateImageFilter filter3(5.0, 8.0);

  TestNotEquals(filter1, filter2, "Radius X differs");
  TestNotEquals(filter1, filter3, "Radius Y differs");
}

TEST(DisplayListImageFilter, DilateBounds) {
  DlDilateImageFilter filter = DlDilateImageFilter(5, 10);
  DlRect input_bounds = DlRect::MakeLTRB(20, 20, 80, 80);
  DlRect expected_output_bounds = input_bounds.Expand(5, 10);
  TestBounds(filter, input_bounds, expected_output_bounds);
}

TEST(DisplayListImageFilter, ErodeConstructor) {
  DlErodeImageFilter filter(5.0, 6.0);
}

TEST(DisplayListImageFilter, ErodeShared) {
  DlErodeImageFilter filter(5.0, 6.0);

  ASSERT_NE(filter.shared().get(), &filter);
  ASSERT_EQ(*filter.shared(), filter);
}

TEST(DisplayListImageFilter, ErodeAsErode) {
  DlErodeImageFilter filter(5.0, 6.0);

  ASSERT_NE(filter.asErode(), nullptr);
  ASSERT_EQ(filter.asErode(), &filter);
}

TEST(DisplayListImageFilter, ErodeContents) {
  DlErodeImageFilter filter(5.0, 6.0);

  ASSERT_EQ(filter.radius_x(), 5.0);
  ASSERT_EQ(filter.radius_y(), 6.0);
}

TEST(DisplayListImageFilter, ErodeEquals) {
  DlErodeImageFilter filter1(5.0, 6.0);
  DlErodeImageFilter filter2(5.0, 6.0);

  TestEquals(filter1, filter2);
}

TEST(DisplayListImageFilter, ErodeWithLocalMatrixEquals) {
  DlErodeImageFilter filter1(5.0, 6.0);
  DlErodeImageFilter filter2(5.0, 6.0);

  DlMatrix local_matrix = DlMatrix::MakeTranslation({10, 10});
  TestEquals(*filter1.makeWithLocalMatrix(local_matrix),
             *filter2.makeWithLocalMatrix(local_matrix));
}

TEST(DisplayListImageFilter, ErodeNotEquals) {
  DlErodeImageFilter filter1(5.0, 6.0);
  DlErodeImageFilter filter2(7.0, 6.0);
  DlErodeImageFilter filter3(5.0, 8.0);

  TestNotEquals(filter1, filter2, "Radius X differs");
  TestNotEquals(filter1, filter3, "Radius Y differs");
}

TEST(DisplayListImageFilter, ErodeBounds) {
  DlErodeImageFilter filter = DlErodeImageFilter(5, 10);
  DlRect input_bounds = DlRect::MakeLTRB(20, 20, 80, 80);
  DlRect expected_output_bounds = input_bounds.Expand(-5, -10);
  TestBounds(filter, input_bounds, expected_output_bounds);
}

TEST(DisplayListImageFilter, MatrixConstructor) {
  DlMatrixImageFilter filter(DlMatrix::MakeRow(2.0, 0.0, 0.0, 10,   //
                                               0.5, 3.0, 0.0, 15,   //
                                               0.0, 0.0, 1.0, 0.0,  //
                                               0.0, 0.0, 0.0, 1.0),
                             DlImageSampling::kLinear);
}

TEST(DisplayListImageFilter, MatrixShared) {
  DlMatrixImageFilter filter(DlMatrix::MakeRow(2.0, 0.0, 0.0, 10,   //
                                               0.5, 3.0, 0.0, 15,   //
                                               0.0, 0.0, 1.0, 0.0,  //
                                               0.0, 0.0, 0.0, 1.0),
                             DlImageSampling::kLinear);

  ASSERT_NE(filter.shared().get(), &filter);
  ASSERT_EQ(*filter.shared(), filter);
}

TEST(DisplayListImageFilter, MatrixAsMatrix) {
  DlMatrixImageFilter filter(DlMatrix::MakeRow(2.0, 0.0, 0.0, 10,   //
                                               0.5, 3.0, 0.0, 15,   //
                                               0.0, 0.0, 1.0, 0.0,  //
                                               0.0, 0.0, 0.0, 1.0),
                             DlImageSampling::kLinear);

  ASSERT_NE(filter.asMatrix(), nullptr);
  ASSERT_EQ(filter.asMatrix(), &filter);
}

TEST(DisplayListImageFilter, MatrixContents) {
  DlMatrix matrix = DlMatrix::MakeRow(2.0, 0.0, 0.0, 10,   //
                                      0.5, 3.0, 0.0, 15,   //
                                      0.0, 0.0, 1.0, 0.0,  //
                                      0.0, 0.0, 0.0, 1.0);
  DlMatrixImageFilter filter(matrix, DlImageSampling::kLinear);

  ASSERT_EQ(filter.matrix(), matrix);
  ASSERT_EQ(filter.sampling(), DlImageSampling::kLinear);
}

TEST(DisplayListImageFilter, MatrixEquals) {
  DlMatrix matrix = DlMatrix::MakeRow(2.0, 0.0, 0.0, 10,   //
                                      0.5, 3.0, 0.0, 15,   //
                                      0.0, 0.0, 1.0, 0.0,  //
                                      0.0, 0.0, 0.0, 1.0);
  DlMatrixImageFilter filter1(matrix, DlImageSampling::kLinear);
  DlMatrixImageFilter filter2(matrix, DlImageSampling::kLinear);

  TestEquals(filter1, filter2);
}

TEST(DisplayListImageFilter, MatrixWithLocalMatrixEquals) {
  DlMatrix matrix = DlMatrix::MakeRow(2.0, 0.0, 0.0, 10,   //
                                      0.5, 3.0, 0.0, 15,   //
                                      0.0, 0.0, 1.0, 0.0,  //
                                      0.0, 0.0, 0.0, 1.0);
  DlMatrixImageFilter filter1(matrix, DlImageSampling::kLinear);
  DlMatrixImageFilter filter2(matrix, DlImageSampling::kLinear);

  DlMatrix local_matrix = DlMatrix::MakeTranslation({10, 10});
  TestEquals(*filter1.makeWithLocalMatrix(local_matrix),
             *filter2.makeWithLocalMatrix(local_matrix));
}

TEST(DisplayListImageFilter, MatrixNotEquals) {
  DlMatrix matrix1 = DlMatrix::MakeRow(2.0, 0.0, 0.0, 10,   //
                                       0.5, 3.0, 0.0, 15,   //
                                       0.0, 0.0, 1.0, 0.0,  //
                                       0.0, 0.0, 0.0, 1.0);
  DlMatrix matrix2 = DlMatrix::MakeRow(5.0, 0.0, 0.0, 10,   //
                                       0.5, 3.0, 0.0, 15,   //
                                       0.0, 0.0, 1.0, 0.0,  //
                                       0.0, 0.0, 0.0, 1.0);
  DlMatrixImageFilter filter1(matrix1, DlImageSampling::kLinear);
  DlMatrixImageFilter filter2(matrix2, DlImageSampling::kLinear);
  DlMatrixImageFilter filter3(matrix1, DlImageSampling::kNearestNeighbor);

  TestNotEquals(filter1, filter2, "Matrix differs");
  TestNotEquals(filter1, filter3, "Sampling differs");
}

TEST(DisplayListImageFilter, MatrixBounds) {
  DlMatrix matrix = DlMatrix::MakeRow(2.0, 0.0, 0.0, 10,   //
                                      0.5, 3.0, 0.0, 7,    //
                                      0.0, 0.0, 1.0, 0.0,  //
                                      0.0, 0.0, 0.0, 1.0);
  EXPECT_TRUE(matrix.IsInvertible());
  DlMatrixImageFilter filter(matrix, DlImageSampling::kLinear);
  DlRect input_bounds = DlRect::MakeLTRB(20, 20, 80, 80);
  DlQuad expectedOutputQuad = {
      DlPoint(50, 77),    // == (20,20) => (20*2 + 10, 20/2 + 20*3 + 7)
      DlPoint(50, 257),   // == (20,80) => (20*2 + 10, 20/2 + 80*3 + 7)
      DlPoint(170, 287),  // == (80,80) => (80*2 + 10, 80/2 + 80*3 + 7)
      DlPoint(170, 107),  // == (80,20) => (80*2 + 10, 80/2 + 20*3 + 7)
  };
  TestBounds(filter, input_bounds, expectedOutputQuad);
}

TEST(DisplayListImageFilter, ComposeConstructor) {
  DlMatrixImageFilter outer(DlMatrix::MakeRow(2.0, 0.0, 0.0, 10,   //
                                              0.5, 3.0, 0.0, 15,   //
                                              0.0, 0.0, 1.0, 0.0,  //
                                              0.0, 0.0, 0.0, 1.0),
                            DlImageSampling::kLinear);
  DlBlurImageFilter inner(5.0, 6.0, DlTileMode::kMirror);
  DlComposeImageFilter filter(outer, inner);
}

TEST(DisplayListImageFilter, ComposeShared) {
  DlMatrixImageFilter outer(DlMatrix::MakeRow(2.0, 0.0, 0.0, 10,   //
                                              0.5, 3.0, 0.0, 15,   //
                                              0.0, 0.0, 1.0, 0.0,  //
                                              0.0, 0.0, 0.0, 1.0),
                            DlImageSampling::kLinear);
  DlBlurImageFilter inner(5.0, 6.0, DlTileMode::kMirror);
  DlComposeImageFilter filter(outer, inner);

  ASSERT_NE(filter.shared().get(), &filter);
  ASSERT_EQ(*filter.shared(), filter);
}

TEST(DisplayListImageFilter, ComposeAsCompose) {
  DlMatrixImageFilter outer(DlMatrix::MakeRow(2.0, 0.0, 0.0, 10,   //
                                              0.5, 3.0, 0.0, 15,   //
                                              0.0, 0.0, 1.0, 0.0,  //
                                              0.0, 0.0, 0.0, 1.0),
                            DlImageSampling::kLinear);
  DlBlurImageFilter inner(5.0, 6.0, DlTileMode::kMirror);
  DlComposeImageFilter filter(outer, inner);

  ASSERT_NE(filter.asCompose(), nullptr);
  ASSERT_EQ(filter.asCompose(), &filter);
}

TEST(DisplayListImageFilter, ComposeContents) {
  DlMatrixImageFilter outer(DlMatrix::MakeRow(2.0, 0.0, 0.0, 10,   //
                                              0.5, 3.0, 0.0, 15,   //
                                              0.0, 0.0, 1.0, 0.0,  //
                                              0.0, 0.0, 0.0, 1.0),
                            DlImageSampling::kLinear);
  DlBlurImageFilter inner(5.0, 6.0, DlTileMode::kMirror);
  DlComposeImageFilter filter(outer, inner);

  ASSERT_EQ(*filter.outer().get(), outer);
  ASSERT_EQ(*filter.inner().get(), inner);
}

TEST(DisplayListImageFilter, ComposeEquals) {
  DlMatrixImageFilter outer1(DlMatrix::MakeRow(2.0, 0.0, 0.0, 10,   //
                                               0.5, 3.0, 0.0, 15,   //
                                               0.0, 0.0, 1.0, 0.0,  //
                                               0.0, 0.0, 0.0, 1.0),
                             DlImageSampling::kLinear);
  DlBlurImageFilter inner1(5.0, 6.0, DlTileMode::kMirror);
  DlComposeImageFilter filter1(outer1, inner1);

  DlMatrixImageFilter outer2(DlMatrix::MakeRow(2.0, 0.0, 0.0, 10,   //
                                               0.5, 3.0, 0.0, 15,   //
                                               0.0, 0.0, 1.0, 0.0,  //
                                               0.0, 0.0, 0.0, 1.0),
                             DlImageSampling::kLinear);
  DlBlurImageFilter inner2(5.0, 6.0, DlTileMode::kMirror);
  DlComposeImageFilter filter2(outer1, inner1);

  TestEquals(filter1, filter2);
}

TEST(DisplayListImageFilter, ComposeWithLocalMatrixEquals) {
  DlMatrixImageFilter outer1(DlMatrix::MakeRow(2.0, 0.0, 0.0, 10,   //
                                               0.5, 3.0, 0.0, 15,   //
                                               0.0, 0.0, 1.0, 0.0,  //
                                               0.0, 0.0, 0.0, 1.0),
                             DlImageSampling::kLinear);
  DlBlurImageFilter inner1(5.0, 6.0, DlTileMode::kMirror);
  DlComposeImageFilter filter1(outer1, inner1);

  DlMatrixImageFilter outer2(DlMatrix::MakeRow(2.0, 0.0, 0.0, 10,   //
                                               0.5, 3.0, 0.0, 15,   //
                                               0.0, 0.0, 1.0, 0.0,  //
                                               0.0, 0.0, 0.0, 1.0),
                             DlImageSampling::kLinear);
  DlBlurImageFilter inner2(5.0, 6.0, DlTileMode::kMirror);
  DlComposeImageFilter filter2(outer1, inner1);

  DlMatrix local_matrix = DlMatrix::MakeTranslation({10, 10});
  TestEquals(*filter1.makeWithLocalMatrix(local_matrix),
             *filter2.makeWithLocalMatrix(local_matrix));
}

TEST(DisplayListImageFilter, ComposeNotEquals) {
  DlMatrixImageFilter outer1(DlMatrix::MakeRow(2.0, 0.0, 0.0, 10,   //
                                               0.5, 3.0, 0.0, 15,   //
                                               0.0, 0.0, 1.0, 0.0,  //
                                               0.0, 0.0, 0.0, 1.0),
                             DlImageSampling::kLinear);
  DlBlurImageFilter inner1(5.0, 6.0, DlTileMode::kMirror);

  DlMatrixImageFilter outer2(DlMatrix::MakeRow(5.0, 0.0, 0.0, 10,   //
                                               0.5, 3.0, 0.0, 15,   //
                                               0.0, 0.0, 1.0, 0.0,  //
                                               0.0, 0.0, 0.0, 1.0),
                             DlImageSampling::kLinear);
  DlBlurImageFilter inner2(7.0, 6.0, DlTileMode::kMirror);

  DlComposeImageFilter filter1(outer1, inner1);
  DlComposeImageFilter filter2(outer2, inner1);
  DlComposeImageFilter filter3(outer1, inner2);

  TestNotEquals(filter1, filter2, "Outer differs");
  TestNotEquals(filter1, filter3, "Inner differs");
}

TEST(DisplayListImageFilter, ComposeBounds) {
  DlDilateImageFilter outer = DlDilateImageFilter(5, 10);
  DlBlurImageFilter inner = DlBlurImageFilter(12, 5, DlTileMode::kDecal);
  DlComposeImageFilter filter = DlComposeImageFilter(outer, inner);
  DlRect input_bounds = DlRect::MakeLTRB(20, 20, 80, 80);
  DlRect expected_output_bounds = input_bounds.Expand(36, 15).Expand(5, 10);
  TestBounds(filter, input_bounds, expected_output_bounds);
}

static void TestUnboundedBounds(DlImageFilter& filter,
                                const DlRect& sourceBounds,
                                const DlRect& expectedOutputBounds,
                                const DlRect& expectedInputBounds) {
  DlRect bounds;
  EXPECT_EQ(filter.map_local_bounds(sourceBounds, bounds), nullptr);
  EXPECT_EQ(bounds, expectedOutputBounds);

  DlIRect ibounds;
  EXPECT_EQ(filter.map_device_bounds(DlIRect::RoundOut(sourceBounds),
                                     DlMatrix(), ibounds),
            nullptr);
  EXPECT_EQ(ibounds, DlIRect::RoundOut(expectedOutputBounds));

  EXPECT_EQ(filter.get_input_device_bounds(DlIRect::RoundOut(sourceBounds),
                                           DlMatrix(), ibounds),
            nullptr);
  EXPECT_EQ(ibounds, DlIRect::RoundOut(expectedInputBounds));
}

TEST(DisplayListImageFilter, ComposeBoundsWithUnboundedInner) {
  auto input_bounds = DlRect::MakeLTRB(20, 20, 80, 80);
  auto expected_bounds = DlRect::MakeLTRB(5, 2, 95, 98);

  DlBlendColorFilter color_filter(DlColor::kRed(), DlBlendMode::kSrcOver);
  auto outer = DlBlurImageFilter(5.0, 6.0, DlTileMode::kRepeat);
  auto inner = DlColorFilterImageFilter(color_filter.shared());
  auto composed = DlComposeImageFilter(outer.shared(), inner.shared());

  TestUnboundedBounds(composed, input_bounds, expected_bounds, expected_bounds);
}

TEST(DisplayListImageFilter, ComposeBoundsWithUnboundedOuter) {
  auto input_bounds = DlRect::MakeLTRB(20, 20, 80, 80);
  auto expected_bounds = DlRect::MakeLTRB(5, 2, 95, 98);

  DlBlendColorFilter color_filter(DlColor::kRed(), DlBlendMode::kSrcOver);
  auto outer = DlColorFilterImageFilter(color_filter.shared());
  auto inner = DlBlurImageFilter(5.0, 6.0, DlTileMode::kRepeat);
  auto composed = DlComposeImageFilter(outer.shared(), inner.shared());

  TestUnboundedBounds(composed, input_bounds, expected_bounds, expected_bounds);
}

TEST(DisplayListImageFilter, ComposeBoundsWithUnboundedInnerAndOuter) {
  auto input_bounds = DlRect::MakeLTRB(20, 20, 80, 80);
  auto expected_bounds = input_bounds;

  DlBlendColorFilter color_filter1(DlColor::kRed(), DlBlendMode::kSrcOver);
  DlBlendColorFilter color_filter2(DlColor::kBlue(), DlBlendMode::kSrcOver);
  auto outer = DlColorFilterImageFilter(color_filter1.shared());
  auto inner = DlColorFilterImageFilter(color_filter2.shared());
  auto composed = DlComposeImageFilter(outer.shared(), inner.shared());

  TestUnboundedBounds(composed, input_bounds, expected_bounds, expected_bounds);
}

// See https://github.com/flutter/flutter/issues/108433
TEST(DisplayListImageFilter, Issue108433) {
  auto input_bounds = DlIRect::MakeLTRB(20, 20, 80, 80);
  auto expected_bounds = DlIRect::MakeLTRB(5, 2, 95, 98);

  DlBlendColorFilter dl_color_filter(DlColor::kRed(), DlBlendMode::kSrcOver);
  auto dl_outer = DlBlurImageFilter(5.0, 6.0, DlTileMode::kRepeat);
  auto dl_inner = DlColorFilterImageFilter(dl_color_filter.shared());
  auto dl_compose = DlComposeImageFilter(dl_outer, dl_inner);

  DlIRect dl_bounds;
  ASSERT_EQ(dl_compose.map_device_bounds(input_bounds, DlMatrix(), dl_bounds),
            nullptr);
  ASSERT_EQ(dl_bounds, expected_bounds);
}

TEST(DisplayListImageFilter, ColorFilterConstructor) {
  DlBlendColorFilter dl_color_filter(DlColor::kRed(), DlBlendMode::kLighten);
  DlColorFilterImageFilter filter(dl_color_filter);
}

TEST(DisplayListImageFilter, ColorFilterShared) {
  DlBlendColorFilter dl_color_filter(DlColor::kRed(), DlBlendMode::kLighten);
  DlColorFilterImageFilter filter(dl_color_filter);

  ASSERT_EQ(*filter.shared(), filter);
}

TEST(DisplayListImageFilter, ColorFilterAsColorFilter) {
  DlBlendColorFilter dl_color_filter(DlColor::kRed(), DlBlendMode::kLighten);
  DlColorFilterImageFilter filter(dl_color_filter);

  ASSERT_NE(filter.asColorFilter(), nullptr);
  ASSERT_EQ(filter.asColorFilter(), &filter);
}

TEST(DisplayListImageFilter, ColorFilterContents) {
  DlBlendColorFilter dl_color_filter(DlColor::kRed(), DlBlendMode::kLighten);
  DlColorFilterImageFilter filter(dl_color_filter);

  ASSERT_EQ(*filter.color_filter().get(), dl_color_filter);
}

TEST(DisplayListImageFilter, ColorFilterEquals) {
  DlBlendColorFilter dl_color_filter1(DlColor::kRed(), DlBlendMode::kLighten);
  DlColorFilterImageFilter filter1(dl_color_filter1);

  DlBlendColorFilter dl_color_filter2(DlColor::kRed(), DlBlendMode::kLighten);
  DlColorFilterImageFilter filter2(dl_color_filter2);

  TestEquals(filter1, filter2);
}

TEST(DisplayListImageFilter, ColorFilterWithLocalMatrixEquals) {
  DlBlendColorFilter dl_color_filter1(DlColor::kRed(), DlBlendMode::kLighten);
  DlColorFilterImageFilter filter1(dl_color_filter1);

  DlBlendColorFilter dl_color_filter2(DlColor::kRed(), DlBlendMode::kLighten);
  DlColorFilterImageFilter filter2(dl_color_filter2);

  DlMatrix local_matrix = DlMatrix::MakeTranslation({10, 10});
  TestEquals(*filter1.makeWithLocalMatrix(local_matrix),
             *filter2.makeWithLocalMatrix(local_matrix));
}

TEST(DisplayListImageFilter, ColorFilterNotEquals) {
  DlBlendColorFilter dl_color_filter1(DlColor::kRed(), DlBlendMode::kLighten);
  DlColorFilterImageFilter filter1(dl_color_filter1);

  DlBlendColorFilter dl_color_filter2(DlColor::kBlue(), DlBlendMode::kLighten);
  DlColorFilterImageFilter filter2(dl_color_filter2);

  DlBlendColorFilter dl_color_filter3(DlColor::kRed(), DlBlendMode::kDarken);
  DlColorFilterImageFilter filter3(dl_color_filter3);

  TestNotEquals(filter1, filter2, "Color differs");
  TestNotEquals(filter1, filter3, "Blend Mode differs");
}

TEST(DisplayListImageFilter, ColorFilterBounds) {
  DlBlendColorFilter dl_color_filter(DlColor::kRed(), DlBlendMode::kSrcIn);
  DlColorFilterImageFilter filter(dl_color_filter);
  DlRect input_bounds = DlRect::MakeLTRB(20, 20, 80, 80);
  TestBounds(filter, input_bounds, input_bounds);
}

TEST(DisplayListImageFilter, ColorFilterModifiesTransparencyBounds) {
  DlBlendColorFilter dl_color_filter(DlColor::kRed(), DlBlendMode::kSrcOver);
  DlColorFilterImageFilter filter(dl_color_filter);
  DlRect input_bounds = DlRect::MakeLTRB(20, 20, 80, 80);
  TestInvalidBounds(filter, DlMatrix(), input_bounds);
}

TEST(DisplayListImageFilter, LocalImageFilterBounds) {
  auto filter_matrix = DlMatrix::MakeRow(2.0, 0.0, 0.0, 10,   //
                                         0.5, 3.0, 0.0, 15,   //
                                         0.0, 0.0, 1.0, 0.0,  //
                                         0.0, 0.0, 0.0, 1.0);
  std::vector<sk_sp<SkImageFilter>> sk_filters{
      SkImageFilters::Blur(5.0, 6.0, SkTileMode::kRepeat, nullptr),
      SkImageFilters::ColorFilter(
          SkColorFilters::Blend(SK_ColorRED, SkBlendMode::kSrcOver), nullptr),
      SkImageFilters::Dilate(5.0, 10.0, nullptr),
      SkImageFilters::MatrixTransform(ToSkMatrix(filter_matrix),
                                      SkSamplingOptions(SkFilterMode::kLinear),
                                      nullptr),
      SkImageFilters::Compose(
          SkImageFilters::Blur(5.0, 6.0, SkTileMode::kRepeat, nullptr),
          SkImageFilters::ColorFilter(
              SkColorFilters::Blend(SK_ColorRED, SkBlendMode::kSrcOver),
              nullptr))};

  DlBlendColorFilter dl_color_filter(DlColor::kRed(), DlBlendMode::kSrcOver);
  std::vector<std::shared_ptr<DlImageFilter>> dl_filters{
      DlImageFilter::MakeBlur(5.0, 6.0, DlTileMode::kRepeat),
      DlImageFilter::MakeColorFilter(dl_color_filter.shared()),
      DlImageFilter::MakeDilate(5, 10),
      DlImageFilter::MakeMatrix(filter_matrix, DlImageSampling::kLinear),
      DlImageFilter::MakeCompose(
          DlImageFilter::MakeBlur(5.0, 6.0, DlTileMode::kRepeat),
          DlImageFilter::MakeColorFilter(dl_color_filter.shared())),
  };

  auto persp = SkMatrix::I();
  persp.setPerspY(0.001);
  std::vector<SkMatrix> sk_matrices = {
      SkMatrix::Translate(10.0, 10.0),
      SkMatrix::Scale(2.0, 2.0).preTranslate(10.0, 10.0),
      SkMatrix::RotateDeg(45).preTranslate(5.0, 5.0),  //
      persp};
  std::vector<DlMatrix> dl_matrices = {
      DlMatrix::MakeTranslation({10.0, 10.0}),
      DlMatrix::MakeScale({2.0, 2.0, 1.0}).Translate({10.0, 10.0}),
      DlMatrix::MakeRotationZ(DlDegrees(45)).Translate({5.0, 5.0}),
      ToDlMatrix(persp)};
  std::vector<SkMatrix> sk_bounds_matrices{
      SkMatrix::Translate(5.0, 10.0),
      SkMatrix::Scale(2.0, 2.0),
  };
  std::vector<DlMatrix> dl_bounds_matrices{
      DlMatrix::MakeTranslation({5.0, 10.0}),
      DlMatrix::MakeScale({2.0, 2.0, 1.0}),
  };

  for (unsigned j = 0; j < dl_matrices.size(); j++) {
    DlLocalMatrixImageFilter filter(dl_matrices[j], nullptr);
    {
      const auto input_bounds = DlRect::MakeLTRB(20, 20, 80, 80);
      DlRect output_bounds;
      EXPECT_EQ(filter.map_local_bounds(input_bounds, output_bounds),
                &output_bounds);
      EXPECT_EQ(input_bounds, output_bounds);
    }
    for (unsigned k = 0; k < dl_bounds_matrices.size(); k++) {
      auto& bounds_matrix = dl_bounds_matrices[k];
      {
        const auto input_bounds = DlIRect::MakeLTRB(20, 20, 80, 80);
        DlIRect output_bounds;
        EXPECT_EQ(filter.map_device_bounds(input_bounds, bounds_matrix,
                                           output_bounds),
                  &output_bounds);
        EXPECT_EQ(input_bounds, output_bounds);
      }
      {
        const auto output_bounds = DlIRect::MakeLTRB(20, 20, 80, 80);
        DlIRect input_bounds;
        EXPECT_EQ(filter.get_input_device_bounds(output_bounds, bounds_matrix,
                                                 input_bounds),
                  &input_bounds);
        EXPECT_EQ(input_bounds, output_bounds);
      }
    }
  }

  for (unsigned i = 0; i < sk_filters.size(); i++) {
    for (unsigned j = 0; j < dl_matrices.size(); j++) {
      for (unsigned k = 0; k < dl_bounds_matrices.size(); k++) {
        auto desc = "filter " + std::to_string(i + 1)             //
                    + ", filter matrix " + std::to_string(j + 1)  //
                    + ", bounds matrix " + std::to_string(k + 1);
        auto sk_local_filter =
            sk_filters[i]->makeWithLocalMatrix(sk_matrices[j]);
        auto dl_local_filter =
            dl_filters[i]->makeWithLocalMatrix(dl_matrices[j]);
        if (!sk_local_filter || !dl_local_filter) {
          // Temporarily relax the equivalence testing to allow Skia to expand
          // their behavior. Once the Skia fixes are rolled in, the
          // DlImageFilter should adapt  to the new rules.
          // See https://github.com/flutter/flutter/issues/114723
          ASSERT_TRUE(sk_local_filter || !dl_local_filter) << desc;
          continue;
        }
        {
          auto input_bounds = SkIRect::MakeLTRB(20, 20, 80, 80);
          SkIRect sk_rect;
          DlIRect dl_rect;
          sk_rect = sk_local_filter->filterBounds(
              input_bounds, sk_bounds_matrices[k],
              SkImageFilter::MapDirection::kForward_MapDirection);
          if (dl_local_filter->map_device_bounds(
                  ToDlIRect(input_bounds), dl_bounds_matrices[k], dl_rect)) {
            ASSERT_EQ(sk_rect, ToSkIRect(dl_rect)) << desc;
          } else {
            ASSERT_TRUE(dl_local_filter->modifies_transparent_black()) << desc;
            ASSERT_FALSE(sk_local_filter->canComputeFastBounds()) << desc;
          }
        }
        {
          // Test for: Know the outset bounds to get the inset bounds
          // Skia have some bounds calculate error of DilateFilter and
          // MatrixFilter
          // Skia issue: https://bugs.chromium.org/p/skia/issues/detail?id=13444
          // flutter issue: https://github.com/flutter/flutter/issues/108693
          if (i == 2 || i == 3) {
            continue;
          }
          auto outset_bounds = SkIRect::MakeLTRB(20, 20, 80, 80);
          SkIRect sk_rect;
          DlIRect dl_rect;
          sk_rect = sk_local_filter->filterBounds(
              outset_bounds, sk_bounds_matrices[k],
              SkImageFilter::MapDirection::kReverse_MapDirection);
          if (dl_local_filter->get_input_device_bounds(
                  ToDlIRect(outset_bounds), dl_bounds_matrices[k], dl_rect)) {
            ASSERT_EQ(sk_rect, ToSkIRect(dl_rect)) << desc;
          } else {
            ASSERT_TRUE(dl_local_filter->modifies_transparent_black());
            ASSERT_FALSE(sk_local_filter->canComputeFastBounds());
          }
        }
      }
    }
  }
}

TEST(DisplayListImageFilter, RuntimeEffectEquality) {
  DlRuntimeEffectImageFilter filter_a(nullptr, {nullptr},
                                      std::make_shared<std::vector<uint8_t>>());
  DlRuntimeEffectImageFilter filter_b(nullptr, {nullptr},
                                      std::make_shared<std::vector<uint8_t>>());

  EXPECT_EQ(filter_a, filter_b);

  DlRuntimeEffectImageFilter filter_c(
      nullptr, {nullptr}, std::make_shared<std::vector<uint8_t>>(1));

  EXPECT_NE(filter_a, filter_c);
}

TEST(DisplayListImageFilter, RuntimeEffectEqualityWithSamplers) {
  auto image_a =
      DlColorSource::MakeImage(nullptr, DlTileMode::kClamp, DlTileMode::kDecal);
  auto image_b =
      DlColorSource::MakeImage(nullptr, DlTileMode::kClamp, DlTileMode::kClamp);

  DlRuntimeEffectImageFilter filter_a(nullptr, {nullptr, image_a},
                                      std::make_shared<std::vector<uint8_t>>());
  DlRuntimeEffectImageFilter filter_b(nullptr, {nullptr, image_a},
                                      std::make_shared<std::vector<uint8_t>>());

  EXPECT_EQ(filter_a, filter_b);

  DlRuntimeEffectImageFilter filter_c(nullptr, {nullptr, image_b},
                                      std::make_shared<std::vector<uint8_t>>());

  EXPECT_NE(filter_a, filter_c);
}

TEST(DisplayListImageFilter, RuntimeEffectMapDeviceBounds) {
  DlRuntimeEffectImageFilter filter_a(nullptr, {nullptr},
                                      std::make_shared<std::vector<uint8_t>>());

  auto input_bounds = DlIRect::MakeLTRB(0, 0, 100, 100);
  DlMatrix identity;
  DlIRect output_bounds;
  DlIRect* result =
      filter_a.map_device_bounds(input_bounds, identity, output_bounds);

  EXPECT_NE(result, nullptr);
  EXPECT_EQ(result, &output_bounds);
  EXPECT_EQ(output_bounds, input_bounds);
}

TEST(DisplayListImageFilter, RuntimeEffectMapInputBounds) {
  DlRuntimeEffectImageFilter filter_a(nullptr, {nullptr},
                                      std::make_shared<std::vector<uint8_t>>());

  auto input_bounds = DlRect::MakeLTRB(0, 0, 100, 100);

  DlRect output_bounds;
  DlRect* result = filter_a.map_local_bounds(input_bounds, output_bounds);

  EXPECT_NE(result, nullptr);
  EXPECT_EQ(result, &output_bounds);
  EXPECT_EQ(output_bounds, input_bounds);
}

TEST(DisplayListImageFilter, RuntimeEffectGetInputDeviceBounds) {
  DlRuntimeEffectImageFilter filter_a(nullptr, {nullptr},
                                      std::make_shared<std::vector<uint8_t>>());

  auto output_bounds = DlIRect::MakeLTRB(0, 0, 100, 100);

  DlMatrix identity;
  DlIRect input_bounds;
  DlIRect* result =
      filter_a.get_input_device_bounds(output_bounds, identity, input_bounds);

  EXPECT_NE(result, nullptr);
  EXPECT_EQ(result, &input_bounds);
  EXPECT_EQ(output_bounds, input_bounds);
}

TEST(DisplayListImageFilter, RuntimeEffectModifiesTransparentBlack) {
  DlRuntimeEffectImageFilter filter_a(nullptr, {nullptr},
                                      std::make_shared<std::vector<uint8_t>>());

  EXPECT_FALSE(filter_a.modifies_transparent_black());
}

}  // namespace testing
}  // namespace flutter
