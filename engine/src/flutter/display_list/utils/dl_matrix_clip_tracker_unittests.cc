// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/utils/dl_matrix_clip_tracker.h"
#include "flutter/testing/assertions_skia.h"
#include "gtest/gtest.h"

namespace flutter {
namespace testing {

TEST(DisplayListMatrixClipState, Constructor) {
  const SkRect cull_rect = SkRect::MakeLTRB(20, 40, 60, 80);
  const DlRect dl_cull_rect = DlRect::MakeLTRB(20, 40, 60, 80);
  const SkMatrix matrix = SkMatrix::Scale(4, 4);
  const SkM44 m44 = SkM44::Scale(4, 4);
  const DlMatrix dl_matrix = DlMatrix::MakeScale({4.0, 4.0, 1.0});
  const SkRect local_cull_rect = SkRect::MakeLTRB(5, 10, 15, 20);

  DisplayListMatrixClipState state1(cull_rect, matrix);
  DisplayListMatrixClipState state2(cull_rect, m44);
  DisplayListMatrixClipState state3(dl_cull_rect, dl_matrix);

  EXPECT_FALSE(state1.using_4x4_matrix());
  EXPECT_EQ(state1.device_cull_rect(), cull_rect);
  EXPECT_EQ(state1.local_cull_rect(), local_cull_rect);
  EXPECT_EQ(state1.matrix_3x3(), matrix);
  EXPECT_EQ(state1.matrix_4x4(), m44);
  EXPECT_EQ(state1.matrix(), dl_matrix);

  EXPECT_FALSE(state2.using_4x4_matrix());
  EXPECT_EQ(state2.device_cull_rect(), cull_rect);
  EXPECT_EQ(state2.local_cull_rect(), local_cull_rect);
  EXPECT_EQ(state2.matrix_3x3(), matrix);
  EXPECT_EQ(state2.matrix_4x4(), m44);
  EXPECT_EQ(state2.matrix(), dl_matrix);

  EXPECT_FALSE(state3.using_4x4_matrix());
  EXPECT_EQ(state3.device_cull_rect(), cull_rect);
  EXPECT_EQ(state3.local_cull_rect(), local_cull_rect);
  EXPECT_EQ(state3.matrix_3x3(), matrix);
  EXPECT_EQ(state3.matrix_4x4(), m44);
  EXPECT_EQ(state3.matrix(), dl_matrix);
}

TEST(DisplayListMatrixClipState, Constructor4x4) {
  const SkRect cull_rect = SkRect::MakeLTRB(20, 40, 60, 80);
  const DlRect dl_cull_rect = DlRect::MakeLTRB(20, 40, 60, 80);
  // clang-format off
  const SkM44 m44 = SkM44(4, 0, 0.5, 0,
                          0, 4, 0.5, 0,
                          0, 0, 4.0, 0,
                          0, 0, 0.0, 1);
  const DlMatrix dl_matrix = DlMatrix::MakeRow(4, 0, 0.5, 0,
                                               0, 4, 0.5, 0,
                                               0, 0, 4.0, 0,
                                               0, 0, 0.0, 1);
  // clang-format on
  const SkRect local_cull_rect = SkRect::MakeLTRB(5, 10, 15, 20);

  DisplayListMatrixClipState state1(cull_rect, m44);
  DisplayListMatrixClipState state2(dl_cull_rect, dl_matrix);

  EXPECT_TRUE(state1.using_4x4_matrix());
  EXPECT_EQ(state1.device_cull_rect(), cull_rect);
  EXPECT_EQ(state1.local_cull_rect(), local_cull_rect);
  EXPECT_EQ(state1.matrix_4x4(), m44);
  EXPECT_EQ(state1.matrix(), dl_matrix);

  EXPECT_TRUE(state2.using_4x4_matrix());
  EXPECT_EQ(state2.device_cull_rect(), cull_rect);
  EXPECT_EQ(state2.local_cull_rect(), local_cull_rect);
  EXPECT_EQ(state2.matrix_4x4(), m44);
  EXPECT_EQ(state2.matrix(), dl_matrix);
}

TEST(DisplayListMatrixClipState, TransformTo4x4) {
  const SkRect cull_rect = SkRect::MakeLTRB(20, 40, 60, 80);
  const DlRect dl_cull_rect = DlRect::MakeLTRB(20, 40, 60, 80);
  // clang-format off
  const SkM44 m44 = SkM44(4, 0, 0.5, 0,
                          0, 4, 0.5, 0,
                          0, 0, 4.0, 0,
                          0, 0, 0.0, 1);
  const DlMatrix dl_matrix = DlMatrix::MakeRow(4, 0, 0.5, 0,
                                               0, 4, 0.5, 0,
                                               0, 0, 4.0, 0,
                                               0, 0, 0.0, 1);
  // clang-format on
  const SkRect local_cull_rect = SkRect::MakeLTRB(5, 10, 15, 20);

  DisplayListMatrixClipState state1(cull_rect, SkMatrix::I());
  DisplayListMatrixClipState state2(dl_cull_rect, DlMatrix());
  EXPECT_FALSE(state1.using_4x4_matrix());
  EXPECT_FALSE(state2.using_4x4_matrix());

  state1.transform(m44);
  EXPECT_TRUE(state1.using_4x4_matrix());
  EXPECT_EQ(state1.device_cull_rect(), cull_rect);
  EXPECT_EQ(state1.local_cull_rect(), local_cull_rect);
  EXPECT_EQ(state1.matrix_4x4(), m44);
  EXPECT_EQ(state1.matrix(), dl_matrix);

  state2.transform(dl_matrix);
  EXPECT_TRUE(state2.using_4x4_matrix());
  EXPECT_EQ(state2.device_cull_rect(), cull_rect);
  EXPECT_EQ(state2.local_cull_rect(), local_cull_rect);
  EXPECT_EQ(state2.matrix_4x4(), m44);
  EXPECT_EQ(state2.matrix(), dl_matrix);
}

TEST(DisplayListMatrixClipState, SetTo4x4) {
  const SkRect cull_rect = SkRect::MakeLTRB(20, 40, 60, 80);
  const DlRect dl_cull_rect = DlRect::MakeLTRB(20, 40, 60, 80);
  // clang-format off
  const SkM44 m44 = SkM44(4, 0, 0.5, 0,
                          0, 4, 0.5, 0,
                          0, 0, 4.0, 0,
                          0, 0, 0.0, 1);
  const DlMatrix dl_matrix = DlMatrix::MakeRow(4, 0, 0.5, 0,
                                               0, 4, 0.5, 0,
                                               0, 0, 4.0, 0,
                                               0, 0, 0.0, 1);
  // clang-format on
  const SkRect local_cull_rect = SkRect::MakeLTRB(5, 10, 15, 20);

  DisplayListMatrixClipState state1(cull_rect, SkMatrix::I());
  DisplayListMatrixClipState state2(dl_cull_rect, DlMatrix());
  EXPECT_FALSE(state1.using_4x4_matrix());
  EXPECT_FALSE(state2.using_4x4_matrix());

  state1.setTransform(m44);
  EXPECT_TRUE(state1.using_4x4_matrix());
  EXPECT_EQ(state1.device_cull_rect(), cull_rect);
  EXPECT_EQ(state1.local_cull_rect(), local_cull_rect);
  EXPECT_EQ(state1.matrix_4x4(), m44);
  EXPECT_EQ(state1.matrix(), dl_matrix);

  state2.setTransform(dl_matrix);
  EXPECT_TRUE(state2.using_4x4_matrix());
  EXPECT_EQ(state2.device_cull_rect(), cull_rect);
  EXPECT_EQ(state2.local_cull_rect(), local_cull_rect);
  EXPECT_EQ(state2.matrix_4x4(), m44);
  EXPECT_EQ(state2.matrix(), dl_matrix);
}

TEST(DisplayListMatrixClipState, Translate) {
  const SkRect cull_rect = SkRect::MakeLTRB(20, 40, 60, 80);
  const DlRect dl_cull_rect = DlRect::MakeLTRB(20, 40, 60, 80);
  const SkMatrix matrix = SkMatrix::Scale(4, 4);
  const SkM44 m44 = SkM44::Scale(4, 4);
  const DlMatrix dl_matrix = DlMatrix::MakeScale({4.0, 4.0, 1.0});
  const SkMatrix translated_matrix =
      SkMatrix::Concat(matrix, SkMatrix::Translate(5, 1));
  const SkM44 translated_m44 = SkM44(translated_matrix);
  const DlMatrix dl_translated_matrix =
      dl_matrix * DlMatrix::MakeTranslation({5.0, 1.0});
  const SkRect local_cull_rect = SkRect::MakeLTRB(0, 9, 10, 19);

  DisplayListMatrixClipState state1(cull_rect, matrix);
  DisplayListMatrixClipState state2(cull_rect, m44);
  DisplayListMatrixClipState state3(dl_cull_rect, dl_matrix);
  state1.translate(5, 1);
  state2.translate(5, 1);
  state3.translate(5, 1);

  EXPECT_FALSE(state1.using_4x4_matrix());
  EXPECT_EQ(state1.device_cull_rect(), cull_rect);
  EXPECT_EQ(state1.local_cull_rect(), local_cull_rect);
  EXPECT_EQ(state1.matrix_3x3(), translated_matrix);
  EXPECT_EQ(state1.matrix_4x4(), translated_m44);
  EXPECT_EQ(state1.matrix(), dl_translated_matrix);

  EXPECT_FALSE(state2.using_4x4_matrix());
  EXPECT_EQ(state2.device_cull_rect(), cull_rect);
  EXPECT_EQ(state2.local_cull_rect(), local_cull_rect);
  EXPECT_EQ(state2.matrix_3x3(), translated_matrix);
  EXPECT_EQ(state2.matrix_4x4(), translated_m44);
  EXPECT_EQ(state2.matrix(), dl_translated_matrix);

  EXPECT_FALSE(state3.using_4x4_matrix());
  EXPECT_EQ(state3.device_cull_rect(), cull_rect);
  EXPECT_EQ(state3.local_cull_rect(), local_cull_rect);
  EXPECT_EQ(state3.matrix_3x3(), translated_matrix);
  EXPECT_EQ(state3.matrix_4x4(), translated_m44);
  EXPECT_EQ(state3.matrix(), dl_translated_matrix);
}

TEST(DisplayListMatrixClipState, Scale) {
  const SkRect cull_rect = SkRect::MakeLTRB(20, 40, 60, 80);
  const DlRect dl_cull_rect = DlRect::MakeLTRB(20, 40, 60, 80);
  const SkMatrix matrix = SkMatrix::Scale(4, 4);
  const SkM44 m44 = SkM44::Scale(4, 4);
  const DlMatrix dl_matrix = DlMatrix::MakeScale({4.0, 4.0, 1.0});
  // Scale factor carefully chosen to multiply cleanly and invert
  // without any non-binary-power-of-2 approximation errors.
  const SkMatrix scaled_matrix =
      SkMatrix::Concat(matrix, SkMatrix::Scale(0.5, 2));
  const SkM44 scaled_m44 = SkM44(scaled_matrix);
  const DlMatrix scaled_dl_matrix = dl_matrix.Scale({0.5, 2, 1});
  const SkRect local_cull_rect = SkRect::MakeLTRB(10, 5, 30, 10);

  DisplayListMatrixClipState state1(cull_rect, matrix);
  DisplayListMatrixClipState state2(cull_rect, m44);
  DisplayListMatrixClipState state3(dl_cull_rect, dl_matrix);
  state1.scale(0.5, 2);
  state2.scale(0.5, 2);
  state3.scale(0.5, 2);

  EXPECT_FALSE(state1.using_4x4_matrix());
  EXPECT_EQ(state1.device_cull_rect(), cull_rect);
  EXPECT_EQ(state1.local_cull_rect(), local_cull_rect);
  EXPECT_EQ(state1.matrix_3x3(), scaled_matrix);
  EXPECT_EQ(state1.matrix_4x4(), scaled_m44);
  EXPECT_EQ(state1.matrix(), scaled_dl_matrix);

  EXPECT_FALSE(state2.using_4x4_matrix());
  EXPECT_EQ(state2.device_cull_rect(), cull_rect);
  EXPECT_EQ(state2.local_cull_rect(), local_cull_rect);
  EXPECT_EQ(state2.matrix_3x3(), scaled_matrix);
  EXPECT_EQ(state2.matrix_4x4(), scaled_m44);
  EXPECT_EQ(state2.matrix(), scaled_dl_matrix);

  EXPECT_FALSE(state3.using_4x4_matrix());
  EXPECT_EQ(state3.device_cull_rect(), cull_rect);
  EXPECT_EQ(state3.local_cull_rect(), local_cull_rect);
  EXPECT_EQ(state3.matrix_3x3(), scaled_matrix);
  EXPECT_EQ(state3.matrix_4x4(), scaled_m44);
  EXPECT_EQ(state3.matrix(), scaled_dl_matrix);
}

TEST(DisplayListMatrixClipState, Skew) {
  const SkRect cull_rect = SkRect::MakeLTRB(20, 40, 60, 80);
  const DlRect dl_cull_rect = DlRect::MakeLTRB(20, 40, 60, 80);
  const SkMatrix matrix = SkMatrix::Scale(4, 4);
  const SkM44 m44 = SkM44::Scale(4, 4);
  const DlMatrix dl_matrix = DlMatrix::MakeScale({4, 4, 1});
  const SkMatrix skewed_matrix =
      SkMatrix::Concat(matrix, SkMatrix::Skew(.25, 0));
  const SkM44 skewed_m44 = SkM44(skewed_matrix);
  const DlMatrix skewed_dl_matrix = dl_matrix * DlMatrix::MakeSkew(0.25, 0);
  const SkRect local_cull_rect = SkRect::MakeLTRB(0, 10, 12.5, 20);

  DisplayListMatrixClipState state1(cull_rect, matrix);
  DisplayListMatrixClipState state2(cull_rect, m44);
  DisplayListMatrixClipState state3(dl_cull_rect, dl_matrix);
  state1.skew(.25, 0);
  state2.skew(.25, 0);
  state3.skew(.25, 0);

  EXPECT_FALSE(state1.using_4x4_matrix());
  EXPECT_EQ(state1.device_cull_rect(), cull_rect);
  EXPECT_EQ(state1.local_cull_rect(), local_cull_rect);
  EXPECT_EQ(state1.matrix_3x3(), skewed_matrix);
  EXPECT_EQ(state1.matrix_4x4(), skewed_m44);
  EXPECT_EQ(state1.matrix(), skewed_dl_matrix);

  EXPECT_FALSE(state2.using_4x4_matrix());
  EXPECT_EQ(state2.device_cull_rect(), cull_rect);
  EXPECT_EQ(state2.local_cull_rect(), local_cull_rect);
  EXPECT_EQ(state2.matrix_3x3(), skewed_matrix);
  EXPECT_EQ(state2.matrix_4x4(), skewed_m44);
  EXPECT_EQ(state2.matrix(), skewed_dl_matrix);

  EXPECT_FALSE(state3.using_4x4_matrix());
  EXPECT_EQ(state3.device_cull_rect(), cull_rect);
  EXPECT_EQ(state3.local_cull_rect(), local_cull_rect);
  EXPECT_EQ(state3.matrix_3x3(), skewed_matrix);
  EXPECT_EQ(state3.matrix_4x4(), skewed_m44);
  EXPECT_EQ(state3.matrix(), skewed_dl_matrix);
}

TEST(DisplayListMatrixClipState, Rotate) {
  const SkRect cull_rect = SkRect::MakeLTRB(20, 40, 60, 80);
  const DlRect dl_cull_rect = DlRect::MakeLTRB(20, 40, 60, 80);
  const SkMatrix matrix = SkMatrix::Scale(4, 4);
  const SkM44 m44 = SkM44::Scale(4, 4);
  const DlMatrix dl_matrix = DlMatrix::MakeScale({4, 4, 1});
  const SkMatrix rotated_matrix =
      SkMatrix::Concat(matrix, SkMatrix::RotateDeg(90));
  const SkM44 rotated_m44 = SkM44(rotated_matrix);
  const DlMatrix rotated_dl_matrix =
      dl_matrix * DlMatrix::MakeRotationZ(DlDegrees(90));
  const SkRect local_cull_rect = SkRect::MakeLTRB(10, -15, 20, -5);

  DisplayListMatrixClipState state1(cull_rect, matrix);
  DisplayListMatrixClipState state2(cull_rect, m44);
  DisplayListMatrixClipState state3(dl_cull_rect, dl_matrix);
  state1.rotate(90);
  state2.rotate(90);
  state3.rotate(90);

  EXPECT_FALSE(state1.using_4x4_matrix());
  EXPECT_EQ(state1.device_cull_rect(), cull_rect);
  EXPECT_EQ(state1.local_cull_rect(), local_cull_rect);
  EXPECT_EQ(state1.matrix_3x3(), rotated_matrix);
  EXPECT_EQ(state1.matrix_4x4(), rotated_m44);
  EXPECT_EQ(state1.matrix(), rotated_dl_matrix);

  EXPECT_FALSE(state2.using_4x4_matrix());
  EXPECT_EQ(state2.device_cull_rect(), cull_rect);
  EXPECT_EQ(state2.local_cull_rect(), local_cull_rect);
  EXPECT_EQ(state2.matrix_3x3(), rotated_matrix);
  EXPECT_EQ(state2.matrix_4x4(), rotated_m44);
  EXPECT_EQ(state2.matrix(), rotated_dl_matrix);

  EXPECT_FALSE(state3.using_4x4_matrix());
  EXPECT_EQ(state3.device_cull_rect(), cull_rect);
  EXPECT_EQ(state3.local_cull_rect(), local_cull_rect);
  EXPECT_EQ(state3.matrix_3x3(), rotated_matrix);
  EXPECT_EQ(state3.matrix_4x4(), rotated_m44);
  EXPECT_EQ(state3.matrix(), rotated_dl_matrix);
}

TEST(DisplayListMatrixClipState, Transform2DAffine) {
  const SkRect cull_rect = SkRect::MakeLTRB(20, 40, 60, 80);
  const DlRect dl_cull_rect = DlRect::MakeLTRB(20, 40, 60, 80);
  const SkMatrix matrix = SkMatrix::Scale(4, 4);
  const SkM44 m44 = SkM44::Scale(4, 4);
  const DlMatrix dl_matrix = DlMatrix::MakeScale({4, 4, 1});

  const SkMatrix transformed_matrix =
      SkMatrix::Concat(matrix, SkMatrix::MakeAll(2, 0, 5,  //
                                                 0, 2, 6,  //
                                                 0, 0, 1));
  const SkM44 transformed_m44 = SkM44(transformed_matrix);
  const DlMatrix transformed_dl_matrix =
      dl_matrix * DlMatrix::MakeRow(2, 0, 0, 5,  //
                                    0, 2, 0, 6,  //
                                    0, 0, 1, 0,  //
                                    0, 0, 0, 1);
  const SkRect local_cull_rect = SkRect::MakeLTRB(0, 2, 5, 7);

  DisplayListMatrixClipState state1(cull_rect, matrix);
  DisplayListMatrixClipState state2(cull_rect, m44);
  DisplayListMatrixClipState state3(dl_cull_rect, dl_matrix);
  state1.transform2DAffine(2, 0, 5,  //
                           0, 2, 6);
  state2.transform2DAffine(2, 0, 5,  //
                           0, 2, 6);
  state3.transform2DAffine(2, 0, 5,  //
                           0, 2, 6);

  EXPECT_FALSE(state1.using_4x4_matrix());
  EXPECT_EQ(state1.device_cull_rect(), cull_rect);
  EXPECT_EQ(state1.local_cull_rect(), local_cull_rect);
  EXPECT_EQ(state1.matrix_3x3(), transformed_matrix);
  EXPECT_EQ(state1.matrix_4x4(), transformed_m44);
  EXPECT_EQ(state1.matrix(), transformed_dl_matrix);

  EXPECT_FALSE(state2.using_4x4_matrix());
  EXPECT_EQ(state2.device_cull_rect(), cull_rect);
  EXPECT_EQ(state2.local_cull_rect(), local_cull_rect);
  EXPECT_EQ(state2.matrix_3x3(), transformed_matrix);
  EXPECT_EQ(state2.matrix_4x4(), transformed_m44);
  EXPECT_EQ(state2.matrix(), transformed_dl_matrix);

  EXPECT_FALSE(state3.using_4x4_matrix());
  EXPECT_EQ(state3.device_cull_rect(), cull_rect);
  EXPECT_EQ(state3.local_cull_rect(), local_cull_rect);
  EXPECT_EQ(state3.matrix_3x3(), transformed_matrix);
  EXPECT_EQ(state3.matrix_4x4(), transformed_m44);
  EXPECT_EQ(state3.matrix(), transformed_dl_matrix);
}

TEST(DisplayListMatrixClipState, TransformFullPerspectiveUsing3x3Matrix) {
  const SkRect cull_rect = SkRect::MakeLTRB(20, 40, 60, 80);
  const DlRect dl_cull_rect = DlRect::MakeLTRB(20, 40, 60, 80);
  const SkMatrix matrix = SkMatrix::Scale(4, 4);
  const SkM44 m44 = SkM44::Scale(4, 4);
  const DlMatrix dl_matrix = DlMatrix::MakeScale({4, 4, 1});

  const SkMatrix transformed_matrix =
      SkMatrix::Concat(matrix, SkMatrix::MakeAll(2, 0, 5,  //
                                                 0, 2, 6,  //
                                                 0, 0, 1));
  const SkM44 transformed_m44 = SkM44(transformed_matrix);
  const DlMatrix transformed_dl_matrix =
      dl_matrix * DlMatrix::MakeRow(2, 0, 0, 5,  //
                                    0, 2, 0, 6,  //
                                    0, 0, 1, 0,  //
                                    0, 0, 0, 1);
  const SkRect local_cull_rect = SkRect::MakeLTRB(0, 2, 5, 7);

  DisplayListMatrixClipState state1(cull_rect, matrix);
  DisplayListMatrixClipState state2(cull_rect, m44);
  DisplayListMatrixClipState state3(dl_cull_rect, dl_matrix);
  state1.transformFullPerspective(2, 0, 0, 5,  //
                                  0, 2, 0, 6,  //
                                  0, 0, 1, 0,  //
                                  0, 0, 0, 1);
  state2.transformFullPerspective(2, 0, 0, 5,  //
                                  0, 2, 0, 6,  //
                                  0, 0, 1, 0,  //
                                  0, 0, 0, 1);
  state3.transformFullPerspective(2, 0, 0, 5,  //
                                  0, 2, 0, 6,  //
                                  0, 0, 1, 0,  //
                                  0, 0, 0, 1);

  EXPECT_FALSE(state1.using_4x4_matrix());
  EXPECT_EQ(state1.device_cull_rect(), cull_rect);
  EXPECT_EQ(state1.local_cull_rect(), local_cull_rect);
  EXPECT_EQ(state1.matrix_3x3(), transformed_matrix);
  EXPECT_EQ(state1.matrix_4x4(), transformed_m44);
  EXPECT_EQ(state1.matrix(), transformed_dl_matrix);

  EXPECT_FALSE(state2.using_4x4_matrix());
  EXPECT_EQ(state2.device_cull_rect(), cull_rect);
  EXPECT_EQ(state2.local_cull_rect(), local_cull_rect);
  EXPECT_EQ(state2.matrix_3x3(), transformed_matrix);
  EXPECT_EQ(state2.matrix_4x4(), transformed_m44);
  EXPECT_EQ(state2.matrix(), transformed_dl_matrix);

  EXPECT_FALSE(state3.using_4x4_matrix());
  EXPECT_EQ(state3.device_cull_rect(), cull_rect);
  EXPECT_EQ(state3.local_cull_rect(), local_cull_rect);
  EXPECT_EQ(state3.matrix_3x3(), transformed_matrix);
  EXPECT_EQ(state3.matrix_4x4(), transformed_m44);
  EXPECT_EQ(state3.matrix(), transformed_dl_matrix);
}

TEST(DisplayListMatrixClipState, TransformFullPerspectiveUsing4x4Matrix) {
  const SkRect cull_rect = SkRect::MakeLTRB(20, 40, 60, 80);
  const DlRect dl_cull_rect = DlRect::MakeLTRB(20, 40, 60, 80);
  const SkMatrix matrix = SkMatrix::Scale(4, 4);
  const SkM44 m44 = SkM44::Scale(4, 4);
  const DlMatrix dl_matrix = DlMatrix::MakeScale({4, 4, 1});

  const SkM44 transformed_m44 = SkM44(m44, SkM44(2, 0, 0, 5,  //
                                                 0, 2, 0, 6,  //
                                                 0, 0, 1, 7,  //
                                                 0, 0, 0, 1));
  const DlMatrix transformed_dl_matrix =
      dl_matrix * DlMatrix::MakeRow(2, 0, 0, 5,  //
                                    0, 2, 0, 6,  //
                                    0, 0, 1, 7,  //
                                    0, 0, 0, 1);
  const SkRect local_cull_rect = SkRect::MakeLTRB(0, 2, 5, 7);

  DisplayListMatrixClipState state1(cull_rect, matrix);
  DisplayListMatrixClipState state2(cull_rect, m44);
  DisplayListMatrixClipState state3(dl_cull_rect, dl_matrix);
  state1.transformFullPerspective(2, 0, 0, 5,  //
                                  0, 2, 0, 6,  //
                                  0, 0, 1, 7,  //
                                  0, 0, 0, 1);
  state2.transformFullPerspective(2, 0, 0, 5,  //
                                  0, 2, 0, 6,  //
                                  0, 0, 1, 7,  //
                                  0, 0, 0, 1);
  state3.transformFullPerspective(2, 0, 0, 5,  //
                                  0, 2, 0, 6,  //
                                  0, 0, 1, 7,  //
                                  0, 0, 0, 1);

  EXPECT_TRUE(state1.using_4x4_matrix());
  EXPECT_EQ(state1.device_cull_rect(), cull_rect);
  EXPECT_EQ(state1.local_cull_rect(), local_cull_rect);
  EXPECT_EQ(state1.matrix_4x4(), transformed_m44);
  EXPECT_EQ(state1.matrix(), transformed_dl_matrix);

  EXPECT_TRUE(state2.using_4x4_matrix());
  EXPECT_EQ(state2.device_cull_rect(), cull_rect);
  EXPECT_EQ(state2.local_cull_rect(), local_cull_rect);
  EXPECT_EQ(state2.matrix_4x4(), transformed_m44);
  EXPECT_EQ(state2.matrix(), transformed_dl_matrix);

  EXPECT_TRUE(state3.using_4x4_matrix());
  EXPECT_EQ(state3.device_cull_rect(), cull_rect);
  EXPECT_EQ(state3.local_cull_rect(), local_cull_rect);
  EXPECT_EQ(state3.matrix_4x4(), transformed_m44);
  EXPECT_EQ(state3.matrix(), transformed_dl_matrix);
}

TEST(DisplayListMatrixClipState, ClipDifference) {
  SkRect cull_rect = SkRect::MakeLTRB(20, 20, 40, 40);

  auto non_reducing = [&cull_rect](const SkRect& diff_rect,
                                   const std::string& label) {
    {
      DisplayListMatrixClipState state(cull_rect, SkMatrix::I());
      state.clipRect(diff_rect, DlCanvas::ClipOp::kDifference, false);
      EXPECT_EQ(state.device_cull_rect(), cull_rect) << label;
    }
    {
      DisplayListMatrixClipState state(cull_rect, SkMatrix::I());
      const SkRRect diff_rrect = SkRRect::MakeRect(diff_rect);
      state.clipRRect(diff_rrect, DlCanvas::ClipOp::kDifference, false);
      EXPECT_EQ(state.device_cull_rect(), cull_rect) << label << " (RRect)";
    }
    {
      DisplayListMatrixClipState state(cull_rect, SkMatrix::I());
      const SkPath diff_path = SkPath().addRect(diff_rect);
      state.clipPath(diff_path, DlCanvas::ClipOp::kDifference, false);
      EXPECT_EQ(state.device_cull_rect(), cull_rect) << label << " (RRect)";
    }
  };

  auto reducing = [&cull_rect](const SkRect& diff_rect,
                               const SkRect& result_rect,
                               const std::string& label) {
    EXPECT_TRUE(result_rect.isEmpty() || cull_rect.contains(result_rect));
    {
      DisplayListMatrixClipState state(cull_rect, SkMatrix::I());
      state.clipRect(diff_rect, DlCanvas::ClipOp::kDifference, false);
      EXPECT_EQ(state.device_cull_rect(), result_rect) << label;
    }
    {
      DisplayListMatrixClipState state(cull_rect, SkMatrix::I());
      const SkRRect diff_rrect = SkRRect::MakeRect(diff_rect);
      state.clipRRect(diff_rrect, DlCanvas::ClipOp::kDifference, false);
      EXPECT_EQ(state.device_cull_rect(), result_rect) << label << " (RRect)";
    }
    {
      DisplayListMatrixClipState state(cull_rect, SkMatrix::I());
      const SkPath diff_path = SkPath().addRect(diff_rect);
      state.clipPath(diff_path, DlCanvas::ClipOp::kDifference, false);
      EXPECT_EQ(state.device_cull_rect(), result_rect) << label << " (RRect)";
    }
  };

  // Skim the corners and edge
  non_reducing(SkRect::MakeLTRB(10, 10, 20, 20), "outside UL corner");
  non_reducing(SkRect::MakeLTRB(20, 10, 40, 20), "Above");
  non_reducing(SkRect::MakeLTRB(40, 10, 50, 20), "outside UR corner");
  non_reducing(SkRect::MakeLTRB(40, 20, 50, 40), "Right");
  non_reducing(SkRect::MakeLTRB(40, 40, 50, 50), "outside LR corner");
  non_reducing(SkRect::MakeLTRB(20, 40, 40, 50), "Below");
  non_reducing(SkRect::MakeLTRB(10, 40, 20, 50), "outside LR corner");
  non_reducing(SkRect::MakeLTRB(10, 20, 20, 40), "Left");

  // Overlap corners
  non_reducing(SkRect::MakeLTRB(15, 15, 25, 25), "covering UL corner");
  non_reducing(SkRect::MakeLTRB(35, 15, 45, 25), "covering UR corner");
  non_reducing(SkRect::MakeLTRB(35, 35, 45, 45), "covering LR corner");
  non_reducing(SkRect::MakeLTRB(15, 35, 25, 45), "covering LL corner");

  // Overlap edges, but not across an entire side
  non_reducing(SkRect::MakeLTRB(20, 15, 39, 25), "Top edge left-biased");
  non_reducing(SkRect::MakeLTRB(21, 15, 40, 25), "Top edge, right biased");
  non_reducing(SkRect::MakeLTRB(35, 20, 45, 39), "Right edge, top-biased");
  non_reducing(SkRect::MakeLTRB(35, 21, 45, 40), "Right edge, bottom-biased");
  non_reducing(SkRect::MakeLTRB(20, 35, 39, 45), "Bottom edge, left-biased");
  non_reducing(SkRect::MakeLTRB(21, 35, 40, 45), "Bottom edge, right-biased");
  non_reducing(SkRect::MakeLTRB(15, 20, 25, 39), "Left edge, top-biased");
  non_reducing(SkRect::MakeLTRB(15, 21, 25, 40), "Left edge, bottom-biased");

  // Slice all the way through the middle
  non_reducing(SkRect::MakeLTRB(25, 15, 35, 45), "Vertical interior slice");
  non_reducing(SkRect::MakeLTRB(15, 25, 45, 35), "Horizontal interior slice");

  // Slice off each edge
  reducing(SkRect::MakeLTRB(20, 15, 40, 25),  //
           SkRect::MakeLTRB(20, 25, 40, 40),  //
           "Slice off top");
  reducing(SkRect::MakeLTRB(35, 20, 45, 40),  //
           SkRect::MakeLTRB(20, 20, 35, 40),  //
           "Slice off right");
  reducing(SkRect::MakeLTRB(20, 35, 40, 45),  //
           SkRect::MakeLTRB(20, 20, 40, 35),  //
           "Slice off bottom");
  reducing(SkRect::MakeLTRB(15, 20, 25, 40),  //
           SkRect::MakeLTRB(25, 20, 40, 40),  //
           "Slice off left");

  // cull rect contains diff rect
  non_reducing(SkRect::MakeLTRB(21, 21, 39, 39), "Contained, non-covering");

  // cull rect equals diff rect
  reducing(cull_rect, SkRect::MakeEmpty(), "Perfectly covering");

  // diff rect contains cull rect
  reducing(SkRect::MakeLTRB(15, 15, 45, 45), SkRect::MakeEmpty(), "Smothering");
}

TEST(DisplayListMatrixClipState, ClipPathWithInvertFillType) {
  SkRect cull_rect = SkRect::MakeLTRB(0, 0, 100.0, 100.0);
  DisplayListMatrixClipState state(cull_rect, SkMatrix::I());
  SkPath clip = SkPath().addCircle(10.2, 11.3, 2).addCircle(20.4, 25.7, 2);
  clip.setFillType(SkPathFillType::kInverseWinding);
  state.clipPath(clip, DlCanvas::ClipOp::kIntersect, false);

  EXPECT_EQ(state.local_cull_rect(), cull_rect);
  EXPECT_EQ(state.device_cull_rect(), cull_rect);
}

TEST(DisplayListMatrixClipState, DiffClipPathWithInvertFillType) {
  SkRect cull_rect = SkRect::MakeLTRB(0, 0, 100.0, 100.0);
  DisplayListMatrixClipState state(cull_rect, SkMatrix::I());

  SkPath clip = SkPath().addCircle(10.2, 11.3, 2).addCircle(20.4, 25.7, 2);
  clip.setFillType(SkPathFillType::kInverseWinding);
  SkRect clip_bounds = SkRect::MakeLTRB(8.2, 9.3, 22.4, 27.7);
  state.clipPath(clip, DlCanvas::ClipOp::kDifference, false);

  EXPECT_EQ(state.local_cull_rect(), clip_bounds);
  EXPECT_EQ(state.device_cull_rect(), clip_bounds);
}

TEST(DisplayListMatrixClipState, MapAndClipRectTranslation) {
  DlRect cull_rect = DlRect::MakeLTRB(100.0f, 100.0f, 200.0f, 200.0f);
  DlMatrix matrix = DlMatrix::MakeTranslation({10.0f, 20.0f, 1.0f});
  DisplayListMatrixClipState state(cull_rect, matrix);

  {
    // Empty width in src rect (before and after translation)
    SkRect rect = SkRect::MakeLTRB(150.0f, 150.0f, 150.0f, 160.0f);
    EXPECT_FALSE(state.mapAndClipRect(&rect));
    EXPECT_TRUE(rect.isEmpty());
  }

  {
    // Empty height in src rect (before and after translation)
    SkRect rect = SkRect::MakeLTRB(150.0f, 150.0f, 160.0f, 150.0f);
    EXPECT_FALSE(state.mapAndClipRect(&rect));
    EXPECT_TRUE(rect.isEmpty());
  }

  {
    // rect far outside of clip, even after translation
    SkRect rect = SkRect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f);
    EXPECT_FALSE(state.mapAndClipRect(&rect));
    EXPECT_TRUE(rect.isEmpty());
  }

  {
    // Rect abuts clip left side after translation
    SkRect rect = SkRect::MakeLTRB(80.0f, 100.0f, 90.0f, 110.0f);
    EXPECT_FALSE(state.mapAndClipRect(&rect));
    EXPECT_TRUE(rect.isEmpty());
  }

  {
    // Rect barely grazes clip left side after translation
    SkRect rect = SkRect::MakeLTRB(80.0f, 100.0f, 91.0f, 110.0f);
    EXPECT_TRUE(state.mapAndClipRect(&rect));
    EXPECT_EQ(rect, SkRect::MakeLTRB(100.0f, 120.0f, 101.0f, 130.0f));
  }

  {
    // Rect abuts clip top after translation
    SkRect rect = SkRect::MakeLTRB(100.0f, 70.0f, 110.0f, 80.0f);
    EXPECT_FALSE(state.mapAndClipRect(&rect));
    EXPECT_TRUE(rect.isEmpty());
  }

  {
    // Rect barely grazes clip top after translation
    SkRect rect = SkRect::MakeLTRB(100.0f, 70.0f, 110.0f, 81.0f);
    EXPECT_TRUE(state.mapAndClipRect(&rect));
    EXPECT_EQ(rect, SkRect::MakeLTRB(110.0f, 100.0f, 120.0f, 101.0f));
  }

  {
    // Rect abuts clip right side after translation
    SkRect rect = SkRect::MakeLTRB(190.0f, 100.0f, 200.0f, 110.0f);
    EXPECT_FALSE(state.mapAndClipRect(&rect));
    EXPECT_TRUE(rect.isEmpty());
  }

  {
    // Rect barely grazes clip right side after translation
    SkRect rect = SkRect::MakeLTRB(189.0f, 100.0f, 200.0f, 110.0f);
    EXPECT_TRUE(state.mapAndClipRect(&rect));
    EXPECT_EQ(rect, SkRect::MakeLTRB(199.0f, 120.0f, 200.0f, 130.0f));
  }

  {
    // Rect abuts clip bottom after translation
    SkRect rect = SkRect::MakeLTRB(100.0f, 180.0f, 110.0f, 190.0f);
    EXPECT_FALSE(state.mapAndClipRect(&rect));
    EXPECT_TRUE(rect.isEmpty());
  }

  {
    // Rect barely grazes clip bottom after translation
    SkRect rect = SkRect::MakeLTRB(100.0f, 179.0f, 110.0f, 190.0f);
    EXPECT_TRUE(state.mapAndClipRect(&rect));
    EXPECT_EQ(rect, SkRect::MakeLTRB(110.0f, 199.0f, 120.0f, 200.0f));
  }
}

TEST(DisplayListMatrixClipState, MapAndClipRectScale) {
  DlRect cull_rect = DlRect::MakeLTRB(100.0f, 100.0f, 500.0f, 500.0f);
  DlMatrix matrix = DlMatrix::MakeScale({2.0f, 4.0f, 1.0f});
  DisplayListMatrixClipState state(cull_rect, matrix);

  {
    // Empty width in src rect (before and after scaling)
    SkRect rect = SkRect::MakeLTRB(100.0f, 100.0f, 100.0f, 110.0f);
    EXPECT_FALSE(state.mapAndClipRect(&rect));
    EXPECT_TRUE(rect.isEmpty());
  }

  {
    // Empty height in src rect (before and after scaling)
    SkRect rect = SkRect::MakeLTRB(100.0f, 100.0f, 110.0f, 100.0f);
    EXPECT_FALSE(state.mapAndClipRect(&rect));
    EXPECT_TRUE(rect.isEmpty());
  }

  {
    // rect far outside of clip, even after scaling
    SkRect rect = SkRect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f);
    EXPECT_FALSE(state.mapAndClipRect(&rect));
    EXPECT_TRUE(rect.isEmpty());
  }

  {
    // Rect abuts clip left side after scaling
    SkRect rect = SkRect::MakeLTRB(40.0f, 100.0f, 50.0f, 110.0f);
    EXPECT_FALSE(state.mapAndClipRect(&rect));
    EXPECT_TRUE(rect.isEmpty());
  }

  {
    // Rect barely grazes clip left side after scaling
    SkRect rect = SkRect::MakeLTRB(40.0f, 100.0f, 51.0f, 110.0f);
    EXPECT_TRUE(state.mapAndClipRect(&rect));
    EXPECT_EQ(rect, SkRect::MakeLTRB(100.0f, 400.0f, 102.0f, 440.0f));
  }

  {
    // Rect abuts clip top after scaling
    SkRect rect = SkRect::MakeLTRB(100.0f, 15.0f, 110.0f, 25.0f);
    EXPECT_FALSE(state.mapAndClipRect(&rect));
    EXPECT_TRUE(rect.isEmpty());
  }

  {
    // Rect barely grazes clip top after scaling
    SkRect rect = SkRect::MakeLTRB(100.0f, 15.0f, 110.0f, 26.0f);
    EXPECT_TRUE(state.mapAndClipRect(&rect));
    EXPECT_EQ(rect, SkRect::MakeLTRB(200.0f, 100.0f, 220.0f, 104.0f));
  }

  {
    // Rect abuts clip right side after scaling
    SkRect rect = SkRect::MakeLTRB(250.0f, 100.0f, 260.0f, 110.0f);
    EXPECT_FALSE(state.mapAndClipRect(&rect));
    EXPECT_TRUE(rect.isEmpty());
  }

  {
    // Rect barely grazes clip right side after scaling
    SkRect rect = SkRect::MakeLTRB(249.0f, 100.0f, 260.0f, 110.0f);
    EXPECT_TRUE(state.mapAndClipRect(&rect));
    EXPECT_EQ(rect, SkRect::MakeLTRB(498.0f, 400.0f, 500.0f, 440.0f));
  }

  {
    // Rect abuts clip bottom after scaling
    SkRect rect = SkRect::MakeLTRB(100.0f, 125.0f, 110.0f, 135.0f);
    EXPECT_FALSE(state.mapAndClipRect(&rect));
    EXPECT_TRUE(rect.isEmpty());
  }

  {
    // Rect barely grazes clip bottom after scaling
    SkRect rect = SkRect::MakeLTRB(100.0f, 124.0f, 110.0f, 135.0f);
    EXPECT_TRUE(state.mapAndClipRect(&rect));
    EXPECT_EQ(rect, SkRect::MakeLTRB(200.0f, 496.0f, 220.0f, 500.0f));
  }
}

TEST(DisplayListMatrixClipState, RectCoverage) {
  DlRect rect = DlRect::MakeLTRB(100.0f, 100.0f, 200.0f, 200.0f);
  DisplayListMatrixClipState state(rect);

  EXPECT_TRUE(state.rect_covers_cull(rect));
  EXPECT_TRUE(state.rect_covers_cull(rect.Expand(0.1f, 0.0f, 0.0f, 0.0f)));
  EXPECT_TRUE(state.rect_covers_cull(rect.Expand(0.0f, 0.1f, 0.0f, 0.0f)));
  EXPECT_TRUE(state.rect_covers_cull(rect.Expand(0.0f, 0.0f, 0.1f, 0.0f)));
  EXPECT_TRUE(state.rect_covers_cull(rect.Expand(0.0f, 0.0f, 0.0f, 0.1f)));
  EXPECT_FALSE(state.rect_covers_cull(rect.Expand(-0.1f, 0.0f, 0.0f, 0.0f)));
  EXPECT_FALSE(state.rect_covers_cull(rect.Expand(0.0f, -0.1f, 0.0f, 0.0f)));
  EXPECT_FALSE(state.rect_covers_cull(rect.Expand(0.0f, 0.0f, -0.1f, 0.0f)));
  EXPECT_FALSE(state.rect_covers_cull(rect.Expand(0.0f, 0.0f, 0.0f, -0.1f)));
}

TEST(DisplayListMatrixClipState, RectCoverageAccuracy) {
  // These particular values create bit errors if we use the path that
  // tests for inclusion in local space, but work OK if we use a forward
  // path that tests for inclusion in device space, due to the fact that
  // the extra matrix inversion is just enough math to cause the transform
  // to place the local space cull corners just outside the original rect.
  // The test in device space only works under a simple scale, such as we
  // use for DPR adjustments (and which are not always inversion friendly).

  DlRect cull = DlRect::MakeLTRB(0.0f, 0.0f, 1080.0f, 2400.0f);
  DlScalar DPR = 2.625;
  DlRect rect = DlRect::MakeLTRB(0.0f, 0.0f, 1080.0f / DPR, 2400.0f / DPR);

  DisplayListMatrixClipState state(cull);
  state.scale(DPR, DPR);

  EXPECT_TRUE(state.rect_covers_cull(rect));
  EXPECT_TRUE(state.rect_covers_cull(rect.Expand(0.1f, 0.0f, 0.0f, 0.0f)));
  EXPECT_TRUE(state.rect_covers_cull(rect.Expand(0.0f, 0.1f, 0.0f, 0.0f)));
  EXPECT_TRUE(state.rect_covers_cull(rect.Expand(0.0f, 0.0f, 0.1f, 0.0f)));
  EXPECT_TRUE(state.rect_covers_cull(rect.Expand(0.0f, 0.0f, 0.0f, 0.1f)));
  EXPECT_FALSE(state.rect_covers_cull(rect.Expand(-0.1f, 0.0f, 0.0f, 0.0f)));
  EXPECT_FALSE(state.rect_covers_cull(rect.Expand(0.0f, -0.1f, 0.0f, 0.0f)));
  EXPECT_FALSE(state.rect_covers_cull(rect.Expand(0.0f, 0.0f, -0.1f, 0.0f)));
  EXPECT_FALSE(state.rect_covers_cull(rect.Expand(0.0f, 0.0f, 0.0f, -0.1f)));
}

TEST(DisplayListMatrixClipState, RectCoverageUnderScale) {
  DlRect rect = DlRect::MakeLTRB(100.0f, 100.0f, 200.0f, 200.0f);
  DisplayListMatrixClipState state(rect);
  state.scale(2.0f, 2.0f);

  EXPECT_FALSE(state.rect_covers_cull(DlRect::MakeLTRB(100, 100, 200, 200)));
  EXPECT_TRUE(state.rect_covers_cull(DlRect::MakeLTRB(50, 50, 100, 100)));
  EXPECT_TRUE(state.rect_covers_cull(DlRect::MakeLTRB(49, 50, 100, 100)));
  EXPECT_TRUE(state.rect_covers_cull(DlRect::MakeLTRB(50, 49, 100, 100)));
  EXPECT_TRUE(state.rect_covers_cull(DlRect::MakeLTRB(50, 50, 101, 100)));
  EXPECT_TRUE(state.rect_covers_cull(DlRect::MakeLTRB(50, 50, 100, 101)));
  EXPECT_FALSE(state.rect_covers_cull(DlRect::MakeLTRB(51, 50, 100, 100)));
  EXPECT_FALSE(state.rect_covers_cull(DlRect::MakeLTRB(50, 51, 100, 100)));
  EXPECT_FALSE(state.rect_covers_cull(DlRect::MakeLTRB(50, 50, 99, 100)));
  EXPECT_FALSE(state.rect_covers_cull(DlRect::MakeLTRB(50, 50, 100, 99)));
}

TEST(DisplayListMatrixClipState, RectCoverageUnderRotation) {
  DlRect rect = DlRect::MakeLTRB(-1.0f, -1.0f, 1.0f, 1.0f);
  DlRect cull = rect.Scale(impeller::kSqrt2 * 25);
  DlRect test = rect.Scale(50.0f);
  DlRect test_true = test.Expand(0.002f);
  DlRect test_false = test.Expand(-0.002f);

  for (int i = 0; i <= 360; i++) {
    DisplayListMatrixClipState state(cull);
    state.rotate(i);
    EXPECT_TRUE(state.rect_covers_cull(test_true))
        << "  testing " << test_true << std::endl
        << "    contains " << state.local_cull_rect() << std::endl
        << "    at " << i << " degrees";
    if ((i % 90) == 45) {
      // The cull rect is largest when viewed at multiples of 45
      // degrees so we will fail to contain it at those angles
      EXPECT_FALSE(state.rect_covers_cull(test_false))
          << "  testing " << test_false << std::endl
          << "    contains " << state.local_cull_rect() << std::endl
          << "    at " << i << " degrees";
    } else {
      // At other angles, the cull rect is not quite so big as to encroach
      // upon the expanded test rectangle.
      EXPECT_TRUE(state.rect_covers_cull(test_false))
          << "  testing " << test_false << std::endl
          << "    contains " << state.local_cull_rect() << std::endl
          << "    at " << i << " degrees";
    }
  }
}

TEST(DisplayListMatrixClipState, OvalCoverage) {
  DlRect cull = DlRect::MakeLTRB(-50.0f, -50.0f, 50.0f, 50.0f);
  DisplayListMatrixClipState state(cull);
  // The cull rect corners will be at (50, 50) so the oval needs to have
  // a radius large enough to cover that - sqrt(2*50*50) == sqrt(2) * 50
  // We pad by an ever so slight 0.02f to account for round off error and
  // then use larger expansion/contractions of 0.1f to cover/not-cover it.
  DlRect test = cull.Scale(impeller::kSqrt2).Expand(0.02f);

  EXPECT_TRUE(state.oval_covers_cull(test));
  EXPECT_TRUE(state.oval_covers_cull(test.Expand(0.1f, 0.0f, 0.0f, 0.0f)));
  EXPECT_TRUE(state.oval_covers_cull(test.Expand(0.0f, 0.1f, 0.0f, 0.0f)));
  EXPECT_TRUE(state.oval_covers_cull(test.Expand(0.0f, 0.0f, 0.1f, 0.0f)));
  EXPECT_TRUE(state.oval_covers_cull(test.Expand(0.0f, 0.0f, 0.0f, 0.1f)));
  EXPECT_FALSE(state.oval_covers_cull(test.Expand(-0.1f, 0.0f, 0.0f, 0.0f)));
  EXPECT_FALSE(state.oval_covers_cull(test.Expand(0.0f, -0.1f, 0.0f, 0.0f)));
  EXPECT_FALSE(state.oval_covers_cull(test.Expand(0.0f, 0.0f, -0.1f, 0.0f)));
  EXPECT_FALSE(state.oval_covers_cull(test.Expand(0.0f, 0.0f, 0.0f, -0.1f)));
}

TEST(DisplayListMatrixClipState, OvalCoverageUnderScale) {
  DlRect cull = DlRect::MakeLTRB(-50.0f, -50.0f, 50.0f, 50.0f);
  DisplayListMatrixClipState state(cull);
  state.scale(2.0f, 2.0f);
  // The cull rect corners will be at (50, 50) so the oval needs to have
  // a radius large enough to cover that - sqrt(2*50*50) == sqrt(2) * 50
  // We pad by an ever so slight 0.02f to account for round off error and
  // then use larger expansion/contractions of 0.1f to cover/not-cover it.
  // We combine that with an additional scale 0.5f since we are viewing
  // the cull rect under a 2.0 scale.
  DlRect test = cull.Scale(0.5f * impeller::kSqrt2).Expand(0.02f);

  EXPECT_TRUE(state.oval_covers_cull(test));
  EXPECT_TRUE(state.oval_covers_cull(test.Expand(0.1f, 0.0f, 0.0f, 0.0f)));
  EXPECT_TRUE(state.oval_covers_cull(test.Expand(0.0f, 0.1f, 0.0f, 0.0f)));
  EXPECT_TRUE(state.oval_covers_cull(test.Expand(0.0f, 0.0f, 0.1f, 0.0f)));
  EXPECT_TRUE(state.oval_covers_cull(test.Expand(0.0f, 0.0f, 0.0f, 0.1f)));
  EXPECT_FALSE(state.oval_covers_cull(test.Expand(-0.1f, 0.0f, 0.0f, 0.0f)));
  EXPECT_FALSE(state.oval_covers_cull(test.Expand(0.0f, -0.1f, 0.0f, 0.0f)));
  EXPECT_FALSE(state.oval_covers_cull(test.Expand(0.0f, 0.0f, -0.1f, 0.0f)));
  EXPECT_FALSE(state.oval_covers_cull(test.Expand(0.0f, 0.0f, 0.0f, -0.1f)));
}

TEST(DisplayListMatrixClipState, OvalCoverageUnderRotation) {
  DlRect unit = DlRect::MakeLTRB(-1.0f, -1.0f, 1.0f, 1.0f);
  DlRect cull = unit.Scale(50.0f);
  // See above, test bounds need to be sqrt(2) larger for the inscribed
  // oval to contain the cull rect. These tests are simpler than the scaled
  // rectangle coverage tests because this expanded test oval will
  // precisely cover the cull rect at all angles.
  DlRect test = cull.Scale(impeller::kSqrt2);
  DlRect test_true = test.Expand(0.002f);
  DlRect test_false = test.Expand(-0.002f);

  for (int i = 0; i <= 360; i++) {
    DisplayListMatrixClipState state(cull);
    state.rotate(i);
    EXPECT_TRUE(state.oval_covers_cull(test_true))
        << "  testing " << test_true << std::endl
        << "    contains " << state.local_cull_rect() << std::endl
        << "    at " << i << " degrees";
    EXPECT_FALSE(state.oval_covers_cull(test_false))
        << "  testing " << test_false << std::endl
        << "    contains " << state.local_cull_rect() << std::endl
        << "    at " << i << " degrees";
  }
}

TEST(DisplayListMatrixClipState, RRectCoverage) {
  SkRect cull = SkRect::MakeLTRB(-50.0f, -50.0f, 50.0f, 50.0f);
  DisplayListMatrixClipState state(cull);
  // test_bounds need to contain
  SkRect test = cull.makeOutset(2.0f, 2.0f);

  // RRect of cull with no corners covers
  EXPECT_TRUE(state.rrect_covers_cull(SkRRect::MakeRectXY(cull, 0.0f, 0.0f)));
  // RRect of cull with even the tiniest corners does not cover
  EXPECT_FALSE(
      state.rrect_covers_cull(SkRRect::MakeRectXY(cull, 0.01f, 0.01f)));

  // Expanded by 2.0 and then with a corner of 2.0 obviously still covers
  EXPECT_TRUE(state.rrect_covers_cull(SkRRect::MakeRectXY(test, 2.0f, 2.0f)));
  // The corner point of the cull rect is at (c-2, c-2) relative to the
  // corner of the rrect bounds so we compute its distance to the center
  // of the circular part and compare it to the radius of the corner (c)
  // to find the corner radius where it will start to leave the rounded
  // rectangle:
  //
  //     +-----------      +
  //     |    __---^^      |
  //     |  +/-------  +   |
  //     |  / \        |   c
  //     | /|   \     c-2  |
  //     |/ |     \    |   |
  //     || |       *  +   +
  //
  // sqrt(2*(c-2)*(c-2)) > c
  // 2*(c-2)*(c-2) > c*c
  // 2*(cc - 4c + 4) > cc
  // 2cc - 8c + 8 > cc
  // cc - 8c + 8 > 0
  // c > 8 +/- sqrt(64 - 32) / 2
  // c > ~6.828
  // corners set to 6.82 should still cover the cull rect
  EXPECT_TRUE(state.rrect_covers_cull(SkRRect::MakeRectXY(test, 6.82f, 6.82f)));
  // but corners set to 6.83 should not cover the cull rect
  EXPECT_FALSE(
      state.rrect_covers_cull(SkRRect::MakeRectXY(test, 6.84f, 6.84f)));
}

}  // namespace testing
}  // namespace flutter
