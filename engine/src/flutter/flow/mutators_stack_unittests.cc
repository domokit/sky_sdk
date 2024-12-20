// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/effects/dl_image_filters.h"
#include "flutter/flow/embedded_views.h"
#include "gtest/gtest.h"

namespace flutter {
namespace testing {

TEST(MutatorsStack, Initialization) {
  MutatorsStack stack;
  ASSERT_TRUE(true);
}

TEST(MutatorsStack, CopyConstructor) {
  MutatorsStack stack;
  auto rrect = SkRRect::MakeEmpty();
  auto rect = SkRect::MakeEmpty();
  stack.PushClipRect(rect);
  stack.PushClipRRect(rrect);
  MutatorsStack copy = MutatorsStack(stack);
  ASSERT_TRUE(copy == stack);
}

TEST(MutatorsStack, CopyAndUpdateTheCopy) {
  MutatorsStack stack;
  auto rrect = SkRRect::MakeEmpty();
  auto rect = SkRect::MakeEmpty();
  stack.PushClipRect(rect);
  stack.PushClipRRect(rrect);
  MutatorsStack copy = MutatorsStack(stack);
  copy.Pop();
  copy.Pop();
  ASSERT_TRUE(copy != stack);
  ASSERT_TRUE(copy.is_empty());
  ASSERT_TRUE(!stack.is_empty());
  auto iter = stack.Bottom();
  ASSERT_TRUE(iter->get()->GetType() == MutatorType::kClipRRect);
  ASSERT_TRUE(iter->get()->GetRRect() == rrect);
  ++iter;
  ASSERT_TRUE(iter->get()->GetType() == MutatorType::kClipRect);
  ASSERT_TRUE(iter->get()->GetRect() == rect);
}

TEST(MutatorsStack, PushClipRect) {
  MutatorsStack stack;
  auto rect = SkRect::MakeEmpty();
  stack.PushClipRect(rect);
  auto iter = stack.Bottom();
  ASSERT_TRUE(iter->get()->GetType() == MutatorType::kClipRect);
  ASSERT_TRUE(iter->get()->GetRect() == rect);
}

TEST(MutatorsStack, PushClipRRect) {
  MutatorsStack stack;
  auto rrect = SkRRect::MakeEmpty();
  stack.PushClipRRect(rrect);
  auto iter = stack.Bottom();
  ASSERT_TRUE(iter->get()->GetType() == MutatorType::kClipRRect);
  ASSERT_TRUE(iter->get()->GetRRect() == rrect);
}

TEST(MutatorsStack, PushClipPath) {
  MutatorsStack stack;
  SkPath path;
  stack.PushClipPath(path);
  auto iter = stack.Bottom();
  ASSERT_TRUE(iter->get()->GetType() == flutter::MutatorType::kClipPath);
  ASSERT_TRUE(iter->get()->GetPath() == path);
}

TEST(MutatorsStack, PushTransform) {
  MutatorsStack stack;
  SkMatrix matrix;
  matrix.setIdentity();
  stack.PushTransform(matrix);
  auto iter = stack.Bottom();
  ASSERT_TRUE(iter->get()->GetType() == MutatorType::kTransform);
  ASSERT_TRUE(iter->get()->GetMatrix() == matrix);
}

TEST(MutatorsStack, PushOpacity) {
  MutatorsStack stack;
  int alpha = 240;
  stack.PushOpacity(alpha);
  auto iter = stack.Bottom();
  ASSERT_TRUE(iter->get()->GetType() == MutatorType::kOpacity);
  ASSERT_TRUE(iter->get()->GetAlpha() == 240);
}

TEST(MutatorsStack, PushBackdropFilter) {
  MutatorsStack stack;
  const int num_of_mutators = 10;
  for (int i = 0; i < num_of_mutators; i++) {
    auto filter = DlImageFilter::MakeBlur(i, 5, DlTileMode::kClamp);
    stack.PushBackdropFilter(filter, SkRect::MakeXYWH(i, i, i, i));
  }

  auto iter = stack.Begin();
  int i = 0;
  while (iter != stack.End()) {
    ASSERT_EQ(iter->get()->GetType(), MutatorType::kBackdropFilter);
    ASSERT_EQ(iter->get()->GetFilterMutation().GetFilter().asBlur()->sigma_x(),
              i);
    ASSERT_EQ(iter->get()->GetFilterMutation().GetFilterRect().x(), i);
    ASSERT_EQ(iter->get()->GetFilterMutation().GetFilterRect().x(), i);
    ASSERT_EQ(iter->get()->GetFilterMutation().GetFilterRect().width(), i);
    ASSERT_EQ(iter->get()->GetFilterMutation().GetFilterRect().height(), i);
    ++iter;
    ++i;
  }
  ASSERT_EQ(i, num_of_mutators);
}

TEST(MutatorsStack, Pop) {
  MutatorsStack stack;
  SkMatrix matrix;
  matrix.setIdentity();
  stack.PushTransform(matrix);
  stack.Pop();
  auto iter = stack.Bottom();
  ASSERT_TRUE(iter == stack.Top());
}

TEST(MutatorsStack, Traversal) {
  MutatorsStack stack;
  SkMatrix matrix;
  matrix.setIdentity();
  stack.PushTransform(matrix);
  auto rect = SkRect::MakeEmpty();
  stack.PushClipRect(rect);
  auto rrect = SkRRect::MakeEmpty();
  stack.PushClipRRect(rrect);
  auto iter = stack.Bottom();
  int index = 0;
  while (iter != stack.Top()) {
    switch (index) {
      case 0:
        ASSERT_TRUE(iter->get()->GetType() == MutatorType::kClipRRect);
        ASSERT_TRUE(iter->get()->GetRRect() == rrect);
        break;
      case 1:
        ASSERT_TRUE(iter->get()->GetType() == MutatorType::kClipRect);
        ASSERT_TRUE(iter->get()->GetRect() == rect);
        break;
      case 2:
        ASSERT_TRUE(iter->get()->GetType() == MutatorType::kTransform);
        ASSERT_TRUE(iter->get()->GetMatrix() == matrix);
        break;
      default:
        break;
    }
    ++iter;
    ++index;
  }
}

TEST(MutatorsStack, Equality) {
  MutatorsStack stack;
  SkMatrix matrix = SkMatrix::Scale(1, 1);
  stack.PushTransform(matrix);
  SkRect rect = SkRect::MakeEmpty();
  stack.PushClipRect(rect);
  SkRRect rrect = SkRRect::MakeEmpty();
  stack.PushClipRRect(rrect);
  SkPath path;
  stack.PushClipPath(path);
  int alpha = 240;
  stack.PushOpacity(alpha);
  auto filter = DlImageFilter::MakeBlur(5, 5, DlTileMode::kClamp);
  stack.PushBackdropFilter(filter, SkRect::MakeEmpty());

  MutatorsStack stack_other;
  SkMatrix matrix_other = SkMatrix::Scale(1, 1);
  stack_other.PushTransform(matrix_other);
  SkRect rect_other = SkRect::MakeEmpty();
  stack_other.PushClipRect(rect_other);
  SkRRect rrect_other = SkRRect::MakeEmpty();
  stack_other.PushClipRRect(rrect_other);
  SkPath other_path;
  stack_other.PushClipPath(other_path);
  int other_alpha = 240;
  stack_other.PushOpacity(other_alpha);
  auto other_filter = DlImageFilter::MakeBlur(5, 5, DlTileMode::kClamp);
  stack_other.PushBackdropFilter(other_filter, SkRect::MakeEmpty());

  ASSERT_TRUE(stack == stack_other);
}

TEST(Mutator, Initialization) {
  SkRect rect = SkRect::MakeEmpty();
  Mutator mutator = Mutator(rect);
  ASSERT_TRUE(mutator.GetType() == MutatorType::kClipRect);
  ASSERT_TRUE(mutator.GetRect() == rect);

  SkRRect rrect = SkRRect::MakeEmpty();
  Mutator mutator2 = Mutator(rrect);
  ASSERT_TRUE(mutator2.GetType() == MutatorType::kClipRRect);
  ASSERT_TRUE(mutator2.GetRRect() == rrect);

  SkPath path;
  Mutator mutator3 = Mutator(path);
  ASSERT_TRUE(mutator3.GetType() == MutatorType::kClipPath);
  ASSERT_TRUE(mutator3.GetPath() == path);

  SkMatrix matrix;
  matrix.setIdentity();
  Mutator mutator4 = Mutator(matrix);
  ASSERT_TRUE(mutator4.GetType() == MutatorType::kTransform);
  ASSERT_TRUE(mutator4.GetMatrix() == matrix);

  int alpha = 240;
  Mutator mutator5 = Mutator(alpha);
  ASSERT_TRUE(mutator5.GetType() == MutatorType::kOpacity);

  auto filter = DlImageFilter::MakeBlur(5, 5, DlTileMode::kClamp);
  Mutator mutator6 = Mutator(filter, SkRect::MakeEmpty());
  ASSERT_TRUE(mutator6.GetType() == MutatorType::kBackdropFilter);
  ASSERT_TRUE(mutator6.GetFilterMutation().GetFilter() == *filter);
}

TEST(Mutator, CopyConstructor) {
  SkRect rect = SkRect::MakeEmpty();
  Mutator mutator = Mutator(rect);
  Mutator copy = Mutator(mutator);
  ASSERT_TRUE(mutator == copy);

  SkRRect rrect = SkRRect::MakeEmpty();
  Mutator mutator2 = Mutator(rrect);
  Mutator copy2 = Mutator(mutator2);
  ASSERT_TRUE(mutator2 == copy2);

  SkPath path;
  Mutator mutator3 = Mutator(path);
  Mutator copy3 = Mutator(mutator3);
  ASSERT_TRUE(mutator3 == copy3);

  SkMatrix matrix;
  matrix.setIdentity();
  Mutator mutator4 = Mutator(matrix);
  Mutator copy4 = Mutator(mutator4);
  ASSERT_TRUE(mutator4 == copy4);

  int alpha = 240;
  Mutator mutator5 = Mutator(alpha);
  Mutator copy5 = Mutator(mutator5);
  ASSERT_TRUE(mutator5 == copy5);

  auto filter = DlImageFilter::MakeBlur(5, 5, DlTileMode::kClamp);
  Mutator mutator6 = Mutator(filter, SkRect::MakeEmpty());
  Mutator copy6 = Mutator(mutator6);
  ASSERT_TRUE(mutator6 == copy6);
}

TEST(Mutator, Equality) {
  SkMatrix matrix;
  matrix.setIdentity();
  Mutator mutator = Mutator(matrix);
  Mutator other_mutator = Mutator(matrix);
  ASSERT_TRUE(mutator == other_mutator);

  SkRect rect = SkRect::MakeEmpty();
  Mutator mutator2 = Mutator(rect);
  Mutator other_mutator2 = Mutator(rect);
  ASSERT_TRUE(mutator2 == other_mutator2);

  SkRRect rrect = SkRRect::MakeEmpty();
  Mutator mutator3 = Mutator(rrect);
  Mutator other_mutator3 = Mutator(rrect);
  ASSERT_TRUE(mutator3 == other_mutator3);

  SkPath path;
  flutter::Mutator mutator4 = flutter::Mutator(path);
  flutter::Mutator other_mutator4 = flutter::Mutator(path);
  ASSERT_TRUE(mutator4 == other_mutator4);
  ASSERT_FALSE(mutator2 == mutator);
  int alpha = 240;
  Mutator mutator5 = Mutator(alpha);
  Mutator other_mutator5 = Mutator(alpha);
  ASSERT_TRUE(mutator5 == other_mutator5);

  auto filter1 = DlImageFilter::MakeBlur(5, 5, DlTileMode::kClamp);
  auto filter2 = DlImageFilter::MakeBlur(5, 5, DlTileMode::kClamp);
  Mutator mutator6 = Mutator(filter1, SkRect::MakeEmpty());
  Mutator other_mutator6 = Mutator(filter2, SkRect::MakeEmpty());
  ASSERT_TRUE(mutator6 == other_mutator6);
}

TEST(Mutator, UnEquality) {
  SkRect rect = SkRect::MakeEmpty();
  Mutator mutator = Mutator(rect);
  SkMatrix matrix;
  matrix.setIdentity();
  Mutator not_equal_mutator = Mutator(matrix);
  ASSERT_TRUE(not_equal_mutator != mutator);

  int alpha = 240;
  int alpha2 = 241;
  Mutator mutator2 = Mutator(alpha);
  Mutator other_mutator2 = Mutator(alpha2);
  ASSERT_TRUE(mutator2 != other_mutator2);

  auto filter = DlImageFilter::MakeBlur(5, 5, DlTileMode::kClamp);
  auto filter2 = DlImageFilter::MakeBlur(10, 10, DlTileMode::kClamp);
  Mutator mutator3 = Mutator(filter, SkRect::MakeEmpty());
  Mutator other_mutator3 = Mutator(filter2, SkRect::MakeEmpty());
  ASSERT_TRUE(mutator3 != other_mutator3);
}

}  // namespace testing
}  // namespace flutter
