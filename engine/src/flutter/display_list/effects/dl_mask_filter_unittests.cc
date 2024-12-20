// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/effects/dl_mask_filter.h"
#include "flutter/display_list/testing/dl_test_equality.h"
#include "flutter/display_list/utils/dl_comparable.h"
#include "gtest/gtest.h"

namespace flutter {
namespace testing {

TEST(DisplayListMaskFilter, BlurConstructor) {
  DlBlurMaskFilter filter(DlBlurStyle::kNormal, 5.0);
}

TEST(DisplayListMaskFilter, BlurShared) {
  DlBlurMaskFilter filter(DlBlurStyle::kNormal, 5.0);
  ASSERT_NE(filter.shared().get(), &filter);
  ASSERT_EQ(*filter.shared(), filter);
}

TEST(DisplayListMaskFilter, BlurAsBlur) {
  DlBlurMaskFilter filter(DlBlurStyle::kNormal, 5.0);
  ASSERT_NE(filter.asBlur(), nullptr);
  ASSERT_EQ(filter.asBlur(), &filter);
}

TEST(DisplayListMaskFilter, BlurContents) {
  DlBlurMaskFilter filter(DlBlurStyle::kNormal, 5.0);
  ASSERT_EQ(filter.style(), DlBlurStyle::kNormal);
  ASSERT_EQ(filter.sigma(), 5.0);
}

TEST(DisplayListMaskFilter, BlurEquals) {
  DlBlurMaskFilter filter1(DlBlurStyle::kNormal, 5.0);
  DlBlurMaskFilter filter2(DlBlurStyle::kNormal, 5.0);
  TestEquals(filter1, filter2);
}

TEST(DisplayListMaskFilter, BlurNotEquals) {
  DlBlurMaskFilter filter1(DlBlurStyle::kNormal, 5.0);
  DlBlurMaskFilter filter2(DlBlurStyle::kInner, 5.0);
  DlBlurMaskFilter filter3(DlBlurStyle::kNormal, 6.0);
  TestNotEquals(filter1, filter2, "Blur style differs");
  TestNotEquals(filter1, filter3, "blur radius differs");
}

void testEquals(DlMaskFilter* a, DlMaskFilter* b) {
  // a and b have the same nullness or values
  ASSERT_TRUE(Equals(a, b));
  ASSERT_FALSE(NotEquals(a, b));
  ASSERT_TRUE(Equals(b, a));
  ASSERT_FALSE(NotEquals(b, a));
}

void testNotEquals(DlMaskFilter* a, DlMaskFilter* b) {
  // a and b do not have the same nullness or values
  ASSERT_FALSE(Equals(a, b));
  ASSERT_TRUE(NotEquals(a, b));
  ASSERT_FALSE(Equals(b, a));
  ASSERT_TRUE(NotEquals(b, a));
}

void testEquals(const std::shared_ptr<const DlMaskFilter>& a, DlMaskFilter* b) {
  // a and b have the same nullness or values
  ASSERT_TRUE(Equals(a, b));
  ASSERT_FALSE(NotEquals(a, b));
  ASSERT_TRUE(Equals(b, a));
  ASSERT_FALSE(NotEquals(b, a));
}

void testNotEquals(const std::shared_ptr<const DlMaskFilter>& a,
                   DlMaskFilter* b) {
  // a and b do not have the same nullness or values
  ASSERT_FALSE(Equals(a, b));
  ASSERT_TRUE(NotEquals(a, b));
  ASSERT_FALSE(Equals(b, a));
  ASSERT_TRUE(NotEquals(b, a));
}

void testEquals(const std::shared_ptr<const DlMaskFilter>& a,
                const std::shared_ptr<const DlMaskFilter>& b) {
  // a and b have the same nullness or values
  ASSERT_TRUE(Equals(a, b));
  ASSERT_FALSE(NotEquals(a, b));
  ASSERT_TRUE(Equals(b, a));
  ASSERT_FALSE(NotEquals(b, a));
}

void testNotEquals(const std::shared_ptr<const DlMaskFilter>& a,
                   const std::shared_ptr<const DlMaskFilter>& b) {
  // a and b do not have the same nullness or values
  ASSERT_FALSE(Equals(a, b));
  ASSERT_TRUE(NotEquals(a, b));
  ASSERT_FALSE(Equals(b, a));
  ASSERT_TRUE(NotEquals(b, a));
}

TEST(DisplayListMaskFilter, ComparableTemplates) {
  DlBlurMaskFilter filter1a(DlBlurStyle::kNormal, 3.0);
  DlBlurMaskFilter filter1b(DlBlurStyle::kNormal, 3.0);
  DlBlurMaskFilter filter2(DlBlurStyle::kNormal, 5.0);
  std::shared_ptr<DlMaskFilter> shared_null;

  // null to null
  testEquals(nullptr, nullptr);
  testEquals(shared_null, nullptr);
  testEquals(shared_null, shared_null);

  // ptr to null
  testNotEquals(&filter1a, nullptr);
  testNotEquals(&filter1b, nullptr);
  testNotEquals(&filter2, nullptr);

  // shared_ptr to null and shared_null to ptr
  testNotEquals(filter1a.shared(), nullptr);
  testNotEquals(filter1b.shared(), nullptr);
  testNotEquals(filter2.shared(), nullptr);
  testNotEquals(shared_null, &filter1a);
  testNotEquals(shared_null, &filter1b);
  testNotEquals(shared_null, &filter2);

  // ptr to ptr
  testEquals(&filter1a, &filter1a);
  testEquals(&filter1a, &filter1b);
  testEquals(&filter1b, &filter1b);
  testEquals(&filter2, &filter2);
  testNotEquals(&filter1a, &filter2);

  // shared_ptr to ptr
  testEquals(filter1a.shared(), &filter1a);
  testEquals(filter1a.shared(), &filter1b);
  testEquals(filter1b.shared(), &filter1b);
  testEquals(filter2.shared(), &filter2);
  testNotEquals(filter1a.shared(), &filter2);
  testNotEquals(filter1b.shared(), &filter2);
  testNotEquals(filter2.shared(), &filter1a);
  testNotEquals(filter2.shared(), &filter1b);

  // shared_ptr to shared_ptr
  testEquals(filter1a.shared(), filter1a.shared());
  testEquals(filter1a.shared(), filter1b.shared());
  testEquals(filter1b.shared(), filter1b.shared());
  testEquals(filter2.shared(), filter2.shared());
  testNotEquals(filter1a.shared(), filter2.shared());
  testNotEquals(filter1b.shared(), filter2.shared());
  testNotEquals(filter2.shared(), filter1a.shared());
  testNotEquals(filter2.shared(), filter1b.shared());
}

}  // namespace testing
}  // namespace flutter
