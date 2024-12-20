// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_DISPLAY_LIST_TESTING_DL_TEST_EQUALITY_H_
#define FLUTTER_DISPLAY_LIST_TESTING_DL_TEST_EQUALITY_H_

#include "flutter/display_list/dl_attributes.h"
#include "flutter/display_list/utils/dl_comparable.h"
#include "gtest/gtest.h"

namespace flutter {
namespace testing {

template <class T, class U>
static void TestEquals(const T& source1, const U& source2) {
  ASSERT_TRUE(source1 == source2);
  ASSERT_TRUE(source2 == source1);
  ASSERT_FALSE(source1 != source2);
  ASSERT_FALSE(source2 != source1);
  ASSERT_EQ(source1, source2);
  ASSERT_EQ(source2, source1);
  ASSERT_TRUE(Equals(&source1, &source2));
  ASSERT_TRUE(Equals(&source2, &source1));
}

template <class T, class U>
static void TestNotEquals(T& source1, U& source2, const std::string& label) {
  ASSERT_FALSE(source1 == source2) << label;
  ASSERT_FALSE(source2 == source1) << label;
  ASSERT_TRUE(source1 != source2) << label;
  ASSERT_TRUE(source2 != source1) << label;
  ASSERT_NE(source1, source2) << label;
  ASSERT_NE(source2, source1) << label;
  ASSERT_TRUE(NotEquals(&source1, &source2));
  ASSERT_TRUE(NotEquals(&source2, &source1));
}

}  // namespace testing
}  // namespace flutter

#endif  // FLUTTER_DISPLAY_LIST_TESTING_DL_TEST_EQUALITY_H_
