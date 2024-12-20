// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/common/text_range.h"

#include "gtest/gtest.h"

namespace flutter {

TEST(TextRange, TextRangeFromPositionZero) {
  TextRange range(0);
  EXPECT_EQ(range.base(), size_t(0));
  EXPECT_EQ(range.extent(), size_t(0));
  EXPECT_EQ(range.start(), size_t(0));
  EXPECT_EQ(range.end(), size_t(0));
  EXPECT_EQ(range.length(), size_t(0));
  EXPECT_EQ(range.position(), size_t(0));
  EXPECT_TRUE(range.collapsed());
}

TEST(TextRange, TextRangeFromPositionNonZero) {
  TextRange range(3);
  EXPECT_EQ(range.base(), size_t(3));
  EXPECT_EQ(range.extent(), size_t(3));
  EXPECT_EQ(range.start(), size_t(3));
  EXPECT_EQ(range.end(), size_t(3));
  EXPECT_EQ(range.length(), size_t(0));
  EXPECT_EQ(range.position(), size_t(3));
  EXPECT_TRUE(range.collapsed());
}

TEST(TextRange, TextRangeFromRange) {
  TextRange range(3, 7);
  EXPECT_EQ(range.base(), size_t(3));
  EXPECT_EQ(range.extent(), size_t(7));
  EXPECT_EQ(range.start(), size_t(3));
  EXPECT_EQ(range.end(), size_t(7));
  EXPECT_EQ(range.length(), size_t(4));
  EXPECT_FALSE(range.collapsed());
}

TEST(TextRange, TextRangeFromReversedRange) {
  TextRange range(7, 3);
  EXPECT_EQ(range.base(), size_t(7));
  EXPECT_EQ(range.extent(), size_t(3));
  EXPECT_EQ(range.start(), size_t(3));
  EXPECT_EQ(range.end(), size_t(7));
  EXPECT_EQ(range.length(), size_t(4));
  EXPECT_FALSE(range.collapsed());
}

TEST(TextRange, SetBase) {
  TextRange range(3, 7);
  range.set_base(4);
  EXPECT_EQ(range.base(), size_t(4));
  EXPECT_EQ(range.extent(), size_t(7));
}

TEST(TextRange, SetBaseReversed) {
  TextRange range(7, 3);
  range.set_base(5);
  EXPECT_EQ(range.base(), size_t(5));
  EXPECT_EQ(range.extent(), size_t(3));
}

TEST(TextRange, SetExtent) {
  TextRange range(3, 7);
  range.set_extent(6);
  EXPECT_EQ(range.base(), size_t(3));
  EXPECT_EQ(range.extent(), size_t(6));
}

TEST(TextRange, SetExtentReversed) {
  TextRange range(7, 3);
  range.set_extent(4);
  EXPECT_EQ(range.base(), size_t(7));
  EXPECT_EQ(range.extent(), size_t(4));
}

TEST(TextRange, SetStart) {
  TextRange range(3, 7);
  range.set_start(5);
  EXPECT_EQ(range.base(), size_t(5));
  EXPECT_EQ(range.extent(), size_t(7));
}

TEST(TextRange, SetStartReversed) {
  TextRange range(7, 3);
  range.set_start(5);
  EXPECT_EQ(range.base(), size_t(7));
  EXPECT_EQ(range.extent(), size_t(5));
}

TEST(TextRange, SetEnd) {
  TextRange range(3, 7);
  range.set_end(6);
  EXPECT_EQ(range.base(), size_t(3));
  EXPECT_EQ(range.extent(), size_t(6));
}

TEST(TextRange, SetEndReversed) {
  TextRange range(7, 3);
  range.set_end(5);
  EXPECT_EQ(range.base(), size_t(5));
  EXPECT_EQ(range.extent(), size_t(3));
}

TEST(TextRange, ContainsPreStartPosition) {
  TextRange range(2, 6);
  EXPECT_FALSE(range.Contains(1));
}

TEST(TextRange, ContainsStartPosition) {
  TextRange range(2, 6);
  EXPECT_TRUE(range.Contains(2));
}

TEST(TextRange, ContainsMiddlePosition) {
  TextRange range(2, 6);
  EXPECT_TRUE(range.Contains(3));
  EXPECT_TRUE(range.Contains(4));
}

TEST(TextRange, ContainsEndPosition) {
  TextRange range(2, 6);
  EXPECT_TRUE(range.Contains(6));
}

TEST(TextRange, ContainsPostEndPosition) {
  TextRange range(2, 6);
  EXPECT_FALSE(range.Contains(7));
}

TEST(TextRange, ContainsPreStartPositionReversed) {
  TextRange range(6, 2);
  EXPECT_FALSE(range.Contains(1));
}

TEST(TextRange, ContainsStartPositionReversed) {
  TextRange range(6, 2);
  EXPECT_TRUE(range.Contains(2));
}

TEST(TextRange, ContainsMiddlePositionReversed) {
  TextRange range(6, 2);
  EXPECT_TRUE(range.Contains(3));
  EXPECT_TRUE(range.Contains(4));
}

TEST(TextRange, ContainsEndPositionReversed) {
  TextRange range(6, 2);
  EXPECT_TRUE(range.Contains(6));
}

TEST(TextRange, ContainsPostEndPositionReversed) {
  TextRange range(6, 2);
  EXPECT_FALSE(range.Contains(7));
}

TEST(TextRange, ContainsRangePreStartPosition) {
  TextRange range(2, 6);
  EXPECT_FALSE(range.Contains(TextRange(0, 1)));
}

TEST(TextRange, ContainsRangeSpanningStartPosition) {
  TextRange range(2, 6);
  EXPECT_FALSE(range.Contains(TextRange(1, 3)));
}

TEST(TextRange, ContainsRangeStartPosition) {
  TextRange range(2, 6);
  EXPECT_TRUE(range.Contains(TextRange(2)));
}

TEST(TextRange, ContainsRangeMiddlePosition) {
  TextRange range(2, 6);
  EXPECT_TRUE(range.Contains(TextRange(3, 4)));
  EXPECT_TRUE(range.Contains(TextRange(4, 5)));
}

TEST(TextRange, ContainsRangeEndPosition) {
  TextRange range(2, 6);
  EXPECT_TRUE(range.Contains(TextRange(6)));
}

TEST(TextRange, ContainsRangeSpanningEndPosition) {
  TextRange range(2, 6);
  EXPECT_FALSE(range.Contains(TextRange(5, 7)));
}

TEST(TextRange, ContainsRangePostEndPosition) {
  TextRange range(2, 6);
  EXPECT_FALSE(range.Contains(TextRange(6, 7)));
}

TEST(TextRange, ContainsRangePreStartPositionReversed) {
  TextRange range(6, 2);
  EXPECT_FALSE(range.Contains(TextRange(0, 1)));
}

TEST(TextRange, ContainsRangeSpanningStartPositionReversed) {
  TextRange range(6, 2);
  EXPECT_FALSE(range.Contains(TextRange(1, 3)));
}

TEST(TextRange, ContainsRangeStartPositionReversed) {
  TextRange range(6, 2);
  EXPECT_TRUE(range.Contains(TextRange(2)));
}

TEST(TextRange, ContainsRangeMiddlePositionReversed) {
  TextRange range(6, 2);
  EXPECT_TRUE(range.Contains(TextRange(3, 4)));
  EXPECT_TRUE(range.Contains(TextRange(4, 5)));
}

TEST(TextRange, ContainsRangeSpanningEndPositionReversed) {
  TextRange range(6, 2);
  EXPECT_FALSE(range.Contains(TextRange(5, 7)));
}

TEST(TextRange, ContainsRangeEndPositionReversed) {
  TextRange range(6, 2);
  EXPECT_TRUE(range.Contains(TextRange(5)));
}

TEST(TextRange, ContainsRangePostEndPositionReversed) {
  TextRange range(6, 2);
  EXPECT_FALSE(range.Contains(TextRange(6, 7)));
}

TEST(TextRange, ReversedForwardRange) {
  TextRange range(2, 6);
  EXPECT_FALSE(range.reversed());
}

TEST(TextRange, ReversedCollapsedRange) {
  TextRange range(2, 2);
  EXPECT_FALSE(range.reversed());
}

TEST(TextRange, ReversedReversedRange) {
  TextRange range(6, 2);
  EXPECT_TRUE(range.reversed());
}

}  // namespace flutter
