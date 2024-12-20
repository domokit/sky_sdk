// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/dl_storage.h"

#include "flutter/testing/testing.h"

namespace flutter {
namespace testing {

TEST(DisplayListStorage, DefaultConstructed) {
  DisplayListStorage storage;
  EXPECT_EQ(storage.base(), nullptr);
  EXPECT_EQ(storage.size(), 0u);
  EXPECT_EQ(storage.capacity(), 0u);
}

TEST(DisplayListStorage, Allocation) {
  DisplayListStorage storage;
  EXPECT_NE(storage.allocate(10u), nullptr);
  EXPECT_NE(storage.base(), nullptr);
  EXPECT_EQ(storage.size(), 10u);
  EXPECT_EQ(storage.capacity(), DisplayListStorage::kDLPageSize);
}

TEST(DisplayListStorage, PostMove) {
  DisplayListStorage original;
  EXPECT_NE(original.allocate(10u), nullptr);

  DisplayListStorage moved = std::move(original);

  // NOLINTBEGIN(bugprone-use-after-move)
  // NOLINTBEGIN(clang-analyzer-cplusplus.Move)
  EXPECT_EQ(original.base(), nullptr);
  EXPECT_EQ(original.size(), 0u);
  EXPECT_EQ(original.capacity(), 0u);
  // NOLINTEND(clang-analyzer-cplusplus.Move)
  // NOLINTEND(bugprone-use-after-move)

  EXPECT_NE(moved.base(), nullptr);
  EXPECT_EQ(moved.size(), 10u);
  EXPECT_EQ(moved.capacity(), DisplayListStorage::kDLPageSize);
}

}  // namespace testing
}  // namespace flutter
