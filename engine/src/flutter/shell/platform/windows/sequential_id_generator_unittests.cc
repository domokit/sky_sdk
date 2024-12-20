// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/windows/sequential_id_generator.h"

#include "gtest/gtest.h"

namespace flutter {
namespace testing {

TEST(SequentialIdGeneratorTest, RemoveMultipleNumbers) {
  const uint32_t kMinId = 4;
  const uint32_t kMaxId = 128;

  SequentialIdGenerator generator(kMinId, kMaxId);

  EXPECT_EQ(4U, generator.GetGeneratedId(45));
  EXPECT_EQ(5U, generator.GetGeneratedId(55));
  EXPECT_EQ(6U, generator.GetGeneratedId(15));

  generator.ReleaseNumber(45);
  EXPECT_FALSE(generator.HasGeneratedIdFor(45));
  generator.ReleaseNumber(15);
  EXPECT_FALSE(generator.HasGeneratedIdFor(15));

  EXPECT_EQ(5U, generator.GetGeneratedId(55));
  EXPECT_EQ(4U, generator.GetGeneratedId(12));

  generator.ReleaseNumber(12);
  generator.ReleaseNumber(55);
  EXPECT_EQ(4U, generator.GetGeneratedId(0));
}

TEST(SequentialIdGeneratorTest, MaybeRemoveNumbers) {
  const uint32_t kMinId = 0;
  const uint32_t kMaxId = 128;

  SequentialIdGenerator generator(kMinId, kMaxId);

  EXPECT_EQ(0U, generator.GetGeneratedId(42));

  generator.ReleaseNumber(42);
  EXPECT_FALSE(generator.HasGeneratedIdFor(42));
  generator.ReleaseNumber(42);
}

}  // namespace testing
}  // namespace flutter
