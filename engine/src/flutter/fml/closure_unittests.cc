// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "fml/closure.h"
#include "gtest/gtest.h"

TEST(ScopedCleanupClosureTest, DestructorDoesNothingWhenNoClosureSet) {
  fml::ScopedCleanupClosure cleanup;

  // Nothing should happen.
}

TEST(ScopedCleanupClosureTest, ReleaseDoesNothingWhenNoClosureSet) {
  fml::ScopedCleanupClosure cleanup;

  // Nothing should happen.
  EXPECT_EQ(nullptr, cleanup.Release());
}

TEST(ScopedCleanupClosureTest, ClosureInvokedOnDestructorWhenSetInConstructor) {
  auto invoked = false;

  {
    fml::ScopedCleanupClosure cleanup([&invoked]() { invoked = true; });

    EXPECT_FALSE(invoked);
  }

  EXPECT_TRUE(invoked);
}

TEST(ScopedCleanupClosureTest, ClosureInvokedOnDestructorWhenSet) {
  auto invoked = false;

  {
    fml::ScopedCleanupClosure cleanup;
    cleanup.SetClosure([&invoked]() { invoked = true; });

    EXPECT_FALSE(invoked);
  }

  EXPECT_TRUE(invoked);
}

TEST(ScopedCleanupClosureTest, ClosureNotInvokedWhenMoved) {
  auto invoked = 0;

  {
    fml::ScopedCleanupClosure cleanup([&invoked]() { invoked++; });
    fml::ScopedCleanupClosure cleanup2(std::move(cleanup));

    EXPECT_EQ(0, invoked);
  }

  EXPECT_EQ(1, invoked);
}

TEST(ScopedCleanupClosureTest, ClosureNotInvokedWhenMovedViaAssignment) {
  auto invoked = 0;

  {
    fml::ScopedCleanupClosure cleanup([&invoked]() { invoked++; });
    fml::ScopedCleanupClosure cleanup2;
    cleanup2 = std::move(cleanup);

    EXPECT_EQ(0, invoked);
  }

  EXPECT_EQ(1, invoked);
}
