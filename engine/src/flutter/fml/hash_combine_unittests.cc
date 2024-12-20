// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/hash_combine.h"

#include "flutter/testing/testing.h"

namespace fml {
namespace testing {

TEST(HashCombineTest, CanHash) {
  ASSERT_EQ(HashCombine(), HashCombine());
  ASSERT_EQ(HashCombine("Hello"), HashCombine("Hello"));
  ASSERT_NE(HashCombine("Hello"), HashCombine("World"));
  ASSERT_EQ(HashCombine("Hello", "World"), HashCombine("Hello", "World"));
  ASSERT_NE(HashCombine("World", "Hello"), HashCombine("Hello", "World"));
  ASSERT_EQ(HashCombine(12u), HashCombine(12u));
  ASSERT_NE(HashCombine(12u), HashCombine(12.0f));
  ASSERT_EQ(HashCombine('a'), HashCombine('a'));
}

}  // namespace testing
}  // namespace fml
