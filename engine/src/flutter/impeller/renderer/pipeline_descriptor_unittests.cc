// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <unordered_set>

#include "flutter/testing/testing.h"
#include "impeller/renderer/pipeline_descriptor.h"

namespace impeller {
namespace testing {

TEST(PipelineDescriptorTest, PrimitiveTypeHashEquality) {
  PipelineDescriptor descA;
  PipelineDescriptor descB;

  ASSERT_TRUE(descA.IsEqual(descB));
  ASSERT_EQ(descA.GetHash(), descB.GetHash());

  descA.SetPrimitiveType(PrimitiveType::kTriangleStrip);

  ASSERT_FALSE(descA.IsEqual(descB));
  ASSERT_NE(descA.GetHash(), descB.GetHash());
}

}  // namespace  testing
}  // namespace impeller
