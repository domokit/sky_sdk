// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <memory>
#include "flutter/testing/testing.h"
#include "gtest/gtest.h"
#include "impeller/entity/contents/filters/inputs/filter_input.h"
#include "impeller/entity/entity.h"
#include "impeller/geometry/geometry_asserts.h"

namespace impeller {
namespace testing {

TEST(FilterInputTest, CanSetLocalTransformForTexture) {
  std::shared_ptr<Texture> texture = nullptr;
  auto input =
      FilterInput::Make(texture, Matrix::MakeTranslation({1.0, 0.0, 0.0}));
  Entity e;
  e.SetTransform(Matrix::MakeTranslation({0.0, 2.0, 0.0}));

  ASSERT_MATRIX_NEAR(input->GetLocalTransform(e),
                     Matrix::MakeTranslation({1.0, 0.0, 0.0}));
  ASSERT_MATRIX_NEAR(input->GetTransform(e),
                     Matrix::MakeTranslation({1.0, 2.0, 0.0}));
}

}  // namespace testing
}  // namespace impeller
