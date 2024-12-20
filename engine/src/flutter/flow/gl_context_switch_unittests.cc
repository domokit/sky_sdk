// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#define FML_USED_ON_EMBEDDER

#include "flutter/common/graphics/gl_context_switch.h"

#include <functional>
#include <future>
#include <memory>

#include "flutter/flow/testing/gl_context_switch_test.h"
#include "gtest/gtest.h"

namespace flutter {
namespace testing {

TEST(GLContextSwitchTest, SwitchKeepsContextCurrentWhileInScope) {
  {
    auto test_gl_context = std::make_unique<TestSwitchableGLContext>(0);
    auto context_switch = GLContextSwitch(std::move(test_gl_context));
    ASSERT_EQ(TestSwitchableGLContext::GetCurrentContext(), 0);
  }
  ASSERT_EQ(TestSwitchableGLContext::GetCurrentContext(), -1);
}

}  // namespace testing
}  // namespace flutter
