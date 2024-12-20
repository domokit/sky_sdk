// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/testing/testing.h"  // IWYU pragma: keep
#include "gtest/gtest.h"
#include "impeller/renderer/backend/gles/proc_table_gles.h"
#include "impeller/renderer/backend/gles/test/mock_gles.h"

namespace impeller {
namespace testing {

// This test just checks that the proc table is initialized correctly.
//
// If this test doesn't pass, no test that uses the proc table will pass.
TEST(MockGLES, CanInitialize) {
  auto mock_gles = MockGLES::Init();

  std::string_view vendor(reinterpret_cast<const char*>(
      mock_gles->GetProcTable().GetString(GL_VENDOR)));
  EXPECT_EQ(vendor, "MockGLES");
}

// Tests that if we call a function we have not mocked, it's OK.
TEST(MockGLES, CanCallUnmockedFunction) {
  auto mock_gles = MockGLES::Init();

  auto& gl = mock_gles->GetProcTable();
  gl.DeleteFramebuffers(1, nullptr);

  // Test should still complete.
  // If we end up mocking DeleteFramebuffers, delete this test.
}

}  // namespace testing
}  // namespace impeller
