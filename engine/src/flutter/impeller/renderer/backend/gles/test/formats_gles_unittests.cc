// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/testing/testing.h"  // IWYU pragma: keep
#include "gtest/gtest.h"
#include "impeller/renderer/backend/gles/formats_gles.h"

namespace impeller {
namespace testing {

TEST(FormatsGLES, CanFormatFramebufferErrorMessage) {
  ASSERT_EQ(DebugToFramebufferError(GL_FRAMEBUFFER_UNDEFINED),
            "GL_FRAMEBUFFER_UNDEFINED");
  ASSERT_EQ(DebugToFramebufferError(GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT),
            "GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT");
  ASSERT_EQ(
      DebugToFramebufferError(GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT),
      "GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT");
  ASSERT_EQ(DebugToFramebufferError(GL_FRAMEBUFFER_UNSUPPORTED),
            "GL_FRAMEBUFFER_UNSUPPORTED");
  ASSERT_EQ(DebugToFramebufferError(GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE),
            "GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE");
  ASSERT_EQ(DebugToFramebufferError(0), "Unknown error code: 0");
}

}  // namespace testing
}  // namespace impeller
