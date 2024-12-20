// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "context_options.h"

#include "gtest/gtest.h"

namespace flutter {
namespace testing {

TEST(ContextOptionsTest, OpenGLDisablesStencilBuffers) {
  auto options = MakeDefaultContextOptions(flutter::ContextType::kRender,
                                           GrBackendApi::kOpenGL);
  EXPECT_TRUE(options.fAvoidStencilBuffers);
}

}  // namespace testing
}  // namespace flutter
