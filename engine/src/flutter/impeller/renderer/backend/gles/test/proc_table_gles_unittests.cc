// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <optional>

#include "flutter/testing/testing.h"  // IWYU pragma: keep
#include "gtest/gtest.h"
#include "impeller/renderer/backend/gles/proc_table_gles.h"
#include "impeller/renderer/backend/gles/test/mock_gles.h"

namespace impeller {
namespace testing {

#define EXPECT_AVAILABLE(proc_ivar) \
  EXPECT_TRUE(mock_gles->GetProcTable().proc_ivar.IsAvailable());
#define EXPECT_UNAVAILABLE(proc_ivar) \
  EXPECT_FALSE(mock_gles->GetProcTable().proc_ivar.IsAvailable());

TEST(ProcTableGLES, ResolvesCorrectClearDepthProcOnES) {
  auto mock_gles = MockGLES::Init(std::nullopt, "OpenGL ES 3.0");
  EXPECT_TRUE(mock_gles->GetProcTable().GetDescription()->IsES());

  FOR_EACH_IMPELLER_ES_ONLY_PROC(EXPECT_AVAILABLE);
  FOR_EACH_IMPELLER_DESKTOP_ONLY_PROC(EXPECT_UNAVAILABLE);
}

TEST(ProcTableGLES, ResolvesCorrectClearDepthProcOnDesktopGL) {
  auto mock_gles = MockGLES::Init(std::nullopt, "OpenGL 4.0");
  EXPECT_FALSE(mock_gles->GetProcTable().GetDescription()->IsES());

  FOR_EACH_IMPELLER_DESKTOP_ONLY_PROC(EXPECT_AVAILABLE);
  FOR_EACH_IMPELLER_ES_ONLY_PROC(EXPECT_UNAVAILABLE);
}

}  // namespace testing
}  // namespace impeller
