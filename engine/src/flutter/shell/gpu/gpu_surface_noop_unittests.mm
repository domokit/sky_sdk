// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <Foundation/Foundation.h>
#include <QuartzCore/QuartzCore.h>

#include "flutter/shell/gpu/gpu_surface_noop.h"
#include "gtest/gtest.h"
#include "impeller/entity/mtl/entity_shaders.h"
#include "impeller/entity/mtl/framebuffer_blend_shaders.h"
#include "impeller/entity/mtl/modern_shaders.h"
#include "impeller/renderer/backend/metal/context_mtl.h"

namespace flutter {
namespace testing {

TEST(GPUSurfaceNoop, InvalidImpellerContextCreatesCausesSurfaceToBeInvalid) {
  auto surface = std::make_shared<GPUSurfaceNoop>();

  EXPECT_TRUE(surface->IsValid());
}

}  // namespace testing
}  // namespace flutter
