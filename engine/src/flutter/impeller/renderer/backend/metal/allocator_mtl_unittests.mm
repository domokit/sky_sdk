// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/testing/testing.h"
#include "impeller/core/formats.h"
#include "impeller/core/texture_descriptor.h"
#include "impeller/playground/playground_test.h"
#include "impeller/renderer/backend/metal/allocator_mtl.h"
#include "impeller/renderer/backend/metal/context_mtl.h"
#include "impeller/renderer/backend/metal/formats_mtl.h"
#include "impeller/renderer/backend/metal/lazy_drawable_holder.h"
#include "impeller/renderer/backend/metal/texture_mtl.h"
#include "impeller/renderer/capabilities.h"

#include <QuartzCore/CAMetalLayer.h>
#include <memory>
#include <thread>

#include "gtest/gtest.h"

namespace impeller {
namespace testing {

using AllocatorMTLTest = PlaygroundTest;
INSTANTIATE_METAL_PLAYGROUND_SUITE(AllocatorMTLTest);

TEST_P(AllocatorMTLTest, DebugTraceMemoryStatistics) {
  auto& context_mtl = ContextMTL::Cast(*GetContext());
  const auto& allocator = context_mtl.GetResourceAllocator();

  EXPECT_EQ(allocator->DebugGetHeapUsage().ConvertTo<MebiBytes>().GetSize(),
            0u);

  // Memoryless texture does not increase allocated size.
  {
    TextureDescriptor desc;
    desc.format = PixelFormat::kR8G8B8A8UNormInt;
    desc.storage_mode = StorageMode::kDeviceTransient;
    desc.size = {1024, 1024};
    auto texture_1 = allocator->CreateTexture(desc);

    // Private storage texture increases allocated size.
    desc.storage_mode = StorageMode::kDevicePrivate;
    auto texture_2 = allocator->CreateTexture(desc);

#ifdef IMPELLER_DEBUG
    EXPECT_EQ(allocator->DebugGetHeapUsage().ConvertTo<MebiBytes>().GetSize(),
              4u);
#else
    EXPECT_EQ(allocator->DebugGetHeapUsage().ConvertTo<MebiBytes>().GetSize(),
              0u);
#endif  // IMPELLER_DEBUG

    // Host storage texture increases allocated size.
    desc.storage_mode = StorageMode::kHostVisible;
    auto texture_3 = allocator->CreateTexture(desc);

#ifdef IMPELLER_DEBUG
    EXPECT_EQ(allocator->DebugGetHeapUsage().ConvertTo<MebiBytes>().GetSize(),
              8u);
#else
    EXPECT_EQ(allocator->DebugGetHeapUsage().ConvertTo<MebiBytes>().GetSize(),
              0u);
#endif  // IMPELLER_DEBUG
  }

  // After all textures are out of scope, memory has been decremented.
  EXPECT_EQ(allocator->DebugGetHeapUsage().ConvertTo<MebiBytes>().GetSize(),
            0u);
}

}  // namespace testing
}  // namespace impeller
