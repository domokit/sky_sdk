// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_TESTING_TEST_METAL_CONTEXT_H_
#define FLUTTER_TESTING_TEST_METAL_CONTEXT_H_

#include <map>
#include <memory>
#include <mutex>

#include <Metal/Metal.h>

#include "third_party/skia/include/gpu/ganesh/GrDirectContext.h"
#include "third_party/skia/include/ports/SkCFObject.h"

namespace flutter::testing {

struct MetalObjCFields;

class TestMetalContext {
 public:
  struct TextureInfo {
    int64_t texture_id;
    void* texture;
  };

  TestMetalContext();

  ~TestMetalContext();

  id<MTLDevice> GetMetalDevice() const;

  id<MTLCommandQueue> GetMetalCommandQueue() const;

  sk_sp<GrDirectContext> GetSkiaContext() const;

  /// Returns texture_id = -1 when texture creation fails.
  TextureInfo CreateMetalTexture(const SkISize& size);

  bool Present(int64_t texture_id);

  TextureInfo GetTextureInfo(int64_t texture_id);

 private:
  id<MTLDevice> device_;
  id<MTLCommandQueue> command_queue_;
  sk_sp<GrDirectContext> skia_context_;
  std::mutex textures_mutex_;
  int64_t texture_id_ctr_ = 1;                 // guarded by textures_mutex
  std::map<int64_t, sk_cfp<void*>> textures_;  // guarded by textures_mutex
};

}  // namespace flutter::testing

#endif  // FLUTTER_TESTING_TEST_METAL_CONTEXT_H_
