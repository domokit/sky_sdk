// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_TESTING_TEST_METAL_SURFACE_IMPL_H_
#define FLUTTER_TESTING_TEST_METAL_SURFACE_IMPL_H_

#include "flutter/fml/macros.h"
#include "flutter/testing/test_metal_context.h"
#include "flutter/testing/test_metal_surface.h"

#include "third_party/skia/include/core/SkSurface.h"

namespace flutter::testing {

class TestMetalSurfaceImpl : public TestMetalSurface {
 public:
  TestMetalSurfaceImpl(const TestMetalContext& test_metal_context,
                       const SkISize& surface_size);

  TestMetalSurfaceImpl(const TestMetalContext& test_metal_context,
                       int64_t texture_id,
                       const SkISize& surface_size);

  // |TestMetalSurface|
  ~TestMetalSurfaceImpl() override;

 private:
  void Init(const TestMetalContext::TextureInfo& texture_info,
            const SkISize& surface_size);

  const TestMetalContext& test_metal_context_;
  bool is_valid_ = false;
  sk_sp<SkSurface> surface_;
  TestMetalContext::TextureInfo texture_info_;

  // |TestMetalSurface|
  bool IsValid() const override;

  // |TestMetalSurface|
  sk_sp<GrDirectContext> GetGrContext() const override;

  // |TestMetalSurface|
  sk_sp<SkSurface> GetSurface() const override;

  // |TestMetalSurface|
  sk_sp<SkImage> GetRasterSurfaceSnapshot() override;

  // |TestMetalSurface|
  TestMetalContext::TextureInfo GetTextureInfo() override;

  FML_DISALLOW_COPY_AND_ASSIGN(TestMetalSurfaceImpl);
};

}  // namespace flutter::testing

#endif  // FLUTTER_TESTING_TEST_METAL_SURFACE_IMPL_H_
