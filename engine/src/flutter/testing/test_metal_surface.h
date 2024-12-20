// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_TESTING_TEST_METAL_SURFACE_H_
#define FLUTTER_TESTING_TEST_METAL_SURFACE_H_

#include "flutter/fml/macros.h"
#include "flutter/testing/test_metal_context.h"
#include "third_party/skia/include/core/SkSize.h"
#include "third_party/skia/include/core/SkSurface.h"
#include "third_party/skia/include/gpu/ganesh/GrDirectContext.h"

namespace flutter::testing {

//------------------------------------------------------------------------------
/// @brief      Creates a MTLTexture backed SkSurface and context that can be
///             used to render to in unit-tests.
///
class TestMetalSurface {
 public:
  static bool PlatformSupportsMetal();

  static std::unique_ptr<TestMetalSurface> Create(
      const TestMetalContext& test_metal_context,
      SkISize surface_size = SkISize::MakeEmpty());

  static std::unique_ptr<TestMetalSurface> Create(
      const TestMetalContext& test_metal_context,
      int64_t texture_id,
      SkISize surface_size = SkISize::MakeEmpty());

  virtual ~TestMetalSurface();

  virtual bool IsValid() const;

  virtual sk_sp<GrDirectContext> GetGrContext() const;

  virtual sk_sp<SkSurface> GetSurface() const;

  virtual sk_sp<SkImage> GetRasterSurfaceSnapshot();

  virtual TestMetalContext::TextureInfo GetTextureInfo();

 protected:
  TestMetalSurface();

 private:
  std::unique_ptr<TestMetalSurface> impl_;

  explicit TestMetalSurface(std::unique_ptr<TestMetalSurface> impl);

  FML_DISALLOW_COPY_AND_ASSIGN(TestMetalSurface);
};

}  // namespace flutter::testing

#endif  // FLUTTER_TESTING_TEST_METAL_SURFACE_H_
