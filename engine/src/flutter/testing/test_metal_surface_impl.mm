// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/testing/test_metal_surface_impl.h"

#include <Metal/Metal.h>

#include "flutter/fml/logging.h"
#include "flutter/testing/test_metal_context.h"
#include "third_party/skia/include/core/SkCanvas.h"
#include "third_party/skia/include/core/SkColorSpace.h"
#include "third_party/skia/include/core/SkImage.h"
#include "third_party/skia/include/core/SkRefCnt.h"
#include "third_party/skia/include/core/SkSize.h"
#include "third_party/skia/include/core/SkSurface.h"
#include "third_party/skia/include/gpu/GpuTypes.h"
#include "third_party/skia/include/gpu/ganesh/GrBackendSurface.h"
#include "third_party/skia/include/gpu/ganesh/SkSurfaceGanesh.h"
#include "third_party/skia/include/gpu/ganesh/mtl/GrMtlBackendSurface.h"
#include "third_party/skia/include/gpu/ganesh/mtl/GrMtlTypes.h"

static_assert(__has_feature(objc_arc), "ARC must be enabled.");

namespace flutter::testing {

void TestMetalSurfaceImpl::Init(const TestMetalContext::TextureInfo& texture_info,
                                const SkISize& surface_size) {
  id<MTLTexture> texture = (__bridge id<MTLTexture>)texture_info.texture;

  GrMtlTextureInfo skia_texture_info;
  skia_texture_info.fTexture.retain((__bridge GrMTLHandle)texture);
  GrBackendTexture backend_texture = GrBackendTextures::MakeMtl(
      surface_size.width(), surface_size.height(), skgpu::Mipmapped::kNo, skia_texture_info);

  sk_sp<SkSurface> surface = SkSurfaces::WrapBackendTexture(
      test_metal_context_.GetSkiaContext().get(), backend_texture, kTopLeft_GrSurfaceOrigin, 1,
      kBGRA_8888_SkColorType, nullptr, nullptr);

  if (!surface) {
    FML_LOG(ERROR) << "Could not create Skia surface from a Metal texture.";
    return;
  }

  surface_ = std::move(surface);
  texture_info_ = texture_info;
  is_valid_ = true;
}

TestMetalSurfaceImpl::TestMetalSurfaceImpl(const TestMetalContext& test_metal_context,
                                           int64_t texture_id,
                                           const SkISize& surface_size)
    : test_metal_context_(test_metal_context) {
  TestMetalContext::TextureInfo texture_info =
      const_cast<TestMetalContext&>(test_metal_context_).GetTextureInfo(texture_id);
  Init(texture_info, surface_size);
}

TestMetalSurfaceImpl::TestMetalSurfaceImpl(const TestMetalContext& test_metal_context,
                                           const SkISize& surface_size)
    : test_metal_context_(test_metal_context) {
  if (surface_size.isEmpty()) {
    FML_LOG(ERROR) << "Size of test Metal surface was empty.";
    return;
  }
  TestMetalContext::TextureInfo texture_info =
      const_cast<TestMetalContext&>(test_metal_context_).CreateMetalTexture(surface_size);
  Init(texture_info, surface_size);
}

sk_sp<SkImage> TestMetalSurfaceImpl::GetRasterSurfaceSnapshot() {
  if (!IsValid()) {
    return nullptr;
  }

  if (!surface_) {
    FML_LOG(ERROR) << "Aborting snapshot because of on-screen surface "
                      "acquisition failure.";
    return nullptr;
  }

  auto device_snapshot = surface_->makeImageSnapshot();

  if (!device_snapshot) {
    FML_LOG(ERROR) << "Could not create the device snapshot while attempting "
                      "to snapshot the Metal surface.";
    return nullptr;
  }

  auto host_snapshot = device_snapshot->makeRasterImage();

  if (!host_snapshot) {
    FML_LOG(ERROR) << "Could not create the host snapshot while attempting to "
                      "snapshot the Metal surface.";
    return nullptr;
  }

  return host_snapshot;
}

// |TestMetalSurface|
TestMetalSurfaceImpl::~TestMetalSurfaceImpl() = default;

// |TestMetalSurface|
bool TestMetalSurfaceImpl::IsValid() const {
  return is_valid_;
}

// |TestMetalSurface|
sk_sp<GrDirectContext> TestMetalSurfaceImpl::GetGrContext() const {
  return IsValid() ? test_metal_context_.GetSkiaContext() : nullptr;
}

// |TestMetalSurface|
sk_sp<SkSurface> TestMetalSurfaceImpl::GetSurface() const {
  return IsValid() ? surface_ : nullptr;
}

// |TestMetalSurface|
TestMetalContext::TextureInfo TestMetalSurfaceImpl::GetTextureInfo() {
  return IsValid() ? texture_info_ : TestMetalContext::TextureInfo();
}

}  // namespace flutter::testing
