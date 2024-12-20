// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/embedder/tests/embedder_test_context_metal.h"

#include <memory>
#include <utility>

#include "embedder.h"
#include "flutter/fml/logging.h"
#include "flutter/shell/platform/embedder/tests/embedder_test_compositor_metal.h"

namespace flutter::testing {

EmbedderTestContextMetal::EmbedderTestContextMetal(std::string assets_path)
    : EmbedderTestContext(std::move(assets_path)) {
  metal_context_ = std::make_unique<TestMetalContext>();
  renderer_config_.type = FlutterRendererType::kMetal;
  renderer_config_.metal = {
      .struct_size = sizeof(FlutterMetalRendererConfig),
      .device = (__bridge FlutterMetalDeviceHandle)metal_context_->GetMetalDevice(),
      .present_command_queue =
          (__bridge FlutterMetalCommandQueueHandle)metal_context_->GetMetalCommandQueue(),
      .get_next_drawable_callback =
          [](void* user_data, const FlutterFrameInfo* frame_info) {
            return reinterpret_cast<EmbedderTestContextMetal*>(user_data)->GetNextDrawable(
                frame_info);
          },
      .present_drawable_callback = [](void* user_data, const FlutterMetalTexture* texture) -> bool {
        return reinterpret_cast<EmbedderTestContextMetal*>(user_data)->Present(texture->texture_id);
      },
      .external_texture_frame_callback = [](void* user_data, int64_t texture_id, size_t width,
                                            size_t height,
                                            FlutterMetalExternalTexture* texture_out) -> bool {
        return reinterpret_cast<EmbedderTestContextMetal*>(user_data)->PopulateExternalTexture(
            texture_id, width, height, texture_out);
      },
  };
}

EmbedderTestContextMetal::~EmbedderTestContextMetal() {}

EmbedderTestContextType EmbedderTestContextMetal::GetContextType() const {
  return EmbedderTestContextType::kMetalContext;
}

size_t EmbedderTestContextMetal::GetSurfacePresentCount() const {
  return present_count_;
}

TestMetalContext* EmbedderTestContextMetal::GetTestMetalContext() {
  return metal_context_.get();
}

TestMetalSurface* EmbedderTestContextMetal::GetTestMetalSurface() {
  return metal_surface_.get();
}

void EmbedderTestContextMetal::SetPresentCallback(PresentCallback present_callback) {
  present_callback_ = std::move(present_callback);
}

bool EmbedderTestContextMetal::Present(int64_t texture_id) {
  FireRootSurfacePresentCallbackIfPresent(
      [&]() { return metal_surface_->GetRasterSurfaceSnapshot(); });
  present_count_++;
  if (present_callback_ != nullptr) {
    return present_callback_(texture_id);
  }
  return metal_context_->Present(texture_id);
}

void EmbedderTestContextMetal::SetExternalTextureCallback(
    TestExternalTextureCallback external_texture_frame_callback) {
  external_texture_frame_callback_ = std::move(external_texture_frame_callback);
}

bool EmbedderTestContextMetal::PopulateExternalTexture(int64_t texture_id,
                                                       size_t w,
                                                       size_t h,
                                                       FlutterMetalExternalTexture* output) {
  if (external_texture_frame_callback_ != nullptr) {
    return external_texture_frame_callback_(texture_id, w, h, output);
  } else {
    return false;
  }
}

void EmbedderTestContextMetal::SetNextDrawableCallback(
    NextDrawableCallback next_drawable_callback) {
  next_drawable_callback_ = std::move(next_drawable_callback);
}

FlutterMetalTexture EmbedderTestContextMetal::GetNextDrawable(const FlutterFrameInfo* frame_info) {
  if (next_drawable_callback_ != nullptr) {
    return next_drawable_callback_(frame_info);
  }

  auto texture_info = metal_surface_->GetTextureInfo();
  FlutterMetalTexture texture = {};
  texture.struct_size = sizeof(FlutterMetalTexture);
  texture.texture_id = texture_info.texture_id;
  texture.texture = reinterpret_cast<FlutterMetalTextureHandle>(texture_info.texture);
  return texture;
}

void EmbedderTestContextMetal::SetSurface(SkISize surface_size) {
  FML_CHECK(surface_size_.isEmpty());
  surface_size_ = surface_size;
  metal_surface_ = TestMetalSurface::Create(*metal_context_, surface_size_);
}

void EmbedderTestContextMetal::SetupCompositor() {
  FML_CHECK(!compositor_) << "Already set up a compositor in this context.";
  FML_CHECK(metal_surface_) << "Set up the Metal surface before setting up a compositor.";
  compositor_ =
      std::make_unique<EmbedderTestCompositorMetal>(surface_size_, metal_surface_->GetGrContext());
}

}  // namespace flutter::testing
