// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/android/surface_texture_external_texture_gl_skia.h"

#include <GLES2/gl2.h>
#define GL_GLEXT_PROTOTYPES
#include <GLES2/gl2ext.h>

#include "flutter/third_party/skia/include/core/SkColorSpace.h"
#include "flutter/third_party/skia/include/gpu/ganesh/GrBackendSurface.h"
#include "flutter/third_party/skia/include/gpu/ganesh/GrDirectContext.h"
#include "flutter/third_party/skia/include/gpu/ganesh/SkImageGanesh.h"
#include "flutter/third_party/skia/include/gpu/ganesh/gl/GrGLBackendSurface.h"
#include "flutter/third_party/skia/include/gpu/ganesh/gl/GrGLTypes.h"

namespace flutter {

SurfaceTextureExternalTextureGLSkia::SurfaceTextureExternalTextureGLSkia(
    int64_t id,
    const fml::jni::ScopedJavaGlobalRef<jobject>& surface_texture,
    const std::shared_ptr<PlatformViewAndroidJNI>& jni_facade)
    : SurfaceTextureExternalTexture(id, surface_texture, jni_facade) {}

SurfaceTextureExternalTextureGLSkia::~SurfaceTextureExternalTextureGLSkia() {
  if (texture_name_ != 0) {
    glDeleteTextures(1, &texture_name_);
  }
}

void SurfaceTextureExternalTextureGLSkia::ProcessFrame(PaintContext& context,
                                                       const SkRect& bounds) {
  if (state_ == AttachmentState::kUninitialized) {
    // Generate the texture handle.
    glGenTextures(1, &texture_name_);
    Attach(texture_name_);
  }
  FML_CHECK(state_ == AttachmentState::kAttached);

  // Updates the texture contents and transformation matrix.
  Update();

  GrGLTextureInfo textureInfo = {GL_TEXTURE_EXTERNAL_OES, texture_name_,
                                 GL_RGBA8_OES};
  auto backendTexture =
      GrBackendTextures::MakeGL(1, 1, skgpu::Mipmapped::kNo, textureInfo);
  dl_image_ = DlImage::Make(SkImages::BorrowTextureFrom(
      context.gr_context, backendTexture, kTopLeft_GrSurfaceOrigin,
      kRGBA_8888_SkColorType, kPremul_SkAlphaType, nullptr));
}

void SurfaceTextureExternalTextureGLSkia::Detach() {
  SurfaceTextureExternalTexture::Detach();
  if (texture_name_ != 0) {
    glDeleteTextures(1, &texture_name_);
    texture_name_ = 0;
  }
}

}  // namespace flutter
