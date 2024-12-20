// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/android/surface_texture_external_texture_gl_impeller.h"

#include "flutter/impeller/display_list/dl_image_impeller.h"

namespace flutter {

SurfaceTextureExternalTextureGLImpeller::
    SurfaceTextureExternalTextureGLImpeller(
        const std::shared_ptr<impeller::ContextGLES>& context,
        int64_t id,
        const fml::jni::ScopedJavaGlobalRef<jobject>& surface_texture,
        const std::shared_ptr<PlatformViewAndroidJNI>& jni_facade)
    : SurfaceTextureExternalTexture(id, surface_texture, jni_facade),
      impeller_context_(context) {}

SurfaceTextureExternalTextureGLImpeller::
    ~SurfaceTextureExternalTextureGLImpeller() = default;

void SurfaceTextureExternalTextureGLImpeller::ProcessFrame(
    PaintContext& context,
    const SkRect& bounds) {
  if (state_ == AttachmentState::kUninitialized) {
    // Generate the texture handle.
    impeller::TextureDescriptor desc;
    desc.type = impeller::TextureType::kTextureExternalOES;
    desc.storage_mode = impeller::StorageMode::kDevicePrivate;
    desc.format = impeller::PixelFormat::kR8G8B8A8UNormInt;
    desc.size = {static_cast<int>(bounds.width()),
                 static_cast<int>(bounds.height())};
    desc.mip_count = 1;
    texture_ = std::make_shared<impeller::TextureGLES>(
        impeller_context_->GetReactor(), desc);
    // The contents will be initialized later in the call to `Attach` instead of
    // by Impeller.
    texture_->MarkContentsInitialized();
    texture_->SetCoordinateSystem(
        impeller::TextureCoordinateSystem::kUploadFromHost);
    auto maybe_handle = texture_->GetGLHandle();
    if (!maybe_handle.has_value()) {
      FML_LOG(ERROR) << "Could not get GL handle from impeller::TextureGLES!";
      return;
    }
    Attach(maybe_handle.value());
  }
  FML_CHECK(state_ == AttachmentState::kAttached);

  // Updates the texture contents and transformation matrix.
  Update();

  dl_image_ = impeller::DlImageImpeller::Make(texture_);
}

void SurfaceTextureExternalTextureGLImpeller::Detach() {
  SurfaceTextureExternalTexture::Detach();
  texture_.reset();
}

}  // namespace flutter
