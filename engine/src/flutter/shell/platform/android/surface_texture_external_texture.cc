// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/android/surface_texture_external_texture.h"

#include <GLES/glext.h>

#include <utility>

#include "flutter/display_list/effects/dl_color_source.h"
#include "third_party/skia/include/core/SkAlphaType.h"
#include "third_party/skia/include/core/SkColorSpace.h"
#include "third_party/skia/include/core/SkColorType.h"
#include "third_party/skia/include/core/SkImage.h"

namespace flutter {

SurfaceTextureExternalTexture::SurfaceTextureExternalTexture(
    int64_t id,
    const fml::jni::ScopedJavaGlobalRef<jobject>& surface_texture,
    const std::shared_ptr<PlatformViewAndroidJNI>& jni_facade)
    : Texture(id),
      jni_facade_(jni_facade),
      surface_texture_(surface_texture),
      transform_(SkMatrix::I()) {}

SurfaceTextureExternalTexture::~SurfaceTextureExternalTexture() {}

void SurfaceTextureExternalTexture::OnGrContextCreated() {
  state_ = AttachmentState::kUninitialized;
}

void SurfaceTextureExternalTexture::MarkNewFrameAvailable() {
  // NOOP.
}

void SurfaceTextureExternalTexture::Paint(PaintContext& context,
                                          const SkRect& bounds,
                                          bool freeze,
                                          const DlImageSampling sampling) {
  if (state_ == AttachmentState::kDetached) {
    return;
  }
  const bool should_process_frame =
      !freeze || ShouldUpdate() || dl_image_ == nullptr;
  if (should_process_frame) {
    ProcessFrame(context, bounds);
  }
  // If process frame failed, this may not be in attached state.
  if (state_ != AttachmentState::kAttached) {
    return;
  }

  if (!dl_image_) {
    FML_LOG(WARNING)
        << "No DlImage available for SurfaceTextureExternalTexture to paint.";
    return;
  }

  DrawFrame(context, bounds, sampling);
}

void SurfaceTextureExternalTexture::DrawFrame(
    PaintContext& context,
    const SkRect& bounds,
    const DlImageSampling sampling) const {
  auto transform = ToDlMatrix(GetCurrentUVTransformation());

  // Android's SurfaceTexture transform matrix works on texture coordinate
  // lookups in the range 0.0-1.0, while Skia's Shader transform matrix works on
  // the image itself, as if it were inscribed inside a clip rect.
  // An Android transform that scales lookup by 0.5 (displaying 50% of the
  // texture) is the same as a Skia transform by 2.0 (scaling 50% of the image
  // outside of the virtual "clip rect"), so we invert the incoming matrix.

  if (transform.IsIdentity()) {
    context.canvas->DrawImage(dl_image_, SkPoint{0, 0}, sampling,
                              context.paint);
    return;
  }

  if (!transform.IsInvertible()) {
    FML_LOG(FATAL)
        << "Invalid (not invertable) SurfaceTexture transformation matrix";
  }
  transform = transform.Invert();

  DlAutoCanvasRestore autoRestore(context.canvas, true);

  // The incoming texture is vertically flipped, so we flip it
  // back. OpenGL's coordinate system has Positive Y equivalent to up, while
  // Skia's coordinate system has Negative Y equvalent to up.
  context.canvas->Translate(bounds.x(), bounds.y() + bounds.height());
  context.canvas->Scale(bounds.width(), -bounds.height());

  auto source = DlColorSource::MakeImage(
      dl_image_, DlTileMode::kClamp, DlTileMode::kClamp, sampling, &transform);

  DlPaint paintWithShader;
  if (context.paint) {
    paintWithShader = *context.paint;
  }
  paintWithShader.setColorSource(source);
  context.canvas->DrawRect(SkRect::MakeWH(1, 1), paintWithShader);
}

void SurfaceTextureExternalTexture::OnGrContextDestroyed() {
  if (state_ == AttachmentState::kAttached) {
    Detach();
  }
  state_ = AttachmentState::kDetached;
}

void SurfaceTextureExternalTexture::OnTextureUnregistered() {}

void SurfaceTextureExternalTexture::Detach() {
  jni_facade_->SurfaceTextureDetachFromGLContext(
      fml::jni::ScopedJavaLocalRef<jobject>(surface_texture_));
  dl_image_.reset();
}

void SurfaceTextureExternalTexture::Attach(int gl_tex_id) {
  jni_facade_->SurfaceTextureAttachToGLContext(
      fml::jni::ScopedJavaLocalRef<jobject>(surface_texture_), gl_tex_id);
  state_ = AttachmentState::kAttached;
}

bool SurfaceTextureExternalTexture::ShouldUpdate() {
  return jni_facade_->SurfaceTextureShouldUpdate(
      fml::jni::ScopedJavaLocalRef<jobject>(surface_texture_));
}

void SurfaceTextureExternalTexture::Update() {
  jni_facade_->SurfaceTextureUpdateTexImage(
      fml::jni::ScopedJavaLocalRef<jobject>(surface_texture_));
  transform_ = jni_facade_->SurfaceTextureGetTransformMatrix(
      fml::jni::ScopedJavaLocalRef<jobject>(surface_texture_));
}

const SkM44& SurfaceTextureExternalTexture::GetCurrentUVTransformation() const {
  return transform_;
}

}  // namespace flutter
