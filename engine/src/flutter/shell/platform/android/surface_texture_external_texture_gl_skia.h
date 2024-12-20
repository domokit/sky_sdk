// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_ANDROID_SURFACE_TEXTURE_EXTERNAL_TEXTURE_GL_SKIA_H_
#define FLUTTER_SHELL_PLATFORM_ANDROID_SURFACE_TEXTURE_EXTERNAL_TEXTURE_GL_SKIA_H_

#include <memory>

#include "flutter/shell/platform/android/surface_texture_external_texture.h"

namespace flutter {

//------------------------------------------------------------------------------
/// @brief      Binds the surface texture to a Skia SkImage.
///
class SurfaceTextureExternalTextureGLSkia
    : public SurfaceTextureExternalTexture {
 public:
  SurfaceTextureExternalTextureGLSkia(
      int64_t id,
      const fml::jni::ScopedJavaGlobalRef<jobject>& surface_texture,
      const std::shared_ptr<PlatformViewAndroidJNI>& jni_facade);

  // |SurfaceTextureExternalTexture|
  ~SurfaceTextureExternalTextureGLSkia() override;

 private:
  // |SurfaceTextureExternalTexture|
  virtual void ProcessFrame(PaintContext& context,
                            const SkRect& bounds) override;

  // |SurfaceTextureExternalTexture|
  virtual void Detach() override;

  GLuint texture_name_ = 0;

  FML_DISALLOW_COPY_AND_ASSIGN(SurfaceTextureExternalTextureGLSkia);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_ANDROID_SURFACE_TEXTURE_EXTERNAL_TEXTURE_GL_SKIA_H_
