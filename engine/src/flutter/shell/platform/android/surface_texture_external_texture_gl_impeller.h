// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_ANDROID_SURFACE_TEXTURE_EXTERNAL_TEXTURE_GL_IMPELLER_H_
#define FLUTTER_SHELL_PLATFORM_ANDROID_SURFACE_TEXTURE_EXTERNAL_TEXTURE_GL_IMPELLER_H_

#include <memory>

#include "flutter/fml/macros.h"
#include "flutter/impeller/renderer/backend/gles/context_gles.h"
#include "flutter/impeller/renderer/backend/gles/texture_gles.h"
#include "flutter/shell/platform/android/surface_texture_external_texture.h"

namespace flutter {

//------------------------------------------------------------------------------
/// @brief      Binds the surface texture to an Impeller texture.
///
class SurfaceTextureExternalTextureGLImpeller
    : public SurfaceTextureExternalTexture {
 public:
  SurfaceTextureExternalTextureGLImpeller(
      const std::shared_ptr<impeller::ContextGLES>& context,
      int64_t id,
      const fml::jni::ScopedJavaGlobalRef<jobject>& surface_texture,
      const std::shared_ptr<PlatformViewAndroidJNI>& jni_facade);

  // |SurfaceTextureExternalTexture|
  ~SurfaceTextureExternalTextureGLImpeller() override;

 private:
  // |SurfaceTextureExternalTexture|
  virtual void ProcessFrame(PaintContext& context,
                            const SkRect& bounds) override;

  // |SurfaceTextureExternalTexture|
  virtual void Detach() override;

  const std::shared_ptr<impeller::ContextGLES> impeller_context_;
  std::shared_ptr<impeller::TextureGLES> texture_;

  FML_DISALLOW_COPY_AND_ASSIGN(SurfaceTextureExternalTextureGLImpeller);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_ANDROID_SURFACE_TEXTURE_EXTERNAL_TEXTURE_GL_IMPELLER_H_
