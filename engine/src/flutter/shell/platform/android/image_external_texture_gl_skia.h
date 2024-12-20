// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_ANDROID_IMAGE_EXTERNAL_TEXTURE_GL_SKIA_H_
#define FLUTTER_SHELL_PLATFORM_ANDROID_IMAGE_EXTERNAL_TEXTURE_GL_SKIA_H_

#include <memory>
#include <optional>

#include "flutter/shell/platform/android/android_context_gl_skia.h"
#include "flutter/shell/platform/android/image_external_texture_gl.h"

namespace flutter {

class ImageExternalTextureGLSkia : public ImageExternalTextureGL {
 public:
  ImageExternalTextureGLSkia(
      const std::shared_ptr<AndroidContextGLSkia>& context,
      int64_t id,
      const fml::jni::ScopedJavaGlobalRef<jobject>& image_textury_entry,
      const std::shared_ptr<PlatformViewAndroidJNI>& jni_facade);

 private:
  // |ImageExternalTexture|
  void Attach(PaintContext& context) override;

  // |ImageExternalTexture|
  void Detach() override;

  // |ImageExternalTextureGL|
  sk_sp<flutter::DlImage> CreateDlImage(
      PaintContext& context,
      const SkRect& bounds,
      std::optional<HardwareBufferKey> id,
      impeller::UniqueEGLImageKHR&& egl_image) override;

  void BindImageToTexture(const impeller::UniqueEGLImageKHR& image, GLuint tex);

  FML_DISALLOW_COPY_AND_ASSIGN(ImageExternalTextureGLSkia);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_ANDROID_IMAGE_EXTERNAL_TEXTURE_GL_SKIA_H_
