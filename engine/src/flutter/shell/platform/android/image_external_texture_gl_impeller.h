// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_ANDROID_IMAGE_EXTERNAL_TEXTURE_GL_IMPELLER_H_
#define FLUTTER_SHELL_PLATFORM_ANDROID_IMAGE_EXTERNAL_TEXTURE_GL_IMPELLER_H_

#include <memory>
#include <optional>

#include "flutter/fml/macros.h"
#include "flutter/impeller/renderer/backend/gles/context_gles.h"
#include "flutter/shell/platform/android/image_external_texture_gl.h"
#include "flutter/shell/platform/android/jni/platform_view_android_jni.h"

namespace flutter {

class ImageExternalTextureGLImpeller : public ImageExternalTextureGL {
 public:
  ImageExternalTextureGLImpeller(
      const std::shared_ptr<impeller::ContextGLES>& context,
      int64_t id,
      const fml::jni::ScopedJavaGlobalRef<jobject>&
          hardware_buffer_texture_entry,
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

  const std::shared_ptr<impeller::ContextGLES> impeller_context_;

  FML_DISALLOW_COPY_AND_ASSIGN(ImageExternalTextureGLImpeller);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_ANDROID_IMAGE_EXTERNAL_TEXTURE_GL_IMPELLER_H_
