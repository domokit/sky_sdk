// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_ANDROID_IMAGE_EXTERNAL_TEXTURE_GL_H_
#define FLUTTER_SHELL_PLATFORM_ANDROID_IMAGE_EXTERNAL_TEXTURE_GL_H_

#include <memory>
#include <optional>
#include <unordered_map>

#include "flutter/impeller/toolkit/egl/image.h"
#include "flutter/impeller/toolkit/gles/texture.h"
#include "flutter/shell/platform/android/image_external_texture.h"

namespace flutter {

class ImageExternalTextureGL : public ImageExternalTexture {
 public:
  ImageExternalTextureGL(
      int64_t id,
      const fml::jni::ScopedJavaGlobalRef<jobject>& image_textury_entry,
      const std::shared_ptr<PlatformViewAndroidJNI>& jni_facade);

 protected:
  // |ImageExternalTexture|
  void Attach(PaintContext& context) override;

  // |ImageExternalTexture|
  void Detach() override;

  // |ImageExternalTexture|
  void ProcessFrame(PaintContext& context, const SkRect& bounds) override;

  virtual sk_sp<flutter::DlImage> CreateDlImage(
      PaintContext& context,
      const SkRect& bounds,
      std::optional<HardwareBufferKey> id,
      impeller::UniqueEGLImageKHR&& egl_image) = 0;

  void UpdateImage(JavaLocalRef& hardware_buffer,
                   const SkRect& bounds,
                   PaintContext& context);

  impeller::UniqueEGLImageKHR CreateEGLImage(AHardwareBuffer* buffer);

  struct GlEntry {
    impeller::UniqueEGLImageKHR egl_image;
    impeller::UniqueGLTexture texture;
  };

  // Each GL entry is keyed off of the currently active
  // hardware buffers and evicted when the hardware buffer
  // is removed from the LRU cache.
  std::unordered_map<HardwareBufferKey, GlEntry> gl_entries_;

  FML_DISALLOW_COPY_AND_ASSIGN(ImageExternalTextureGL);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_ANDROID_IMAGE_EXTERNAL_TEXTURE_GL_H_
