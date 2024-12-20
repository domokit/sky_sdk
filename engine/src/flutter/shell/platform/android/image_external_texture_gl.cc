// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/android/image_external_texture_gl.h"

#include <android/hardware_buffer_jni.h>
#include <android/sensor.h>

#include "flutter/common/graphics/texture.h"
#include "flutter/impeller/core/formats.h"
#include "flutter/impeller/display_list/dl_image_impeller.h"
#include "flutter/impeller/toolkit/android/hardware_buffer.h"
#include "flutter/impeller/toolkit/egl/image.h"
#include "flutter/impeller/toolkit/gles/texture.h"
#include "third_party/skia/include/core/SkAlphaType.h"
#include "third_party/skia/include/core/SkColorType.h"
#include "third_party/skia/include/gpu/ganesh/SkImageGanesh.h"
#include "third_party/skia/include/gpu/ganesh/gl/GrGLBackendSurface.h"
#include "third_party/skia/include/gpu/ganesh/gl/GrGLTypes.h"

namespace flutter {

ImageExternalTextureGL::ImageExternalTextureGL(
    int64_t id,
    const fml::jni::ScopedJavaGlobalRef<jobject>& image_texture_entry,
    const std::shared_ptr<PlatformViewAndroidJNI>& jni_facade)
    : ImageExternalTexture(id, image_texture_entry, jni_facade) {}

void ImageExternalTextureGL::Attach(PaintContext& context) {
  if (state_ == AttachmentState::kUninitialized) {
    // TODO(johnmccurtchan): We currently display the first frame after an
    // attach-detach cycle as blank. There seems to be an issue on some
    // devices where ImageReaders/Images from before the detach aren't
    // valid after the attach. According to Android folks this doesn't
    // match the spec. Revisit this in the future.
    // See https://github.com/flutter/flutter/issues/142978 and
    // https://github.com/flutter/flutter/issues/139039.
    state_ = AttachmentState::kAttached;
  }
}

void ImageExternalTextureGL::UpdateImage(JavaLocalRef& hardware_buffer,
                                         const SkRect& bounds,
                                         PaintContext& context) {
  AHardwareBuffer* latest_hardware_buffer = AHardwareBufferFor(hardware_buffer);
  std::optional<HardwareBufferKey> key =
      impeller::android::HardwareBuffer::GetSystemUniqueID(
          latest_hardware_buffer);
  auto existing_image = image_lru_.FindImage(key);
  if (existing_image != nullptr) {
    dl_image_ = existing_image;
    return;
  }

  auto egl_image = CreateEGLImage(latest_hardware_buffer);
  if (!egl_image.is_valid()) {
    return;
  }

  dl_image_ = CreateDlImage(context, bounds, key, std::move(egl_image));
  if (key.has_value()) {
    gl_entries_.erase(image_lru_.AddImage(dl_image_, key.value()));
  }
}

void ImageExternalTextureGL::ProcessFrame(PaintContext& context,
                                          const SkRect& bounds) {
  JavaLocalRef image = AcquireLatestImage();
  if (image.is_null()) {
    return;
  }
  JavaLocalRef hardware_buffer = HardwareBufferFor(image);
  UpdateImage(hardware_buffer, bounds, context);
  CloseHardwareBuffer(hardware_buffer);
}

void ImageExternalTextureGL::Detach() {
  image_lru_.Clear();
  gl_entries_.clear();
}

impeller::UniqueEGLImageKHR ImageExternalTextureGL::CreateEGLImage(
    AHardwareBuffer* hardware_buffer) {
  if (hardware_buffer == nullptr) {
    return impeller::UniqueEGLImageKHR();
  }

  EGLDisplay display = eglGetCurrentDisplay();
  if (display == EGL_NO_DISPLAY) {
    // This could happen when running in a deferred task that executes after
    // the thread has lost its EGL state.
    return impeller::UniqueEGLImageKHR();
  }

  EGLClientBuffer client_buffer =
      impeller::android::GetProcTable().eglGetNativeClientBufferANDROID(
          hardware_buffer);
  FML_DCHECK(client_buffer != nullptr);
  if (client_buffer == nullptr) {
    FML_LOG(ERROR) << "eglGetNativeClientBufferAndroid returned null.";
    return impeller::UniqueEGLImageKHR();
  }

  impeller::EGLImageKHRWithDisplay maybe_image =
      impeller::EGLImageKHRWithDisplay{
          eglCreateImageKHR(display, EGL_NO_CONTEXT, EGL_NATIVE_BUFFER_ANDROID,
                            client_buffer, 0),
          display};

  return impeller::UniqueEGLImageKHR(maybe_image);
}

}  // namespace flutter
