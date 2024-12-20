// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/android/image_external_texture.h"

#include <android/hardware_buffer_jni.h>
#include <android/sensor.h>

#include "flutter/fml/platform/android/jni_util.h"
#include "flutter/impeller/toolkit/android/proc_table.h"
#include "flutter/shell/platform/android/jni/platform_view_android_jni.h"

namespace flutter {

ImageExternalTexture::ImageExternalTexture(
    int64_t id,
    const fml::jni::ScopedJavaGlobalRef<jobject>& image_texture_entry,
    const std::shared_ptr<PlatformViewAndroidJNI>& jni_facade)
    : Texture(id),
      image_texture_entry_(image_texture_entry),
      jni_facade_(jni_facade) {}

ImageExternalTexture::~ImageExternalTexture() = default;

// Implementing flutter::Texture.
void ImageExternalTexture::Paint(PaintContext& context,
                                 const SkRect& bounds,
                                 bool freeze,
                                 const DlImageSampling sampling) {
  if (state_ == AttachmentState::kDetached) {
    return;
  }
  Attach(context);
  const bool should_process_frame = !freeze;
  if (should_process_frame) {
    ProcessFrame(context, bounds);
  }
  if (dl_image_) {
    context.canvas->DrawImageRect(
        dl_image_,                                     // image
        SkRect::Make(dl_image_->bounds()),             // source rect
        bounds,                                        // destination rect
        sampling,                                      // sampling
        context.paint,                                 // paint
        flutter::DlCanvas::SrcRectConstraint::kStrict  // enforce edges
    );
  } else {
    FML_LOG(INFO) << "No DlImage available for ImageExternalTexture to paint.";
  }
}

// Implementing flutter::Texture.
void ImageExternalTexture::MarkNewFrameAvailable() {
  // NOOP.
}

// Implementing flutter::Texture.
void ImageExternalTexture::OnTextureUnregistered() {}

// Implementing flutter::ContextListener.
void ImageExternalTexture::OnGrContextCreated() {
  state_ = AttachmentState::kUninitialized;
}

// Implementing flutter::ContextListener.
void ImageExternalTexture::OnGrContextDestroyed() {
  if (state_ == AttachmentState::kAttached) {
    dl_image_.reset();
    image_lru_.Clear();
    Detach();
  }
  state_ = AttachmentState::kDetached;
}

JavaLocalRef ImageExternalTexture::AcquireLatestImage() {
  JNIEnv* env = fml::jni::AttachCurrentThread();
  FML_CHECK(env != nullptr);

  // ImageTextureEntry.acquireLatestImage.
  JavaLocalRef image_java =
      jni_facade_->ImageProducerTextureEntryAcquireLatestImage(
          JavaLocalRef(image_texture_entry_));
  return image_java;
}

void ImageExternalTexture::CloseImage(const fml::jni::JavaRef<jobject>& image) {
  if (image.obj() == nullptr) {
    return;
  }
  jni_facade_->ImageClose(JavaLocalRef(image));
}

void ImageExternalTexture::CloseHardwareBuffer(
    const fml::jni::JavaRef<jobject>& hardware_buffer) {
  if (hardware_buffer.obj() == nullptr) {
    return;
  }
  jni_facade_->HardwareBufferClose(JavaLocalRef(hardware_buffer));
}

JavaLocalRef ImageExternalTexture::HardwareBufferFor(
    const fml::jni::JavaRef<jobject>& image) {
  if (image.obj() == nullptr) {
    return JavaLocalRef();
  }
  // Image.getHardwareBuffer.
  return jni_facade_->ImageGetHardwareBuffer(JavaLocalRef(image));
}

AHardwareBuffer* ImageExternalTexture::AHardwareBufferFor(
    const fml::jni::JavaRef<jobject>& hardware_buffer) {
  JNIEnv* env = fml::jni::AttachCurrentThread();
  FML_CHECK(env != nullptr);
  const auto& proc =
      impeller::android::GetProcTable().AHardwareBuffer_fromHardwareBuffer;
  return proc ? proc(env, hardware_buffer.obj()) : nullptr;
}

}  // namespace flutter
