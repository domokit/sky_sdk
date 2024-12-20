// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/android/android_image_generator.h"

#include <memory>
#include <utility>

#include <android/bitmap.h>
#include <android/hardware_buffer.h>

#include "flutter/fml/platform/android/jni_util.h"

#include "third_party/skia/include/codec/SkCodecAnimation.h"

namespace flutter {

static fml::jni::ScopedJavaGlobalRef<jclass>* g_flutter_jni_class = nullptr;
static jmethodID g_decode_image_method = nullptr;

AndroidImageGenerator::~AndroidImageGenerator() = default;

AndroidImageGenerator::AndroidImageGenerator(sk_sp<SkData> data)
    : data_(std::move(data)), image_info_(SkImageInfo::MakeUnknown(-1, -1)) {}

const SkImageInfo& AndroidImageGenerator::GetInfo() {
  header_decoded_latch_.Wait();
  return image_info_;
}

unsigned int AndroidImageGenerator::GetFrameCount() const {
  return 1;
}

unsigned int AndroidImageGenerator::GetPlayCount() const {
  return 1;
}

const ImageGenerator::FrameInfo AndroidImageGenerator::GetFrameInfo(
    unsigned int frame_index) {
  return {.required_frame = std::nullopt,
          .duration = 0,
          .disposal_method = SkCodecAnimation::DisposalMethod::kKeep};
}

SkISize AndroidImageGenerator::GetScaledDimensions(float desired_scale) {
  return GetInfo().dimensions();
}

bool AndroidImageGenerator::GetPixels(const SkImageInfo& info,
                                      void* pixels,
                                      size_t row_bytes,
                                      unsigned int frame_index,
                                      std::optional<unsigned int> prior_frame) {
  fully_decoded_latch_.Wait();

  if (!software_decoded_data_) {
    return false;
  }

  if (kRGBA_8888_SkColorType != info.colorType()) {
    return false;
  }

  switch (info.alphaType()) {
    case kOpaque_SkAlphaType:
      if (kOpaque_SkAlphaType != GetInfo().alphaType()) {
        return false;
      }
      break;
    case kPremul_SkAlphaType:
      break;
    default:
      return false;
  }

  // TODO(bdero): Override `GetImage()` to use `SkImage::FromAHardwareBuffer` on
  // API level 30+ once it's updated to do symbol lookups and not get
  // preprocessed out in Skia. This will allow for avoiding this copy in
  // cases where the result image doesn't need to be resized.
  memcpy(pixels, software_decoded_data_->data(),
         software_decoded_data_->size());
  return true;
}

void AndroidImageGenerator::DecodeImage() {
  DoDecodeImage();

  header_decoded_latch_.Signal();
  fully_decoded_latch_.Signal();
}

void AndroidImageGenerator::DoDecodeImage() {
  FML_DCHECK(g_flutter_jni_class);
  FML_DCHECK(g_decode_image_method);

  // Call FlutterJNI.decodeImage

  JNIEnv* env = fml::jni::AttachCurrentThread();

  // This task is run on the IO thread.  Create a frame to ensure that all
  // local JNI references used here are freed.
  fml::jni::ScopedJavaLocalFrame scoped_local_reference_frame(env);

  jobject direct_buffer =
      env->NewDirectByteBuffer(const_cast<void*>(data_->data()), data_->size());

  auto bitmap = std::make_unique<fml::jni::ScopedJavaGlobalRef<jobject>>(
      env, env->CallStaticObjectMethod(g_flutter_jni_class->obj(),
                                       g_decode_image_method, direct_buffer,
                                       reinterpret_cast<jlong>(this)));
  FML_CHECK(fml::jni::CheckException(env));

  if (bitmap->is_null()) {
    return;
  }

  AndroidBitmapInfo info;
  [[maybe_unused]] int status;
  if ((status = AndroidBitmap_getInfo(env, bitmap->obj(), &info)) < 0) {
    FML_DLOG(ERROR) << "Failed to get bitmap info, status=" << status;
    return;
  }
  FML_DCHECK(info.format == ANDROID_BITMAP_FORMAT_RGBA_8888);

  // Lock the android buffer in a shared pointer

  void* pixel_lock;
  if ((status = AndroidBitmap_lockPixels(env, bitmap->obj(), &pixel_lock)) <
      0) {
    FML_DLOG(ERROR) << "Failed to lock pixels, error=" << status;
    return;
  }

  SkData::ReleaseProc on_release = [](const void* ptr, void* context) -> void {
    fml::jni::ScopedJavaGlobalRef<jobject>* bitmap =
        reinterpret_cast<fml::jni::ScopedJavaGlobalRef<jobject>*>(context);
    auto env = fml::jni::AttachCurrentThread();
    AndroidBitmap_unlockPixels(env, bitmap->obj());
    delete bitmap;
  };

  software_decoded_data_ = SkData::MakeWithProc(
      pixel_lock, info.width * info.height * sizeof(uint32_t), on_release,
      bitmap.release());
}

bool AndroidImageGenerator::Register(JNIEnv* env) {
  g_flutter_jni_class = new fml::jni::ScopedJavaGlobalRef<jclass>(
      env, env->FindClass("io/flutter/embedding/engine/FlutterJNI"));
  FML_DCHECK(!g_flutter_jni_class->is_null());

  g_decode_image_method = env->GetStaticMethodID(
      g_flutter_jni_class->obj(), "decodeImage",
      "(Ljava/nio/ByteBuffer;J)Landroid/graphics/Bitmap;");
  FML_DCHECK(g_decode_image_method);

  static const JNINativeMethod header_decoded_method = {
      .name = "nativeImageHeaderCallback",
      .signature = "(JII)V",
      .fnPtr = reinterpret_cast<void*>(
          &AndroidImageGenerator::NativeImageHeaderCallback),
  };
  if (env->RegisterNatives(g_flutter_jni_class->obj(), &header_decoded_method,
                           1) != 0) {
    FML_LOG(ERROR)
        << "Failed to register FlutterJNI.nativeImageHeaderCallback method";
    return false;
  }

  return true;
}

std::shared_ptr<ImageGenerator> AndroidImageGenerator::MakeFromData(
    sk_sp<SkData> data,
    const fml::RefPtr<fml::TaskRunner>& task_runner) {
  std::shared_ptr<AndroidImageGenerator> generator(
      new AndroidImageGenerator(std::move(data)));

  fml::TaskRunner::RunNowOrPostTask(
      task_runner, [generator]() { generator->DecodeImage(); });

  if (generator->IsValidImageData()) {
    return generator;
  }

  return nullptr;
}

void AndroidImageGenerator::NativeImageHeaderCallback(JNIEnv* env,
                                                      jclass jcaller,
                                                      jlong generator_address,
                                                      int width,
                                                      int height) {
  AndroidImageGenerator* generator =
      reinterpret_cast<AndroidImageGenerator*>(generator_address);

  generator->image_info_ = SkImageInfo::Make(
      width, height, kRGBA_8888_SkColorType, kPremul_SkAlphaType);
  generator->header_decoded_latch_.Signal();
}

bool AndroidImageGenerator::IsValidImageData() {
  // The generator kicks off an IO task to decode everything, and calls to
  // "GetInfo()" block until either the header has been decoded or decoding has
  // failed, whichever is sooner. The decoder is initialized with a width and
  // height of -1 and will update the dimensions if the image is able to be
  // decoded.
  return GetInfo().height() != -1;
}

}  // namespace flutter
