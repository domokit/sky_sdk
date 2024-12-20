// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_ANDROID_ANDROID_IMAGE_GENERATOR_H_
#define FLUTTER_SHELL_PLATFORM_ANDROID_ANDROID_IMAGE_GENERATOR_H_

#include <jni.h>

#include "flutter/fml/memory/ref_ptr.h"
#include "flutter/fml/synchronization/waitable_event.h"
#include "flutter/fml/task_runner.h"
#include "flutter/lib/ui/painting/image_generator.h"

namespace flutter {

class AndroidImageGenerator : public ImageGenerator {
 private:
  explicit AndroidImageGenerator(sk_sp<SkData> buffer);

 public:
  ~AndroidImageGenerator();

  // |ImageGenerator|
  const SkImageInfo& GetInfo() override;

  // |ImageGenerator|
  unsigned int GetFrameCount() const override;

  // |ImageGenerator|
  unsigned int GetPlayCount() const override;

  // |ImageGenerator|
  const ImageGenerator::FrameInfo GetFrameInfo(
      unsigned int frame_index) override;

  // |ImageGenerator|
  SkISize GetScaledDimensions(float desired_scale) override;

  // |ImageGenerator|
  bool GetPixels(const SkImageInfo& info,
                 void* pixels,
                 size_t row_bytes,
                 unsigned int frame_index,
                 std::optional<unsigned int> prior_frame) override;

  void DecodeImage();

  static bool Register(JNIEnv* env);

  static std::shared_ptr<ImageGenerator> MakeFromData(
      sk_sp<SkData> data,
      const fml::RefPtr<fml::TaskRunner>& task_runner);

  static void NativeImageHeaderCallback(JNIEnv* env,
                                        jclass jcaller,
                                        jlong generator_pointer,
                                        int width,
                                        int height);

 private:
  sk_sp<SkData> data_;
  sk_sp<SkData> software_decoded_data_;

  SkImageInfo image_info_;

  /// Blocks until the header of the image has been decoded and the image
  /// dimensions have been determined.
  fml::ManualResetWaitableEvent header_decoded_latch_;

  /// Blocks until the image has been fully decoded.
  fml::ManualResetWaitableEvent fully_decoded_latch_;

  void DoDecodeImage();

  bool IsValidImageData();

  FML_DISALLOW_COPY_ASSIGN_AND_MOVE(AndroidImageGenerator);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_ANDROID_ANDROID_IMAGE_GENERATOR_H_
