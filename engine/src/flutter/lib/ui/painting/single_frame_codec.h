// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_PAINTING_SINGLE_FRAME_CODEC_H_
#define FLUTTER_LIB_UI_PAINTING_SINGLE_FRAME_CODEC_H_

#include "flutter/fml/macros.h"
#include "flutter/lib/ui/painting/codec.h"
#include "flutter/lib/ui/painting/image.h"
#include "flutter/lib/ui/painting/image_decoder.h"
#include "flutter/lib/ui/painting/image_descriptor.h"

namespace flutter {

class SingleFrameCodec : public Codec {
 public:
  SingleFrameCodec(const fml::RefPtr<ImageDescriptor>& descriptor,
                   uint32_t target_width,
                   uint32_t target_height);

  ~SingleFrameCodec() override;

  // |Codec|
  int frameCount() const override;

  // |Codec|
  int repetitionCount() const override;

  // |Codec|
  Dart_Handle getNextFrame(Dart_Handle args) override;

 private:
  enum class Status { kNew, kInProgress, kComplete };
  Status status_ = Status::kNew;
  fml::RefPtr<ImageDescriptor> descriptor_;
  uint32_t target_width_;
  uint32_t target_height_;
  fml::RefPtr<CanvasImage> cached_image_;
  std::vector<tonic::DartPersistentValue> pending_callbacks_;

  FML_FRIEND_MAKE_REF_COUNTED(SingleFrameCodec);
  FML_FRIEND_REF_COUNTED_THREAD_SAFE(SingleFrameCodec);
};

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_PAINTING_SINGLE_FRAME_CODEC_H_
