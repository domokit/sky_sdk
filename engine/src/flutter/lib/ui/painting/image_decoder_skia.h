// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_PAINTING_IMAGE_DECODER_SKIA_H_
#define FLUTTER_LIB_UI_PAINTING_IMAGE_DECODER_SKIA_H_

#if !SLIMPELLER

#include "flutter/fml/macros.h"
#include "flutter/lib/ui/painting/image_decoder.h"

namespace flutter {

class ImageDecoderSkia final : public ImageDecoder {
 public:
  ImageDecoderSkia(
      const TaskRunners& runners,
      std::shared_ptr<fml::ConcurrentTaskRunner> concurrent_task_runner,
      fml::WeakPtr<IOManager> io_manager);

  ~ImageDecoderSkia() override;

  // |ImageDecoder|
  void Decode(fml::RefPtr<ImageDescriptor> descriptor,
              uint32_t target_width,
              uint32_t target_height,
              const ImageResult& result) override;

  static sk_sp<SkImage> ImageFromCompressedData(
      ImageDescriptor* descriptor,
      uint32_t target_width,
      uint32_t target_height,
      const fml::tracing::TraceFlow& flow);

 private:
  FML_DISALLOW_COPY_AND_ASSIGN(ImageDecoderSkia);
};

}  // namespace flutter

#endif  //  !SLIMPELLER

#endif  // FLUTTER_LIB_UI_PAINTING_IMAGE_DECODER_SKIA_H_
