// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_PAINTING_DISPLAY_LIST_IMAGE_GPU_H_
#define FLUTTER_LIB_UI_PAINTING_DISPLAY_LIST_IMAGE_GPU_H_

#include "flutter/display_list/image/dl_image.h"
#include "flutter/flow/skia_gpu_object.h"
#include "flutter/fml/macros.h"

namespace flutter {

class DlImageGPU final : public DlImage {
 public:
  static sk_sp<DlImageGPU> Make(SkiaGPUObject<SkImage> image);

  // |DlImage|
  ~DlImageGPU() override;

  // |DlImage|
  sk_sp<SkImage> skia_image() const override;

  // |DlImage|
  std::shared_ptr<impeller::Texture> impeller_texture() const override;

  // |DlImage|
  bool isOpaque() const override;

  // |DlImage|
  bool isTextureBacked() const override;

  // |DlImage|
  bool isUIThreadSafe() const override;

  // |DlImage|
  SkISize dimensions() const override;

  // |DlImage|
  virtual size_t GetApproximateByteSize() const override;

 private:
  SkiaGPUObject<SkImage> image_;

  explicit DlImageGPU(SkiaGPUObject<SkImage> image);

  FML_DISALLOW_COPY_AND_ASSIGN(DlImageGPU);
};

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_PAINTING_DISPLAY_LIST_IMAGE_GPU_H_
