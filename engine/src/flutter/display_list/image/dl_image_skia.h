// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_DISPLAY_LIST_IMAGE_DL_IMAGE_SKIA_H_
#define FLUTTER_DISPLAY_LIST_IMAGE_DL_IMAGE_SKIA_H_

#include "flutter/display_list/image/dl_image.h"
#include "flutter/fml/macros.h"

namespace flutter {

class DlImageSkia final : public DlImage {
 public:
  explicit DlImageSkia(sk_sp<SkImage> image);

  // |DlImage|
  ~DlImageSkia() override;

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
  size_t GetApproximateByteSize() const override;

 private:
  sk_sp<SkImage> image_;

  FML_DISALLOW_COPY_AND_ASSIGN(DlImageSkia);
};

}  // namespace flutter

#endif  // FLUTTER_DISPLAY_LIST_IMAGE_DL_IMAGE_SKIA_H_
