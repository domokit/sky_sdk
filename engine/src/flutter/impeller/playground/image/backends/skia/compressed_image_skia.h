// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_PLAYGROUND_IMAGE_BACKENDS_SKIA_COMPRESSED_IMAGE_SKIA_H_
#define FLUTTER_IMPELLER_PLAYGROUND_IMAGE_BACKENDS_SKIA_COMPRESSED_IMAGE_SKIA_H_

#include "impeller/playground/image/compressed_image.h"

namespace impeller {

class CompressedImageSkia final : public CompressedImage {
 public:
  static std::shared_ptr<CompressedImage> Create(
      std::shared_ptr<const fml::Mapping> allocation);

  explicit CompressedImageSkia(std::shared_ptr<const fml::Mapping> allocation);

  ~CompressedImageSkia() override;

  // |CompressedImage|
  DecompressedImage Decode() const override;

 private:
  CompressedImageSkia(const CompressedImageSkia&) = delete;

  CompressedImageSkia& operator=(const CompressedImageSkia&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_PLAYGROUND_IMAGE_BACKENDS_SKIA_COMPRESSED_IMAGE_SKIA_H_
