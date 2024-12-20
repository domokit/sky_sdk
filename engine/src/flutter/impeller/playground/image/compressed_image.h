// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_PLAYGROUND_IMAGE_COMPRESSED_IMAGE_H_
#define FLUTTER_IMPELLER_PLAYGROUND_IMAGE_COMPRESSED_IMAGE_H_

#include <memory>

#include "flutter/fml/mapping.h"
#include "impeller/playground/image/decompressed_image.h"

namespace impeller {

class ImageSource;

class CompressedImage {
 public:
  virtual ~CompressedImage();

  [[nodiscard]] virtual DecompressedImage Decode() const = 0;

  bool IsValid() const;

 protected:
  const std::shared_ptr<const fml::Mapping> source_;

  explicit CompressedImage(std::shared_ptr<const fml::Mapping> allocation);
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_PLAYGROUND_IMAGE_COMPRESSED_IMAGE_H_
