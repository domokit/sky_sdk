// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/playground/image/compressed_image.h"

namespace impeller {

CompressedImage::CompressedImage(std::shared_ptr<const fml::Mapping> allocation)
    : source_(std::move(allocation)) {}

CompressedImage::~CompressedImage() = default;

bool CompressedImage::IsValid() const {
  return static_cast<bool>(source_);
}

}  // namespace impeller
