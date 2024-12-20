// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/image/dl_image.h"

#include "flutter/display_list/image/dl_image_skia.h"

namespace flutter {

sk_sp<DlImage> DlImage::Make(const SkImage* image) {
  return Make(sk_ref_sp(image));
}

sk_sp<DlImage> DlImage::Make(sk_sp<SkImage> image) {
  return sk_make_sp<DlImageSkia>(std::move(image));
}

DlImage::DlImage() = default;

DlImage::~DlImage() = default;

int DlImage::width() const {
  return dimensions().fWidth;
};

int DlImage::height() const {
  return dimensions().fHeight;
};

SkIRect DlImage::bounds() const {
  return SkIRect::MakeSize(dimensions());
}

std::optional<std::string> DlImage::get_error() const {
  return std::nullopt;
}

}  // namespace flutter
