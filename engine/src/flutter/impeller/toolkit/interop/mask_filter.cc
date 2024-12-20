// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/toolkit/interop/mask_filter.h"

namespace impeller::interop {

ScopedObject<MaskFilter> MaskFilter::MakeBlur(flutter::DlBlurStyle style,
                                              float sigma) {
  auto filter = flutter::DlBlurMaskFilter::Make(style, sigma);
  if (!filter) {
    return nullptr;
  }
  return Create<MaskFilter>(std::move(filter));
}

MaskFilter::MaskFilter(std::shared_ptr<flutter::DlMaskFilter> mask_filter)
    : mask_filter_(std::move(mask_filter)) {}

MaskFilter::~MaskFilter() = default;

const std::shared_ptr<flutter::DlMaskFilter>& MaskFilter::GetMaskFilter()
    const {
  return mask_filter_;
}

}  // namespace impeller::interop
