// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/toolkit/interop/color_filter.h"

namespace impeller::interop {

ScopedObject<ColorFilter> ColorFilter::MakeBlend(Color color, BlendMode mode) {
  auto filter = flutter::DlColorFilter::MakeBlend(ToDisplayListType(color),
                                                  ToDisplayListType(mode));
  if (!filter) {
    return nullptr;
  }
  return Create<ColorFilter>(std::move(filter));
}

ScopedObject<ColorFilter> ColorFilter::MakeMatrix(const float matrix[20]) {
  auto filter = flutter::DlColorFilter::MakeMatrix(matrix);
  if (!filter) {
    return nullptr;
  }
  return Create<ColorFilter>(std::move(filter));
}

ColorFilter::ColorFilter(std::shared_ptr<const flutter::DlColorFilter> filter)
    : filter_(std::move(filter)) {}

ColorFilter::~ColorFilter() = default;

const std::shared_ptr<const flutter::DlColorFilter>&
ColorFilter::GetColorFilter() const {
  return filter_;
}

}  // namespace impeller::interop
