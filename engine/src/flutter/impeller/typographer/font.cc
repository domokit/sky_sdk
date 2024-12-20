// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/typographer/font.h"

namespace impeller {

Font::Font(std::shared_ptr<Typeface> typeface,
           Metrics metrics,
           AxisAlignment axis_alignment)
    : typeface_(std::move(typeface)),
      metrics_(metrics),
      axis_alignment_(axis_alignment) {
  if (!typeface_) {
    return;
  }
  is_valid_ = true;
}

Font::~Font() = default;

bool Font::IsValid() const {
  return is_valid_;
}

const std::shared_ptr<Typeface>& Font::GetTypeface() const {
  return typeface_;
}

std::size_t Font::GetHash() const {
  return fml::HashCombine(is_valid_, typeface_ ? typeface_->GetHash() : 0u,
                          metrics_);
}

bool Font::IsEqual(const Font& other) const {
  return DeepComparePointer(typeface_, other.typeface_) &&
         is_valid_ == other.is_valid_ && metrics_ == other.metrics_;
}

AxisAlignment Font::GetAxisAlignment() const {
  return axis_alignment_;
}

const Font::Metrics& Font::GetMetrics() const {
  return metrics_;
}

}  // namespace impeller
