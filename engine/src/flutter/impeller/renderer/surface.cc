// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/surface.h"

#include "flutter/fml/logging.h"

namespace impeller {

Surface::Surface() : Surface(RenderTarget{}) {}

Surface::Surface(const RenderTarget& target_desc) : desc_(target_desc) {
  if (auto size = desc_.GetColorAttachmentSize(0u); size.has_value()) {
    size_ = size.value();
  } else {
    return;
  }

  is_valid_ = true;
}

Surface::~Surface() = default;

const ISize& Surface::GetSize() const {
  return size_;
}

bool Surface::IsValid() const {
  return is_valid_;
}

const RenderTarget& Surface::GetRenderTarget() const {
  return desc_;
}

bool Surface::Present() const {
  return false;
};

}  // namespace impeller
