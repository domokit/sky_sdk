// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/typographer/typographer_context.h"

#include <utility>

namespace impeller {

TypographerContext::TypographerContext() {
  is_valid_ = true;
}

TypographerContext::~TypographerContext() = default;

bool TypographerContext::IsValid() const {
  return is_valid_;
}

}  // namespace impeller
