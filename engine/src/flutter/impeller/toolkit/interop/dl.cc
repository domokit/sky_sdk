// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/toolkit/interop/dl.h"

namespace impeller::interop {

DisplayList::DisplayList(sk_sp<flutter::DisplayList> display_list)
    : display_list_(std::move(display_list)) {}

DisplayList::~DisplayList() = default;

bool DisplayList::IsValid() const {
  return !!display_list_;
}

const sk_sp<flutter::DisplayList> DisplayList::GetDisplayList() const {
  return display_list_;
}

}  // namespace impeller::interop
