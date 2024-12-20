// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/lib/ui/io_manager.h"

namespace flutter {

std::shared_ptr<impeller::Context> IOManager::GetImpellerContext() const {
  return nullptr;
}

}  // namespace flutter
