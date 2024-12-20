// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/common/display_manager.h"

#include "flutter/fml/logging.h"
#include "flutter/fml/macros.h"

namespace flutter {

DisplayManager::DisplayManager() = default;

DisplayManager::~DisplayManager() = default;

double DisplayManager::GetMainDisplayRefreshRate() const {
  std::scoped_lock lock(displays_mutex_);
  if (displays_.empty()) {
    return kUnknownDisplayRefreshRate;
  } else {
    return displays_[0]->GetRefreshRate();
  }
}

void DisplayManager::HandleDisplayUpdates(
    std::vector<std::unique_ptr<Display>> displays) {
  FML_DCHECK(!displays.empty());
  std::scoped_lock lock(displays_mutex_);
  displays_ = std::move(displays);
}

}  // namespace flutter
