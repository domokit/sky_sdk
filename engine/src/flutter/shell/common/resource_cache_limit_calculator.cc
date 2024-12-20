// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/common/resource_cache_limit_calculator.h"

namespace flutter {

size_t ResourceCacheLimitCalculator::GetResourceCacheMaxBytes() {
  size_t max_bytes = 0;
  size_t max_bytes_threshold = max_bytes_threshold_ > 0
                                   ? max_bytes_threshold_
                                   : std::numeric_limits<size_t>::max();
  std::vector<fml::WeakPtr<ResourceCacheLimitItem>> live_items;
  for (const auto& item : items_) {
    if (item) {
      live_items.push_back(item);
      max_bytes += item->GetResourceCacheLimit();
    }
  }
  items_ = std::move(live_items);
  return std::min(max_bytes, max_bytes_threshold);
}

}  // namespace flutter
