// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_COMMON_RESOURCE_CACHE_LIMIT_CALCULATOR_H_
#define FLUTTER_SHELL_COMMON_RESOURCE_CACHE_LIMIT_CALCULATOR_H_

#include <cstdint>
#include <unordered_map>

#include "flutter/fml/macros.h"
#include "flutter/fml/memory/weak_ptr.h"

namespace flutter {
class ResourceCacheLimitItem {
 public:
  // The expected GPU resource cache limit in bytes. This will be called on the
  // platform thread.
  virtual size_t GetResourceCacheLimit() = 0;

 protected:
  virtual ~ResourceCacheLimitItem() = default;
};

class ResourceCacheLimitCalculator {
 public:
  explicit ResourceCacheLimitCalculator(size_t max_bytes_threshold)
      : max_bytes_threshold_(max_bytes_threshold) {}

  ~ResourceCacheLimitCalculator() = default;

  // This will be called on the platform thread.
  void AddResourceCacheLimitItem(
      const fml::WeakPtr<ResourceCacheLimitItem>& item) {
    items_.push_back(item);
  }

  // The maximum GPU resource cache limit in bytes calculated by
  // 'ResourceCacheLimitItem's. This will be called on the platform thread.
  size_t GetResourceCacheMaxBytes();

 private:
  std::vector<fml::WeakPtr<ResourceCacheLimitItem>> items_;
  size_t max_bytes_threshold_;
  FML_DISALLOW_COPY_AND_ASSIGN(ResourceCacheLimitCalculator);
};
}  // namespace flutter

#endif  // FLUTTER_SHELL_COMMON_RESOURCE_CACHE_LIMIT_CALCULATOR_H_
