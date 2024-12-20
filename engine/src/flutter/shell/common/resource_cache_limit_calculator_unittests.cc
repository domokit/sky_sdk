// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/common/resource_cache_limit_calculator.h"

#include "gtest/gtest.h"

namespace flutter {
namespace testing {

class TestResourceCacheLimitItem : public ResourceCacheLimitItem {
 public:
  explicit TestResourceCacheLimitItem(size_t resource_cache_limit)
      : resource_cache_limit_(resource_cache_limit), weak_factory_(this) {}

  size_t GetResourceCacheLimit() override { return resource_cache_limit_; }

  fml::WeakPtr<TestResourceCacheLimitItem> GetWeakPtr() {
    return weak_factory_.GetWeakPtr();
  }

 private:
  size_t resource_cache_limit_;
  fml::WeakPtrFactory<TestResourceCacheLimitItem> weak_factory_;
};

TEST(ResourceCacheLimitCalculatorTest, GetResourceCacheMaxBytes) {
  ResourceCacheLimitCalculator calculator(800U);
  auto item1 = std::make_unique<TestResourceCacheLimitItem>(100.0);
  calculator.AddResourceCacheLimitItem(item1->GetWeakPtr());
  EXPECT_EQ(calculator.GetResourceCacheMaxBytes(), static_cast<size_t>(100U));

  auto item2 = std::make_unique<TestResourceCacheLimitItem>(200.0);
  calculator.AddResourceCacheLimitItem(item2->GetWeakPtr());
  EXPECT_EQ(calculator.GetResourceCacheMaxBytes(), static_cast<size_t>(300U));

  auto item3 = std::make_unique<TestResourceCacheLimitItem>(300.0);
  calculator.AddResourceCacheLimitItem(item3->GetWeakPtr());
  EXPECT_EQ(calculator.GetResourceCacheMaxBytes(), static_cast<size_t>(600U));

  auto item4 = std::make_unique<TestResourceCacheLimitItem>(400.0);
  calculator.AddResourceCacheLimitItem(item4->GetWeakPtr());
  EXPECT_EQ(calculator.GetResourceCacheMaxBytes(), static_cast<size_t>(800U));

  item3.reset();
  EXPECT_EQ(calculator.GetResourceCacheMaxBytes(), static_cast<size_t>(700U));

  item2.reset();
  EXPECT_EQ(calculator.GetResourceCacheMaxBytes(), static_cast<size_t>(500U));
}

}  // namespace testing
}  // namespace flutter
