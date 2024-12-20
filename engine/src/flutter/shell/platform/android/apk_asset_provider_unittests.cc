// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/android/apk_asset_provider.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace flutter {
namespace testing {
class MockAPKAssetProviderImpl : public APKAssetProviderInternal {
 public:
  MOCK_METHOD(std::unique_ptr<fml::Mapping>,
              GetAsMapping,
              (const std::string& asset_name),
              (const, override));
};

TEST(APKAssetProvider, CloneAndEquals) {
  auto first_provider = std::make_unique<APKAssetProvider>(
      std::make_shared<MockAPKAssetProviderImpl>());
  auto second_provider = std::make_unique<APKAssetProvider>(
      std::make_shared<MockAPKAssetProviderImpl>());
  auto third_provider = first_provider->Clone();

  ASSERT_NE(first_provider->GetImpl(), second_provider->GetImpl());
  ASSERT_EQ(first_provider->GetImpl(), third_provider->GetImpl());
  ASSERT_FALSE(*first_provider == *second_provider);
  ASSERT_TRUE(*first_provider == *third_provider);
}
}  // namespace testing
}  // namespace flutter
