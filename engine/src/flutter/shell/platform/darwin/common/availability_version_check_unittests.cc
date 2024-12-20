// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <tuple>

#include "flutter/shell/platform/darwin/common/availability_version_check.h"

#include "gtest/gtest.h"

TEST(AvailabilityVersionCheck, CanDecodeSystemPlist) {
  auto maybe_product_version = flutter::ProductVersionFromSystemVersionPList();
  ASSERT_TRUE(maybe_product_version.has_value());
  if (maybe_product_version.has_value()) {
    auto product_version = maybe_product_version.value();
    ASSERT_GT(product_version, std::make_tuple(0, 0, 0));
  }
}

static inline uint32_t ConstructVersion(uint32_t major,
                                        uint32_t minor,
                                        uint32_t subminor) {
  return ((major & 0xffff) << 16) | ((minor & 0xff) << 8) | (subminor & 0xff);
}

TEST(AvailabilityVersionCheck, CanParseAndCompareVersions) {
  auto rhs_version = std::make_tuple(17, 2, 0);
  uint32_t encoded_lower_version = ConstructVersion(12, 3, 7);
  ASSERT_TRUE(flutter::IsEncodedVersionLessThanOrSame(encoded_lower_version,
                                                      rhs_version));

  uint32_t encoded_higher_version = ConstructVersion(42, 0, 1);
  ASSERT_FALSE(flutter::IsEncodedVersionLessThanOrSame(encoded_higher_version,
                                                       rhs_version));
}
