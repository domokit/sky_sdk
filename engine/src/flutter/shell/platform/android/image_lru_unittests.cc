// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/android/image_lru.h"
#include "gtest/gtest.h"

namespace flutter {
namespace testing {

TEST(ImageLRU, CanStoreSingleImage) {
  auto image = DlImage::Make(nullptr);
  ImageLRU image_lru;

  EXPECT_EQ(image_lru.FindImage(1), nullptr);

  image_lru.AddImage(image, 1);

  EXPECT_EQ(image_lru.FindImage(1), image);
}

TEST(ImageLRU, EvictsLRU) {
  auto image = DlImage::Make(nullptr);
  ImageLRU image_lru;

  // Fill up the cache, nothing is removed
  for (auto i = 0u; i < kImageReaderSwapchainSize; i++) {
    EXPECT_EQ(image_lru.AddImage(image, i + 1), 0u);
  }
  // Confirm each image is in the cache. This should keep the LRU
  // order the same.
  for (auto i = 0u; i < kImageReaderSwapchainSize; i++) {
    EXPECT_EQ(image_lru.FindImage(i + 1), image);
  }

  // Insert new image and verify least recently used was removed.
  EXPECT_EQ(image_lru.AddImage(image, 100), 1u);
}

TEST(ImageLRU, CanClear) {
  auto image = DlImage::Make(nullptr);
  ImageLRU image_lru;

  // Fill up the cache, nothing is removed
  for (auto i = 0u; i < kImageReaderSwapchainSize; i++) {
    EXPECT_EQ(image_lru.AddImage(image, i + 1), 0u);
  }
  image_lru.Clear();

  // Expect no cache entries.
  for (auto i = 0u; i < kImageReaderSwapchainSize; i++) {
    EXPECT_EQ(image_lru.FindImage(i + 1), nullptr);
  }
}

}  // namespace testing
}  // namespace flutter
