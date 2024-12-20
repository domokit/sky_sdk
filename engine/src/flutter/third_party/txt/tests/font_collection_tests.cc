/*
 * Copyright 2017 Google, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "gtest/gtest.h"

#include <sstream>

#include "txt/font_collection.h"

namespace txt {
namespace testing {

class FontCollectionTests : public ::testing::Test {
 public:
  FontCollectionTests() {}

  void SetUp() override {}
};

TEST_F(FontCollectionTests, SettingUpDefaultFontManagerClearsCache) {
  FontCollection font_collection;
  sk_sp<skia::textlayout::FontCollection> sk_font_collection =
      font_collection.CreateSktFontCollection();
  ASSERT_EQ(sk_font_collection->getFallbackManager().get(), nullptr);
  font_collection.SetupDefaultFontManager(0);
  sk_font_collection = font_collection.CreateSktFontCollection();
  ASSERT_NE(sk_font_collection->getFallbackManager().get(), nullptr);
}
}  // namespace testing
}  // namespace txt
