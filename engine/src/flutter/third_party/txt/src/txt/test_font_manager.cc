/*
 * Copyright 2017 Google Inc.
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

#include "txt/test_font_manager.h"
#include "flutter/fml/logging.h"

namespace txt {

TestFontManager::TestFontManager(
    std::unique_ptr<FontAssetProvider> font_provider,
    std::vector<std::string> test_font_family_names)
    : AssetFontManager(std::move(font_provider)),
      test_font_family_names_(test_font_family_names) {}

TestFontManager::~TestFontManager() = default;

sk_sp<SkFontStyleSet> TestFontManager::onMatchFamily(
    const char family_name[]) const {
  // Find the requested name in the list, if not found, default to the first
  // font family in the test font family list.
  std::string requested_name(family_name);
  std::string sanitized_name = test_font_family_names_[0];
  for (const std::string& test_family : test_font_family_names_) {
    if (requested_name == test_family) {
      sanitized_name = test_family;
    }
  }
  return AssetFontManager::onMatchFamily(sanitized_name.c_str());
}

}  // namespace txt
