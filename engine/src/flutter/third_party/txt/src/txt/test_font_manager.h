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

#ifndef TXT_TEST_FONT_MANAGER_H_
#define TXT_TEST_FONT_MANAGER_H_

#include <memory>
#include <string>
#include <vector>

#include "flutter/fml/macros.h"
#include "third_party/skia/include/core/SkFontMgr.h"
#include "txt/asset_font_manager.h"
#include "txt/font_asset_provider.h"

namespace txt {

// A font manager intended for tests that matches all requested fonts using
// one family.
class TestFontManager : public AssetFontManager {
 public:
  TestFontManager(std::unique_ptr<FontAssetProvider> font_provider,
                  std::vector<std::string> test_font_family_names);

  ~TestFontManager() override;

 private:
  std::vector<std::string> test_font_family_names_;

  sk_sp<SkFontStyleSet> onMatchFamily(const char family_name[]) const override;

  FML_DISALLOW_COPY_AND_ASSIGN(TestFontManager);
};

}  // namespace txt

#endif  // TXT_TEST_FONT_MANAGER_H_
