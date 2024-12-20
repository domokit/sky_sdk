/*
 * Copyright 2018 Google Inc.
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

#ifndef TXT_FONT_ASSET_PROVIDER_H_
#define TXT_FONT_ASSET_PROVIDER_H_

#include <string>

#include "third_party/skia/include/core/SkFontMgr.h"

namespace txt {

class FontAssetProvider {
 public:
  virtual ~FontAssetProvider() = default;

  virtual size_t GetFamilyCount() const = 0;
  virtual std::string GetFamilyName(int index) const = 0;
  virtual sk_sp<SkFontStyleSet> MatchFamily(const std::string& family_name) = 0;

 protected:
  static std::string CanonicalFamilyName(std::string family_name);
};

}  // namespace txt

#endif  // TXT_FONT_ASSET_PROVIDER_H_
