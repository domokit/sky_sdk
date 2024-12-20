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

#ifndef TXT_TYPEFACE_FONT_ASSET_PROVIDER_H_
#define TXT_TYPEFACE_FONT_ASSET_PROVIDER_H_

#include <string>
#include <unordered_map>
#include <vector>

#include "flutter/fml/macros.h"
#include "third_party/skia/include/core/SkFontMgr.h"
#include "txt/font_asset_provider.h"

namespace txt {

class TypefaceFontStyleSet : public SkFontStyleSet {
 public:
  TypefaceFontStyleSet();

  ~TypefaceFontStyleSet() override;

  void registerTypeface(sk_sp<SkTypeface> typeface);

  // |SkFontStyleSet|
  int count() override;

  // |SkFontStyleSet|
  void getStyle(int index, SkFontStyle* style, SkString* name) override;

  // |SkFontStyleSet|
  sk_sp<SkTypeface> createTypeface(int index) override;

  // |SkFontStyleSet|
  sk_sp<SkTypeface> matchStyle(const SkFontStyle& pattern) override;

 private:
  std::vector<sk_sp<SkTypeface>> typefaces_;

  FML_DISALLOW_COPY_AND_ASSIGN(TypefaceFontStyleSet);
};

class TypefaceFontAssetProvider : public FontAssetProvider {
 public:
  TypefaceFontAssetProvider();
  ~TypefaceFontAssetProvider() override;

  void RegisterTypeface(sk_sp<SkTypeface> typeface);

  void RegisterTypeface(sk_sp<SkTypeface> typeface,
                        std::string family_name_alias);

  // |FontAssetProvider|
  size_t GetFamilyCount() const override;

  // |FontAssetProvider|
  std::string GetFamilyName(int index) const override;

  // |FontAssetProvider|
  sk_sp<SkFontStyleSet> MatchFamily(const std::string& family_name) override;

 private:
  std::unordered_map<std::string, sk_sp<TypefaceFontStyleSet>>
      registered_families_;
  std::vector<std::string> family_names_;

  FML_DISALLOW_COPY_AND_ASSIGN(TypefaceFontAssetProvider);
};

}  // namespace txt

#endif  // TXT_TYPEFACE_FONT_ASSET_PROVIDER_H_
