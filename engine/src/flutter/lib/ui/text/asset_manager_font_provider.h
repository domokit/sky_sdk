// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_TEXT_ASSET_MANAGER_FONT_PROVIDER_H_
#define FLUTTER_LIB_UI_TEXT_ASSET_MANAGER_FONT_PROVIDER_H_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "flutter/assets/asset_manager.h"
#include "flutter/fml/macros.h"
#include "third_party/skia/include/core/SkFontMgr.h"
#include "third_party/skia/include/core/SkTypeface.h"
#include "txt/font_asset_provider.h"

namespace flutter {

class AssetManagerFontStyleSet : public SkFontStyleSet {
 public:
  AssetManagerFontStyleSet(std::shared_ptr<AssetManager> asset_manager,
                           std::string family_name);

  ~AssetManagerFontStyleSet() override;

  void registerAsset(const std::string& asset);

  // |SkFontStyleSet|
  int count() override;

  // |SkFontStyleSet|
  void getStyle(int index, SkFontStyle*, SkString* style) override;

  // |SkFontStyleSet|
  using CreateTypefaceRet =
      decltype(std::declval<SkFontStyleSet>().createTypeface(0));
  CreateTypefaceRet createTypeface(int index) override;

  // |SkFontStyleSet|
  using MatchStyleRet = decltype(std::declval<SkFontStyleSet>().matchStyle(
      std::declval<SkFontStyle>()));
  MatchStyleRet matchStyle(const SkFontStyle& pattern) override;

 private:
  std::shared_ptr<AssetManager> asset_manager_;
  std::string family_name_;

  struct TypefaceAsset {
    explicit TypefaceAsset(std::string a);

    TypefaceAsset(const TypefaceAsset& other);

    ~TypefaceAsset();

    std::string asset;
    sk_sp<SkTypeface> typeface;
  };
  std::vector<TypefaceAsset> assets_;

  FML_DISALLOW_COPY_AND_ASSIGN(AssetManagerFontStyleSet);
};

class AssetManagerFontProvider : public txt::FontAssetProvider {
 public:
  explicit AssetManagerFontProvider(
      std::shared_ptr<AssetManager> asset_manager);

  ~AssetManagerFontProvider() override;

  void RegisterAsset(const std::string& family_name, const std::string& asset);

  // |FontAssetProvider|
  size_t GetFamilyCount() const override;

  // |FontAssetProvider|
  std::string GetFamilyName(int index) const override;

  // |FontAssetProvider|
  sk_sp<SkFontStyleSet> MatchFamily(const std::string& family_name) override;

 private:
  std::shared_ptr<AssetManager> asset_manager_;
  std::unordered_map<std::string, sk_sp<AssetManagerFontStyleSet>>
      registered_families_;
  std::vector<std::string> family_names_;

  FML_DISALLOW_COPY_AND_ASSIGN(AssetManagerFontProvider);
};

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_TEXT_ASSET_MANAGER_FONT_PROVIDER_H_
