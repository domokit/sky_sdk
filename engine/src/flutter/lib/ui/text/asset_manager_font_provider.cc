// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/lib/ui/text/asset_manager_font_provider.h"

#include <utility>

#include "flutter/fml/logging.h"
#include "third_party/skia/include/core/SkData.h"
#include "third_party/skia/include/core/SkFontMgr.h"
#include "third_party/skia/include/core/SkStream.h"
#include "third_party/skia/include/core/SkString.h"
#include "third_party/skia/include/core/SkTypeface.h"
#include "txt/platform.h"

namespace flutter {

namespace {

void MappingReleaseProc(const void* ptr, void* context) {
  delete reinterpret_cast<fml::Mapping*>(context);
}

}  // anonymous namespace

AssetManagerFontProvider::AssetManagerFontProvider(
    std::shared_ptr<AssetManager> asset_manager)
    : asset_manager_(std::move(asset_manager)) {}

AssetManagerFontProvider::~AssetManagerFontProvider() = default;

// |FontAssetProvider|
size_t AssetManagerFontProvider::GetFamilyCount() const {
  return family_names_.size();
}

// |FontAssetProvider|
std::string AssetManagerFontProvider::GetFamilyName(int index) const {
  FML_DCHECK(index >= 0 && static_cast<size_t>(index) < family_names_.size());
  return family_names_[index];
}

// |FontAssetProvider|
sk_sp<SkFontStyleSet> AssetManagerFontProvider::MatchFamily(
    const std::string& family_name) {
  auto found = registered_families_.find(CanonicalFamilyName(family_name));
  if (found == registered_families_.end()) {
    return nullptr;
  }
  return found->second;
}

void AssetManagerFontProvider::RegisterAsset(const std::string& family_name,
                                             const std::string& asset) {
  std::string canonical_name = CanonicalFamilyName(family_name);
  auto family_it = registered_families_.find(canonical_name);

  if (family_it == registered_families_.end()) {
    family_names_.push_back(family_name);
    auto value = std::make_pair(
        canonical_name,
        sk_make_sp<AssetManagerFontStyleSet>(asset_manager_, family_name));
    family_it = registered_families_.emplace(value).first;
  }

  family_it->second->registerAsset(asset);
}

AssetManagerFontStyleSet::AssetManagerFontStyleSet(
    std::shared_ptr<AssetManager> asset_manager,
    std::string family_name)
    : asset_manager_(std::move(asset_manager)),
      family_name_(std::move(family_name)) {}

AssetManagerFontStyleSet::~AssetManagerFontStyleSet() = default;

void AssetManagerFontStyleSet::registerAsset(const std::string& asset) {
  assets_.emplace_back(asset);
}

int AssetManagerFontStyleSet::count() {
  return assets_.size();
}

void AssetManagerFontStyleSet::getStyle(int index,
                                        SkFontStyle* style,
                                        SkString* name) {
  FML_DCHECK(index < static_cast<int>(assets_.size()));
  if (style) {
    sk_sp<SkTypeface> typeface(createTypeface(index));
    if (typeface) {
      *style = typeface->fontStyle();
    }
  }
  if (name) {
    *name = family_name_.c_str();
  }
}

auto AssetManagerFontStyleSet::createTypeface(int i) -> CreateTypefaceRet {
  size_t index = i;
  if (index >= assets_.size()) {
    return nullptr;
  }

  TypefaceAsset& asset = assets_[index];
  if (!asset.typeface) {
    std::unique_ptr<fml::Mapping> asset_mapping =
        asset_manager_->GetAsMapping(asset.asset);
    if (asset_mapping == nullptr) {
      return nullptr;
    }

    fml::Mapping* asset_mapping_ptr = asset_mapping.release();
    sk_sp<SkData> asset_data = SkData::MakeWithProc(
        asset_mapping_ptr->GetMapping(), asset_mapping_ptr->GetSize(),
        MappingReleaseProc, asset_mapping_ptr);
    std::unique_ptr<SkMemoryStream> stream = SkMemoryStream::Make(asset_data);

    sk_sp<SkFontMgr> font_mgr = txt::GetDefaultFontManager();
    // Ownership of the stream is transferred.
    asset.typeface = font_mgr->makeFromStream(std::move(stream));
    if (!asset.typeface) {
      FML_DLOG(ERROR) << "Unable to load font asset for family: "
                      << family_name_;
      return nullptr;
    }
  }

  return CreateTypefaceRet(SkRef(asset.typeface.get()));
}

auto AssetManagerFontStyleSet::matchStyle(const SkFontStyle& pattern)
    -> MatchStyleRet {
  return matchStyleCSS3(pattern);
}

AssetManagerFontStyleSet::TypefaceAsset::TypefaceAsset(std::string a)
    : asset(std::move(a)) {}

AssetManagerFontStyleSet::TypefaceAsset::TypefaceAsset(
    const AssetManagerFontStyleSet::TypefaceAsset& other) = default;

AssetManagerFontStyleSet::TypefaceAsset::~TypefaceAsset() = default;

}  // namespace flutter
