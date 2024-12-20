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

#include "txt/asset_font_manager.h"

#include <memory>

#include "flutter/fml/logging.h"
#include "third_party/skia/include/core/SkString.h"
#include "third_party/skia/include/core/SkTypeface.h"

namespace txt {

AssetFontManager::AssetFontManager(
    std::unique_ptr<FontAssetProvider> font_provider)
    : font_provider_(std::move(font_provider)) {
  FML_DCHECK(font_provider_ != nullptr);
}

AssetFontManager::~AssetFontManager() = default;

int AssetFontManager::onCountFamilies() const {
  return font_provider_->GetFamilyCount();
}

void AssetFontManager::onGetFamilyName(int index, SkString* familyName) const {
  familyName->set(font_provider_->GetFamilyName(index).c_str());
}

sk_sp<SkFontStyleSet> AssetFontManager::onCreateStyleSet(int index) const {
  FML_DCHECK(false);
  return nullptr;
}

sk_sp<SkFontStyleSet> AssetFontManager::onMatchFamily(
    const char family_name_string[]) const {
  std::string family_name(family_name_string);
  return font_provider_->MatchFamily(family_name);
}

sk_sp<SkTypeface> AssetFontManager::onMatchFamilyStyle(
    const char familyName[],
    const SkFontStyle& style) const {
  sk_sp<SkFontStyleSet> font_style_set =
      font_provider_->MatchFamily(std::string(familyName));
  if (font_style_set == nullptr)
    return nullptr;
  return font_style_set->matchStyle(style);
}

sk_sp<SkTypeface> AssetFontManager::onMatchFamilyStyleCharacter(
    const char familyName[],
    const SkFontStyle&,
    const char* bcp47[],
    int bcp47Count,
    SkUnichar character) const {
  return nullptr;
}

sk_sp<SkTypeface> AssetFontManager::onMakeFromData(sk_sp<SkData>,
                                                   int ttcIndex) const {
  FML_DCHECK(false);
  return nullptr;
}

sk_sp<SkTypeface> AssetFontManager::onMakeFromStreamIndex(
    std::unique_ptr<SkStreamAsset>,
    int ttcIndex) const {
  FML_DCHECK(false);
  return nullptr;
}

sk_sp<SkTypeface> AssetFontManager::onMakeFromStreamArgs(
    std::unique_ptr<SkStreamAsset>,
    const SkFontArguments&) const {
  FML_DCHECK(false);
  return nullptr;
}

sk_sp<SkTypeface> AssetFontManager::onMakeFromFile(const char path[],
                                                   int ttcIndex) const {
  FML_DCHECK(false);
  return nullptr;
}

sk_sp<SkTypeface> AssetFontManager::onLegacyMakeTypeface(
    const char familyName[],
    SkFontStyle) const {
  return nullptr;
}

}  // namespace txt
