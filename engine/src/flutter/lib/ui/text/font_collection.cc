// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/lib/ui/text/font_collection.h"

#include <mutex>

#include "flutter/lib/ui/text/asset_manager_font_provider.h"
#include "flutter/lib/ui/ui_dart_state.h"
#include "flutter/lib/ui/window/platform_configuration.h"
#include "flutter/runtime/test_font_data.h"
#include "rapidjson/document.h"
#include "rapidjson/rapidjson.h"
#include "third_party/skia/include/core/SkFontMgr.h"
#include "third_party/skia/include/core/SkGraphics.h"
#include "third_party/skia/include/core/SkStream.h"
#include "third_party/skia/include/core/SkTypeface.h"
#include "third_party/tonic/dart_args.h"
#include "third_party/tonic/dart_library_natives.h"
#include "third_party/tonic/logging/dart_invoke.h"
#include "third_party/tonic/typed_data/typed_list.h"
#include "txt/asset_font_manager.h"
#include "txt/platform.h"
#include "txt/test_font_manager.h"

#if FML_OS_MACOSX || FML_OS_IOS
#include "txt/platform_mac.h"
#endif

namespace flutter {

FontCollection::FontCollection()
    : collection_(std::make_shared<txt::FontCollection>()) {
  dynamic_font_manager_ = sk_make_sp<txt::DynamicFontManager>();
  collection_->SetDynamicFontManager(dynamic_font_manager_);
}

FontCollection::~FontCollection() {
  collection_.reset();
  SkGraphics::PurgeFontCache();
}

std::shared_ptr<txt::FontCollection> FontCollection::GetFontCollection() const {
  return collection_;
}

void FontCollection::SetupDefaultFontManager(
    uint32_t font_initialization_data) {
  collection_->SetupDefaultFontManager(font_initialization_data);
}

// Font manifest yaml format:
//
// flutter:
//   fonts:
//    - family: Raleway
//      fonts:
//        - asset: fonts/Raleway-Regular.ttf
//        - asset: fonts/Raleway-Italic.ttf
//          style: italic
//    - family: RobotoMono
//      fonts:
//        - asset: fonts/RobotoMono-Regular.ttf
//        - asset: fonts/RobotoMono-Bold.ttf
//          weight: 700
//
// Structure described in https://docs.flutter.dev/cookbook/design/fonts
void FontCollection::RegisterFonts(
    const std::shared_ptr<AssetManager>& asset_manager) {
#if FML_OS_MACOSX || FML_OS_IOS
  RegisterSystemFonts(*dynamic_font_manager_);
#endif
  std::unique_ptr<fml::Mapping> manifest_mapping =
      asset_manager->GetAsMapping("FontManifest.json");
  if (manifest_mapping == nullptr) {
    FML_DLOG(WARNING) << "Could not find the font manifest in the asset store.";
    return;
  }

  rapidjson::Document document;
  static_assert(sizeof(decltype(document)::Ch) == sizeof(uint8_t), "");
  document.Parse(reinterpret_cast<const decltype(document)::Ch*>(
                     manifest_mapping->GetMapping()),
                 manifest_mapping->GetSize());

  if (document.HasParseError()) {
    FML_DLOG(WARNING) << "Error parsing the font manifest in the asset store.";
    return;
  }

  if (!document.IsArray()) {
    return;
  }

  auto font_provider =
      std::make_unique<AssetManagerFontProvider>(asset_manager);

  for (const auto& family : document.GetArray()) {
    auto family_name = family.FindMember("family");
    if (family_name == family.MemberEnd() || !family_name->value.IsString()) {
      continue;
    }

    auto family_fonts = family.FindMember("fonts");
    if (family_fonts == family.MemberEnd() || !family_fonts->value.IsArray()) {
      continue;
    }

    for (const auto& family_font : family_fonts->value.GetArray()) {
      if (!family_font.IsObject()) {
        continue;
      }

      auto font_asset = family_font.FindMember("asset");
      if (font_asset == family_font.MemberEnd() ||
          !font_asset->value.IsString()) {
        continue;
      }

      // TODO(chinmaygarde): Handle weights and styles.
      font_provider->RegisterAsset(family_name->value.GetString(),
                                   font_asset->value.GetString());
    }
  }

  collection_->SetAssetFontManager(
      sk_make_sp<txt::AssetFontManager>(std::move(font_provider)));
}

void FontCollection::RegisterTestFonts() {
  std::vector<sk_sp<SkTypeface>> test_typefaces = GetTestFontData();
  std::unique_ptr<txt::TypefaceFontAssetProvider> font_provider =
      std::make_unique<txt::TypefaceFontAssetProvider>();

  size_t index = 0;
  std::vector<std::string> names = GetTestFontFamilyNames();
  for (sk_sp<SkTypeface> typeface : test_typefaces) {
    if (typeface) {
      font_provider->RegisterTypeface(std::move(typeface), names[index]);
    }
    index++;
  }

  collection_->SetTestFontManager(
      sk_make_sp<txt::TestFontManager>(std::move(font_provider), names));

  collection_->DisableFontFallback();
}

void FontCollection::LoadFontFromList(Dart_Handle font_data_handle,
                                      Dart_Handle callback,
                                      const std::string& family_name) {
  tonic::Uint8List font_data(font_data_handle);
  UIDartState::ThrowIfUIOperationsProhibited();
  FontCollection& font_collection = UIDartState::Current()
                                        ->platform_configuration()
                                        ->client()
                                        ->GetFontCollection();

  std::unique_ptr<SkStreamAsset> font_stream = std::make_unique<SkMemoryStream>(
      font_data.data(), font_data.num_elements(), true);
  sk_sp<SkFontMgr> font_mgr = txt::GetDefaultFontManager();
  sk_sp<SkTypeface> typeface = font_mgr->makeFromStream(std::move(font_stream));
  txt::TypefaceFontAssetProvider& font_provider =
      font_collection.dynamic_font_manager_->font_provider();
  if (family_name.empty()) {
    font_provider.RegisterTypeface(typeface);
  } else {
    font_provider.RegisterTypeface(typeface, family_name);
  }
  font_collection.collection_->ClearFontFamilyCache();

  font_data.Release();
  tonic::DartInvoke(callback, {tonic::ToDart(0)});
}

}  // namespace flutter
