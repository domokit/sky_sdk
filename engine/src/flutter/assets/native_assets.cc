// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "assets/native_assets.h"

#include "flutter/fml/build_config.h"
#include "rapidjson/document.h"

namespace flutter {

#if defined(FML_ARCH_CPU_ARMEL)
#define kTargetArchitectureName "arm"
#elif defined(FML_ARCH_CPU_ARM64)
#define kTargetArchitectureName "arm64"
#elif defined(FML_ARCH_CPU_X86)
#define kTargetArchitectureName "ia32"
#elif defined(FML_ARCH_CPU_X86_64)
#define kTargetArchitectureName "x64"
#else
#error Target architecture detection failed.
#endif

#if defined(FML_OS_ANDROID)
#define kTargetOperatingSystemName "android"
#elif defined(OS_FUCHSIA)
#define kTargetOperatingSystemName "fuchsia"
#elif defined(FML_OS_LINUX)
#define kTargetOperatingSystemName "linux"
#elif defined(FML_OS_IOS) || defined(FML_OS_IOS_SIMULATOR)
#define kTargetOperatingSystemName "ios"
#elif defined(FML_OS_MACOSX)
#define kTargetOperatingSystemName "macos"
#elif defined(FML_OS_WIN)
#define kTargetOperatingSystemName "windows"
#else
#error Target operating system detection failed.
#endif

#define kTarget kTargetOperatingSystemName "_" kTargetArchitectureName

void NativeAssetsManager::RegisterNativeAssets(const uint8_t* manifest,
                                               size_t manifest_size) {
  parsed_mapping_.clear();

  rapidjson::Document document;
  static_assert(sizeof(decltype(document)::Ch) == sizeof(uint8_t), "");
  document.Parse(reinterpret_cast<const decltype(document)::Ch*>(manifest),
                 manifest_size);
  if (document.HasParseError()) {
    FML_DLOG(WARNING) << "NativeAssetsManifest.json is malformed.";
    return;
  }
  if (!document.IsObject()) {
    FML_DLOG(WARNING) << "NativeAssetsManifest.json is malformed.";
    return;
  }
  auto native_assets = document.FindMember("native-assets");
  if (native_assets == document.MemberEnd() ||
      !native_assets->value.IsObject()) {
    FML_DLOG(WARNING) << "NativeAssetsManifest.json is malformed.";
    return;
  }
  auto mapping = native_assets->value.FindMember(kTarget);
  if (mapping == native_assets->value.MemberEnd() ||
      !mapping->value.IsObject()) {
    FML_DLOG(WARNING) << "NativeAssetsManifest.json is malformed.";
    return;
  }
  for (auto entry = mapping->value.MemberBegin();
       entry != mapping->value.MemberEnd(); entry++) {
    std::vector<std::string> parsed_path;
    entry->name.GetString();
    auto& value = entry->value;
    if (!value.IsArray()) {
      FML_DLOG(WARNING) << "NativeAssetsManifest.json is malformed.";
      continue;
    }
    for (const auto& element : value.GetArray()) {
      if (!element.IsString()) {
        FML_DLOG(WARNING) << "NativeAssetsManifest.json is malformed.";
        continue;
      }
      parsed_path.push_back(element.GetString());
    }
    parsed_mapping_[entry->name.GetString()] = std::move(parsed_path);
  }
}

void NativeAssetsManager::RegisterNativeAssets(
    const std::shared_ptr<AssetManager>& asset_manager) {
  std::unique_ptr<fml::Mapping> manifest_mapping =
      asset_manager->GetAsMapping("NativeAssetsManifest.json");
  if (manifest_mapping == nullptr) {
    FML_DLOG(WARNING)
        << "Could not find NativeAssetsManifest.json in the asset store.";
    return;
  }

  RegisterNativeAssets(manifest_mapping->GetMapping(),
                       manifest_mapping->GetSize());
}

std::vector<std::string> NativeAssetsManager::LookupNativeAsset(
    std::string_view asset_id) {
  // Cpp17 does not support unordered_map lookup with std::string_view on a
  // std::string key.
  std::string as_string = std::string(asset_id);
  if (parsed_mapping_.find(as_string) == parsed_mapping_.end()) {
    return std::vector<std::string>();
  }
  return parsed_mapping_[as_string];
}

std::string NativeAssetsManager::AvailableNativeAssets() {
  if (parsed_mapping_.empty()) {
    return std::string("No available native assets.");
  }

  std::string result;
  result.append("Available native assets: ");
  bool first = true;
  for (const auto& n : parsed_mapping_) {
    if (first) {
      first = false;
    } else {
      result.append(", ");
    }
    result.append(n.first);
  }

  result.append(".");
  return result;
}

}  // namespace flutter
