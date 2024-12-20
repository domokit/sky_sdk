// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_ASSETS_NATIVE_ASSETS_H_
#define FLUTTER_ASSETS_NATIVE_ASSETS_H_

#include <memory>
#include <vector>

#include "flutter/assets/asset_manager.h"

namespace flutter {

// Parses the `NativeAssetsManifest.json` and provides a way to look up assets
// and the available assets for the callbacks that are registered to the Dart VM
// via the dart_api.h.
//
// The engine eagerly populates a native assets manager on startup. This native
// assets manager is stored in the `IsolateGroupData` so it can be accessed on
// the native assets callbacks registered in `InitDartFFIForIsolateGroup`.
class NativeAssetsManager {
 public:
  NativeAssetsManager() = default;
  ~NativeAssetsManager() = default;

  // Reads the `NativeAssetsManifest.json` bundled in the Flutter application.
  void RegisterNativeAssets(const uint8_t* manifest, size_t manifest_size);
  void RegisterNativeAssets(const std::shared_ptr<AssetManager>& asset_manager);

  // Looks up the asset path for [asset_id].
  //
  // The asset path consists of a type, and an optional path. For example:
  // `["system", "libsqlite3.so"]`.
  std::vector<std::string> LookupNativeAsset(std::string_view asset_id);

  // Lists the available asset ids.
  //
  // Used when a user tries to look up an asset with an ID that does not exist
  // to report the list of available asset ids.
  std::string AvailableNativeAssets();

 private:
  std::unordered_map<std::string, std::vector<std::string>> parsed_mapping_;

  NativeAssetsManager(const NativeAssetsManager&) = delete;
  NativeAssetsManager(NativeAssetsManager&&) = delete;
  NativeAssetsManager& operator=(const NativeAssetsManager&) = delete;
  NativeAssetsManager& operator=(NativeAssetsManager&&) = delete;
};

}  // namespace flutter

#endif  // FLUTTER_ASSETS_NATIVE_ASSETS_H_
