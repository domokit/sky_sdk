// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_ASSETS_ASSET_RESOLVER_H_
#define FLUTTER_ASSETS_ASSET_RESOLVER_H_

#include <string>
#include <vector>

#include <optional>
#include "flutter/fml/macros.h"
#include "flutter/fml/mapping.h"

namespace flutter {

class AssetManager;
class APKAssetProvider;
class DirectoryAssetBundle;

class AssetResolver {
 public:
  AssetResolver() = default;

  virtual ~AssetResolver() = default;

  //----------------------------------------------------------------------------
  /// @brief      Identifies the type of AssetResolver an instance is.
  ///
  enum AssetResolverType {
    kAssetManager,
    kApkAssetProvider,
    kDirectoryAssetBundle
  };

  virtual const AssetManager* as_asset_manager() const { return nullptr; }
  virtual const APKAssetProvider* as_apk_asset_provider() const {
    return nullptr;
  }
  virtual const DirectoryAssetBundle* as_directory_asset_bundle() const {
    return nullptr;
  }

  virtual bool IsValid() const = 0;

  //----------------------------------------------------------------------------
  /// @brief      Certain asset resolvers are still valid after the asset
  ///             manager is replaced before a hot reload, or after a new run
  ///             configuration is created during a hot restart. By preserving
  ///             these resolvers and re-inserting them into the new resolver or
  ///             run configuration, the tooling can avoid needing to sync all
  ///             application assets through the Dart devFS upon connecting to
  ///             the VM Service. Besides improving the startup performance of
  ///             running a Flutter application, it also reduces the occurrence
  ///             of tool failures due to repeated network flakes caused by
  ///             damaged cables or hereto unknown bugs in the Dart HTTP server
  ///             implementation.
  ///
  /// @return     Returns whether this resolver is valid after the asset manager
  ///             or run configuration is updated.
  ///
  virtual bool IsValidAfterAssetManagerChange() const = 0;

  //----------------------------------------------------------------------------
  /// @brief      Gets the type of AssetResolver this is. Types are defined in
  ///             AssetResolverType.
  ///
  /// @return     Returns the AssetResolverType that this resolver is.
  ///
  virtual AssetResolverType GetType() const = 0;

  [[nodiscard]] virtual std::unique_ptr<fml::Mapping> GetAsMapping(
      const std::string& asset_name) const = 0;

  //--------------------------------------------------------------------------
  /// @brief      Same as GetAsMapping() but returns mappings for all files
  ///             who's name matches a given pattern. Returns empty vector
  ///             if no matching assets are found.
  ///
  /// @param[in]  asset_pattern  The pattern to match file names against.
  ///
  /// @param[in]  subdir  Optional subdirectory in which to search for files.
  ///             If supplied this function does a flat search within the
  ///             subdirectory instead of a recursive search through the entire
  ///             assets directory.
  ///
  /// @return     Returns a vector of mappings of files which match the search
  ///             parameters.
  ///
  [[nodiscard]] virtual std::vector<std::unique_ptr<fml::Mapping>>
  GetAsMappings(const std::string& asset_pattern,
                const std::optional<std::string>& subdir) const {
    return {};
  };

  virtual bool operator==(const AssetResolver& other) const = 0;

  bool operator!=(const AssetResolver& other) const {
    return !operator==(other);
  }

 private:
  FML_DISALLOW_COPY_AND_ASSIGN(AssetResolver);
};

}  // namespace flutter

#endif  // FLUTTER_ASSETS_ASSET_RESOLVER_H_
