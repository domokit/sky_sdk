// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_ASSETS_ASSET_MANAGER_H_
#define FLUTTER_ASSETS_ASSET_MANAGER_H_

#include <deque>
#include <memory>
#include <string>

#include <optional>
#include "flutter/assets/asset_resolver.h"
#include "flutter/fml/macros.h"
#include "flutter/fml/memory/ref_counted.h"

namespace flutter {

class AssetManager final : public AssetResolver {
 public:
  AssetManager();

  ~AssetManager() override;

  //--------------------------------------------------------------------------
  /// @brief      Adds an asset resolver to the front of the resolver queue.
  ///             Assets would be loaded from this resolver before any follwing
  ///             resolvers.
  ///
  /// @return     Returns whether this resolver is valid and has been added to
  ///             the resolver queue.
  bool PushFront(std::unique_ptr<AssetResolver> resolver);

  //--------------------------------------------------------------------------
  /// @brief      Adds an asset resolver to the end of the resolver queue.
  ///             Assets would be loaded from this resolver after any previous
  ///             resolvers.
  ///
  /// @return     Returns whether this resolver is valid and has been added to
  ///             the resolver queue.
  bool PushBack(std::unique_ptr<AssetResolver> resolver);

  //--------------------------------------------------------------------------
  /// @brief      Replaces an asset resolver of the specified `type` with
  ///             `updated_asset_resolver`. The matching AssetResolver is
  ///             removed and replaced with `updated_asset_resolvers`.
  ///
  ///             AssetResolvers should be updated when the existing resolver
  ///             becomes obsolete and a newer one becomes available that
  ///             provides updated access to the same type of assets as the
  ///             existing one. This update process is meant to be performed
  ///             at runtime.
  ///
  ///             If a null resolver is provided, nothing will be done. If no
  ///             matching resolver is found, the provided resolver will be
  ///             added to the end of the AssetManager resolvers queue. The
  ///             replacement only occurs with the first matching resolver.
  ///             Any additional matching resolvers are untouched.
  ///
  /// @param[in]  updated_asset_resolver  The asset resolver to replace the
  ///             resolver of matching type with.
  ///
  /// @param[in]  type  The type of AssetResolver to update. Only resolvers of
  ///                   the specified type will be replaced by the updated
  ///                   resolver.
  ///
  void UpdateResolverByType(
      std::unique_ptr<AssetResolver> updated_asset_resolver,
      AssetResolver::AssetResolverType type);

  std::deque<std::unique_ptr<AssetResolver>> TakeResolvers();

  // |AssetResolver|
  bool IsValid() const override;

  // |AssetResolver|
  bool IsValidAfterAssetManagerChange() const override;

  // |AssetResolver|
  AssetResolver::AssetResolverType GetType() const override;

  // |AssetResolver|
  std::unique_ptr<fml::Mapping> GetAsMapping(
      const std::string& asset_name) const override;

  // |AssetResolver|
  std::vector<std::unique_ptr<fml::Mapping>> GetAsMappings(
      const std::string& asset_pattern,
      const std::optional<std::string>& subdir) const override;

  // |AssetResolver|
  bool operator==(const AssetResolver& other) const override;

  // |AssetResolver|
  const AssetManager* as_asset_manager() const override { return this; }

 private:
  std::deque<std::unique_ptr<AssetResolver>> resolvers_;

  FML_DISALLOW_COPY_AND_ASSIGN(AssetManager);
};

}  // namespace flutter

#endif  // FLUTTER_ASSETS_ASSET_MANAGER_H_
