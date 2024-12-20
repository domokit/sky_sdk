// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_ASSETS_DIRECTORY_ASSET_BUNDLE_H_
#define FLUTTER_ASSETS_DIRECTORY_ASSET_BUNDLE_H_

#include <optional>
#include "flutter/assets/asset_resolver.h"
#include "flutter/fml/macros.h"
#include "flutter/fml/memory/ref_counted.h"
#include "flutter/fml/unique_fd.h"

namespace flutter {

class DirectoryAssetBundle : public AssetResolver {
 public:
  DirectoryAssetBundle(fml::UniqueFD descriptor,
                       bool is_valid_after_asset_manager_change);

  ~DirectoryAssetBundle() override;

 private:
  const fml::UniqueFD descriptor_;
  bool is_valid_ = false;
  bool is_valid_after_asset_manager_change_ = false;

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
  const DirectoryAssetBundle* as_directory_asset_bundle() const override {
    return this;
  }

  FML_DISALLOW_COPY_AND_ASSIGN(DirectoryAssetBundle);
};

}  // namespace flutter

#endif  // FLUTTER_ASSETS_DIRECTORY_ASSET_BUNDLE_H_
