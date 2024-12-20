// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_ANDROID_APK_ASSET_PROVIDER_H_
#define FLUTTER_SHELL_PLATFORM_ANDROID_APK_ASSET_PROVIDER_H_

#include <android/asset_manager_jni.h>
#include <jni.h>

#include "flutter/assets/asset_resolver.h"
#include "flutter/fml/memory/ref_counted.h"
#include "flutter/fml/platform/android/scoped_java_ref.h"

namespace flutter {

class APKAssetProviderInternal {
 public:
  virtual std::unique_ptr<fml::Mapping> GetAsMapping(
      const std::string& asset_name) const = 0;

 protected:
  virtual ~APKAssetProviderInternal() = default;
};

class APKAssetProvider final : public AssetResolver {
 public:
  explicit APKAssetProvider(JNIEnv* env,
                            jobject assetManager,
                            std::string directory);

  explicit APKAssetProvider(std::shared_ptr<APKAssetProviderInternal> impl);

  ~APKAssetProvider() = default;

  // Returns a new 'std::unique_ptr<APKAssetProvider>' with the same 'impl_' as
  // this provider.
  std::unique_ptr<APKAssetProvider> Clone() const;

  // Obtain a raw pointer to the APKAssetProviderInternal.
  //
  // This method is intended for use in tests. Callers must not
  // delete the returned pointer.
  APKAssetProviderInternal* GetImpl() const { return impl_.get(); }

  bool operator==(const AssetResolver& other) const override;

 private:
  std::shared_ptr<APKAssetProviderInternal> impl_;

  // |flutter::AssetResolver|
  bool IsValid() const override;

  // |flutter::AssetResolver|
  bool IsValidAfterAssetManagerChange() const override;

  // |AssetResolver|
  AssetResolver::AssetResolverType GetType() const override;

  // |flutter::AssetResolver|
  std::unique_ptr<fml::Mapping> GetAsMapping(
      const std::string& asset_name) const override;

  // |AssetResolver|
  const APKAssetProvider* as_apk_asset_provider() const override {
    return this;
  }

  FML_DISALLOW_COPY_AND_ASSIGN(APKAssetProvider);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_ANDROID_APK_ASSET_PROVIDER_H_
