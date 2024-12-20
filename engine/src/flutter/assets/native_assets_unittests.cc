// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/assets/native_assets.h"

#include "gtest/gtest.h"

namespace flutter {
namespace testing {

// Manifest containing all hosts on which this test is run.
// In practise, a manifest will only contain a single OS.
// A manifest might contain multiple architectures if the OS supports
// multi-arch apps.
const char* kTestManifest = R"({
    "format-version": [
        1,
        0,
        0
    ],
    "native-assets": {
        "linux_arm64": {
            "package:my_package/my_package_bindings_generated.dart": [
                "absolute",
                "libmy_package.so"
            ]
        },
        "linux_x64": {
            "package:my_package/my_package_bindings_generated.dart": [
                "absolute",
                "libmy_package.so"
            ]
        },
        "macos_arm64": {
            "package:my_package/my_package_bindings_generated.dart": [
                "absolute",
                "my_package.framework/my_package"
            ]
        },
        "macos_x64": {
            "package:my_package/my_package_bindings_generated.dart": [
                "absolute",
                "my_package.framework/my_package"
            ]
        },
        "windows_x64": {
            "package:my_package/my_package_bindings_generated.dart": [
                "absolute",
                "my_package.dll"
            ]
        }
    }
})";

TEST(NativeAssetsManagerTest, NoAvailableAssets) {
  NativeAssetsManager manager;
  std::string available_assets = manager.AvailableNativeAssets();
  ASSERT_EQ(available_assets, "No available native assets.");
}

TEST(NativeAssetsManagerTest, NativeAssetsManifestParsing) {
  NativeAssetsManager manager;
  manager.RegisterNativeAssets(reinterpret_cast<const uint8_t*>(kTestManifest),
                               strlen(kTestManifest));

  std::string available_assets = manager.AvailableNativeAssets();
  ASSERT_EQ(available_assets,
            "Available native assets: "
            "package:my_package/my_package_bindings_generated.dart.");

  std::vector<std::string> existing_asset = manager.LookupNativeAsset(
      "package:my_package/my_package_bindings_generated.dart");
  ASSERT_EQ(existing_asset.size(), 2u);
  ASSERT_EQ(existing_asset[0], "absolute");
#if defined(FML_OS_MACOSX)
  ASSERT_EQ(existing_asset[1], "my_package.framework/my_package");
#elif defined(FML_OS_LINUX)
  ASSERT_EQ(existing_asset[1], "libmy_package.so");
#elif defined(FML_OS_WIN)
  ASSERT_EQ(existing_asset[1], "my_package.dll");
#endif

  std::vector<std::string> non_existing_asset =
      manager.LookupNativeAsset("non_existing_asset");
  ASSERT_EQ(non_existing_asset.size(), 0u);
}

}  // namespace testing
}  // namespace flutter
