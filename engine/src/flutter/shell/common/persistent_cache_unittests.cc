// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/common/graphics/persistent_cache.h"

#include <memory>

#include "flutter/assets/directory_asset_bundle.h"
#include "flutter/flow/layers/container_layer.h"
#include "flutter/flow/layers/layer.h"
#include "flutter/fml/command_line.h"
#include "flutter/fml/file.h"
#include "flutter/fml/log_settings.h"
#include "flutter/fml/unique_fd.h"
#include "flutter/shell/common/shell_test.h"
#include "flutter/shell/common/switches.h"
#include "flutter/shell/version/version.h"
#include "flutter/testing/testing.h"

namespace flutter {
namespace testing {

using PersistentCacheTest = ShellTest;

static void CheckTextSkData(const sk_sp<SkData>& data,
                            const std::string& expected) {
  std::string data_string(reinterpret_cast<const char*>(data->bytes()),
                          data->size());
  ASSERT_EQ(data_string, expected);
}

static void ResetAssetManager() {
  PersistentCache::SetAssetManager(nullptr);
  ASSERT_EQ(PersistentCache::GetCacheForProcess()->LoadSkSLs().size(), 0u);
}

static void CheckTwoSkSLsAreLoaded() {
  auto shaders = PersistentCache::GetCacheForProcess()->LoadSkSLs();
  ASSERT_EQ(shaders.size(), 2u);
}

TEST_F(PersistentCacheTest, CanLoadSkSLsFromAsset) {
  // Avoid polluting unit tests output by hiding INFO level logging.
  fml::LogSettings warning_only = {fml::kLogWarning};
  fml::ScopedSetLogSettings scoped_set_log_settings(warning_only);

  // The SkSL key is Base32 encoded. "IE" is the encoding of "A" and "II" is the
  // encoding of "B".
  //
  // The SkSL data is Base64 encoded. "eA==" is the encoding of "x" and "eQ=="
  // is the encoding of "y".
  const std::string kTestJson =
      "{\n"
      "  \"data\": {\n"
      "    \"IE\": \"eA==\",\n"
      "    \"II\": \"eQ==\"\n"
      "  }\n"
      "}\n";

  // Temp dir for the asset.
  fml::ScopedTemporaryDirectory asset_dir;

  auto data = std::make_unique<fml::DataMapping>(
      std::vector<uint8_t>{kTestJson.begin(), kTestJson.end()});
  fml::WriteAtomically(asset_dir.fd(), PersistentCache::kAssetFileName, *data);

  // 1st, test that RunConfiguration::InferFromSettings sets the asset manager.
  ResetAssetManager();
  auto settings = CreateSettingsForFixture();
  settings.assets_path = asset_dir.path();
  RunConfiguration::InferFromSettings(settings);
  CheckTwoSkSLsAreLoaded();

  // 2nd, test that the RunConfiguration constructor sets the asset manager.
  // (Android is directly calling that constructor without InferFromSettings.)
  ResetAssetManager();
  auto asset_manager = std::make_shared<AssetManager>();
  RunConfiguration config(nullptr, asset_manager);
  asset_manager->PushBack(std::make_unique<DirectoryAssetBundle>(
      fml::OpenDirectory(asset_dir.path().c_str(), false,
                         fml::FilePermission::kRead),
      false));
  CheckTwoSkSLsAreLoaded();

  // 3rd, test the content of the SkSLs in the asset.
  {
    auto shaders = PersistentCache::GetCacheForProcess()->LoadSkSLs();
    ASSERT_EQ(shaders.size(), 2u);

    // Make sure that the 2 shaders are sorted by their keys. Their keys should
    // be "A" and "B" (decoded from "II" and "IE").
    if (shaders[0].key->bytes()[0] == 'B') {
      std::swap(shaders[0], shaders[1]);
    }

    CheckTextSkData(shaders[0].key, "A");
    CheckTextSkData(shaders[1].key, "B");
    CheckTextSkData(shaders[0].value, "x");
    CheckTextSkData(shaders[1].value, "y");
  }

  // Cleanup.
  fml::UnlinkFile(asset_dir.fd(), PersistentCache::kAssetFileName);
}

TEST_F(PersistentCacheTest, CanRemoveOldPersistentCache) {
  fml::ScopedTemporaryDirectory base_dir;
  ASSERT_TRUE(base_dir.fd().is_valid());

  fml::CreateDirectory(base_dir.fd(),
                       {"flutter_engine", GetFlutterEngineVersion(), "skia"},
                       fml::FilePermission::kReadWrite);

  constexpr char kOldEngineVersion[] = "old";
  auto old_created = fml::CreateDirectory(
      base_dir.fd(), {"flutter_engine", kOldEngineVersion, "skia"},
      fml::FilePermission::kReadWrite);
  ASSERT_TRUE(old_created.is_valid());

  PersistentCache::SetCacheDirectoryPath(base_dir.path());
  PersistentCache::ResetCacheForProcess();

  auto engine_dir = fml::OpenDirectoryReadOnly(base_dir.fd(), "flutter_engine");
  auto current_dir =
      fml::OpenDirectoryReadOnly(engine_dir, GetFlutterEngineVersion());
  auto old_dir = fml::OpenDirectoryReadOnly(engine_dir, kOldEngineVersion);

  ASSERT_TRUE(engine_dir.is_valid());
  ASSERT_TRUE(current_dir.is_valid());
  ASSERT_FALSE(old_dir.is_valid());

  // Cleanup
  fml::RemoveFilesInDirectory(base_dir.fd());
}

TEST_F(PersistentCacheTest, CanPurgePersistentCache) {
  fml::ScopedTemporaryDirectory base_dir;
  ASSERT_TRUE(base_dir.fd().is_valid());
  auto cache_dir = fml::CreateDirectory(
      base_dir.fd(),
      {"flutter_engine", GetFlutterEngineVersion(), "skia", GetSkiaVersion()},
      fml::FilePermission::kReadWrite);
  PersistentCache::SetCacheDirectoryPath(base_dir.path());
  PersistentCache::ResetCacheForProcess();

  // Generate a dummy persistent cache.
  fml::DataMapping test_data(std::string("test"));
  ASSERT_TRUE(fml::WriteAtomically(cache_dir, "test", test_data));
  auto file = fml::OpenFileReadOnly(cache_dir, "test");
  ASSERT_TRUE(file.is_valid());

  // Run engine with purge_persistent_cache to remove the dummy cache.
  auto settings = CreateSettingsForFixture();
  settings.purge_persistent_cache = true;
  auto config = RunConfiguration::InferFromSettings(settings);
  std::unique_ptr<Shell> shell = CreateShell(settings);
  RunEngine(shell.get(), std::move(config));

  // Verify that the dummy is purged.
  file = fml::OpenFileReadOnly(cache_dir, "test");
  ASSERT_FALSE(file.is_valid());

  // Cleanup
  fml::RemoveFilesInDirectory(base_dir.fd());
  DestroyShell(std::move(shell));
}

TEST_F(PersistentCacheTest, PurgeAllowsFutureSkSLCache) {
  sk_sp<SkData> shader_key = SkData::MakeWithCString("key");
  sk_sp<SkData> shader_value = SkData::MakeWithCString("value");
  std::string shader_filename = PersistentCache::SkKeyToFilePath(*shader_key);

  fml::ScopedTemporaryDirectory base_dir;
  ASSERT_TRUE(base_dir.fd().is_valid());
  PersistentCache::SetCacheDirectoryPath(base_dir.path());
  PersistentCache::ResetCacheForProcess();

  // Run engine with purge_persistent_cache and cache_sksl.
  auto settings = CreateSettingsForFixture();
  settings.purge_persistent_cache = true;
  settings.cache_sksl = true;
  auto config = RunConfiguration::InferFromSettings(settings);
  std::unique_ptr<Shell> shell = CreateShell(settings);
  RunEngine(shell.get(), std::move(config));
  auto persistent_cache = PersistentCache::GetCacheForProcess();
  ASSERT_EQ(persistent_cache->LoadSkSLs().size(), 0u);

  // Store the cache and verify it's valid.
  StorePersistentCache(persistent_cache, *shader_key, *shader_value);
  std::promise<bool> io_flushed;
  shell->GetTaskRunners().GetIOTaskRunner()->PostTask(
      [&io_flushed]() { io_flushed.set_value(true); });
  io_flushed.get_future().get();  // Wait for the IO thread to flush the file.
  ASSERT_GT(persistent_cache->LoadSkSLs().size(), 0u);

  // Cleanup
  fml::RemoveFilesInDirectory(base_dir.fd());
  DestroyShell(std::move(shell));
}

}  // namespace testing
}  // namespace flutter
