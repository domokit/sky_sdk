// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_COMMON_GRAPHICS_PERSISTENT_CACHE_H_
#define FLUTTER_COMMON_GRAPHICS_PERSISTENT_CACHE_H_

#if !SLIMPELLER

#include <memory>
#include <mutex>
#include <set>

#include "flutter/assets/asset_manager.h"
#include "flutter/fml/macros.h"
#include "flutter/fml/task_runner.h"
#include "flutter/fml/unique_fd.h"
#include "third_party/skia/include/gpu/ganesh/GrContextOptions.h"

class GrDirectContext;

namespace flutter {

namespace testing {
class ShellTest;
}

/// A cache of SkData that gets stored to disk.
///
/// This is mainly used for Shaders but is also written to by Dart.  It is
/// thread-safe for reading and writing from multiple threads.
class PersistentCache : public GrContextOptions::PersistentCache {
 public:
  // Mutable static switch that can be set before GetCacheForProcess. If true,
  // we'll only read existing caches but not generate new ones. Some clients
  // (e.g., embedded devices) prefer generating persistent cache files for the
  // specific device beforehand, and ship them as readonly files in OTA
  // packages.
  static bool gIsReadOnly;

  static PersistentCache* GetCacheForProcess();
  static void ResetCacheForProcess();

  // This must be called before |GetCacheForProcess|. Otherwise, it won't
  // affect the cache directory returned by |GetCacheForProcess|.
  static void SetCacheDirectoryPath(std::string path);

  // Convert a binary SkData key into a Base32 encoded string.
  //
  // This is used to specify persistent cache filenames and service protocol
  // json keys.
  static std::string SkKeyToFilePath(const SkData& key);

  // Allocate a MallocMapping containing the given key and value in the file
  // format used by the cache.
  static std::unique_ptr<fml::MallocMapping> BuildCacheObject(
      const SkData& key,
      const SkData& data);

  // Header written into the files used to store cached Skia objects.
  struct CacheObjectHeader {
    // A prefix used to identify the cache object file format.
    static const uint32_t kSignature = 0xA869593F;
    static const uint32_t kVersion1 = 1;

    explicit CacheObjectHeader(uint32_t p_key_size) : key_size(p_key_size) {}

    uint32_t signature = kSignature;
    uint32_t version = kVersion1;
    uint32_t key_size;
  };

  ~PersistentCache() override;

  void AddWorkerTaskRunner(const fml::RefPtr<fml::TaskRunner>& task_runner);

  void RemoveWorkerTaskRunner(const fml::RefPtr<fml::TaskRunner>& task_runner);

  // Whether Skia tries to store any shader into this persistent cache after
  // |ResetStoredNewShaders| is called. This flag is usually reset before each
  // frame so we can know if Skia tries to compile new shaders in that frame.
  bool StoredNewShaders() const { return stored_new_shaders_; }
  void ResetStoredNewShaders() { stored_new_shaders_ = false; }
  void DumpSkp(const SkData& data);
  bool IsDumpingSkp() const { return is_dumping_skp_; }
  void SetIsDumpingSkp(bool value) { is_dumping_skp_ = value; }

  // Remove all files inside the persistent cache directory.
  // Return whether the purge is successful.
  bool Purge();

  // |GrContextOptions::PersistentCache|
  sk_sp<SkData> load(const SkData& key) override;

  struct SkSLCache {
    sk_sp<SkData> key;
    sk_sp<SkData> value;
  };

  /// Load all the SkSL shader caches in the right directory.
  std::vector<SkSLCache> LoadSkSLs() const;

  //----------------------------------------------------------------------------
  /// @brief      Precompile SkSLs packaged with the application and gathered
  ///             during previous runs in the given context.
  ///
  /// @warning    The context must be the rendering context. This context may be
  ///             destroyed during application suspension and subsequently
  ///             recreated. The SkSLs must be precompiled again in the new
  ///             context.
  ///
  /// @param      context  The rendering context to precompile shaders in.
  ///
  /// @return     The number of SkSLs precompiled.
  ///
  size_t PrecompileKnownSkSLs(GrDirectContext* context) const;

  // Return mappings for all skp's accessible through the AssetManager
  std::vector<std::unique_ptr<fml::Mapping>> GetSkpsFromAssetManager() const;

  /// Set the asset manager from which PersistentCache can load SkLSs. A nullptr
  /// can be provided to clear the asset manager.
  static void SetAssetManager(std::shared_ptr<AssetManager> value);
  static std::shared_ptr<AssetManager> asset_manager() {
    return asset_manager_;
  }

  static bool cache_sksl() { return cache_sksl_; }

  static void SetCacheSkSL(bool value);

  static void MarkStrategySet() { strategy_set_ = true; }

  static constexpr char kSkSLSubdirName[] = "sksl";
  static constexpr char kAssetFileName[] = "io.flutter.shaders.json";

 private:
  static std::string cache_base_path_;

  static std::shared_ptr<AssetManager> asset_manager_;

  static std::mutex instance_mutex_;
  static std::unique_ptr<PersistentCache> gPersistentCache;

  // Mutable static switch that can be set before GetCacheForProcess is called
  // and GrContextOptions.fShaderCacheStrategy is set. If true, it means that
  // we'll set `GrContextOptions::fShaderCacheStrategy` to `kSkSL`, and all the
  // persistent cache should be stored and loaded from the "sksl" directory.
  static std::atomic<bool> cache_sksl_;

  // Guard flag to make sure that cache_sksl_ is not modified after
  // strategy_set_ becomes true.
  static std::atomic<bool> strategy_set_;

  const bool is_read_only_;
  const std::shared_ptr<fml::UniqueFD> cache_directory_;
  const std::shared_ptr<fml::UniqueFD> sksl_cache_directory_;
  mutable std::mutex worker_task_runners_mutex_;
  std::multiset<fml::RefPtr<fml::TaskRunner>> worker_task_runners_;

  bool stored_new_shaders_ = false;
  bool is_dumping_skp_ = false;

  static SkSLCache LoadFile(const fml::UniqueFD& dir,
                            const std::string& file_name,
                            bool need_key);

  bool IsValid() const;

  explicit PersistentCache(bool read_only = false);

  // |GrContextOptions::PersistentCache|
  void store(const SkData& key, const SkData& data) override;

  fml::RefPtr<fml::TaskRunner> GetWorkerTaskRunner() const;

  friend class testing::ShellTest;

  FML_DISALLOW_COPY_AND_ASSIGN(PersistentCache);
};

}  // namespace flutter

#endif  //  !SLIMPELLER

#endif  // FLUTTER_COMMON_GRAPHICS_PERSISTENT_CACHE_H_
