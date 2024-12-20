// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/testing/dart_fixture.h"

#include <utility>
#include "flutter/fml/paths.h"

namespace flutter::testing {

DartFixture::DartFixture()
    : DartFixture("kernel_blob.bin",
                  kDefaultAOTAppELFFileName,
                  kDefaultAOTAppELFSplitFileName) {}

DartFixture::DartFixture(std::string kernel_filename,
                         std::string elf_filename,
                         std::string elf_split_filename)
    : native_resolver_(std::make_shared<TestDartNativeResolver>()),
      split_aot_symbols_(LoadELFSplitSymbolFromFixturesIfNeccessary(
          std::move(elf_split_filename))),
      kernel_filename_(std::move(kernel_filename)),
      assets_dir_(fml::OpenDirectory(GetFixturesPath(),
                                     false,
                                     fml::FilePermission::kRead)),
      aot_symbols_(
          LoadELFSymbolFromFixturesIfNeccessary(std::move(elf_filename))) {}

Settings DartFixture::CreateSettingsForFixture() {
  Settings settings;
  settings.leak_vm = false;
  settings.task_observer_add = [](intptr_t, const fml::closure&) {};
  settings.task_observer_remove = [](intptr_t) {};
  settings.isolate_create_callback = [this]() {
    native_resolver_->SetNativeResolverForIsolate();
  };
  settings.enable_vm_service = false;
  SetSnapshotsAndAssets(settings);
  return settings;
}

void DartFixture::SetSnapshotsAndAssets(Settings& settings) {
  if (!assets_dir_.is_valid()) {
    return;
  }

  settings.assets_dir = assets_dir_.get();

  // In JIT execution, all snapshots are present within the binary itself and
  // don't need to be explicitly supplied by the embedder. In AOT, these
  // snapshots will be present in the application AOT dylib.
  if (DartVM::IsRunningPrecompiledCode()) {
    FML_CHECK(PrepareSettingsForAOTWithSymbols(settings, aot_symbols_));
#if FML_OS_LINUX
    settings.vmservice_snapshot_library_path.emplace_back(fml::paths::JoinPaths(
        {GetTestingAssetsPath(), "libvmservice_snapshot.so"}));
#endif  // FML_OS_LINUX
  } else {
    settings.application_kernels = [this]() -> Mappings {
      std::vector<std::unique_ptr<const fml::Mapping>> kernel_mappings;
      auto kernel_mapping =
          fml::FileMapping::CreateReadOnly(assets_dir_, kernel_filename_);
      if (!kernel_mapping || !kernel_mapping->IsValid()) {
        FML_LOG(ERROR) << "Could not find kernel blob for test fixture not "
                          "running in precompiled mode.";
        return kernel_mappings;
      }
      kernel_mappings.emplace_back(std::move(kernel_mapping));
      return kernel_mappings;
    };
  }
}

void DartFixture::AddNativeCallback(const std::string& name,
                                    Dart_NativeFunction callback) {
  native_resolver_->AddNativeCallback(name, callback);
}

void DartFixture::AddFfiNativeCallback(const std::string& name,
                                       void* callback_ptr) {
  native_resolver_->AddFfiNativeCallback(name, callback_ptr);
}

}  // namespace flutter::testing
