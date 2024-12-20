// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_WINDOWS_FLUTTER_PROJECT_BUNDLE_H_
#define FLUTTER_SHELL_PLATFORM_WINDOWS_FLUTTER_PROJECT_BUNDLE_H_

#include <filesystem>
#include <string>
#include <vector>

#include "flutter/shell/platform/embedder/embedder.h"
#include "flutter/shell/platform/windows/public/flutter_windows.h"

namespace flutter {

using UniqueAotDataPtr =
    std::unique_ptr<_FlutterEngineAOTData, FlutterEngineCollectAOTDataFnPtr>;

// The data associated with a Flutter project needed to run it in an engine.
class FlutterProjectBundle {
 public:
  // Creates a new project bundle from the given properties.
  //
  // Treats any relative paths as relative to the directory of this executable.
  explicit FlutterProjectBundle(
      const FlutterDesktopEngineProperties& properties);

  ~FlutterProjectBundle();

  // Whether or not the bundle is valid. This does not check that the paths
  // exist, or contain valid data, just that paths were able to be constructed.
  bool HasValidPaths();

  // Returns the path to the assets directory.
  const std::filesystem::path& assets_path() { return assets_path_; }

  // Returns the path to the ICU data file.
  const std::filesystem::path& icu_path() { return icu_path_; }

  // Returns any switches that should be passed to the engine.
  const std::vector<std::string> GetSwitches();

  // Sets engine switches.
  void SetSwitches(const std::vector<std::string>& switches);

  // Attempts to load AOT data for this bundle. The returned data must be
  // retained until any engine instance it is passed to has been shut down.
  //
  // Logs and returns nullptr on failure.
  UniqueAotDataPtr LoadAotData(const FlutterEngineProcTable& engine_procs);

  // Returns the Dart entrypoint.
  const std::string& dart_entrypoint() const { return dart_entrypoint_; }

  // Returns the command line arguments to be passed through to the Dart
  // entrypoint.
  const std::vector<std::string>& dart_entrypoint_arguments() const {
    return dart_entrypoint_arguments_;
  }

 private:
  std::filesystem::path assets_path_;
  std::filesystem::path icu_path_;

  // Path to the AOT library file, if any.
  std::filesystem::path aot_library_path_;

  // The Dart entrypoint to launch.
  std::string dart_entrypoint_;

  // Dart entrypoint arguments.
  std::vector<std::string> dart_entrypoint_arguments_;

  // Engine switches.
  std::vector<std::string> engine_switches_;
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_WINDOWS_FLUTTER_PROJECT_BUNDLE_H_
