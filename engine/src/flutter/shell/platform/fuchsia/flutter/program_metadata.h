// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_PROGRAM_METADATA_H_
#define FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_PROGRAM_METADATA_H_

#include <optional>
#include <string>

namespace flutter_runner {

/// The metadata that can be passed by a Flutter component via
/// the `program` field.
struct ProgramMetadata {
  /// The path where data for the Flutter component should
  /// be stored.
  std::string data_path = "";

  /// The path where assets for the Flutter component should
  /// be stored.
  ///
  /// TODO(fxb/89246): No components appear to be using this field. We
  /// may be able to get rid of this.
  std::string assets_path = "";

  /// The preferred heap size for the Flutter component in megabytes.
  std::optional<int64_t> old_gen_heap_size = std::nullopt;

  /// A list of additional directories that we will expose in out/
  std::vector<std::string> expose_dirs;
};

}  // namespace flutter_runner

#endif  // FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_PROGRAM_METADATA_H_
