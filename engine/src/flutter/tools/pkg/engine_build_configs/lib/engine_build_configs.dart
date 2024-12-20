// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// This is a library for parsing the Engine CI configurations that live under
/// flutter/ci/builders. They describe how CI builds, tests, archives, and
/// uploads the engine to cloud storage. The documentation and spec for the
/// format is at:
///
///    https://github.com/flutter/engine/blob/main/ci/builders/README.md
///
/// The code in this library is *not* used by CI to run these configurations.
/// Rather, that code executes these configs on CI is part of the "engine_v2"
/// recipes at:
///
///   https://cs.opensource.google/flutter/recipes/+/main:recipes/engine_v2
///
/// This library exposes two main classes, [BuildConfigLoader], which reads and
/// loads all build configurations under a directory, and [BuildConfig], which
/// is the Dart representation of a single build configuration.
library;

export 'src/build_config.dart';
export 'src/build_config_loader.dart';
export 'src/build_config_runner.dart';
