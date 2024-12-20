// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:file/local.dart';
import 'package:meta/meta.dart';
import 'package:platform/platform.dart';

/// The current host machine running the tests.
HostAgent get hostAgent =>
    HostAgent(platform: const LocalPlatform(), fileSystem: const LocalFileSystem());

/// Host machine running the tests.
class HostAgent {
  HostAgent({required Platform platform, required FileSystem fileSystem})
    : _platform = platform,
      _fileSystem = fileSystem;

  final Platform _platform;
  final FileSystem _fileSystem;

  /// Creates a directory to dump file artifacts.
  Directory? get dumpDirectory {
    if (_dumpDirectory == null) {
      // Set in LUCI recipe.
      final String? directoryPath = _platform.environment['FLUTTER_LOGS_DIR'];
      if (directoryPath != null) {
        _dumpDirectory = _fileSystem.directory(directoryPath)..createSync(recursive: true);
        print('Found FLUTTER_LOGS_DIR dump directory ${_dumpDirectory?.path}');
      }
    }
    return _dumpDirectory;
  }

  static Directory? _dumpDirectory;

  @visibleForTesting
  void resetDumpDirectory() {
    _dumpDirectory = null;
  }
}
