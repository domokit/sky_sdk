// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'base/common.dart';
import 'base/file_system.dart';
import 'cache.dart';

/// A validator that runs before the tool runs any command.
abstract class PreRunValidator {
  factory PreRunValidator({required FileSystem fileSystem}) = _DefaultPreRunValidator;

  void validate();
}

class _DefaultPreRunValidator implements PreRunValidator {
  _DefaultPreRunValidator({required this.fileSystem});

  final FileSystem fileSystem;

  late final Directory _toolsDir = fileSystem.directory(
    fileSystem.path.join(Cache.flutterRoot!, 'packages', 'flutter_tools'),
  );

  @override
  void validate() {
    // If a user downloads the Flutter SDK via a pre-built archive and there is
    // an error during extraction, the user could have a valid Dart snapshot of
    // the tool but not the source directory. We still need the source, so
    // validate the source directory exists and toolExit if not.
    if (!_toolsDir.existsSync()) {
      throwToolExit(
        'Flutter SDK installation appears corrupted: expected to find the '
        'directory ${_toolsDir.path} but it does not exist! Please go to '
        'https://flutter.dev/setup for instructions on how to re-install '
        'Flutter.',
      );
    }
  }
}

class NoOpPreRunValidator implements PreRunValidator {
  const NoOpPreRunValidator();

  @override
  void validate() {}
}
