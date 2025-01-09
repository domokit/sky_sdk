// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file_testing/file_testing.dart';
import 'package:flutter_tools/src/base/file_system.dart';

import '../src/common.dart';
import 'test_utils.dart';

// Test the android/app/build directory not be created unexpectedly after
// `flutter build` commands, see https://github.com/flutter/flutter/issues/91018.
//
// The easiest way to reproduce this issue is to create a plugin project, then run
// `flutter build` command inside the `example` directory, so we create a plugin
// project in the test.
void main() {
  late Directory tempDir;
  late Directory exampleAppDir;

  setUp(() async {
    tempDir = createResolvedTempDirectorySync('flutter_plugin_test.');
    exampleAppDir = tempDir.childDirectory('aaa').childDirectory('example');

    processManager.runSync(<String>[
      flutterBin,
      ...getLocalEngineArguments(),
      'create',
      '--template=plugin',
      '--platforms=android',
      'aaa',
    ], workingDirectory: tempDir.path);
  });

  tearDown(() async {
    tryToDelete(tempDir);
  });

  void checkBuildDir() {
    // The android/app/build directory should not exists
    final Directory appBuildDir = fileSystem.directory(
      fileSystem.path.join(exampleAppDir.path, 'android', 'app', 'build'),
    );
    expect(appBuildDir, isNot(exists));
  }

  test('android/app/build should not exists after flutter build apk', () async {
    processManager.runSync(<String>[
      flutterBin,
      ...getLocalEngineArguments(),
      'build',
      'apk',
      '--target-platform=android-arm',
    ], workingDirectory: exampleAppDir.path);
    checkBuildDir();
  });

  test('android/app/build should not exists after flutter build appbundle', () async {
    processManager.runSync(<String>[
      flutterBin,
      ...getLocalEngineArguments(),
      'build',
      'appbundle',
      '--target-platform=android-arm',
    ], workingDirectory: exampleAppDir.path);
    checkBuildDir();
  });
}
