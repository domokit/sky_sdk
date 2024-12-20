// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:file_testing/file_testing.dart';
import 'package:flutter_tools/src/base/file_system.dart';

import '../src/common.dart';
import 'test_utils.dart';

void main() {
  late Directory tempDir;
  late Directory exampleAppDir;

  setUp(() async {
    tempDir = createResolvedTempDirectorySync('flutter_web_wasm_test.');
    exampleAppDir = tempDir.childDirectory('test_app');

    processManager.runSync(<String>[
      flutterBin,
      'create',
      '--platforms=web',
      'test_app',
    ], workingDirectory: tempDir.path);
  });

  test('building web with --wasm produces expected files', () async {
    final ProcessResult result = processManager.runSync(<String>[
      flutterBin,
      'build',
      'web',
      '--wasm',
    ], workingDirectory: exampleAppDir.path);
    expect(result, const ProcessResultMatcher());

    final Directory appBuildDir = fileSystem.directory(
      fileSystem.path.join(exampleAppDir.path, 'build', 'web'),
    );
    for (final String filename in const <String>[
      'flutter.js',
      'flutter_service_worker.js',
      'index.html',
      'main.dart.wasm',
      'main.dart.mjs',
      'main.dart.js',
    ]) {
      expect(appBuildDir.childFile(filename), exists);
    }
  });
}
