// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/io.dart';

import '../src/common.dart';
import 'test_utils.dart';

// Test that verbosity it propagated to Gradle tasks correctly.
void main() {
  late Directory tempDir;
  late Directory exampleAppDir;

  setUp(() async {
    tempDir = createResolvedTempDirectorySync('flutter_build_test.');
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

  test('flutter build apk -v output should contain gen_snapshot command', () async {
    final ProcessResult result = processManager.runSync(<String>[
      flutterBin,
      ...getLocalEngineArguments(),
      'build',
      'apk',
      '--target-platform=android-arm',
      '-v',
    ], workingDirectory: exampleAppDir.path);
    expect(result.stdout, contains(RegExp(r'executing:\s+.+gen_snapshot\s+')));
  });
}
