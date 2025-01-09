// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:flutter_tools/src/base/io.dart';

import '../src/common.dart';
import 'test_utils.dart';

void main() {
  late Directory tempDir;
  late Directory projectRoot;
  final List<String> targetPlatforms = <String>[
    'apk',
    'web',
    if (platform.isWindows) 'windows',
    if (platform.isMacOS) ...<String>['macos', 'ios'],
  ];

  setUpAll(() {
    tempDir = createResolvedTempDirectorySync('build_compilation_error_test.');
    processManager.runSync(<String>[
      flutterBin,
      'config',
      '--enable-macos-desktop',
      '--enable-windows-desktop',
      '--enable-web',
    ]);

    processManager.runSync(<String>[
      flutterBin,
      ...getLocalEngineArguments(),
      'create',
      'hello',
    ], workingDirectory: tempDir.path);

    projectRoot = tempDir.childDirectory('hello');
    writeFile(fileSystem.path.join(projectRoot.path, 'lib', 'main.dart'), '''
int x = 'String';
''');
  });

  tearDownAll(() {
    tryToDelete(tempDir);
  });

  for (final String targetPlatform in targetPlatforms) {
    testWithoutContext(
      'flutter build $targetPlatform shows dart compilation error in non-verbose',
      () {
        final ProcessResult result = processManager.runSync(<String>[
          flutterBin,
          ...getLocalEngineArguments(),
          'build',
          targetPlatform,
          '--no-pub',
          if (targetPlatform == 'ios') '--no-codesign',
        ], workingDirectory: projectRoot.path);

        const String errorMessage =
            "A value of type 'String' can't be assigned to a variable of type 'int'.";

        // Xcode 16 moved the xcodebuild error details from stderr to stdout.
        // Check that it's contained in one or the other.
        final bool matchStdout = result.stdout.toString().contains(errorMessage);
        final bool matchStderr = result.stderr.toString().contains(errorMessage);

        expect(matchStdout || matchStderr, isTrue);
        expect(
          result.stderr,
          isNot(contains("Warning: The 'dart2js' entrypoint script is deprecated")),
        );
        expect(
          result.stdout,
          isNot(contains("Warning: The 'dart2js' entrypoint script is deprecated")),
        );
      },
    );
  }
}
