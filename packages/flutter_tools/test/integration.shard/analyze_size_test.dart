// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:file_testing/file_testing.dart';
import 'package:flutter_tools/src/base/io.dart';

import '../src/common.dart';
import 'test_utils.dart';

// This test file does not use [getLocalEngineArguments] because it requires
// multiple specific artifact output types.

const String apkDebugMessage = 'A summary of your APK analysis can be found at: ';
const String iosDebugMessage = 'A summary of your iOS bundle analysis can be found at: ';
const String macOSDebugMessage = 'A summary of your macOS bundle analysis can be found at: ';
const String runDevToolsMessage = 'dart devtools ';

void main() {
  testWithoutContext(
    '--analyze-size flag produces expected output on hello_world for Android',
    () async {
      final String workingDirectory = fileSystem.path.join(
        getFlutterRoot(),
        'examples',
        'hello_world',
      );
      final ProcessResult result = await processManager.run(<String>[
        flutterBin,
        'build',
        'apk',
        '--verbose',
        '--analyze-size',
        '--target-platform=android-arm64',
      ], workingDirectory: workingDirectory);

      expect(
        result,
        const ProcessResultMatcher(stdoutPattern: 'app-release.apk (total compressed)'),
      );

      final String line = result.stdout
          .toString()
          .split('\n')
          .firstWhere((String line) => line.contains(apkDebugMessage));

      final String outputFilePath = line.split(apkDebugMessage).last.trim();
      expect(fileSystem.file(fileSystem.path.join(workingDirectory, outputFilePath)), exists);
      expect(outputFilePath, contains('.flutter-devtools'));

      final String devToolsCommand = result.stdout
          .toString()
          .split('\n')
          .firstWhere((String line) => line.contains(runDevToolsMessage));
      final String commandArguments = devToolsCommand.split(runDevToolsMessage).last.trim();
      expect(commandArguments.contains('--appSizeBase=$outputFilePath'), isTrue);
    },
  );

  testWithoutContext(
    '--analyze-size flag produces expected output on hello_world for iOS',
    () async {
      final String workingDirectory = fileSystem.path.join(
        getFlutterRoot(),
        'examples',
        'hello_world',
      );
      final Directory tempDir = fileSystem.systemTempDirectory.createTempSync('flutter_size_test.');
      final Directory codeSizeDir = tempDir.childDirectory('code size dir')..createSync();
      final ProcessResult result = await processManager.run(<String>[
        flutterBin,
        'build',
        'ios',
        '--verbose',
        '--analyze-size',
        '--code-size-directory=${codeSizeDir.path}',
        '--no-codesign',
      ], workingDirectory: workingDirectory);

      expect(
        result,
        const ProcessResultMatcher(stdoutPattern: 'Dart AOT symbols accounted decompressed size'),
      );

      final String line = result.stdout
          .toString()
          .split('\n')
          .firstWhere((String line) => line.contains(iosDebugMessage));

      final String outputFilePath = line.split(iosDebugMessage).last.trim();
      expect(fileSystem.file(fileSystem.path.join(workingDirectory, outputFilePath)), exists);

      final String devToolsCommand = result.stdout
          .toString()
          .split('\n')
          .firstWhere((String line) => line.contains(runDevToolsMessage));
      final String commandArguments = devToolsCommand.split(runDevToolsMessage).last.trim();

      expect(commandArguments.contains('--appSizeBase=$outputFilePath'), isTrue);
      expect(codeSizeDir.existsSync(), true);
      tempDir.deleteSync(recursive: true);
    },
    skip: !platform.isMacOS, // [intended] iOS can only be built on macos.
  );

  testWithoutContext(
    '--analyze-size flag produces expected output on hello_world for macOS',
    () async {
      final String workingDirectory = fileSystem.path.join(
        getFlutterRoot(),
        'examples',
        'hello_world',
      );
      final Directory tempDir = fileSystem.systemTempDirectory.createTempSync('flutter_size_test.');
      final Directory codeSizeDir = tempDir.childDirectory('code size dir')..createSync();

      final ProcessResult configResult = await processManager.run(<String>[
        flutterBin,
        'config',
        '--verbose',
        '--enable-macos-desktop',
      ], workingDirectory: workingDirectory);

      expect(configResult, const ProcessResultMatcher());

      printOnFailure('Output of flutter config:');
      printOnFailure(configResult.stdout.toString());
      printOnFailure(configResult.stderr.toString());

      final ProcessResult result = await processManager.run(<String>[
        flutterBin,
        'build',
        'macos',
        '--analyze-size',
        '--code-size-directory=${codeSizeDir.path}',
      ], workingDirectory: workingDirectory);

      expect(
        result,
        const ProcessResultMatcher(stdoutPattern: 'Dart AOT symbols accounted decompressed size'),
      );

      final String line = result.stdout
          .toString()
          .split('\n')
          .firstWhere((String line) => line.contains(macOSDebugMessage));

      final String outputFilePath = line.split(macOSDebugMessage).last.trim();
      expect(fileSystem.file(fileSystem.path.join(workingDirectory, outputFilePath)), exists);

      final String devToolsCommand = result.stdout
          .toString()
          .split('\n')
          .firstWhere((String line) => line.contains(runDevToolsMessage));
      final String commandArguments = devToolsCommand.split(runDevToolsMessage).last.trim();

      expect(commandArguments.contains('--appSizeBase=$outputFilePath'), isTrue);
      expect(codeSizeDir.existsSync(), true);
      tempDir.deleteSync(recursive: true);
    },
    skip: !platform.isMacOS, // [intended] this is a macos only test.
  );

  testWithoutContext('--analyze-size is only supported in release mode', () async {
    final ProcessResult result = await processManager.run(<String>[
      flutterBin,
      'build',
      'apk',
      '--verbose',
      '--analyze-size',
      '--target-platform=android-arm64',
      '--debug',
    ], workingDirectory: fileSystem.path.join(getFlutterRoot(), 'examples', 'hello_world'));
    expect(
      result,
      const ProcessResultMatcher(
        exitCode: 1,
        stderrPattern: '"--analyze-size" can only be used on release builds',
      ),
    );
  });

  testWithoutContext(
    '--analyze-size is not supported in combination with --split-debug-info',
    () async {
      final List<String> command = <String>[
        flutterBin,
        'build',
        'apk',
        '--verbose',
        '--analyze-size',
        '--target-platform=android-arm64',
        '--split-debug-info=infos',
      ];
      final String workingDirectory = fileSystem.path.join(
        getFlutterRoot(),
        'examples',
        'hello_world',
      );
      final ProcessResult result = await processManager.run(
        command,
        workingDirectory: workingDirectory,
      );

      expect(
        result,
        const ProcessResultMatcher(
          exitCode: 1,
          stderrPattern: '"--analyze-size" cannot be combined with "--split-debug-info"',
        ),
      );
    },
  );

  testWithoutContext(
    '--analyze-size allows overriding the directory for code size files',
    () async {
      final Directory tempDir = fileSystem.systemTempDirectory.createTempSync('flutter_size_test.');

      final List<String> command = <String>[
        flutterBin,
        'build',
        'apk',
        '--verbose',
        '--analyze-size',
        '--code-size-directory=${tempDir.path}',
        '--target-platform=android-arm64',
        '--release',
      ];
      final String workingDirectory = fileSystem.path.join(
        getFlutterRoot(),
        'examples',
        'hello_world',
      );
      final ProcessResult result = await processManager.run(
        command,
        workingDirectory: workingDirectory,
      );

      expect(result, const ProcessResultMatcher());

      expect(tempDir, exists);
      expect(tempDir.childFile('snapshot.arm64-v8a.json'), exists);
      expect(tempDir.childFile('trace.arm64-v8a.json'), exists);

      tempDir.deleteSync(recursive: true);
    },
  );
}
