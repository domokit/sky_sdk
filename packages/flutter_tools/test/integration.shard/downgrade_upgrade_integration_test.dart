// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/process.dart';
import 'package:flutter_tools/src/base/terminal.dart';

import '../src/common.dart';
import 'test_utils.dart';

const String _kInitialVersion = '3.0.0';
const String _kBranch = 'beta';

final Stdio stdio = Stdio();
final BufferLogger logger = BufferLogger.test(
  terminal: AnsiTerminal(platform: platform, stdio: stdio),
  outputPreferences: OutputPreferences.test(wrapText: true),
);
final ProcessUtils processUtils = ProcessUtils(processManager: processManager, logger: logger);

/// A test for flutter upgrade & downgrade that checks out a parallel flutter repo.
void main() {
  late Directory parentDirectory;

  setUp(() {
    parentDirectory = fileSystem.systemTempDirectory.createTempSync('flutter_tools.');
    parentDirectory.createSync(recursive: true);
  });

  tearDown(() {
    tryToDelete(parentDirectory);
  });

  testWithoutContext('Can upgrade and downgrade a Flutter checkout', () async {
    final Directory testDirectory = parentDirectory.childDirectory('flutter');
    testDirectory.createSync(recursive: true);

    // Enable longpaths for windows integration test.
    await processManager.run(<String>['git', 'config', '--system', 'core.longpaths', 'true']);

    void checkExitCode(int code) {
      expect(
        exitCode,
        0,
        reason: '''
trace:
${logger.traceText}

status:
${logger.statusText}

error:
${logger.errorText}''',
      );
    }

    printOnFailure('Step 1 - clone the $_kBranch of flutter into the test directory');
    exitCode = await processUtils.stream(
      <String>['git', 'clone', 'https://github.com/flutter/flutter.git'],
      workingDirectory: parentDirectory.path,
      trace: true,
    );
    checkExitCode(exitCode);

    printOnFailure('Step 2 - switch to the $_kBranch');
    exitCode = await processUtils.stream(
      <String>['git', 'checkout', '--track', '-b', _kBranch, 'origin/$_kBranch'],
      workingDirectory: testDirectory.path,
      trace: true,
    );
    checkExitCode(exitCode);

    printOnFailure('Step 3 - revert back to $_kInitialVersion');
    exitCode = await processUtils.stream(
      <String>['git', 'reset', '--hard', _kInitialVersion],
      workingDirectory: testDirectory.path,
      trace: true,
    );
    checkExitCode(exitCode);

    printOnFailure('Step 4 - upgrade to the newest $_kBranch');
    // This should update the persistent tool state with the sha for HEAD
    // This is probably a source of flakes as it mutates system-global state.
    exitCode = await processUtils.stream(
      <String>[
        flutterBin,
        'upgrade',
        '--verbose',
        '--working-directory=${testDirectory.path}',
        // we intentionally run this in a directory outside the test repo to
        // verify the tool overrides the working directory when invoking git
      ],
      workingDirectory: parentDirectory.path,
      trace: true,
    );
    checkExitCode(exitCode);

    printOnFailure('Step 5 - verify that the version is different');
    final RunResult versionResult = await processUtils.run(<String>[
      'git',
      'describe',
      '--match',
      '*.*.*',
      '--long',
      '--tags',
    ], workingDirectory: testDirectory.path);
    expect(versionResult.stdout, isNot(contains(_kInitialVersion)));
    printOnFailure(
      'current version is ${versionResult.stdout.trim()}\ninitial was $_kInitialVersion',
    );

    printOnFailure('Step 6 - downgrade back to the initial version');
    exitCode = await processUtils.stream(
      <String>[flutterBin, 'downgrade', '--no-prompt', '--working-directory=${testDirectory.path}'],
      workingDirectory: parentDirectory.path,
      trace: true,
    );
    checkExitCode(exitCode);

    printOnFailure('Step 7 - verify downgraded version matches original version');
    final RunResult oldVersionResult = await processUtils.run(<String>[
      'git',
      'describe',
      '--match',
      '*.*.*',
      '--long',
      '--tags',
    ], workingDirectory: testDirectory.path);
    expect(oldVersionResult.stdout, contains(_kInitialVersion));
    printOnFailure(
      'current version is ${oldVersionResult.stdout.trim()}\ninitial was $_kInitialVersion',
    );
  });
}
