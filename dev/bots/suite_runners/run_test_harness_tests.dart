// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' show File, Platform;

import 'package:path/path.dart' as path;

import '../run_command.dart';
import '../utils.dart';

String get platformFolderName {
  if (Platform.isWindows) {
    return 'windows-x64';
  }
  if (Platform.isMacOS) {
    return 'darwin-x64';
  }
  if (Platform.isLinux) {
    return 'linux-x64';
  }
  throw UnsupportedError(
    'The platform ${Platform.operatingSystem} is not supported by this script.',
  );
}

Future<void> testHarnessTestsRunner() async {
  printProgress('${green}Running test harness tests...$reset');

  await _validateEngineHash();

  // Verify that the tests actually return failure on failure and success on
  // success.
  final String automatedTests = path.join(flutterRoot, 'dev', 'automated_tests');

  // We want to run these tests in parallel, because they each take some time
  // to run (e.g. compiling), so we don't want to run them in series, especially
  // on 20-core machines. However, we have a race condition, so for now...
  // Race condition issue: https://github.com/flutter/flutter/issues/90026
  final List<ShardRunner> tests = <ShardRunner>[
    () => runFlutterTest(
      automatedTests,
      script: path.join('test_smoke_test', 'pass_test.dart'),
      printOutput: false,
    ),
    () => runFlutterTest(
      automatedTests,
      script: path.join('test_smoke_test', 'fail_test.dart'),
      expectFailure: true,
      printOutput: false,
    ),
    () => runFlutterTest(
      automatedTests,
      script: path.join('test_smoke_test', 'pending_timer_fail_test.dart'),
      expectFailure: true,
      printOutput: false,
      outputChecker: (CommandResult result) {
        return result.flattenedStdout!.contains('failingPendingTimerTest')
            ? null
            : 'Failed to find the stack trace for the pending Timer.\n\n'
                'stdout:\n${result.flattenedStdout}\n\n'
                'stderr:\n${result.flattenedStderr}';
      },
    ),
    () => runFlutterTest(
      automatedTests,
      script: path.join('test_smoke_test', 'fail_test_on_exception_after_test.dart'),
      expectFailure: true,
      printOutput: false,
      outputChecker: (CommandResult result) {
        const String expectedError =
            '══╡ EXCEPTION CAUGHT BY FLUTTER TEST FRAMEWORK ╞════════════════════════════════════════════════════\n'
            'The following StateError was thrown running a test (but after the test had completed):\n'
            'Bad state: Exception thrown after test completed.';
        if (result.flattenedStdout!.contains(expectedError)) {
          return null;
        }
        return 'Failed to find expected output on stdout.\n\n'
            'Expected output:\n$expectedError\n\n'
            'Actual stdout:\n${result.flattenedStdout}\n\n'
            'Actual stderr:\n${result.flattenedStderr}';
      },
    ),
    () => runFlutterTest(
      automatedTests,
      script: path.join('test_smoke_test', 'crash1_test.dart'),
      expectFailure: true,
      printOutput: false,
    ),
    () => runFlutterTest(
      automatedTests,
      script: path.join('test_smoke_test', 'crash2_test.dart'),
      expectFailure: true,
      printOutput: false,
    ),
    () => runFlutterTest(
      automatedTests,
      script: path.join('test_smoke_test', 'syntax_error_test.broken_dart'),
      expectFailure: true,
      printOutput: false,
    ),
    () => runFlutterTest(
      automatedTests,
      script: path.join('test_smoke_test', 'missing_import_test.broken_dart'),
      expectFailure: true,
      printOutput: false,
    ),
    () => runFlutterTest(
      automatedTests,
      script: path.join('test_smoke_test', 'disallow_error_reporter_modification_test.dart'),
      expectFailure: true,
      printOutput: false,
    ),
  ];

  List<ShardRunner> testsToRun;

  // Run all tests unless sharding is explicitly specified.
  final String? shardName = Platform.environment[kShardKey];
  if (shardName == kTestHarnessShardName) {
    testsToRun = selectIndexOfTotalSubshard<ShardRunner>(tests);
  } else {
    testsToRun = tests;
  }
  for (final ShardRunner test in testsToRun) {
    await test();
  }

  // Verify that we correctly generated the version file.
  final String? versionError = await verifyVersion(File(path.join(flutterRoot, 'version')));
  if (versionError != null) {
    foundError(<String>[versionError]);
  }
}

/// Verify the Flutter Engine is the revision in
/// bin/cache/internal/engine.version.
Future<void> _validateEngineHash() async {
  final String flutterTester = path.join(
    flutterRoot,
    'bin',
    'cache',
    'artifacts',
    'engine',
    platformFolderName,
    'flutter_tester$exe',
  );

  if (runningInDartHHHBot) {
    // The Dart HHH bots intentionally modify the local artifact cache
    // and then use this script to run Flutter's test suites.
    // Because the artifacts have been changed, this particular test will return
    // a false positive and should be skipped.
    print('${yellow}Skipping Flutter Engine Version Validation for swarming bot $luciBotId.');
    return;
  }
  final String expectedVersion = File(engineVersionFile).readAsStringSync().trim();
  final CommandResult result = await runCommand(flutterTester, <String>[
    '--help',
  ], outputMode: OutputMode.capture);
  if (result.flattenedStdout!.isNotEmpty) {
    foundError(<String>[
      '${red}The stdout of `$flutterTester --help` was not empty:$reset',
      ...result.flattenedStdout!.split('\n').map((String line) => ' $gray┆$reset $line'),
    ]);
  }
  final String actualVersion;
  try {
    actualVersion = result.flattenedStderr!.split('\n').firstWhere((final String line) {
      return line.startsWith('Flutter Engine Version:');
    });
  } on StateError {
    foundError(<String>[
      '${red}Could not find "Flutter Engine Version:" line in `${path.basename(flutterTester)} --help` stderr output:$reset',
      ...result.flattenedStderr!.split('\n').map((String line) => ' $gray┆$reset $line'),
    ]);
    return;
  }
  if (!actualVersion.contains(expectedVersion)) {
    foundError(<String>[
      '${red}Expected "Flutter Engine Version: $expectedVersion", but found "$actualVersion".$reset',
    ]);
  }
}
