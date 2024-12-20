// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:dds/dap.dart';
import 'package:file/file.dart';
import 'package:flutter_tools/src/cache.dart';

import '../../src/common.dart';
import '../test_data/integration_tests_project.dart';
import '../test_data/tests_project.dart';
import '../test_utils.dart';
import 'test_client.dart';
import 'test_support.dart';

void main() {
  late Directory tempDir;
  late DapTestSession dap;
  late DapTestClient client;
  late TestsProject project;

  setUpAll(() {
    Cache.flutterRoot = getFlutterRoot();
  });

  setUp(() async {
    tempDir = createResolvedTempDirectorySync('flutter_test_adapter_test.');
    dap = await DapTestSession.setUp(additionalArgs: <String>['--test']);
    client = dap.client;
  });

  tearDown(() async {
    await dap.tearDown();
    tryToDelete(tempDir);
  });

  void standardTests({List<String>? toolArgs}) {
    test('can run in debug mode', () async {
      // Collect output and test events while running the script.
      final TestEvents outputEvents = await client.collectTestOutput(
        launch:
            () => client.launch(
              program: project.testFilePath,
              cwd: project.dir.path,
              toolArgs: toolArgs,
            ),
      );

      // Check the printed output shows that the run finished, and it's exit
      // code (which is 1 due to the failing test).
      final String output = outputEvents.output.map((OutputEventBody e) => e.output).join();
      expectLines(
        output,
        <Object>[startsWith('Connecting to VM Service at'), ..._testsProjectExpectedOutput],
        allowExtras: true, // Allow for printed call stack etc.
      );

      _expectStandardTestsProjectResults(outputEvents);
    });

    test('can run in noDebug mode', () async {
      // Collect output and test events while running the script.
      final TestEvents outputEvents = await client.collectTestOutput(
        launch:
            () => client.launch(
              program: project.testFilePath,
              noDebug: true,
              cwd: project.dir.path,
              toolArgs: toolArgs,
            ),
      );

      // Check the printed output shows that the run finished, and it's exit
      // code (which is 1 due to the failing test).
      final String output = outputEvents.output.map((OutputEventBody e) => e.output).join();
      expectLines(
        output,
        _testsProjectExpectedOutput,
        allowExtras: true, // Allow for printed call stack etc.
      );

      _expectStandardTestsProjectResults(outputEvents);
    });

    test('can run a single test', () async {
      // Collect output and test events while running the script.
      final TestEvents outputEvents = await client.collectTestOutput(
        launch:
            () => client.launch(
              program: project.testFilePath,
              noDebug: true,
              cwd: project.dir.path,
              // It's up to the calling IDE to pass the correct args for
              // 'flutter test' if it wants to run a subset of tests.
              toolArgs: <String>['--plain-name', 'can pass', ...?toolArgs],
            ),
      );

      final List<Object> testsNames =
          outputEvents.testNotifications
              .where((Map<String, Object?> e) => e['type'] == 'testStart')
              .map((Map<String, Object?> e) => (e['test']! as Map<String, Object?>)['name']!)
              .toList();

      expect(testsNames, contains('Flutter tests can pass'));
      expect(testsNames, isNot(contains('Flutter tests can fail')));
    });
  }

  group('widget tests', () {
    setUp(() async {
      project = TestsProject();
      await project.setUpIn(tempDir);
    });

    standardTests();
  });

  group('integration tests', () {
    const List<String> toolArgs = <String>['-d', 'flutter-tester'];

    setUp(() async {
      project = IntegrationTestsProject();
      await project.setUpIn(tempDir);
    });

    standardTests(toolArgs: toolArgs);
  });
}

/// Matchers for the expected console output of [TestsProject].
final List<Object> _testsProjectExpectedOutput = <Object>[
  // First test
  '✓ Flutter tests can pass',
  // Second test
  '══╡ EXCEPTION CAUGHT BY FLUTTER TEST FRAMEWORK ╞════════════════════════════════════════════════════',
  'The following TestFailure was thrown running a test:',
  'Expected: false',
  '  Actual: <true>',
  '',
  'The test description was: can fail',
  '',
  '✖ Flutter tests can fail',
  // Exit
  '',
  'Exited (1).',
];

/// A helper that verifies a full set of expected test results for the
/// [TestsProject] script.
void _expectStandardTestsProjectResults(TestEvents events) {
  // Check we received all expected test events passed through from
  // package:test.
  final List<Object> eventNames =
      events.testNotifications.map((Map<String, Object?> e) => e['type']!).toList();

  // start/done should always be first/last.
  expect(eventNames.first, equals('start'));
  expect(eventNames.last, equals('done'));

  // allSuites should have occurred after start.
  expect(eventNames, containsAllInOrder(<String>['start', 'allSuites']));

  // Expect two tests, with the failing one emitting an error.
  expect(
    eventNames,
    containsAllInOrder(<String>['testStart', 'testDone', 'testStart', 'error', 'testDone']),
  );
}
