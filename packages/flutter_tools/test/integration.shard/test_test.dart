// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';

import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/io.dart';

import '../src/common.dart';
import 'test_utils.dart';

// This test depends on some files in ///dev/automated_tests/flutter_test/*

final String automatedTestsDirectory = fileSystem.path.join('..', '..', 'dev', 'automated_tests');
final String missingDependencyDirectory = fileSystem.path.join(
  '..',
  '..',
  'dev',
  'missing_dependency_tests',
);
final String flutterTestDirectory = fileSystem.path.join(automatedTestsDirectory, 'flutter_test');
final String integrationTestDirectory = fileSystem.path.join(
  automatedTestsDirectory,
  'integration_test',
);

// Running Integration Tests in the Flutter Tester will still exercise the same
// flows specific to Integration Tests.
final List<String> integrationTestExtraArgs = <String>['-d', 'flutter-tester'];

void main() {
  setUpAll(() async {
    expect(
      await processManager.run(<String>[
        flutterBin,
        'pub',
        'get',
      ], workingDirectory: flutterTestDirectory),
      const ProcessResultMatcher(),
    );
    expect(
      await processManager.run(<String>[
        flutterBin,
        'pub',
        'get',
      ], workingDirectory: missingDependencyDirectory),
      const ProcessResultMatcher(),
    );
  });

  testWithoutContext('flutter test should not have extraneous error messages', () async {
    return _testFile(
      'trivial_widget',
      automatedTestsDirectory,
      flutterTestDirectory,
      exitCode: isZero,
    );
  });

  testWithoutContext('integration test should not have extraneous error messages', () async {
    return _testFile(
      'trivial_widget',
      automatedTestsDirectory,
      integrationTestDirectory,
      exitCode: isZero,
      extraArguments: integrationTestExtraArgs,
    );
  });

  testWithoutContext('flutter test set the working directory correctly', () async {
    return _testFile(
      'working_directory',
      automatedTestsDirectory,
      flutterTestDirectory,
      exitCode: isZero,
    );
  });

  testWithoutContext(
    'flutter test should report nice errors for exceptions thrown within testWidgets()',
    () async {
      return _testFile('exception_handling', automatedTestsDirectory, flutterTestDirectory);
    },
  );

  testWithoutContext(
    'integration test should report nice errors for exceptions thrown within testWidgets()',
    () async {
      return _testFile(
        'exception_handling',
        automatedTestsDirectory,
        integrationTestDirectory,
        extraArguments: integrationTestExtraArgs,
      );
    },
  );

  testWithoutContext(
    'flutter test should report a nice error when a guarded function was called without await',
    () async {
      return _testFile('test_async_utils_guarded', automatedTestsDirectory, flutterTestDirectory);
    },
  );

  testWithoutContext(
    'flutter test should report a nice error when an async function was called without await',
    () async {
      return _testFile('test_async_utils_unguarded', automatedTestsDirectory, flutterTestDirectory);
    },
  );

  testWithoutContext(
    'flutter test should report a nice error when a Ticker is left running',
    () async {
      return _testFile('ticker', automatedTestsDirectory, flutterTestDirectory);
    },
  );

  testWithoutContext(
    'flutter test should report a nice error when a pubspec.yaml is missing a flutter_test dependency',
    () async {
      final String missingDependencyTests = fileSystem.path.join(
        '..',
        '..',
        'dev',
        'missing_dependency_tests',
      );
      return _testFile('trivial', missingDependencyTests, missingDependencyTests);
    },
  );

  testWithoutContext(
    'flutter test should report which user-created widget caused the error',
    () async {
      return _testFile(
        'print_user_created_ancestor',
        automatedTestsDirectory,
        flutterTestDirectory,
        extraArguments: const <String>['--track-widget-creation'],
      );
    },
  );

  testWithoutContext(
    'flutter test should report which user-created widget caused the error - no flag',
    () async {
      return _testFile(
        'print_user_created_ancestor_no_flag',
        automatedTestsDirectory,
        flutterTestDirectory,
        extraArguments: const <String>['--no-track-widget-creation'],
      );
    },
  );

  testWithoutContext(
    'flutter test should report the correct user-created widget that caused the error',
    () async {
      return _testFile(
        'print_correct_local_widget',
        automatedTestsDirectory,
        flutterTestDirectory,
        extraArguments: const <String>['--track-widget-creation'],
      );
    },
  );

  testWithoutContext('flutter test should can load assets within its own package', () async {
    return _testFile(
      'package_assets',
      automatedTestsDirectory,
      flutterTestDirectory,
      exitCode: isZero,
    );
  });

  testWithoutContext('flutter test should support dart defines', () async {
    return _testFile(
      'dart_defines',
      automatedTestsDirectory,
      flutterTestDirectory,
      exitCode: isZero,
      extraArguments: <String>['--dart-define=flutter.test.foo=bar'],
    );
  });

  testWithoutContext('flutter test should run a test when its name matches a regexp', () async {
    final ProcessResult result = await _runFlutterTest(
      'filtering',
      automatedTestsDirectory,
      flutterTestDirectory,
      extraArguments: const <String>['--name', 'inc.*de'],
    );
    expect(result, ProcessResultMatcher(stdoutPattern: RegExp(r'\+\d+: All tests passed!')));
  });

  testWithoutContext(
    'flutter test should run a test when its name matches a regexp when --experimental-faster-testing is set',
    () async {
      final ProcessResult result = await _runFlutterTest(
        null,
        automatedTestsDirectory,
        flutterTestDirectory,
        extraArguments: const <String>['--experimental-faster-testing', '--name=inc.*de'],
      );
      expect(result, ProcessResultMatcher(stdoutPattern: RegExp(r'\+\d+: All tests passed!')));
    },
  );

  testWithoutContext('flutter test should run a test when its name contains a string', () async {
    final ProcessResult result = await _runFlutterTest(
      'filtering',
      automatedTestsDirectory,
      flutterTestDirectory,
      extraArguments: const <String>['--plain-name', 'include'],
    );
    expect(result, ProcessResultMatcher(stdoutPattern: RegExp(r'\+\d+: All tests passed!')));
  });

  testWithoutContext(
    'flutter test should run a test when its name contains a string when --experimental-faster-testing is set',
    () async {
      final ProcessResult result = await _runFlutterTest(
        null,
        automatedTestsDirectory,
        flutterTestDirectory,
        extraArguments: const <String>['--experimental-faster-testing', '--plain-name=include'],
      );
      expect(result, ProcessResultMatcher(stdoutPattern: RegExp(r'\+\d+: All tests passed!')));
    },
  );

  testWithoutContext('flutter test should run a test with a given tag', () async {
    final ProcessResult result = await _runFlutterTest(
      'filtering_tag',
      automatedTestsDirectory,
      flutterTestDirectory,
      extraArguments: const <String>['--tags', 'include-tag'],
    );
    expect(result, ProcessResultMatcher(stdoutPattern: RegExp(r'\+\d+: All tests passed!')));
  });

  testWithoutContext(
    'flutter test should run a test with a given tag when --experimental-faster-testing is set',
    () async {
      final ProcessResult result = await _runFlutterTest(
        null,
        automatedTestsDirectory,
        flutterTestDirectory,
        extraArguments: const <String>['--experimental-faster-testing', '--tags=include-tag'],
      );
      expect(result, ProcessResultMatcher(stdoutPattern: RegExp(r'\+\d+: All tests passed!')));
    },
  );

  testWithoutContext('flutter test should not run a test with excluded tag', () async {
    final ProcessResult result = await _runFlutterTest(
      'filtering_tag',
      automatedTestsDirectory,
      flutterTestDirectory,
      extraArguments: const <String>['--exclude-tags', 'exclude-tag'],
    );
    expect(result, ProcessResultMatcher(stdoutPattern: RegExp(r'\+\d+: All tests passed!')));
  });

  testWithoutContext('flutter test should run all tests when tags are unspecified', () async {
    final ProcessResult result = await _runFlutterTest(
      'filtering_tag',
      automatedTestsDirectory,
      flutterTestDirectory,
    );
    expect(
      result,
      ProcessResultMatcher(exitCode: 1, stdoutPattern: RegExp(r'\+\d+ -1: Some tests failed\.')),
    );
  });

  testWithoutContext(
    'flutter test should run all tests when tags are unspecified when --experimental-faster-testing is set',
    () async {
      final ProcessResult result = await _runFlutterTest(
        null,
        automatedTestsDirectory,
        flutterTestDirectory,
        extraArguments: const <String>['--experimental-faster-testing'],
      );
      expect(
        result,
        ProcessResultMatcher(
          exitCode: 1,
          stdoutPattern: RegExp(r'\+\d+ -\d+: Some tests failed\.'),
        ),
      );
    },
  );

  testWithoutContext('flutter test should run a widgetTest with a given tag', () async {
    final ProcessResult result = await _runFlutterTest(
      'filtering_tag_widget',
      automatedTestsDirectory,
      flutterTestDirectory,
      extraArguments: const <String>['--tags', 'include-tag'],
    );
    expect(result, ProcessResultMatcher(stdoutPattern: RegExp(r'\+\d+: All tests passed!')));
  });

  testWithoutContext('flutter test should not run a widgetTest with excluded tag', () async {
    final ProcessResult result = await _runFlutterTest(
      'filtering_tag_widget',
      automatedTestsDirectory,
      flutterTestDirectory,
      extraArguments: const <String>['--exclude-tags', 'exclude-tag'],
    );
    expect(result, ProcessResultMatcher(stdoutPattern: RegExp(r'\+\d+: All tests passed!')));
  });

  testWithoutContext('flutter test should run all widgetTest when tags are unspecified', () async {
    final ProcessResult result = await _runFlutterTest(
      'filtering_tag_widget',
      automatedTestsDirectory,
      flutterTestDirectory,
    );
    expect(
      result,
      ProcessResultMatcher(exitCode: 1, stdoutPattern: RegExp(r'\+\d+ -1: Some tests failed\.')),
    );
  });

  testWithoutContext('flutter test should run a test with an exact name in URI format', () async {
    final ProcessResult result = await _runFlutterTest(
      'uri_format',
      automatedTestsDirectory,
      flutterTestDirectory,
      query: 'full-name=exactTestName',
    );
    expect(result, ProcessResultMatcher(stdoutPattern: RegExp(r'\+\d+: All tests passed!')));
  });

  testWithoutContext('flutter test should run a test by line number in URI format', () async {
    final ProcessResult result = await _runFlutterTest(
      'uri_format',
      automatedTestsDirectory,
      flutterTestDirectory,
      query: 'line=11',
    );
    expect(result, ProcessResultMatcher(stdoutPattern: RegExp(r'\+\d+: All tests passed!')));
  });

  testWithoutContext('flutter test should test runs to completion', () async {
    final ProcessResult result = await _runFlutterTest(
      'trivial',
      automatedTestsDirectory,
      flutterTestDirectory,
      extraArguments: const <String>['--verbose'],
    );
    final String stdout = (result.stdout as String).replaceAll('\r', '\n');
    expect(stdout, contains(RegExp(r'\+\d+: All tests passed\!')));
    expect(stdout, contains('test 0: Starting flutter_tester process with command'));
    expect(stdout, contains('test 0: deleting temporary directory'));
    expect(stdout, contains('test 0: finished'));
    expect(stdout, contains('test package returned with exit code 0'));
    if ((result.stderr as String).isNotEmpty) {
      fail('unexpected error output from test:\n\n${result.stderr}\n-- end stderr --\n\n');
    }
    expect(result, const ProcessResultMatcher());
  });

  testWithoutContext(
    'flutter test should test runs to completion when --experimental-faster-testing is set',
    () async {
      final ProcessResult result = await _runFlutterTest(
        null,
        automatedTestsDirectory,
        '$flutterTestDirectory/child_directory',
        extraArguments: const <String>['--experimental-faster-testing', '--verbose'],
      );
      final String stdout = (result.stdout as String).replaceAll('\r', '\n');
      expect(stdout, contains(RegExp(r'\+\d+: All tests passed\!')));
      expect(stdout, contains('Starting flutter_tester process with command'));
      if ((result.stderr as String).isNotEmpty) {
        fail('unexpected error output from test:\n\n${result.stderr}\n-- end stderr --\n\n');
      }
      expect(result, const ProcessResultMatcher());
    },
  );

  testWithoutContext(
    'flutter test should ignore --experimental-faster-testing when only a single test file is specified',
    () async {
      final ProcessResult result = await _runFlutterTest(
        'trivial',
        automatedTestsDirectory,
        flutterTestDirectory,
        extraArguments: const <String>['--experimental-faster-testing', '--verbose'],
      );
      final String stdout = (result.stdout as String).replaceAll('\r', '\n');
      expect(
        stdout,
        contains(
          '--experimental-faster-testing was parsed but will be ignored. '
          'This option should not be used when running a single test file.',
        ),
      );
      expect(stdout, contains(RegExp(r'\+\d+: All tests passed\!')));
      expect(stdout, contains('test 0: Starting flutter_tester process with command'));
      expect(stdout, contains('test 0: deleting temporary directory'));
      expect(stdout, contains('test 0: finished'));
      expect(stdout, contains('test package returned with exit code 0'));
      if ((result.stderr as String).isNotEmpty) {
        fail('unexpected error output from test:\n\n${result.stderr}\n-- end stderr --\n\n');
      }
      expect(result, const ProcessResultMatcher());
    },
  );

  testWithoutContext(
    'flutter test should ignore --experimental-faster-testing when running integration tests',
    () async {
      final ProcessResult result = await _runFlutterTest(
        'trivial_widget',
        automatedTestsDirectory,
        integrationTestDirectory,
        extraArguments: <String>[...integrationTestExtraArgs, '--experimental-faster-testing'],
      );
      final String stdout = (result.stdout as String).replaceAll('\r', '\n');
      expect(
        stdout,
        contains(
          '--experimental-faster-testing was parsed but will be ignored. '
          'This option is not supported when running integration tests or web '
          'tests.',
        ),
      );
      if ((result.stderr as String).isNotEmpty) {
        fail('unexpected error output from test:\n\n${result.stderr}\n-- end stderr --\n\n');
      }
      expect(result, const ProcessResultMatcher());
    },
  );

  testWithoutContext(
    'flutter test should run all tests inside of a directory with no trailing slash',
    () async {
      final ProcessResult result = await _runFlutterTest(
        null,
        automatedTestsDirectory,
        '$flutterTestDirectory/child_directory',
        extraArguments: const <String>['--verbose'],
      );
      final String stdout = (result.stdout as String).replaceAll('\r', '\n');
      expect(result.stdout, contains(RegExp(r'\+\d+: All tests passed\!')));
      expect(stdout, contains('test 0: Starting flutter_tester process with command'));
      expect(stdout, contains('test 0: deleting temporary directory'));
      expect(stdout, contains('test 0: finished'));
      expect(stdout, contains('test package returned with exit code 0'));
      if ((result.stderr as String).isNotEmpty) {
        fail('unexpected error output from test:\n\n${result.stderr}\n-- end stderr --\n\n');
      }
      expect(result, const ProcessResultMatcher());
    },
  );

  testWithoutContext('flutter gold skips tests where the expectations are missing', () async {
    return _testFile(
      'flutter_gold',
      automatedTestsDirectory,
      flutterTestDirectory,
      exitCode: isZero,
    );
  });

  testWithoutContext('flutter test should respect --serve-observatory', () async {
    Process? process;
    StreamSubscription<String>? sub;
    try {
      process = await _runFlutterTestConcurrent(
        'trivial',
        automatedTestsDirectory,
        flutterTestDirectory,
        extraArguments: const <String>['--start-paused', '--serve-observatory'],
      );
      final Completer<Uri> completer = Completer<Uri>();
      final RegExp vmServiceUriRegExp = RegExp(r'((http)?:\/\/)[^\s]+');
      sub = process.stdout.transform(utf8.decoder).listen((String e) {
        if (!completer.isCompleted && vmServiceUriRegExp.hasMatch(e)) {
          completer.complete(Uri.parse(vmServiceUriRegExp.firstMatch(e)!.group(0)!));
        }
      });
      final Uri vmServiceUri = await completer.future;
      final HttpClient client = HttpClient();
      final HttpClientRequest request = await client.getUrl(vmServiceUri);
      final HttpClientResponse response = await request.close();
      final String content = await response.transform(utf8.decoder).join();
      expect(content, contains('Dart VM Observatory'));
    } finally {
      await sub?.cancel();
      process?.kill();
    }
  });

  testWithoutContext('flutter test should serve DevTools', () async {
    Process? process;
    StreamSubscription<String>? sub;
    try {
      process = await _runFlutterTestConcurrent(
        'trivial',
        automatedTestsDirectory,
        flutterTestDirectory,
        extraArguments: const <String>['--start-paused'],
      );
      final Completer<Uri> completer = Completer<Uri>();
      final RegExp devToolsUriRegExp = RegExp(
        r'The Flutter DevTools debugger and profiler is available at: (http://[^\s]+)',
      );
      sub = process.stdout.transform(utf8.decoder).listen((String e) {
        if (!completer.isCompleted && devToolsUriRegExp.hasMatch(e)) {
          completer.complete(Uri.parse(devToolsUriRegExp.firstMatch(e)!.group(1)!));
        }
      });
      final Uri devToolsUri = await completer.future;
      final HttpClient client = HttpClient();
      final HttpClientRequest request = await client.getUrl(devToolsUri);
      final HttpClientResponse response = await request.close();
      final String content = await response.transform(utf8.decoder).join();
      expect(content, contains('DevTools'));
    } finally {
      await sub?.cancel();
      process?.kill();
    }
  });
}

Future<void> _testFile(
  String testName,
  String workingDirectory,
  String testDirectory, {
  Matcher? exitCode,
  List<String> extraArguments = const <String>[],
}) async {
  exitCode ??= isNonZero;
  final String fullTestExpectation = fileSystem.path.join(
    testDirectory,
    '${testName}_expectation.txt',
  );
  final File expectationFile = fileSystem.file(fullTestExpectation);
  if (!expectationFile.existsSync()) {
    fail('missing expectation file: $expectationFile');
  }

  final ProcessResult exec = await _runFlutterTest(
    testName,
    workingDirectory,
    testDirectory,
    extraArguments: extraArguments,
  );

  expect(
    exec.exitCode,
    exitCode,
    reason:
        '"$testName" returned code ${exec.exitCode}\n\nstdout:\n'
        '${exec.stdout}\nstderr:\n${exec.stderr}',
  );
  List<String> output = (exec.stdout as String).split('\n');

  output = _removeMacFontServerWarning(output);

  if (output.first.startsWith(
    'Waiting for another flutter command to release the startup lock...',
  )) {
    output.removeAt(0);
  }
  if (output.first.startsWith('Running "flutter pub get" in')) {
    output.removeAt(0);
  }
  // Whether cached artifacts need to be downloaded is dependent on what
  // previous tests have run. Disregard these messages.
  output.removeWhere(RegExp(r'Downloading .*\.\.\.').hasMatch);
  output.add('<<stderr>>');
  output.addAll((exec.stderr as String).split('\n'));
  final List<String> expectations = fileSystem.file(fullTestExpectation).readAsLinesSync();
  bool allowSkip = false;
  int expectationLineNumber = 0;
  int outputLineNumber = 0;
  bool haveSeenStdErrMarker = false;
  while (expectationLineNumber < expectations.length) {
    expect(
      output,
      hasLength(greaterThan(outputLineNumber)),
      reason: 'Failure in $testName to compare to $fullTestExpectation',
    );
    final String expectationLine = expectations[expectationLineNumber];
    String outputLine = output[outputLineNumber];
    if (expectationLine == '<<skip until matching line>>') {
      allowSkip = true;
      expectationLineNumber += 1;
      continue;
    }
    if (allowSkip) {
      if (!RegExp(expectationLine).hasMatch(outputLine)) {
        outputLineNumber += 1;
        continue;
      }
      allowSkip = false;
    }
    if (expectationLine == '<<stderr>>') {
      expect(haveSeenStdErrMarker, isFalse);
      haveSeenStdErrMarker = true;
    }
    if (!RegExp(expectationLine).hasMatch(outputLine) && outputLineNumber + 1 < output.length) {
      // Check if the RegExp can match the next two lines in the output so
      // that it is possible to write expectations that still hold even if a
      // line is wrapped slightly differently due to for example a file name
      // being longer on one platform than another.
      final String mergedLines = '$outputLine\n${output[outputLineNumber + 1]}';
      if (RegExp(expectationLine).hasMatch(mergedLines)) {
        outputLineNumber += 1;
        outputLine = mergedLines;
      }
    }

    expect(
      outputLine,
      matches(expectationLine),
      reason:
          'Full output:\n- - - -----8<----- - - -\n${output.join("\n")}\n- - - -----8<----- - - -',
    );
    expectationLineNumber += 1;
    outputLineNumber += 1;
  }
  expect(allowSkip, isFalse);
  if (!haveSeenStdErrMarker) {
    expect(exec.stderr, '');
  }
}

final RegExp _fontServerProtocolPattern = RegExp(
  r'flutter_tester.*Font server protocol version mismatch',
);
final RegExp _unableToConnectToFontDaemonPattern = RegExp(
  r'flutter_tester.*XType: unable to make a connection to the font daemon!',
);
final RegExp _xtFontStaticRegistryPattern = RegExp(
  r'flutter_tester.*XType: XTFontStaticRegistry is enabled as fontd is not available',
);

// https://github.com/flutter/flutter/issues/132990
List<String> _removeMacFontServerWarning(List<String> output) {
  return output.where((String line) {
    if (_fontServerProtocolPattern.hasMatch(line)) {
      return false;
    }
    if (_unableToConnectToFontDaemonPattern.hasMatch(line)) {
      return false;
    }
    if (_xtFontStaticRegistryPattern.hasMatch(line)) {
      return false;
    }
    return true;
  }).toList();
}

Future<ProcessResult> _runFlutterTest(
  String? testName,
  String workingDirectory,
  String testDirectory, {
  List<String> extraArguments = const <String>[],
  String? query,
}) async {
  String testPath;
  if (testName == null) {
    // Test everything in the directory.
    testPath = testDirectory;
    final Directory directoryToTest = fileSystem.directory(testPath);
    if (!directoryToTest.existsSync()) {
      fail('missing test directory: $directoryToTest');
    }
  } else {
    // Test just a specific test file.
    testPath = fileSystem.path.join(testDirectory, '${testName}_test.dart');
    final File testFile = fileSystem.file(testPath);
    if (!testFile.existsSync()) {
      fail('missing test file: $testFile');
    }
  }

  final List<String> args = <String>[
    'test',
    '--no-color',
    '--no-version-check',
    '--no-pub',
    '--reporter',
    'compact',
    ...extraArguments,
    if (query != null) Uri.file(testPath).replace(query: query).toString() else testPath,
  ];

  return Process.run(
    flutterBin, // Uses the precompiled flutter tool for faster tests,
    args,
    workingDirectory: workingDirectory,
    stdoutEncoding: utf8,
    stderrEncoding: utf8,
  );
}

Future<Process> _runFlutterTestConcurrent(
  String? testName,
  String workingDirectory,
  String testDirectory, {
  List<String> extraArguments = const <String>[],
}) async {
  String testPath;
  if (testName == null) {
    // Test everything in the directory.
    testPath = testDirectory;
    final Directory directoryToTest = fileSystem.directory(testPath);
    if (!directoryToTest.existsSync()) {
      fail('missing test directory: $directoryToTest');
    }
  } else {
    // Test just a specific test file.
    testPath = fileSystem.path.join(testDirectory, '${testName}_test.dart');
    final File testFile = fileSystem.file(testPath);
    if (!testFile.existsSync()) {
      fail('missing test file: $testFile');
    }
  }

  final List<String> args = <String>[
    'test',
    '--no-color',
    '--no-version-check',
    '--no-pub',
    '--reporter',
    'compact',
    ...extraArguments,
    testPath,
  ];

  return Process.start(
    flutterBin, // Uses the precompiled flutter tool for faster tests,
    args,
    workingDirectory: workingDirectory,
  );
}
