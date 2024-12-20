// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:integration_test/integration_test.dart';
library;

import 'dart:convert';
import 'dart:io';

import 'package:flutter_test/flutter_test.dart';
import 'package:path/path.dart' as path;

final String bat = Platform.isWindows ? '.bat' : '';
final String _flutterBin = path.join(Directory.current.parent.parent.path, 'bin', 'flutter$bat');
const String _integrationResultsPrefix = 'IntegrationTestWidgetsFlutterBinding test results:';
const String _failureExcerpt = r'Expected: <false>\n  Actual: <true>';

Future<void> main() async {
  group('Integration binding result', () {
    test('when multiple tests pass', () async {
      final Map<String, dynamic>? results = await _runTest(
        path.join('test', 'data', 'pass_test_script.dart'),
      );

      expect(
        results,
        equals(<String, dynamic>{'passing test 1': 'success', 'passing test 2': 'success'}),
      );
    });

    test('when multiple tests fail', () async {
      final Map<String, dynamic>? results = await _runTest(
        path.join('test', 'data', 'fail_test_script.dart'),
      );

      expect(results, hasLength(2));
      expect(results, containsPair('failing test 1', contains(_failureExcerpt)));
      expect(results, containsPair('failing test 2', contains(_failureExcerpt)));
    });

    test('when one test passes, then another fails', () async {
      final Map<String, dynamic>? results = await _runTest(
        path.join('test', 'data', 'pass_then_fail_test_script.dart'),
      );

      expect(results, hasLength(2));
      expect(results, containsPair('passing test', equals('success')));
      expect(results, containsPair('failing test', contains(_failureExcerpt)));
    });

    test('when one test fails, then another passes', () async {
      final Map<String, dynamic>? results = await _runTest(
        path.join('test', 'data', 'fail_then_pass_test_script.dart'),
      );

      expect(results, hasLength(2));
      expect(results, containsPair('failing test', contains(_failureExcerpt)));
      expect(results, containsPair('passing test', equals('success')));
    });
  });
}

/// Runs a test script and returns the [IntegrationTestWidgetsFlutterBinding.results].
///
/// [scriptPath] is relative to the package root.
Future<Map<String, dynamic>?> _runTest(String scriptPath) async {
  final Process process = await Process.start(_flutterBin, <String>[
    'test',
    '--machine',
    scriptPath,
  ]);

  /// In the test [tearDownAll] block, the test results are encoded into JSON and
  /// are printed with the [_integrationResultsPrefix] prefix.
  ///
  /// See the following for the test event spec which we parse the printed lines
  /// out of: https://github.com/dart-lang/test/blob/master/pkgs/test/doc/json_reporter.md
  final String testResults = (await process.stdout
      .transform(utf8.decoder)
      .expand((String text) => text.split('\n'))
      .map<dynamic>((String line) {
        try {
          return jsonDecode(line);
        } on FormatException {
          // Only interested in test events which are JSON.
        }
      })
      .expand<Map<String, dynamic>>((dynamic json) {
        if (json is List<dynamic>) {
          return json.cast();
        }
        return <Map<String, dynamic>>[if (json != null) json as Map<String, dynamic>];
      })
      .where((Map<String, dynamic> testEvent) => testEvent['type'] == 'print')
      .map((Map<String, dynamic> printEvent) => printEvent['message'] as String)
      .firstWhere(
        (String message) => message.startsWith(_integrationResultsPrefix),
      )).replaceAll(_integrationResultsPrefix, '');

  return jsonDecode(testResults) as Map<String, dynamic>?;
}
