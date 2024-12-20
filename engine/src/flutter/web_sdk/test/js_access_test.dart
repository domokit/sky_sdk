// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// ignore_for_file: avoid_print

// Checks that JavaScript API is accessed properly.
//
// JavaScript access needs to be audited to make sure it follows security best
// practices. To do that, all JavaScript access is consolidated into a small
// number of libraries that change infrequently. These libraries are manually
// audited on every change. All other code accesses JavaScript through these
// libraries and does not require audit.

import 'dart:io';

import 'package:test/test.dart';

// Libraries that allow making arbitrary calls to JavaScript.
const List<String> _jsAccessLibraries = <String>['dart:js_util', 'package:js'];

// Libraries that are allowed to make direct calls to JavaScript. These
// libraries must be reviewed carefully to make sure JavaScript APIs are used
// safely.
const List<String> _auditedLibraries = <String>[
  'lib/web_ui/lib/src/engine/canvaskit/canvaskit_api.dart',
  'lib/web_ui/lib/src/engine/safe_browser_api.dart',
];

Future<void> main(List<String> args) async {
  bool shouldThrow = true;
  assert(() {
    shouldThrow = false;
    return true;
  }());

  if (shouldThrow) {
    throw ArgumentError('This test must run with --enable-asserts');
  }

  test('Self-test', () {
    // A library that doesn't directly access JavaScript API should pass.
    {
      final _CheckResult result = _checkFile(File('lib/web_ui/lib/src/engine/alarm_clock.dart'), '''
// A comment
import 'dart:async';
import 'package:ui/ui.dart' as ui;
export 'foo.dart';
''');
      expect(result.passed, isTrue);
      expect(result.failed, isFalse);
      expect(result.violations, isEmpty);
    }

    // A library that doesn't directly access JavaScript API should pass.
    expect(
      _checkFile(File('lib/web_ui/lib/src/engine/alarm_clock.dart'), '''
import 'dart:async';
import 'package:ui/ui.dart' as ui;
''').passed,
      isTrue,
    );

    // A non-audited library that directly accesses JavaScript API should fail.
    for (final String jsAccessLibrary in _jsAccessLibraries) {
      final _CheckResult result = _checkFile(File('lib/web_ui/lib/src/engine/alarm_clock.dart'), '''
  import 'dart:async';
  import 'package:ui/ui.dart' as ui;
  import '$jsAccessLibrary';
  ''');
      expect(result.passed, isFalse);
      expect(result.failed, isTrue);
      expect(result.violations, <String>['on line 3: library accesses $jsAccessLibrary directly']);
    }

    // Audited libraries that directly accesses JavaScript API should pass.
    for (final String auditedLibrary in _auditedLibraries) {
      for (final String jsAccessLibrary in _jsAccessLibraries) {
        expect(
          _checkFile(File(auditedLibrary), '''
    import 'dart:async';
    import 'package:ui/ui.dart' as ui;
    import '$jsAccessLibrary';
    ''').passed,
          isTrue,
        );
      }
    }
  });

  test('Check JavaScript access', () async {
    final Directory webUiLibDir = Directory('lib/web_ui/lib');
    final List<File> dartFiles =
        webUiLibDir
            .listSync(recursive: true)
            .whereType<File>()
            .where((File file) => file.path.endsWith('.dart'))
            .toList();

    expect(dartFiles, isNotEmpty);

    final List<_CheckResult> results = <_CheckResult>[];
    for (final File dartFile in dartFiles) {
      results.add(_checkFile(dartFile, await dartFile.readAsString()));
    }

    if (results.any((_CheckResult result) => result.failed)) {
      // Sort to show failures last.
      results.sort((_CheckResult a, _CheckResult b) {
        final int aSortKey = a.passed ? 1 : 0;
        final int bSortKey = b.passed ? 1 : 0;
        return bSortKey - aSortKey;
      });
      int passedCount = 0;
      int failedCount = 0;
      for (final _CheckResult result in results) {
        if (result.passed) {
          passedCount += 1;
          print('PASSED: ${result.file.path}');
        } else {
          failedCount += 1;
          print('FAILED: ${result.file.path}');
          for (final String violation in result.violations) {
            print('        $violation');
          }
        }
      }
      expect(passedCount + failedCount, dartFiles.length);
      print('$passedCount files passed. $failedCount files contain violations.');
      fail('Some file contain violations. See log messages above for details.');
    }
  });
}

_CheckResult _checkFile(File dartFile, String code) {
  final List<String> violations = <String>[];
  final List<String> lines = code.split('\n');
  for (int i = 0; i < lines.length; i += 1) {
    final int lineNumber = i + 1;
    final String line = lines[i].trim();
    final bool isImport = line.startsWith('import');
    if (!isImport) {
      continue;
    }

    if (line.contains('"')) {
      violations.add(
        'on line $lineNumber: import is using double quotes instead of single quotes: $line',
      );
      continue;
    }

    final bool isAuditedLibrary = _auditedLibraries.contains(dartFile.path);

    if (isAuditedLibrary) {
      // This library is allowed to access JavaScript API directly.
      continue;
    }

    for (final String jsAccessLibrary in _jsAccessLibraries) {
      if (line.contains("'$jsAccessLibrary'")) {
        violations.add('on line $lineNumber: library accesses $jsAccessLibrary directly');
        continue;
      }
    }
  }

  if (violations.isEmpty) {
    return _CheckResult.passed(dartFile);
  } else {
    return _CheckResult.failed(dartFile, violations);
  }
}

class _CheckResult {
  _CheckResult.passed(this.file) : violations = const <String>[];

  _CheckResult.failed(this.file, this.violations) : assert(violations.isNotEmpty);

  /// The Dart file that was checked.
  final File file;

  /// If the check failed, contains the descriptions of violations.
  ///
  /// If the check passed, this is empty.
  final List<String> violations;

  /// Whether the file passed the check.
  bool get passed => violations.isEmpty;

  /// Whether the file failed the check.
  bool get failed => !passed;
}
