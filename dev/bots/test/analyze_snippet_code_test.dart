// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// we ignore these so that the format of the strings below matches what package:test prints, to make maintenance easier
// ignore_for_file: use_raw_strings

import 'dart:io';

import 'common.dart';

const List<String> expectedMainErrors = <String>[
  'dev/bots/test/analyze-snippet-code-test-input/custom_imports_broken.dart:19:11: (statement) (undefined_identifier)',
  'dev/bots/test/analyze-snippet-code-test-input/known_broken_documentation.dart:30:5: (expression) (unnecessary_new)',
  'dev/bots/test/analyze-snippet-code-test-input/known_broken_documentation.dart:103:5: (statement) (always_specify_types)',
  'dev/bots/test/analyze-snippet-code-test-input/known_broken_documentation.dart:111:5: (top-level declaration) (prefer_const_declarations)',
  'dev/bots/test/analyze-snippet-code-test-input/known_broken_documentation.dart:111:19: (top-level declaration) (unnecessary_nullable_for_final_variable_declarations)',
  'dev/bots/test/analyze-snippet-code-test-input/known_broken_documentation.dart:112:5: (top-level declaration) (prefer_const_declarations)',
  'dev/bots/test/analyze-snippet-code-test-input/known_broken_documentation.dart:112:21: (top-level declaration) (invalid_assignment)',
  'dev/bots/test/analyze-snippet-code-test-input/known_broken_documentation.dart:134:14: (top-level declaration) (undefined_identifier)',
  'dev/bots/test/analyze-snippet-code-test-input/known_broken_documentation.dart:136:21: (top-level declaration) (read_potentially_unassigned_final)',
  'dev/bots/test/analyze-snippet-code-test-input/known_broken_documentation.dart:147:12: (self-contained program) (unused_import)',
  'dev/bots/test/analyze-snippet-code-test-input/known_broken_documentation.dart:148:11: (self-contained program) (undefined_class)',
  'dev/bots/test/analyze-snippet-code-test-input/known_broken_documentation.dart:148:22: (self-contained program) (undefined_function)',
  'dev/bots/test/analyze-snippet-code-test-input/known_broken_documentation.dart:153:10: (stateful widget) (annotate_overrides)',
  'dev/bots/test/analyze-snippet-code-test-input/known_broken_documentation.dart:153:10: (stateful widget) (must_call_super)',
  'dev/bots/test/analyze-snippet-code-test-input/known_broken_documentation.dart:161:7: (top-level declaration) (undefined_identifier)',
  'dev/bots/test/analyze-snippet-code-test-input/known_broken_documentation.dart:165: Found "```" in code but it did not match RegExp: pattern=^ */// *```dart\$ flags= so something is wrong. Line was: "/// ```none"',
  'dev/bots/test/analyze-snippet-code-test-input/short_but_still_broken.dart:9:12: (statement) (invalid_assignment)',
  'dev/bots/test/analyze-snippet-code-test-input/short_but_still_broken.dart:18:4: Empty ```dart block in snippet code.',
];

const List<String> expectedUiErrors = <String>[
  'dev/bots/test/analyze-snippet-code-test-dart-ui/ui.dart:14:7: (top-level declaration) (prefer_typing_uninitialized_variables)',
  'dev/bots/test/analyze-snippet-code-test-dart-ui/ui.dart:14:7: (top-level declaration) (inference_failure_on_uninitialized_variable)',
  'dev/bots/test/analyze-snippet-code-test-dart-ui/ui.dart:14:7: (top-level declaration) (missing_const_final_var_or_type)',
  'dev/bots/test/analyze-snippet-code-test-dart-ui/ui.dart:16:20: (top-level declaration) (prefer_final_fields)',
  'dev/bots/test/analyze-snippet-code-test-dart-ui/ui.dart:16:20: (top-level declaration) (unused_field)',
];

final RegExp errorPrefixRE = RegExp(r'^([-a-z0-9/_.:]+): .*(\([-a-z_ ]+\) \([-a-z_ ]+\))$');
String removeLintDescriptions(String error) {
  final RegExpMatch? match = errorPrefixRE.firstMatch(error);
  if (match != null) {
    return '${match[1]}: ${match[2]}';
  }
  return error;
}

void main() {
  // These tests don't run on Windows because the sample analyzer doesn't
  // support Windows as a platform, since it is only run on Linux in the
  // continuous integration tests.
  if (Platform.isWindows) {
    return;
  }

  test(
    'analyze_snippet_code smoke test',
    () {
      final ProcessResult process = Process.runSync('../../bin/cache/dart-sdk/bin/dart', <String>[
        '--enable-asserts',
        'analyze_snippet_code.dart',
        '--no-include-dart-ui',
        'test/analyze-snippet-code-test-input',
      ]);
      expect(process.stdout, isEmpty);
      final List<String> stderrLines = process.stderr.toString().split('\n');
      expect(
        stderrLines.length,
        stderrLines.toSet().length,
        reason: 'found duplicates in $stderrLines',
      );
      final List<String> stderrNoDescriptions = stderrLines.map(removeLintDescriptions).toList();
      expect(stderrNoDescriptions, <String>[
        ...expectedMainErrors,
        'Found 18 snippet code errors.',
        'See the documentation at the top of dev/bots/analyze_snippet_code.dart for details.',
        '', // because we end with a newline, split gives us an extra blank line
      ]);
      expect(process.exitCode, 1);
    },
    // TODO(scheglov): Restore after landing Dart SDK changes, https://github.com/flutter/flutter/issues/154413
    skip: true,
  );

  test(
    'Analyzes dart:ui code',
    () {
      final ProcessResult process = Process.runSync('../../bin/cache/dart-sdk/bin/dart', <String>[
        '--enable-asserts',
        'analyze_snippet_code.dart',
        '--dart-ui-location=test/analyze-snippet-code-test-dart-ui',
        'test/analyze-snippet-code-test-input',
      ]);
      expect(process.stdout, isEmpty);
      final List<String> stderrLines = process.stderr.toString().split('\n');
      expect(
        stderrLines.length,
        stderrLines.toSet().length,
        reason: 'found duplicates in $stderrLines',
      );
      final List<String> stderrNoDescriptions = stderrLines.map(removeLintDescriptions).toList();
      expect(stderrNoDescriptions, <String>[
        ...expectedUiErrors,
        ...expectedMainErrors,
        'Found 23 snippet code errors.',
        'See the documentation at the top of dev/bots/analyze_snippet_code.dart for details.',
        '', // because we end with a newline, split gives us an extra blank line
      ]);
      expect(process.exitCode, 1);
    },
    // TODO(scheglov): Restore after landing Dart SDK changes, https://github.com/flutter/flutter/issues/154413
    skip: true,
  );
}
