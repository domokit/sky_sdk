// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
@TestOn('vm')
library;

import 'dart:io' as io;

import 'package:header_guard_check/src/header_file.dart';
import 'package:path/path.dart' as p;
import 'package:source_span/source_span.dart';
import 'package:test/test.dart';

Future<int> main() async {
  void withTestFile(String path, String contents, void Function(io.File) fn) {
    // Create a temporary file and delete it when we're done.
    final io.Directory tempDir = io.Directory.systemTemp.createTempSync('header_guard_check_test');
    final io.File file = io.File(p.join(tempDir.path, path));
    file.writeAsStringSync(contents);
    try {
      fn(file);
    } finally {
      tempDir.deleteSync(recursive: true);
    }
  }

  group('HeaderGuardSpans', () {
    test('parses #ifndef', () {
      const String input = '#ifndef FOO_H_';
      final HeaderGuardSpans guard = HeaderGuardSpans(
        ifndefSpan: SourceSpanWithContext(
          SourceLocation(0),
          SourceLocation(input.length),
          input,
          input,
        ),
        defineSpan: null,
        endifSpan: null,
      );
      expect(guard.ifndefValue, 'FOO_H_');
    });

    test('ignores #ifndef if omitted', () {
      const HeaderGuardSpans guard = HeaderGuardSpans(
        ifndefSpan: null,
        defineSpan: null,
        endifSpan: null,
      );
      expect(guard.ifndefValue, isNull);
    });

    test('ignores #ifndef if invalid', () {
      const String input = '#oops FOO_H_';
      final HeaderGuardSpans guard = HeaderGuardSpans(
        ifndefSpan: SourceSpanWithContext(
          SourceLocation(0),
          SourceLocation(input.length),
          input,
          input,
        ),
        defineSpan: null,
        endifSpan: null,
      );
      expect(guard.ifndefValue, isNull);
    });

    test('parses #define', () {
      const String input = '#define FOO_H_';
      final HeaderGuardSpans guard = HeaderGuardSpans(
        ifndefSpan: null,
        defineSpan: SourceSpanWithContext(
          SourceLocation(0),
          SourceLocation(input.length),
          input,
          input,
        ),
        endifSpan: null,
      );
      expect(guard.defineValue, 'FOO_H_');
    });

    test('ignores #define if omitted', () {
      const HeaderGuardSpans guard = HeaderGuardSpans(
        ifndefSpan: null,
        defineSpan: null,
        endifSpan: null,
      );
      expect(guard.defineValue, isNull);
    });

    test('ignores #define if invalid', () {
      const String input = '#oops FOO_H_';
      final HeaderGuardSpans guard = HeaderGuardSpans(
        ifndefSpan: null,
        defineSpan: SourceSpanWithContext(
          SourceLocation(0),
          SourceLocation(input.length),
          input,
          input,
        ),
        endifSpan: null,
      );
      expect(guard.defineValue, isNull);
    });

    test('parses #endif', () {
      const String input = '#endif  // FOO_H_';
      final HeaderGuardSpans guard = HeaderGuardSpans(
        ifndefSpan: null,
        defineSpan: null,
        endifSpan: SourceSpanWithContext(
          SourceLocation(0),
          SourceLocation(input.length),
          input,
          input,
        ),
      );
      expect(guard.endifValue, 'FOO_H_');
    });

    test('ignores #endif if omitted', () {
      const HeaderGuardSpans guard = HeaderGuardSpans(
        ifndefSpan: null,
        defineSpan: null,
        endifSpan: null,
      );
      expect(guard.endifValue, isNull);
    });

    test('ignores #endif if invalid', () {
      const String input = '#oops  // FOO_H_';
      final HeaderGuardSpans guard = HeaderGuardSpans(
        ifndefSpan: null,
        defineSpan: null,
        endifSpan: SourceSpanWithContext(
          SourceLocation(0),
          SourceLocation(input.length),
          input,
          input,
        ),
      );
      expect(guard.endifValue, isNull);
    });
  });

  group('HeaderFile', () {
    test('produces a valid header guard name from various file names', () {
      // All of these should produce the name `FOO_BAR_BAZ_H_`.
      const List<String> inputs = <String>[
        'foo_bar_baz.h',
        'foo-bar-baz.h',
        'foo_bar-baz.h',
        'foo-bar_baz.h',
        'foo+bar+baz.h',
      ];
      for (final String input in inputs) {
        final HeaderFile headerFile = HeaderFile.from(input, guard: null, pragmaOnce: null);
        expect(headerFile.computeExpectedName(engineRoot: ''), endsWith('FOO_BAR_BAZ_H_'));
      }
    });

    test('parses a header file with a valid guard', () {
      final String input = <String>[
        '#ifndef FOO_H_',
        '#define FOO_H_',
        '',
        '#endif  // FOO_H_',
      ].join('\n');
      withTestFile('foo.h', input, (io.File file) {
        final HeaderFile headerFile = HeaderFile.parse(file.path);
        expect(headerFile.guard!.ifndefValue, 'FOO_H_');
        expect(headerFile.guard!.defineValue, 'FOO_H_');
        expect(headerFile.guard!.endifValue, 'FOO_H_');
      });
    });

    test('parses a header file with an invalid #endif', () {
      final String input = <String>[
        '#ifndef FOO_H_',
        '#define FOO_H_',
        '',
        // No comment after the #endif.
        '#endif',
      ].join('\n');
      withTestFile('foo.h', input, (io.File file) {
        final HeaderFile headerFile = HeaderFile.parse(file.path);
        expect(headerFile.guard!.ifndefValue, 'FOO_H_');
        expect(headerFile.guard!.defineValue, 'FOO_H_');
        expect(headerFile.guard!.endifValue, isNull);
      });
    });

    test('parses a header file with a missing #define', () {
      final String input = <String>[
        '#ifndef FOO_H_',
        // No #define.
        '',
        '#endif  // FOO_H_',
      ].join('\n');
      withTestFile('foo.h', input, (io.File file) {
        final HeaderFile headerFile = HeaderFile.parse(file.path);
        expect(headerFile.guard!.ifndefValue, 'FOO_H_');
        expect(headerFile.guard!.defineValue, isNull);
        expect(headerFile.guard!.endifValue, 'FOO_H_');
      });
    });

    test('parses a header file with a missing #ifndef', () {
      final String input = <String>[
        // No #ifndef.
        '#define FOO_H_',
        '',
        '#endif  // FOO_H_',
      ].join('\n');
      withTestFile('foo.h', input, (io.File file) {
        final HeaderFile headerFile = HeaderFile.parse(file.path);
        expect(headerFile.guard, isNull);
      });
    });

    test('parses a header file with a #pragma once', () {
      final String input = <String>['#pragma once', ''].join('\n');
      withTestFile('foo.h', input, (io.File file) {
        final HeaderFile headerFile = HeaderFile.parse(file.path);
        expect(headerFile.pragmaOnce, isNotNull);
      });
    });

    test('fixes a file that uses #pragma once', () {
      final String input = <String>['#pragma once', '', '// ...'].join('\n');
      withTestFile('foo.h', input, (io.File file) {
        final HeaderFile headerFile = HeaderFile.parse(file.path);
        expect(headerFile.fix(engineRoot: p.dirname(file.path)), isTrue);
        expect(
          file.readAsStringSync(),
          <String>[
            '#ifndef FLUTTER_FOO_H_',
            '#define FLUTTER_FOO_H_',
            '',
            '// ...',
            '#endif  // FLUTTER_FOO_H_',
            '',
          ].join('\n'),
        );
      });
    });

    test('fixes a file with an incorrect header guard', () {
      final String input = <String>[
        '#ifndef FOO_H_',
        '#define FOO_H_',
        '',
        '#endif  // FOO_H_',
      ].join('\n');
      withTestFile('foo.h', input, (io.File file) {
        final HeaderFile headerFile = HeaderFile.parse(file.path);
        expect(headerFile.fix(engineRoot: p.dirname(file.path)), isTrue);
        expect(
          file.readAsStringSync(),
          <String>[
            '#ifndef FLUTTER_FOO_H_',
            '#define FLUTTER_FOO_H_',
            '',
            '#endif  // FLUTTER_FOO_H_',
            '',
          ].join('\n'),
        );
      });
    });

    test('fixes a file with no header guard', () {
      final String input = <String>[
        '// 1.',
        '// 2.',
        '// 3.',
        '',
        "#import 'flutter/shell/platform/darwin/Flutter.h'",
        '',
        '@protocl Flutter',
        '',
        '@end',
        '',
      ].join('\n');
      withTestFile('foo.h', input, (io.File file) {
        final HeaderFile headerFile = HeaderFile.parse(file.path);
        expect(headerFile.fix(engineRoot: p.dirname(file.path)), isTrue);
        expect(
          file.readAsStringSync(),
          <String>[
            '// 1.',
            '// 2.',
            '// 3.',
            '',
            '#ifndef FLUTTER_FOO_H_',
            '#define FLUTTER_FOO_H_',
            '',
            "#import 'flutter/shell/platform/darwin/Flutter.h'",
            '',
            '@protocl Flutter',
            '',
            '@end',
            '',
            '#endif  // FLUTTER_FOO_H_',
            '',
          ].join('\n'),
        );
      });
    });

    test('does not touch a file with an existing guard and another #define', () {
      final String input = <String>[
        '// 1.',
        '// 2.',
        '// 3.',
        '',
        '#define FML_USED_ON_EMBEDDER',
        '',
        '#ifndef FLUTTER_FOO_H_',
        '#define FLUTTER_FOO_H_',
        '',
        '#endif  // FLUTTER_FOO_H_',
        '',
      ].join('\n');
      withTestFile('foo.h', input, (io.File file) {
        final HeaderFile headerFile = HeaderFile.parse(file.path);
        expect(headerFile.fix(engineRoot: p.dirname(file.path)), isFalse);
      });
    });

    test('is OK with windows-style CRLF file with a valid header guard', () {
      final String input = <String>[
        '#ifndef FLUTTER_FOO_H_',
        '#define FLUTTER_FOO_H_',
        '',
        '// ...',
        '',
        '#endif  // FLUTTER_FOO_H_',
      ].join('\r\n');
      withTestFile('foo.h', input, (io.File file) {
        final HeaderFile headerFile = HeaderFile.parse(file.path);
        expect(headerFile.pragmaOnce, isNull);
        expect(headerFile.guard!.ifndefValue, 'FLUTTER_FOO_H_');
        expect(headerFile.guard!.defineValue, 'FLUTTER_FOO_H_');
        expect(headerFile.guard!.endifValue, 'FLUTTER_FOO_H_');
      });
    });
  });

  return 0;
}
