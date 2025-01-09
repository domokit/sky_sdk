// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/terminal.dart';
import 'package:flutter_tools/src/base/utils.dart';
import 'package:flutter_tools/src/base/version.dart';

import '../src/common.dart';

void main() {
  group('SettingsFile', () {
    testWithoutContext('parse', () {
      final SettingsFile file = SettingsFile.parse('''
# ignore comment
foo=bar
baz=qux
''');
      expect(file.values['foo'], 'bar');
      expect(file.values['baz'], 'qux');
      expect(file.values, hasLength(2));
    });
  });

  group('Version', () {
    testWithoutContext('can parse and compare', () {
      expect(Version(0, null, null).toString(), equals('0'));
      expect(const Version.withText(1, 2, 3, 'versionText').toString(), 'versionText');

      final Version v1 = Version.parse('1')!;
      expect(v1.major, equals(1));
      expect(v1.minor, equals(0));
      expect(v1.patch, equals(0));

      expect(v1, greaterThan(Version(0, 0, 0)));

      final Version v2 = Version.parse('1.2')!;
      expect(v2.major, equals(1));
      expect(v2.minor, equals(2));
      expect(v2.patch, equals(0));

      final Version v3 = Version.parse('1.2.3')!;
      expect(v3.major, equals(1));
      expect(v3.minor, equals(2));
      expect(v3.patch, equals(3));

      final Version v4 = Version.parse('1.12')!;
      expect(v4, greaterThan(v2));

      expect(v3, greaterThan(v2));
      expect(v2, greaterThan(v1));

      final Version v5 = Version(1, 2, 0, text: 'foo');
      expect(v5, equals(v2));

      expect(Version.parse('Preview2.2'), isNull);
    });

    group('isWithinVersionRange', () {
      test('unknown not included', () {
        expect(isWithinVersionRange('unknown', min: '1.0.0', max: '1.1.3'), isFalse);
      });

      test('pre java 8 format included', () {
        expect(isWithinVersionRange('1.0.0_201', min: '1.0.0', max: '1.1.3'), isTrue);
      });

      test('min included by default', () {
        expect(isWithinVersionRange('1.0.0', min: '1.0.0', max: '1.1.3'), isTrue);
      });

      test('max included by default', () {
        expect(isWithinVersionRange('1.1.3', min: '1.0.0', max: '1.1.3'), isTrue);
      });

      test('inclusive min excluded', () {
        expect(
          isWithinVersionRange('1.0.0', min: '1.0.0', max: '1.1.3', inclusiveMin: false),
          isFalse,
        );
      });

      test('inclusive max excluded', () {
        expect(
          isWithinVersionRange('1.1.3', min: '1.0.0', max: '1.1.3', inclusiveMax: false),
          isFalse,
        );
      });

      test('lower value excluded', () {
        expect(isWithinVersionRange('0.1.0', min: '1.0.0', max: '1.1.3'), isFalse);
      });

      test('higher value excluded', () {
        expect(isWithinVersionRange('1.1.4', min: '1.0.0', max: '1.1.3'), isFalse);
      });

      test('middle value included', () {
        expect(isWithinVersionRange('1.1.0', min: '1.0.0', max: '1.1.3'), isTrue);
      });
    });
  });

  group('Misc', () {
    testWithoutContext('snakeCase', () async {
      expect(snakeCase('abc'), equals('abc'));
      expect(snakeCase('abC'), equals('ab_c'));
      expect(snakeCase('aBc'), equals('a_bc'));
      expect(snakeCase('aBC'), equals('a_b_c'));
      expect(snakeCase('Abc'), equals('abc'));
      expect(snakeCase('AbC'), equals('ab_c'));
      expect(snakeCase('ABc'), equals('a_bc'));
      expect(snakeCase('ABC'), equals('a_b_c'));
    });

    testWithoutContext('kebabCase', () async {
      expect(kebabCase('abc'), equals('abc'));
      expect(kebabCase('abC'), equals('ab-c'));
      expect(kebabCase('aBc'), equals('a-bc'));
      expect(kebabCase('aBC'), equals('a-b-c'));
      expect(kebabCase('Abc'), equals('abc'));
      expect(kebabCase('AbC'), equals('ab-c'));
      expect(kebabCase('ABc'), equals('a-bc'));
      expect(kebabCase('ABC'), equals('a-b-c'));
    });

    testWithoutContext('sentenceCase', () async {
      expect(sentenceCase('abc'), equals('Abc'));
      expect(sentenceCase('ab_c'), equals('Ab_c'));
      expect(sentenceCase('a b c'), equals('A b c'));
      expect(sentenceCase('a B c'), equals('A B c'));
      expect(sentenceCase('Abc'), equals('Abc'));
      expect(sentenceCase('ab_c'), equals('Ab_c'));
      expect(sentenceCase('a_bc'), equals('A_bc'));
      expect(sentenceCase('a_b_c'), equals('A_b_c'));
    });

    testWithoutContext('snakeCaseToTitleCase', () async {
      expect(snakeCaseToTitleCase('abc'), equals('Abc'));
      expect(snakeCaseToTitleCase('ab_c'), equals('Ab C'));
      expect(snakeCaseToTitleCase('a_b_c'), equals('A B C'));
      expect(snakeCaseToTitleCase('a_B_c'), equals('A B C'));
      expect(snakeCaseToTitleCase('Abc'), equals('Abc'));
      expect(snakeCaseToTitleCase('ab_c'), equals('Ab C'));
      expect(snakeCaseToTitleCase('a_bc'), equals('A Bc'));
      expect(snakeCaseToTitleCase('a_b_c'), equals('A B C'));
    });
  });

  group('text wrapping', () {
    const int lineLength = 40;
    const String longLine = 'This is a long line that needs to be wrapped.';
    final String longLineWithNewlines =
        'This is a long line with newlines that\n'
        'needs to be wrapped.\n\n'
        '${'0123456789' * 5}';
    final String longAnsiLineWithNewlines =
        '${AnsiTerminal.red}This${AnsiTerminal.resetAll} is a long line with newlines that\n'
        'needs to be wrapped.\n\n'
        '${AnsiTerminal.green}0123456789${AnsiTerminal.resetAll}'
        '${'0123456789' * 3}'
        '${AnsiTerminal.green}0123456789${AnsiTerminal.resetAll}';
    const String onlyAnsiSequences = '${AnsiTerminal.red}${AnsiTerminal.resetAll}';
    final String indentedLongLineWithNewlines =
        '    This is an indented long line with newlines that\n'
        'needs to be wrapped.\n\tAnd preserves tabs.\n      \n  '
        '${'0123456789' * 5}';
    const String shortLine = 'Short line.';
    const String indentedLongLine =
        '    This is an indented long line that needs to be '
        'wrapped and indentation preserved.';
    testWithoutContext('does not wrap by default in tests', () {
      expect(wrapText(longLine, columnWidth: 80, shouldWrap: true), equals(longLine));
    });

    testWithoutContext('can override wrap preference if preference is off', () {
      expect(
        wrapText(longLine, columnWidth: lineLength, shouldWrap: true),
        equals('''
This is a long line that needs to be
wrapped.'''),
      );
    });

    testWithoutContext('can override wrap preference if preference is on', () {
      expect(wrapText(longLine, shouldWrap: false, columnWidth: 80), equals(longLine));
    });

    testWithoutContext('does not wrap at all if not told to wrap', () {
      expect(wrapText(longLine, columnWidth: 80, shouldWrap: false), equals(longLine));
    });

    testWithoutContext('does not wrap short lines.', () {
      expect(wrapText(shortLine, columnWidth: lineLength, shouldWrap: true), equals(shortLine));
    });

    testWithoutContext('able to wrap long lines', () {
      expect(
        wrapText(longLine, columnWidth: lineLength, shouldWrap: true),
        equals('''
This is a long line that needs to be
wrapped.'''),
      );
    });

    testWithoutContext('able to handle dynamically changing terminal column size', () {
      expect(
        wrapText(longLine, columnWidth: 20, shouldWrap: true),
        equals('''
This is a long line
that needs to be
wrapped.'''),
      );

      expect(
        wrapText(longLine, columnWidth: lineLength, shouldWrap: true),
        equals('''
This is a long line that needs to be
wrapped.'''),
      );
    });

    testWithoutContext('wrap long lines with no whitespace', () {
      expect(
        wrapText('0123456789' * 5, columnWidth: lineLength, shouldWrap: true),
        equals('''
0123456789012345678901234567890123456789
0123456789'''),
      );
    });

    testWithoutContext('refuses to wrap to a column smaller than 10 characters', () {
      expect(
        wrapText('$longLine ${'0123456789' * 4}', columnWidth: 1, shouldWrap: true),
        equals('''
This is a
long line
that needs
to be
wrapped.
0123456789
0123456789
0123456789
0123456789'''),
      );
    });
    testWithoutContext('preserves indentation', () {
      expect(
        wrapText(indentedLongLine, columnWidth: lineLength, shouldWrap: true),
        equals('''
    This is an indented long line that
    needs to be wrapped and indentation
    preserved.'''),
      );
    });

    testWithoutContext('preserves indentation and stripping trailing whitespace', () {
      expect(
        wrapText('$indentedLongLine   ', columnWidth: lineLength, shouldWrap: true),
        equals('''
    This is an indented long line that
    needs to be wrapped and indentation
    preserved.'''),
      );
    });

    testWithoutContext('wraps text with newlines', () {
      expect(
        wrapText(longLineWithNewlines, columnWidth: lineLength, shouldWrap: true),
        equals('''
This is a long line with newlines that
needs to be wrapped.

0123456789012345678901234567890123456789
0123456789'''),
      );
    });

    testWithoutContext('wraps text with ANSI sequences embedded', () {
      expect(
        wrapText(longAnsiLineWithNewlines, columnWidth: lineLength, shouldWrap: true),
        equals('''
${AnsiTerminal.red}This${AnsiTerminal.resetAll} is a long line with newlines that
needs to be wrapped.

${AnsiTerminal.green}0123456789${AnsiTerminal.resetAll}012345678901234567890123456789
${AnsiTerminal.green}0123456789${AnsiTerminal.resetAll}'''),
      );
    });

    testWithoutContext('wraps text with only ANSI sequences', () {
      expect(
        wrapText(onlyAnsiSequences, columnWidth: lineLength, shouldWrap: true),
        equals('${AnsiTerminal.red}${AnsiTerminal.resetAll}'),
      );
    });

    testWithoutContext('preserves indentation in the presence of newlines', () {
      expect(
        wrapText(indentedLongLineWithNewlines, columnWidth: lineLength, shouldWrap: true),
        equals('''
    This is an indented long line with
    newlines that
needs to be wrapped.
\tAnd preserves tabs.

  01234567890123456789012345678901234567
  890123456789'''),
      );
    });

    testWithoutContext('removes trailing whitespace when wrapping', () {
      expect(
        wrapText('$longLine     \t', columnWidth: lineLength, shouldWrap: true),
        equals('''
This is a long line that needs to be
wrapped.'''),
      );
    });

    testWithoutContext('honors hangingIndent parameter', () {
      expect(
        wrapText(longLine, columnWidth: lineLength, hangingIndent: 6, shouldWrap: true),
        equals('''
This is a long line that needs to be
      wrapped.'''),
      );
    });

    testWithoutContext('handles hangingIndent with a single unwrapped line.', () {
      expect(
        wrapText(shortLine, columnWidth: lineLength, hangingIndent: 6, shouldWrap: true),
        equals('''
Short line.'''),
      );
    });

    testWithoutContext(
      'handles hangingIndent with two unwrapped lines and the second is empty.',
      () {
        expect(
          wrapText('$shortLine\n', columnWidth: lineLength, hangingIndent: 6, shouldWrap: true),
          equals('''
Short line.
'''),
        );
      },
    );

    testWithoutContext('honors hangingIndent parameter on already indented line.', () {
      expect(
        wrapText(indentedLongLine, columnWidth: lineLength, hangingIndent: 6, shouldWrap: true),
        equals('''
    This is an indented long line that
          needs to be wrapped and
          indentation preserved.'''),
      );
    });

    testWithoutContext('honors hangingIndent and indent parameters at the same time.', () {
      expect(
        wrapText(
          indentedLongLine,
          columnWidth: lineLength,
          indent: 6,
          hangingIndent: 6,
          shouldWrap: true,
        ),
        equals('''
          This is an indented long line
                that needs to be wrapped
                and indentation
                preserved.'''),
      );
    });

    testWithoutContext('honors indent parameter on already indented line.', () {
      expect(
        wrapText(indentedLongLine, columnWidth: lineLength, indent: 6, shouldWrap: true),
        equals('''
          This is an indented long line
          that needs to be wrapped and
          indentation preserved.'''),
      );
    });

    testWithoutContext('honors hangingIndent parameter on already indented line.', () {
      expect(
        wrapText(
          indentedLongLineWithNewlines,
          columnWidth: lineLength,
          hangingIndent: 6,
          shouldWrap: true,
        ),
        equals('''
    This is an indented long line with
          newlines that
needs to be wrapped.
	And preserves tabs.

  01234567890123456789012345678901234567
        890123456789'''),
      );
    });

    testWithoutContext('', () {
      expect(
        wrapText(
          '${' ' * 7}abc def ghi',
          columnWidth: 20,
          hangingIndent: 5,
          indent: 3,
          shouldWrap: true,
        ),
        equals(
          '          abc def\n'
          '          ghi',
        ),
      );
      expect(
        wrapText('abc def ghi', columnWidth: 0, hangingIndent: 5, shouldWrap: true),
        equals(
          'abc def\n'
          'ghi',
        ),
      );
      expect(
        wrapText('abc def ghi', columnWidth: 0, indent: 5, shouldWrap: true),
        equals(
          'abc def\n'
          'ghi',
        ),
      );
      expect(
        wrapText('     abc def ghi', columnWidth: 0, shouldWrap: true),
        equals(
          'abc def\n'
          'ghi',
        ),
      );
      expect(
        wrapText(
          'abc def ghi',
          columnWidth: kMinColumnWidth - 2,
          hangingIndent: 5,
          shouldWrap: true,
        ),
        equals(
          'abc def\n'
          'ghi',
        ),
      );
      expect(
        wrapText('abc def ghi', columnWidth: kMinColumnWidth - 2, indent: 5, shouldWrap: true),
        equals(
          'abc def\n'
          'ghi',
        ),
      );
      expect(
        wrapText('     abc def ghi', columnWidth: kMinColumnWidth - 2, shouldWrap: true),
        equals(
          'abc def\n'
          'ghi',
        ),
      );
      expect(
        wrapText(
          'abc def ghi jkl',
          columnWidth: kMinColumnWidth + 2,
          hangingIndent: 5,
          shouldWrap: true,
        ),
        equals(
          'abc def ghi\n'
          '  jkl',
        ),
      );
      expect(
        wrapText('abc def ghi', columnWidth: kMinColumnWidth + 2, indent: 5, shouldWrap: true),
        equals(
          '  abc def\n'
          '  ghi',
        ),
      );
      expect(
        wrapText('     abc def ghi', columnWidth: kMinColumnWidth + 2, shouldWrap: true),
        equals(
          '  abc def\n'
          '  ghi',
        ),
      );
    });
  });

  testWithoutContext('getSizeAsMB', () async {
    expect(
      // ignore: avoid_redundant_argument_values
      getSizeAsPlatformMB(10 * 1000 * 1000, platform: FakePlatform(operatingSystem: 'linux')),
      '10.0MB',
    );
    expect(
      getSizeAsPlatformMB(10 * 1000 * 1000, platform: FakePlatform(operatingSystem: 'macos')),
      '10.0MB',
    );
    expect(
      getSizeAsPlatformMB(10 * 1000 * 1000, platform: FakePlatform(operatingSystem: 'windows')),
      '9.5MB',
    );
    expect(
      getSizeAsPlatformMB(10 * 1000 * 1000, platform: FakePlatform(operatingSystem: 'android')),
      '10.0MB',
    );
    expect(
      getSizeAsPlatformMB(10 * 1000 * 1000, platform: FakePlatform(operatingSystem: 'ios')),
      '10.0MB',
    );
    expect(
      getSizeAsPlatformMB(10 * 1000 * 1000, platform: FakePlatform(operatingSystem: 'web')),
      '10.0MB',
    );
  });
}
