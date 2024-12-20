// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// flutter_ignore_for_file: golden_tag (see analyze.dart)

import 'dart:math' as math;
import 'dart:typed_data';

import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:vector_math/vector_math_64.dart' show Matrix3;

/// Class that makes it easy to mock common toStringDeep behavior.
class _MockToStringDeep {
  _MockToStringDeep(String str) : _lines = <String>[] {
    final List<String> lines = str.split('\n');
    for (int i = 0; i < lines.length - 1; ++i) {
      _lines.add('${lines[i]}\n');
    }

    // If the last line is empty, that really just means that the previous
    // line was terminated with a line break.
    if (lines.isNotEmpty && lines.last.isNotEmpty) {
      _lines.add(lines.last);
    }
  }

  _MockToStringDeep.fromLines(this._lines);

  /// Lines in the message to display when [toStringDeep] is called.
  /// For correct toStringDeep behavior, each line should be terminated with a
  /// line break.
  final List<String> _lines;

  String toStringDeep({String prefixLineOne = '', String prefixOtherLines = ''}) {
    final StringBuffer sb = StringBuffer();
    if (_lines.isNotEmpty) {
      sb.write('$prefixLineOne${_lines.first}');
    }

    for (int i = 1; i < _lines.length; ++i) {
      sb.write('$prefixOtherLines${_lines[i]}');
    }

    return sb.toString();
  }

  @override
  String toString() => toStringDeep();
}

void main() {
  test('hasOneLineDescription', () {
    expect('Hello', hasOneLineDescription);
    expect('Hello\nHello', isNot(hasOneLineDescription));
    expect(' Hello', isNot(hasOneLineDescription));
    expect('Hello ', isNot(hasOneLineDescription));
    expect(Object(), isNot(hasOneLineDescription));
  });

  test('hasAGoodToStringDeep', () {
    expect(_MockToStringDeep('Hello\n World\n'), hasAGoodToStringDeep);
    // Not terminated with a line break.
    expect(_MockToStringDeep('Hello\n World'), isNot(hasAGoodToStringDeep));
    // Trailing whitespace on last line.
    expect(_MockToStringDeep('Hello\n World \n'), isNot(hasAGoodToStringDeep));
    expect(_MockToStringDeep('Hello\n World\t\n'), isNot(hasAGoodToStringDeep));
    // Leading whitespace on line 1.
    expect(_MockToStringDeep(' Hello\n World \n'), isNot(hasAGoodToStringDeep));

    // Single line.
    expect(_MockToStringDeep('Hello World'), isNot(hasAGoodToStringDeep));
    expect(_MockToStringDeep('Hello World\n'), isNot(hasAGoodToStringDeep));

    expect(_MockToStringDeep('Hello: World\nFoo: bar\n'), hasAGoodToStringDeep);
    expect(_MockToStringDeep('Hello: World\nFoo: 42\n'), hasAGoodToStringDeep);
    // Contains default Object.toString().
    expect(_MockToStringDeep('Hello: World\nFoo: ${Object()}\n'), isNot(hasAGoodToStringDeep));
    expect(_MockToStringDeep('A\n├─B\n'), hasAGoodToStringDeep);
    expect(_MockToStringDeep('A\n├─B\n╘══════\n'), hasAGoodToStringDeep);
    // Last line is all whitespace or vertical line art.
    expect(_MockToStringDeep('A\n├─B\n\n'), isNot(hasAGoodToStringDeep));
    expect(_MockToStringDeep('A\n├─B\n│\n'), isNot(hasAGoodToStringDeep));
    expect(_MockToStringDeep('A\n├─B\n│\n'), isNot(hasAGoodToStringDeep));
    expect(_MockToStringDeep('A\n├─B\n│\n'), isNot(hasAGoodToStringDeep));
    expect(_MockToStringDeep('A\n├─B\n╎\n'), isNot(hasAGoodToStringDeep));
    expect(_MockToStringDeep('A\n├─B\n║\n'), isNot(hasAGoodToStringDeep));
    expect(_MockToStringDeep('A\n├─B\n │\n'), isNot(hasAGoodToStringDeep));
    expect(_MockToStringDeep('A\n├─B\n ╎\n'), isNot(hasAGoodToStringDeep));
    expect(_MockToStringDeep('A\n├─B\n ║\n'), isNot(hasAGoodToStringDeep));
    expect(_MockToStringDeep('A\n├─B\n ││\n'), isNot(hasAGoodToStringDeep));

    expect(
      _MockToStringDeep(
        'A\n'
        '├─B\n'
        '│\n'
        '└─C\n',
      ),
      hasAGoodToStringDeep,
    );
    // Last line is all whitespace or vertical line art.
    expect(
      _MockToStringDeep(
        'A\n'
        '├─B\n'
        '│\n',
      ),
      isNot(hasAGoodToStringDeep),
    );

    expect(
      _MockToStringDeep.fromLines(<String>[
        'Paragraph#00000\n',
        ' │ size: (400x200)\n',
        ' ╘═╦══ text ═══\n',
        '   ║ TextSpan:\n',
        '   ║   "I polished up that handle so carefullee\n',
        '   ║   That now I am the Ruler of the Queen\'s Navee!"\n',
        '   ╚═══════════\n',
      ]),
      hasAGoodToStringDeep,
    );

    // Text span
    expect(
      _MockToStringDeep.fromLines(<String>[
        'Paragraph#00000\n',
        ' │ size: (400x200)\n',
        ' ╘═╦══ text ═══\n',
        '   ║ TextSpan:\n',
        '   ║   "I polished up that handle so carefullee\nThat now I am the Ruler of the Queen\'s Navee!"\n',
        '   ╚═══════════\n',
      ]),
      isNot(hasAGoodToStringDeep),
    );
  });

  test('equalsIgnoringHashCodes', () {
    expect('Foo#34219', equalsIgnoringHashCodes('Foo#00000'));
    expect('Foo#34219', equalsIgnoringHashCodes('Foo#12345'));
    expect('Foo#34219', equalsIgnoringHashCodes('Foo#abcdf'));
    expect('Foo#34219', isNot(equalsIgnoringHashCodes('Foo')));
    expect('Foo#34219', isNot(equalsIgnoringHashCodes('Foo#')));
    expect('Foo#34219', isNot(equalsIgnoringHashCodes('Foo#0')));
    expect('Foo#34219', isNot(equalsIgnoringHashCodes('Foo#00')));
    expect('Foo#34219', isNot(equalsIgnoringHashCodes('Foo#00000 ')));
    expect('Foo#34219', isNot(equalsIgnoringHashCodes('Foo#000000')));
    expect('Foo#34219', isNot(equalsIgnoringHashCodes('Foo#123456')));

    expect('Foo#34219:', equalsIgnoringHashCodes('Foo#00000:'));
    expect('Foo#34219:', isNot(equalsIgnoringHashCodes('Foo#00000')));

    expect('Foo#a3b4d', equalsIgnoringHashCodes('Foo#00000'));
    expect('Foo#a3b4d', equalsIgnoringHashCodes('Foo#12345'));
    expect('Foo#a3b4d', equalsIgnoringHashCodes('Foo#abcdf'));
    expect('Foo#a3b4d', isNot(equalsIgnoringHashCodes('Foo')));
    expect('Foo#a3b4d', isNot(equalsIgnoringHashCodes('Foo#')));
    expect('Foo#a3b4d', isNot(equalsIgnoringHashCodes('Foo#0')));
    expect('Foo#a3b4d', isNot(equalsIgnoringHashCodes('Foo#00')));
    expect('Foo#a3b4d', isNot(equalsIgnoringHashCodes('Foo#00000 ')));
    expect('Foo#a3b4d', isNot(equalsIgnoringHashCodes('Foo#000000')));
    expect('Foo#a3b4d', isNot(equalsIgnoringHashCodes('Foo#123456')));

    expect('FOO#A3b4D', equalsIgnoringHashCodes('FOO#00000'));
    expect('FOO#A3b4J', isNot(equalsIgnoringHashCodes('FOO#00000')));

    expect('Foo#12345(Bar#9110f)', equalsIgnoringHashCodes('Foo#00000(Bar#00000)'));
    expect('Foo#12345(Bar#9110f)', isNot(equalsIgnoringHashCodes('Foo#00000(Bar#)')));

    expect('Foo', isNot(equalsIgnoringHashCodes('Foo#00000')));
    expect('Foo#', isNot(equalsIgnoringHashCodes('Foo#00000')));
    expect('Foo#3421', isNot(equalsIgnoringHashCodes('Foo#00000')));
    expect('Foo#342193', isNot(equalsIgnoringHashCodes('Foo#00000')));
    expect(<String>['Foo#a3b4d'], equalsIgnoringHashCodes(<String>['Foo#12345']));
    expect(<String>[
      'Foo#a3b4d',
      'Foo#12345',
    ], equalsIgnoringHashCodes(<String>['Foo#00000', 'Foo#00000']));
    expect(<String>[
      'Foo#a3b4d',
      'Bar#12345',
    ], equalsIgnoringHashCodes(<String>['Foo#00000', 'Bar#00000']));
    expect(<String>[
      'Foo#a3b4d',
      'Bar#12345',
    ], isNot(equalsIgnoringHashCodes(<String>['Bar#00000', 'Foo#00000'])));
    expect(<String>['Foo#a3b4d'], isNot(equalsIgnoringHashCodes(<String>['Foo'])));
    expect(<String>[
      'Foo#a3b4d',
    ], isNot(equalsIgnoringHashCodes(<String>['Foo#00000', 'Bar#00000'])));
  });

  test('equalsIgnoringHashCodes - wrong line', () {
    TestFailure? failure;
    try {
      expect(
        '1\n2\n3\n4\n5\n6\n7\n8\n9\n10',
        equalsIgnoringHashCodes('1\n2\n3\n4\n5\n6\na\n8\n9\n10'),
      );
    } on TestFailure catch (e) {
      failure = e;
    }

    expect(failure, isNotNull);
    if (failure != null) {
      final String? message = failure.message;
      expect(message, contains('Lines 7 differed'));
      expect(message, contains("'a'"));
      expect(message, contains("'7'"));
    }
  });

  test('moreOrLessEquals', () {
    expect(0.0, moreOrLessEquals(1e-11));
    expect(1e-11, moreOrLessEquals(0.0));
    expect(-1e-11, moreOrLessEquals(0.0));

    expect(0.0, isNot(moreOrLessEquals(1e11)));
    expect(1e11, isNot(moreOrLessEquals(0.0)));
    expect(-1e11, isNot(moreOrLessEquals(0.0)));

    expect(0.0, isNot(moreOrLessEquals(1.0)));
    expect(1.0, isNot(moreOrLessEquals(0.0)));
    expect(-1.0, isNot(moreOrLessEquals(0.0)));

    expect(1e-11, moreOrLessEquals(-1e-11));
    expect(-1e-11, moreOrLessEquals(1e-11));

    expect(11.0, isNot(moreOrLessEquals(-11.0, epsilon: 1.0)));
    expect(-11.0, isNot(moreOrLessEquals(11.0, epsilon: 1.0)));

    expect(11.0, moreOrLessEquals(-11.0, epsilon: 100.0));
    expect(-11.0, moreOrLessEquals(11.0, epsilon: 100.0));

    expect(0, moreOrLessEquals(0.0));
    expect(0.0, moreOrLessEquals(0));
  });

  test('matrixMoreOrLessEquals', () {
    expect(
      Matrix4.rotationZ(math.pi),
      matrixMoreOrLessEquals(
        Matrix4.fromList(<double>[-1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]),
      ),
    );

    expect(
      Matrix4.rotationZ(math.pi),
      matrixMoreOrLessEquals(
        Matrix4.fromList(<double>[-2, 0, 0, 0, 0, -2, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]),
        epsilon: 2,
      ),
    );

    expect(
      Matrix4.rotationZ(math.pi),
      isNot(
        matrixMoreOrLessEquals(
          Matrix4.fromList(<double>[-2, 0, 0, 0, 0, -2, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]),
        ),
      ),
    );
  });

  test('matrix3MoreOrLessEquals', () {
    expect(
      Matrix3.rotationZ(math.pi),
      matrix3MoreOrLessEquals(Matrix3.fromList(<double>[-1, 0, 0, 0, -1, 0, 0, 0, 1])),
    );

    expect(
      Matrix3.rotationZ(math.pi),
      matrix3MoreOrLessEquals(Matrix3.fromList(<double>[-2, 0, 0, 0, -2, 0, 0, 0, 1]), epsilon: 2),
    );

    expect(
      Matrix3.rotationZ(math.pi),
      isNot(matrix3MoreOrLessEquals(Matrix3.fromList(<double>[-2, 0, 0, 0, -2, 0, 0, 0, 1]))),
    );
  });

  test('rectMoreOrLessEquals', () {
    expect(
      const Rect.fromLTRB(0.0, 0.0, 10.0, 10.0),
      rectMoreOrLessEquals(const Rect.fromLTRB(0.0, 0.0, 10.0, 10.00000000001)),
    );

    expect(
      const Rect.fromLTRB(11.0, 11.0, 20.0, 20.0),
      isNot(rectMoreOrLessEquals(const Rect.fromLTRB(-11.0, -11.0, 20.0, 20.0), epsilon: 1.0)),
    );

    expect(
      const Rect.fromLTRB(11.0, 11.0, 20.0, 20.0),
      rectMoreOrLessEquals(const Rect.fromLTRB(-11.0, -11.0, 20.0, 20.0), epsilon: 100.0),
    );
  });

  test('within', () {
    expect(0.0, within<double>(distance: 0.1, from: 0.05));
    expect(0.0, isNot(within<double>(distance: 0.1, from: 0.2)));

    expect(0, within<int>(distance: 1, from: 1));
    expect(0, isNot(within<int>(distance: 1, from: 2)));

    expect(const Color(0x00000000), within<Color>(distance: 1, from: const Color(0x01000000)));
    expect(const Color(0x00000000), within<Color>(distance: 1, from: const Color(0x00010000)));
    expect(const Color(0x00000000), within<Color>(distance: 1, from: const Color(0x00000100)));
    expect(const Color(0x00000000), within<Color>(distance: 1, from: const Color(0x00000001)));
    expect(const Color(0x00000000), within<Color>(distance: 1, from: const Color(0x01010101)));
    expect(
      const Color(0x00000000),
      isNot(within<Color>(distance: 1, from: const Color(0x02000000))),
    );

    expect(const Offset(1.0, 0.0), within(distance: 1.0, from: Offset.zero));
    expect(const Offset(1.0, 0.0), isNot(within(distance: 1.0, from: const Offset(-1.0, 0.0))));

    expect(
      const Rect.fromLTRB(0.0, 1.0, 2.0, 3.0),
      within<Rect>(distance: 4.0, from: const Rect.fromLTRB(1.0, 3.0, 5.0, 7.0)),
    );
    expect(
      const Rect.fromLTRB(0.0, 1.0, 2.0, 3.0),
      isNot(within<Rect>(distance: 3.9, from: const Rect.fromLTRB(1.0, 3.0, 5.0, 7.0))),
    );

    expect(const Size(1.0, 1.0), within<Size>(distance: 1.415, from: const Size(2.0, 2.0)));
    expect(const Size(1.0, 1.0), isNot(within<Size>(distance: 1.414, from: const Size(2.0, 2.0))));

    expect(() => within<bool>(distance: 1, from: false), throwsArgumentError);

    expect(
      () => within<int>(
        distance: 1,
        from: 2,
        distanceFunction: (int a, int b) => -1,
      ).matches(1, <dynamic, dynamic>{}),
      throwsArgumentError,
    );
  });

  test('isSameColorSwatchAs', () {
    expect(
      const ColorSwatch<String>(0xaaaaaaaa, <String, Color>{
        'foo': Color(0xaaaaaaaa),
        'bar': Color(0xbbbbbbbb),
      }),
      isSameColorSwatchAs(
        const ColorSwatch<String>(0xaaaaaaaa, <String, Color>{
          'foo': Color(0xaaaaaaaa),
          'bar': Color(0xbbbbbbbb),
        }),
      ),
    );

    expect(
      const ColorSwatch<String>(0xaaaaaaaa, <String, Color>{
        'foo': Color(0xaaaaaaaa),
        'bar': Color(0xbbbbbbbb),
      }),
      isNot(
        isSameColorSwatchAs(
          const ColorSwatch<String>(0xaaaaaaaa, <String, Color>{
            'foo': Color(0xaaaaaaaa),
            'bar': Color(0xcccccccc),
          }),
        ),
      ),
    );
  });

  test('isSameColorAs', () {
    expect(const Color(0x87654321), isSameColorAs(const _CustomColor(0x87654321)));

    expect(const _CustomColor(0x87654321), isSameColorAs(const Color(0x87654321)));

    expect(const Color(0x12345678), isNot(isSameColorAs(const _CustomColor(0x87654321))));

    expect(const _CustomColor(0x87654321), isNot(isSameColorAs(const Color(0x12345678))));

    expect(
      const _CustomColor(0xFF123456),
      isSameColorAs(const _CustomColor(0xFF123456, isEqual: false)),
    );

    expect(const Color(0x00000000), isNot(isSameColorAs(const Color(0x00000002))));

    expect(const Color(0x00000000), isSameColorAs(const Color(0x00000002), threshold: 0.008));
  });

  group('coversSameAreaAs', () {
    test('empty Paths', () {
      expect(
        Path(),
        coversSameAreaAs(Path(), areaToCompare: const Rect.fromLTRB(0.0, 0.0, 10.0, 10.0)),
      );
    });

    test('mismatch', () {
      final Path rectPath = Path()..addRect(const Rect.fromLTRB(5.0, 5.0, 6.0, 6.0));
      expect(
        Path(),
        isNot(coversSameAreaAs(rectPath, areaToCompare: const Rect.fromLTRB(0.0, 0.0, 10.0, 10.0))),
      );
    });

    test('mismatch out of examined area', () {
      final Path rectPath = Path()..addRect(const Rect.fromLTRB(5.0, 5.0, 6.0, 6.0));
      rectPath.addRect(const Rect.fromLTRB(5.0, 5.0, 6.0, 6.0));
      expect(
        Path(),
        coversSameAreaAs(rectPath, areaToCompare: const Rect.fromLTRB(0.0, 0.0, 4.0, 4.0)),
      );
    });

    test('differently constructed rects match', () {
      final Path rectPath = Path()..addRect(const Rect.fromLTRB(5.0, 5.0, 6.0, 6.0));
      final Path linePath =
          Path()
            ..moveTo(5.0, 5.0)
            ..lineTo(5.0, 6.0)
            ..lineTo(6.0, 6.0)
            ..lineTo(6.0, 5.0)
            ..close();
      expect(
        linePath,
        coversSameAreaAs(rectPath, areaToCompare: const Rect.fromLTRB(0.0, 0.0, 10.0, 10.0)),
      );
    });

    test('partially overlapping paths', () {
      final Path rectPath = Path()..addRect(const Rect.fromLTRB(5.0, 5.0, 6.0, 6.0));
      final Path linePath =
          Path()
            ..moveTo(5.0, 5.0)
            ..lineTo(5.0, 6.0)
            ..lineTo(6.0, 6.0)
            ..lineTo(6.0, 5.5)
            ..close();
      expect(
        linePath,
        isNot(coversSameAreaAs(rectPath, areaToCompare: const Rect.fromLTRB(0.0, 0.0, 10.0, 10.0))),
      );
    });
  });

  group('matchesGoldenFile', () {
    late _FakeComparator comparator;

    Widget boilerplate(Widget child) {
      return Directionality(textDirection: TextDirection.ltr, child: child);
    }

    setUp(() {
      comparator = _FakeComparator();
      goldenFileComparator = comparator;
    });

    group('matches', () {
      testWidgets('if comparator succeeds', (WidgetTester tester) async {
        await tester.pumpWidget(boilerplate(const Text('hello')));
        final Finder finder = find.byType(Text);
        await expectLater(finder, matchesGoldenFile('foo.png'));
        expect(comparator.invocation, _ComparatorInvocation.compare);
        expect(comparator.imageBytes, hasLength(greaterThan(0)));
        expect(comparator.golden, Uri.parse('foo.png'));
      });

      testWidgets('list of integers', (WidgetTester tester) async {
        await expectLater(<int>[1, 2], matchesGoldenFile('foo.png'));
        expect(comparator.invocation, _ComparatorInvocation.compare);
        expect(comparator.imageBytes, equals(<int>[1, 2]));
        expect(comparator.golden, Uri.parse('foo.png'));
      });

      testWidgets('future list of integers', (WidgetTester tester) async {
        await expectLater(Future<List<int>>.value(<int>[1, 2]), matchesGoldenFile('foo.png'));
        expect(comparator.invocation, _ComparatorInvocation.compare);
        expect(comparator.imageBytes, equals(<int>[1, 2]));
        expect(comparator.golden, Uri.parse('foo.png'));
      });

      testWidgets('future nullable list of integers', (WidgetTester tester) async {
        await expectLater(Future<List<int>?>.value(<int>[1, 2]), matchesGoldenFile('foo.png'));
        expect(comparator.invocation, _ComparatorInvocation.compare);
        expect(comparator.imageBytes, equals(<int>[1, 2]));
        expect(comparator.golden, Uri.parse('foo.png'));
      });
    });

    group('does not match', () {
      testWidgets('if comparator returns false', (WidgetTester tester) async {
        comparator.behavior = _ComparatorBehavior.returnFalse;
        await tester.pumpWidget(boilerplate(const Text('hello')));
        final Finder finder = find.byType(Text);
        await expectLater(
          () => expectLater(finder, matchesGoldenFile('foo.png')),
          throwsA(
            isA<TestFailure>().having(
              (TestFailure error) => error.message,
              'message',
              contains('does not match'),
            ),
          ),
        );
        expect(comparator.invocation, _ComparatorInvocation.compare);
      });

      testWidgets('if comparator throws', (WidgetTester tester) async {
        comparator.behavior = _ComparatorBehavior.throwTestFailure;
        await tester.pumpWidget(boilerplate(const Text('hello')));
        final Finder finder = find.byType(Text);
        await expectLater(
          () => expectLater(finder, matchesGoldenFile('foo.png')),
          throwsA(
            isA<TestFailure>().having(
              (TestFailure error) => error.message,
              'message',
              contains('fake message'),
            ),
          ),
        );
        expect(comparator.invocation, _ComparatorInvocation.compare);
      });

      testWidgets('if finder finds no widgets', (WidgetTester tester) async {
        await tester.pumpWidget(boilerplate(Container()));
        final Finder finder = find.byType(Text);
        await expectLater(
          () => expectLater(finder, matchesGoldenFile('foo.png')),
          throwsA(
            isA<TestFailure>().having(
              (TestFailure error) => error.message,
              'message',
              contains('no widget was found'),
            ),
          ),
        );
        expect(comparator.invocation, isNull);
      });

      testWidgets('if finder finds multiple widgets', (WidgetTester tester) async {
        await tester.pumpWidget(
          boilerplate(const Column(children: <Widget>[Text('hello'), Text('world')])),
        );
        final Finder finder = find.byType(Text);
        await expectLater(
          () => expectLater(finder, matchesGoldenFile('foo.png')),
          throwsA(
            isA<TestFailure>().having(
              (TestFailure error) => error.message,
              'message',
              contains('too many widgets'),
            ),
          ),
        );
        expect(comparator.invocation, isNull);
      });
    });

    testWidgets('calls update on comparator if autoUpdateGoldenFiles is true', (
      WidgetTester tester,
    ) async {
      autoUpdateGoldenFiles = true;
      await tester.pumpWidget(boilerplate(const Text('hello')));
      final Finder finder = find.byType(Text);
      await expectLater(finder, matchesGoldenFile('foo.png'));
      expect(comparator.invocation, _ComparatorInvocation.update);
      expect(comparator.imageBytes, hasLength(greaterThan(0)));
      expect(comparator.golden, Uri.parse('foo.png'));
      autoUpdateGoldenFiles = false;
    });
  });

  group('matchesSemanticsData', () {
    testWidgets('matches SemanticsData', (WidgetTester tester) async {
      final SemanticsHandle handle = tester.ensureSemantics();
      const Key key = Key('semantics');
      await tester.pumpWidget(
        Semantics(
          key: key,
          namesRoute: true,
          header: true,
          button: true,
          link: true,
          onTap: () {},
          onLongPress: () {},
          label: 'foo',
          hint: 'bar',
          value: 'baz',
          increasedValue: 'a',
          decreasedValue: 'b',
          textDirection: TextDirection.rtl,
          onTapHint: 'scan',
          onLongPressHint: 'fill',
          customSemanticsActions: <CustomSemanticsAction, VoidCallback>{
            const CustomSemanticsAction(label: 'foo'): () {},
            const CustomSemanticsAction(label: 'bar'): () {},
          },
        ),
      );

      expect(
        tester.getSemantics(find.byKey(key)),
        matchesSemantics(
          label: 'foo',
          hint: 'bar',
          value: 'baz',
          increasedValue: 'a',
          decreasedValue: 'b',
          textDirection: TextDirection.rtl,
          hasTapAction: true,
          hasLongPressAction: true,
          isButton: true,
          isLink: true,
          isHeader: true,
          namesRoute: true,
          onTapHint: 'scan',
          onLongPressHint: 'fill',
          customActions: <CustomSemanticsAction>[
            const CustomSemanticsAction(label: 'foo'),
            const CustomSemanticsAction(label: 'bar'),
          ],
        ),
      );

      // Doesn't match custom actions
      expect(
        tester.getSemantics(find.byKey(key)),
        isNot(
          matchesSemantics(
            label: 'foo',
            hint: 'bar',
            value: 'baz',
            textDirection: TextDirection.rtl,
            hasTapAction: true,
            hasLongPressAction: true,
            isButton: true,
            isLink: true,
            isHeader: true,
            namesRoute: true,
            onTapHint: 'scan',
            onLongPressHint: 'fill',
            customActions: <CustomSemanticsAction>[
              const CustomSemanticsAction(label: 'foo'),
              const CustomSemanticsAction(label: 'barz'),
            ],
          ),
        ),
      );

      // Doesn't match wrong hints
      expect(
        tester.getSemantics(find.byKey(key)),
        isNot(
          matchesSemantics(
            label: 'foo',
            hint: 'bar',
            value: 'baz',
            textDirection: TextDirection.rtl,
            hasTapAction: true,
            hasLongPressAction: true,
            isButton: true,
            isLink: true,
            isHeader: true,
            namesRoute: true,
            onTapHint: 'scans',
            onLongPressHint: 'fills',
            customActions: <CustomSemanticsAction>[
              const CustomSemanticsAction(label: 'foo'),
              const CustomSemanticsAction(label: 'bar'),
            ],
          ),
        ),
      );

      handle.dispose();
    });

    testWidgets('Can match all semantics flags and actions', (WidgetTester tester) async {
      int actions = 0;
      int flags = 0;
      const CustomSemanticsAction action = CustomSemanticsAction(label: 'test');
      for (final SemanticsAction action in SemanticsAction.values) {
        actions |= action.index;
      }
      for (final SemanticsFlag flag in SemanticsFlag.values) {
        flags |= flag.index;
      }
      final SemanticsData data = SemanticsData(
        flags: flags,
        actions: actions,
        identifier: 'i',
        attributedLabel: AttributedString('a'),
        attributedIncreasedValue: AttributedString('b'),
        attributedValue: AttributedString('c'),
        attributedDecreasedValue: AttributedString('d'),
        attributedHint: AttributedString('e'),
        tooltip: 'f',
        textDirection: TextDirection.ltr,
        rect: const Rect.fromLTRB(0.0, 0.0, 10.0, 10.0),
        elevation: 3.0,
        thickness: 4.0,
        textSelection: null,
        scrollIndex: null,
        scrollChildCount: null,
        scrollPosition: null,
        scrollExtentMax: null,
        scrollExtentMin: null,
        platformViewId: 105,
        customSemanticsActionIds: <int>[CustomSemanticsAction.getIdentifier(action)],
        currentValueLength: 10,
        maxValueLength: 15,
        headingLevel: 0,
        linkUrl: Uri(path: 'l'),
      );
      final _FakeSemanticsNode node = _FakeSemanticsNode(data);

      expect(
        node,
        matchesSemantics(
          rect: const Rect.fromLTRB(0.0, 0.0, 10.0, 10.0),
          size: const Size(10.0, 10.0),
          elevation: 3.0,
          thickness: 4.0,
          platformViewId: 105,
          currentValueLength: 10,
          maxValueLength: 15,
          /* Flags */
          hasCheckedState: true,
          isChecked: true,
          isCheckStateMixed: true,
          isSelected: true,
          isButton: true,
          isSlider: true,
          isKeyboardKey: true,
          isLink: true,
          isTextField: true,
          isReadOnly: true,
          hasEnabledState: true,
          isFocused: true,
          isFocusable: true,
          isEnabled: true,
          isInMutuallyExclusiveGroup: true,
          isHeader: true,
          isObscured: true,
          isMultiline: true,
          namesRoute: true,
          scopesRoute: true,
          isHidden: true,
          isImage: true,
          isLiveRegion: true,
          hasToggledState: true,
          isToggled: true,
          hasImplicitScrolling: true,
          hasExpandedState: true,
          isExpanded: true,
          /* Actions */
          hasTapAction: true,
          hasLongPressAction: true,
          hasScrollLeftAction: true,
          hasScrollRightAction: true,
          hasScrollUpAction: true,
          hasScrollDownAction: true,
          hasIncreaseAction: true,
          hasDecreaseAction: true,
          hasShowOnScreenAction: true,
          hasMoveCursorForwardByCharacterAction: true,
          hasMoveCursorBackwardByCharacterAction: true,
          hasMoveCursorForwardByWordAction: true,
          hasMoveCursorBackwardByWordAction: true,
          hasSetTextAction: true,
          hasSetSelectionAction: true,
          hasCopyAction: true,
          hasCutAction: true,
          hasPasteAction: true,
          hasDidGainAccessibilityFocusAction: true,
          hasDidLoseAccessibilityFocusAction: true,
          hasDismissAction: true,
          hasFocusAction: true,
          customActions: <CustomSemanticsAction>[action],
        ),
      );
    });

    testWidgets('Can match child semantics', (WidgetTester tester) async {
      final SemanticsHandle handle = tester.ensureSemantics();
      const Key key = Key('a');
      await tester.pumpWidget(
        Semantics(
          key: key,
          label: 'Foo',
          container: true,
          explicitChildNodes: true,
          textDirection: TextDirection.ltr,
          child: Semantics(label: 'Bar', textDirection: TextDirection.ltr),
        ),
      );
      final SemanticsNode node = tester.getSemantics(find.byKey(key));

      expect(
        node,
        matchesSemantics(
          label: 'Foo',
          textDirection: TextDirection.ltr,
          children: <Matcher>[matchesSemantics(label: 'Bar', textDirection: TextDirection.ltr)],
        ),
      );
      handle.dispose();
    });

    testWidgets('failure does not throw unexpected errors', (WidgetTester tester) async {
      final SemanticsHandle handle = tester.ensureSemantics();

      const Key key = Key('semantics');
      await tester.pumpWidget(
        Semantics(
          key: key,
          namesRoute: true,
          header: true,
          button: true,
          link: true,
          onTap: () {},
          onLongPress: () {},
          identifier: 'ident',
          label: 'foo',
          hint: 'bar',
          value: 'baz',
          increasedValue: 'a',
          decreasedValue: 'b',
          textDirection: TextDirection.rtl,
          onTapHint: 'scan',
          onLongPressHint: 'fill',
          customSemanticsActions: <CustomSemanticsAction, VoidCallback>{
            const CustomSemanticsAction(label: 'foo'): () {},
            const CustomSemanticsAction(label: 'bar'): () {},
          },
        ),
      );

      // This should fail due to the mis-match between the `namesRoute` value.
      void failedExpectation() => expect(
        tester.getSemantics(find.byKey(key)),
        matchesSemantics(
          // Adding the explicit `false` for test readability
          // ignore: avoid_redundant_argument_values
          namesRoute: false,
          label: 'foo',
          hint: 'bar',
          value: 'baz',
          increasedValue: 'a',
          decreasedValue: 'b',
          textDirection: TextDirection.rtl,
          hasTapAction: true,
          hasLongPressAction: true,
          isButton: true,
          isLink: true,
          isHeader: true,
          onTapHint: 'scan',
          onLongPressHint: 'fill',
          customActions: <CustomSemanticsAction>[
            const CustomSemanticsAction(label: 'foo'),
            const CustomSemanticsAction(label: 'bar'),
          ],
        ),
      );

      expect(failedExpectation, throwsA(isA<TestFailure>()));
      handle.dispose();
    });
  });

  group('containsSemantics', () {
    testWidgets('matches SemanticsData', (WidgetTester tester) async {
      final SemanticsHandle handle = tester.ensureSemantics();

      const Key key = Key('semantics');
      await tester.pumpWidget(
        Semantics(
          key: key,
          namesRoute: true,
          header: true,
          button: true,
          link: true,
          onTap: () {},
          onLongPress: () {},
          label: 'foo',
          hint: 'bar',
          value: 'baz',
          increasedValue: 'a',
          decreasedValue: 'b',
          textDirection: TextDirection.rtl,
          onTapHint: 'scan',
          onLongPressHint: 'fill',
          customSemanticsActions: <CustomSemanticsAction, VoidCallback>{
            const CustomSemanticsAction(label: 'foo'): () {},
            const CustomSemanticsAction(label: 'bar'): () {},
          },
        ),
      );

      expect(
        tester.getSemantics(find.byKey(key)),
        containsSemantics(
          label: 'foo',
          hint: 'bar',
          value: 'baz',
          increasedValue: 'a',
          decreasedValue: 'b',
          textDirection: TextDirection.rtl,
          hasTapAction: true,
          hasLongPressAction: true,
          isButton: true,
          isLink: true,
          isHeader: true,
          namesRoute: true,
          onTapHint: 'scan',
          onLongPressHint: 'fill',
          customActions: <CustomSemanticsAction>[
            const CustomSemanticsAction(label: 'foo'),
            const CustomSemanticsAction(label: 'bar'),
          ],
        ),
      );

      expect(
        tester.getSemantics(find.byKey(key)),
        isNot(
          containsSemantics(
            label: 'foo',
            hint: 'bar',
            value: 'baz',
            textDirection: TextDirection.rtl,
            hasTapAction: true,
            hasLongPressAction: true,
            isButton: true,
            isLink: true,
            isHeader: true,
            namesRoute: true,
            onTapHint: 'scan',
            onLongPressHint: 'fill',
            customActions: <CustomSemanticsAction>[
              const CustomSemanticsAction(label: 'foo'),
              const CustomSemanticsAction(label: 'barz'),
            ],
          ),
        ),
        reason: 'CustomSemanticsAction "barz" should not have matched "bar".',
      );

      expect(
        tester.getSemantics(find.byKey(key)),
        isNot(
          matchesSemantics(
            label: 'foo',
            hint: 'bar',
            value: 'baz',
            textDirection: TextDirection.rtl,
            hasTapAction: true,
            hasLongPressAction: true,
            isButton: true,
            isLink: true,
            isHeader: true,
            namesRoute: true,
            onTapHint: 'scans',
            onLongPressHint: 'fills',
            customActions: <CustomSemanticsAction>[
              const CustomSemanticsAction(label: 'foo'),
              const CustomSemanticsAction(label: 'bar'),
            ],
          ),
        ),
        reason: 'onTapHint "scans" should not have matched "scan".',
      );
      handle.dispose();
    });

    testWidgets('can match all semantics flags and actions enabled', (WidgetTester tester) async {
      int actions = 0;
      int flags = 0;
      const CustomSemanticsAction action = CustomSemanticsAction(label: 'test');
      for (final SemanticsAction action in SemanticsAction.values) {
        actions |= action.index;
      }
      for (final SemanticsFlag flag in SemanticsFlag.values) {
        flags |= flag.index;
      }
      final SemanticsData data = SemanticsData(
        flags: flags,
        actions: actions,
        identifier: 'i',
        attributedLabel: AttributedString('a'),
        attributedIncreasedValue: AttributedString('b'),
        attributedValue: AttributedString('c'),
        attributedDecreasedValue: AttributedString('d'),
        attributedHint: AttributedString('e'),
        tooltip: 'f',
        textDirection: TextDirection.ltr,
        rect: const Rect.fromLTRB(0.0, 0.0, 10.0, 10.0),
        elevation: 3.0,
        thickness: 4.0,
        textSelection: null,
        scrollIndex: null,
        scrollChildCount: null,
        scrollPosition: null,
        scrollExtentMax: null,
        scrollExtentMin: null,
        platformViewId: 105,
        customSemanticsActionIds: <int>[CustomSemanticsAction.getIdentifier(action)],
        currentValueLength: 10,
        maxValueLength: 15,
        headingLevel: 0,
        linkUrl: Uri(path: 'l'),
      );
      final _FakeSemanticsNode node = _FakeSemanticsNode(data);

      expect(
        node,
        containsSemantics(
          rect: const Rect.fromLTRB(0.0, 0.0, 10.0, 10.0),
          size: const Size(10.0, 10.0),
          elevation: 3.0,
          thickness: 4.0,
          platformViewId: 105,
          currentValueLength: 10,
          maxValueLength: 15,
          /* Flags */
          hasCheckedState: true,
          isChecked: true,
          isSelected: true,
          isButton: true,
          isSlider: true,
          isKeyboardKey: true,
          isLink: true,
          isTextField: true,
          isReadOnly: true,
          hasEnabledState: true,
          isFocused: true,
          isFocusable: true,
          isEnabled: true,
          isInMutuallyExclusiveGroup: true,
          isHeader: true,
          isObscured: true,
          isMultiline: true,
          namesRoute: true,
          scopesRoute: true,
          isHidden: true,
          isImage: true,
          isLiveRegion: true,
          hasToggledState: true,
          isToggled: true,
          hasImplicitScrolling: true,
          hasExpandedState: true,
          isExpanded: true,
          /* Actions */
          hasTapAction: true,
          hasLongPressAction: true,
          hasScrollLeftAction: true,
          hasScrollRightAction: true,
          hasScrollUpAction: true,
          hasScrollDownAction: true,
          hasIncreaseAction: true,
          hasDecreaseAction: true,
          hasShowOnScreenAction: true,
          hasMoveCursorForwardByCharacterAction: true,
          hasMoveCursorBackwardByCharacterAction: true,
          hasMoveCursorForwardByWordAction: true,
          hasMoveCursorBackwardByWordAction: true,
          hasSetTextAction: true,
          hasSetSelectionAction: true,
          hasCopyAction: true,
          hasCutAction: true,
          hasPasteAction: true,
          hasDidGainAccessibilityFocusAction: true,
          hasDidLoseAccessibilityFocusAction: true,
          hasDismissAction: true,
          hasFocusAction: true,
          customActions: <CustomSemanticsAction>[action],
        ),
      );
    });

    testWidgets('can match all flags and actions disabled', (WidgetTester tester) async {
      final SemanticsData data = SemanticsData(
        flags: 0,
        actions: 0,
        identifier: 'i',
        attributedLabel: AttributedString('a'),
        attributedIncreasedValue: AttributedString('b'),
        attributedValue: AttributedString('c'),
        attributedDecreasedValue: AttributedString('d'),
        attributedHint: AttributedString('e'),
        tooltip: 'f',
        textDirection: TextDirection.ltr,
        rect: const Rect.fromLTRB(0.0, 0.0, 10.0, 10.0),
        elevation: 3.0,
        thickness: 4.0,
        textSelection: null,
        scrollIndex: null,
        scrollChildCount: null,
        scrollPosition: null,
        scrollExtentMax: null,
        scrollExtentMin: null,
        platformViewId: 105,
        currentValueLength: 10,
        maxValueLength: 15,
        headingLevel: 0,
        linkUrl: null,
      );
      final _FakeSemanticsNode node = _FakeSemanticsNode(data);

      expect(
        node,
        containsSemantics(
          rect: const Rect.fromLTRB(0.0, 0.0, 10.0, 10.0),
          size: const Size(10.0, 10.0),
          elevation: 3.0,
          thickness: 4.0,
          platformViewId: 105,
          currentValueLength: 10,
          maxValueLength: 15,
          /* Flags */
          hasCheckedState: false,
          isChecked: false,
          isSelected: false,
          isButton: false,
          isSlider: false,
          isKeyboardKey: false,
          isLink: false,
          isTextField: false,
          isReadOnly: false,
          hasEnabledState: false,
          isFocused: false,
          isFocusable: false,
          isEnabled: false,
          isInMutuallyExclusiveGroup: false,
          isHeader: false,
          isObscured: false,
          isMultiline: false,
          namesRoute: false,
          scopesRoute: false,
          isHidden: false,
          isImage: false,
          isLiveRegion: false,
          hasToggledState: false,
          isToggled: false,
          hasImplicitScrolling: false,
          hasExpandedState: false,
          isExpanded: false,
          /* Actions */
          hasTapAction: false,
          hasLongPressAction: false,
          hasScrollLeftAction: false,
          hasScrollRightAction: false,
          hasScrollUpAction: false,
          hasScrollDownAction: false,
          hasIncreaseAction: false,
          hasDecreaseAction: false,
          hasShowOnScreenAction: false,
          hasMoveCursorForwardByCharacterAction: false,
          hasMoveCursorBackwardByCharacterAction: false,
          hasMoveCursorForwardByWordAction: false,
          hasMoveCursorBackwardByWordAction: false,
          hasSetTextAction: false,
          hasSetSelectionAction: false,
          hasCopyAction: false,
          hasCutAction: false,
          hasPasteAction: false,
          hasDidGainAccessibilityFocusAction: false,
          hasDidLoseAccessibilityFocusAction: false,
          hasDismissAction: false,
          hasFocusAction: false,
        ),
      );
    });

    testWidgets('only matches given flags and actions', (WidgetTester tester) async {
      int allActions = 0;
      int allFlags = 0;
      for (final SemanticsAction action in SemanticsAction.values) {
        allActions |= action.index;
      }
      for (final SemanticsFlag flag in SemanticsFlag.values) {
        allFlags |= flag.index;
      }
      final SemanticsData emptyData = SemanticsData(
        flags: 0,
        actions: 0,
        identifier: 'i',
        attributedLabel: AttributedString('a'),
        attributedIncreasedValue: AttributedString('b'),
        attributedValue: AttributedString('c'),
        attributedDecreasedValue: AttributedString('d'),
        attributedHint: AttributedString('e'),
        tooltip: 'f',
        textDirection: TextDirection.ltr,
        rect: const Rect.fromLTRB(0.0, 0.0, 10.0, 10.0),
        elevation: 3.0,
        thickness: 4.0,
        textSelection: null,
        scrollIndex: null,
        scrollChildCount: null,
        scrollPosition: null,
        scrollExtentMax: null,
        scrollExtentMin: null,
        platformViewId: 105,
        currentValueLength: 10,
        maxValueLength: 15,
        headingLevel: 0,
        linkUrl: null,
      );
      final _FakeSemanticsNode emptyNode = _FakeSemanticsNode(emptyData);

      const CustomSemanticsAction action = CustomSemanticsAction(label: 'test');
      final SemanticsData fullData = SemanticsData(
        flags: allFlags,
        actions: allActions,
        identifier: 'i',
        attributedLabel: AttributedString('a'),
        attributedIncreasedValue: AttributedString('b'),
        attributedValue: AttributedString('c'),
        attributedDecreasedValue: AttributedString('d'),
        attributedHint: AttributedString('e'),
        tooltip: 'f',
        textDirection: TextDirection.ltr,
        rect: const Rect.fromLTRB(0.0, 0.0, 10.0, 10.0),
        elevation: 3.0,
        thickness: 4.0,
        textSelection: null,
        scrollIndex: null,
        scrollChildCount: null,
        scrollPosition: null,
        scrollExtentMax: null,
        scrollExtentMin: null,
        platformViewId: 105,
        currentValueLength: 10,
        maxValueLength: 15,
        customSemanticsActionIds: <int>[CustomSemanticsAction.getIdentifier(action)],
        headingLevel: 0,
        linkUrl: Uri(path: 'l'),
      );
      final _FakeSemanticsNode fullNode = _FakeSemanticsNode(fullData);

      expect(
        emptyNode,
        containsSemantics(
          rect: const Rect.fromLTRB(0.0, 0.0, 10.0, 10.0),
          size: const Size(10.0, 10.0),
          elevation: 3.0,
          thickness: 4.0,
          platformViewId: 105,
          currentValueLength: 10,
          maxValueLength: 15,
        ),
      );

      expect(
        fullNode,
        containsSemantics(
          rect: const Rect.fromLTRB(0.0, 0.0, 10.0, 10.0),
          size: const Size(10.0, 10.0),
          elevation: 3.0,
          thickness: 4.0,
          platformViewId: 105,
          currentValueLength: 10,
          maxValueLength: 15,
          customActions: <CustomSemanticsAction>[action],
        ),
      );
    });

    testWidgets('can match child semantics', (WidgetTester tester) async {
      final SemanticsHandle handle = tester.ensureSemantics();
      const Key key = Key('a');
      await tester.pumpWidget(
        Semantics(
          key: key,
          label: 'Foo',
          container: true,
          explicitChildNodes: true,
          textDirection: TextDirection.ltr,
          child: Semantics(label: 'Bar', textDirection: TextDirection.ltr),
        ),
      );
      final SemanticsNode node = tester.getSemantics(find.byKey(key));

      expect(
        node,
        containsSemantics(
          label: 'Foo',
          textDirection: TextDirection.ltr,
          children: <Matcher>[containsSemantics(label: 'Bar', textDirection: TextDirection.ltr)],
        ),
      );

      handle.dispose();
    });

    testWidgets('can match only custom actions', (WidgetTester tester) async {
      const CustomSemanticsAction action = CustomSemanticsAction(label: 'test');
      final SemanticsData data = SemanticsData(
        flags: 0,
        actions: SemanticsAction.customAction.index,
        identifier: 'i',
        attributedLabel: AttributedString('a'),
        attributedIncreasedValue: AttributedString('b'),
        attributedValue: AttributedString('c'),
        attributedDecreasedValue: AttributedString('d'),
        attributedHint: AttributedString('e'),
        tooltip: 'f',
        textDirection: TextDirection.ltr,
        rect: const Rect.fromLTRB(0.0, 0.0, 10.0, 10.0),
        elevation: 3.0,
        thickness: 4.0,
        textSelection: null,
        scrollIndex: null,
        scrollChildCount: null,
        scrollPosition: null,
        scrollExtentMax: null,
        scrollExtentMin: null,
        platformViewId: 105,
        currentValueLength: 10,
        maxValueLength: 15,
        customSemanticsActionIds: <int>[CustomSemanticsAction.getIdentifier(action)],
        headingLevel: 0,
        linkUrl: null,
      );
      final _FakeSemanticsNode node = _FakeSemanticsNode(data);

      expect(node, containsSemantics(customActions: <CustomSemanticsAction>[action]));
    });

    testWidgets('failure does not throw unexpected errors', (WidgetTester tester) async {
      final SemanticsHandle handle = tester.ensureSemantics();

      const Key key = Key('semantics');
      await tester.pumpWidget(
        Semantics(
          key: key,
          namesRoute: true,
          header: true,
          button: true,
          link: true,
          onTap: () {},
          onLongPress: () {},
          label: 'foo',
          hint: 'bar',
          value: 'baz',
          increasedValue: 'a',
          decreasedValue: 'b',
          textDirection: TextDirection.rtl,
          onTapHint: 'scan',
          onLongPressHint: 'fill',
          customSemanticsActions: <CustomSemanticsAction, VoidCallback>{
            const CustomSemanticsAction(label: 'foo'): () {},
            const CustomSemanticsAction(label: 'bar'): () {},
          },
        ),
      );

      // This should fail due to the mis-match between the `namesRoute` value.
      void failedExpectation() => expect(
        tester.getSemantics(find.byKey(key)),
        containsSemantics(
          label: 'foo',
          hint: 'bar',
          value: 'baz',
          increasedValue: 'a',
          decreasedValue: 'b',
          textDirection: TextDirection.rtl,
          hasTapAction: true,
          hasLongPressAction: true,
          isButton: true,
          isLink: true,
          isHeader: true,
          namesRoute: false,
          onTapHint: 'scan',
          onLongPressHint: 'fill',
          customActions: <CustomSemanticsAction>[
            const CustomSemanticsAction(label: 'foo'),
            const CustomSemanticsAction(label: 'bar'),
          ],
        ),
      );

      expect(failedExpectation, throwsA(isA<TestFailure>()));
      handle.dispose();
    });
  });

  group('findsAtLeastNWidgets', () {
    Widget boilerplate(Widget child) {
      return Directionality(textDirection: TextDirection.ltr, child: child);
    }

    testWidgets('succeeds when finds more then the specified count', (WidgetTester tester) async {
      await tester.pumpWidget(
        boilerplate(const Column(children: <Widget>[Text('1'), Text('2'), Text('3')])),
      );

      expect(find.byType(Text), findsAtLeastNWidgets(2));
    });

    testWidgets('succeeds when finds the exact specified count', (WidgetTester tester) async {
      await tester.pumpWidget(boilerplate(const Column(children: <Widget>[Text('1'), Text('2')])));

      expect(find.byType(Text), findsAtLeastNWidgets(2));
    });

    testWidgets('fails when finds less then specified count', (WidgetTester tester) async {
      await tester.pumpWidget(boilerplate(const Column(children: <Widget>[Text('1'), Text('2')])));

      expect(find.byType(Text), isNot(findsAtLeastNWidgets(3)));
    });
  });

  group('findsOneWidget', () {
    testWidgets('finds exactly one widget', (WidgetTester tester) async {
      await tester.pumpWidget(const Text('foo', textDirection: TextDirection.ltr));
      expect(find.text('foo'), findsOneWidget);
    });

    testWidgets('fails with a descriptive message', (WidgetTester tester) async {
      late TestFailure failure;
      try {
        expect(find.text('foo', skipOffstage: false), findsOneWidget);
      } on TestFailure catch (e) {
        failure = e;
      }

      expect(failure, isNotNull);
      final String? message = failure.message;
      expect(message, contains('Expected: exactly one matching candidate\n'));
      expect(message, contains('Actual: _TextWidgetFinder:<Found 0 widgets with text "foo"'));
      expect(message, contains('Which: means none were found but one was expected\n'));
    });
  });

  group('findsNothing', () {
    testWidgets('finds no widgets', (WidgetTester tester) async {
      expect(find.text('foo'), findsNothing);
    });

    testWidgets('fails with a descriptive message', (WidgetTester tester) async {
      await tester.pumpWidget(const Text('foo', textDirection: TextDirection.ltr));

      late TestFailure failure;
      try {
        expect(find.text('foo', skipOffstage: false), findsNothing);
      } on TestFailure catch (e) {
        failure = e;
      }

      expect(failure, isNotNull);
      final String? message = failure.message;

      expect(message, contains('Expected: no matching candidates\n'));
      expect(message, contains('Actual: _TextWidgetFinder:<Found 1 widget with text "foo"'));
      expect(message, contains('Text("foo", textDirection: ltr, dependencies: [MediaQuery])'));
      expect(message, contains('Which: means one was found but none were expected\n'));
    });

    testWidgets('fails with a descriptive message when skipping', (WidgetTester tester) async {
      await tester.pumpWidget(const Text('foo', textDirection: TextDirection.ltr));

      late TestFailure failure;
      try {
        expect(find.text('foo'), findsNothing);
      } on TestFailure catch (e) {
        failure = e;
      }

      expect(failure, isNotNull);
      final String? message = failure.message;

      expect(message, contains('Expected: no matching candidates\n'));
      expect(message, contains('Actual: _TextWidgetFinder:<Found 1 widget with text "foo"'));
      expect(message, contains('Text("foo", textDirection: ltr, dependencies: [MediaQuery])'));
      expect(message, contains('Which: means one was found but none were expected\n'));
    });
  });
}

enum _ComparatorBehavior { returnTrue, returnFalse, throwTestFailure }

enum _ComparatorInvocation { compare, update }

class _FakeComparator implements GoldenFileComparator {
  _ComparatorBehavior behavior = _ComparatorBehavior.returnTrue;
  _ComparatorInvocation? invocation;
  Uint8List? imageBytes;
  Uri? golden;

  @override
  Future<bool> compare(Uint8List imageBytes, Uri golden) {
    invocation = _ComparatorInvocation.compare;
    this.imageBytes = imageBytes;
    this.golden = golden;
    switch (behavior) {
      case _ComparatorBehavior.returnTrue:
        return Future<bool>.value(true);
      case _ComparatorBehavior.returnFalse:
        return Future<bool>.value(false);
      case _ComparatorBehavior.throwTestFailure:
        fail('fake message');
    }
  }

  @override
  Future<void> update(Uri golden, Uint8List imageBytes) {
    invocation = _ComparatorInvocation.update;
    this.golden = golden;
    this.imageBytes = imageBytes;
    return Future<void>.value();
  }

  @override
  Uri getTestUri(Uri key, int? version) {
    return key;
  }
}

class _FakeSemanticsNode extends SemanticsNode {
  _FakeSemanticsNode(this.data);

  SemanticsData data;
  @override
  SemanticsData getSemanticsData() => data;
}

@immutable
class _CustomColor extends Color {
  const _CustomColor(super.value, {this.isEqual});
  final bool? isEqual;

  @override
  bool operator ==(Object other) => isEqual ?? super == other;

  @override
  int get hashCode => Object.hash(super.hashCode, isEqual);
}
