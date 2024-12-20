// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  group('text contrast guideline', () {
    testWidgets('black text on white background - Text Widget - direct style', (
      WidgetTester tester,
    ) async {
      final SemanticsHandle handle = tester.ensureSemantics();
      await tester.pumpWidget(
        _boilerplate(
          const Text('this is a test', style: TextStyle(fontSize: 14.0, color: Colors.black)),
        ),
      );
      await expectLater(tester, meetsGuideline(textContrastGuideline));
      handle.dispose();
    });

    testWidgets('Multiple text with same label', (WidgetTester tester) async {
      final SemanticsHandle handle = tester.ensureSemantics();
      await tester.pumpWidget(
        _boilerplate(
          const Column(
            children: <Widget>[
              Text('this is a test', style: TextStyle(fontSize: 14.0, color: Colors.black)),
              Text('this is a test', style: TextStyle(fontSize: 14.0, color: Colors.black)),
            ],
          ),
        ),
      );
      await expectLater(tester, meetsGuideline(textContrastGuideline));
      handle.dispose();
    });

    testWidgets('Multiple text with same label but Nodes excluded from '
        'semantic tree have failing contrast should pass a11y guideline ', (
      WidgetTester tester,
    ) async {
      final SemanticsHandle handle = tester.ensureSemantics();
      await tester.pumpWidget(
        _boilerplate(
          const Column(
            children: <Widget>[
              Text('this is a test', style: TextStyle(fontSize: 14.0, color: Colors.black)),
              SizedBox(height: 50),
              Text('this is a test', style: TextStyle(fontSize: 14.0, color: Colors.black)),
              SizedBox(height: 50),
              ExcludeSemantics(
                child: Text(
                  'this is a test',
                  style: TextStyle(fontSize: 14.0, color: Colors.white),
                ),
              ),
            ],
          ),
        ),
      );
      await expectLater(tester, meetsGuideline(textContrastGuideline));
      handle.dispose();
    });

    testWidgets('white text on black background - Text Widget - direct style', (
      WidgetTester tester,
    ) async {
      final SemanticsHandle handle = tester.ensureSemantics();
      await tester.pumpWidget(
        _boilerplate(
          Container(
            width: 200.0,
            height: 200.0,
            color: Colors.black,
            child: const Text(
              'this is a test',
              style: TextStyle(fontSize: 14.0, color: Colors.white),
            ),
          ),
        ),
      );
      await expectLater(tester, meetsGuideline(textContrastGuideline));
      handle.dispose();
    });

    testWidgets('White text on white background fails contrast test', (WidgetTester tester) async {
      final SemanticsHandle handle = tester.ensureSemantics();
      await tester.pumpWidget(
        _boilerplate(
          Container(
            width: 200.0,
            height: 300.0,
            color: Colors.white,
            child: const Column(
              children: <Widget>[
                Text('this is a white text', style: TextStyle(fontSize: 14.0, color: Colors.white)),
                SizedBox(height: 50),
                Text(
                  'this is a black text test1',
                  style: TextStyle(fontSize: 14.0, color: Colors.black),
                ),
                SizedBox(height: 50),
                Text(
                  'this is a black text test2',
                  style: TextStyle(fontSize: 14.0, color: Colors.black),
                ),
              ],
            ),
          ),
        ),
      );
      await expectLater(tester, doesNotMeetGuideline(textContrastGuideline));
      handle.dispose();
    });

    const Color surface = Color(0xFFF0F0F0);

    /// Shades of blue with contrast ratio of 2.9, 4.4, 4.5 from [surface].
    const Color blue29 = Color(0xFF7E7EFB);
    const Color blue44 = Color(0xFF5757FF);
    const Color blue45 = Color(0xFF5252FF);
    const List<TextStyle> textStylesMeetingGuideline = <TextStyle>[
      TextStyle(color: blue44, backgroundColor: surface, fontSize: 18),
      TextStyle(color: blue44, backgroundColor: surface, fontSize: 14, fontWeight: FontWeight.bold),
      TextStyle(color: blue45, backgroundColor: surface),
    ];
    const List<TextStyle> textStylesDoesNotMeetingGuideline = <TextStyle>[
      TextStyle(color: blue44, backgroundColor: surface),
      TextStyle(color: blue29, backgroundColor: surface, fontSize: 18),
    ];

    Widget appWithTextWidget(TextStyle style) =>
        _boilerplate(Text('this is text', style: style.copyWith(height: 30.0)));

    for (final TextStyle style in textStylesMeetingGuideline) {
      testWidgets('text with style $style', (WidgetTester tester) async {
        final SemanticsHandle handle = tester.ensureSemantics();
        await tester.pumpWidget(appWithTextWidget(style));
        await expectLater(tester, meetsGuideline(textContrastGuideline));
        handle.dispose();
      });
    }

    for (final TextStyle style in textStylesDoesNotMeetingGuideline) {
      testWidgets('text with $style', (WidgetTester tester) async {
        final SemanticsHandle handle = tester.ensureSemantics();
        await tester.pumpWidget(appWithTextWidget(style));
        await expectLater(tester, doesNotMeetGuideline(textContrastGuideline));
        handle.dispose();
      });
    }

    testWidgets('black text on white background - Text Widget - direct style', (
      WidgetTester tester,
    ) async {
      final SemanticsHandle handle = tester.ensureSemantics();
      await tester.pumpWidget(
        _boilerplate(
          const Text('this is a test', style: TextStyle(fontSize: 14.0, color: Colors.black)),
        ),
      );
      await expectLater(tester, meetsGuideline(textContrastGuideline));
      handle.dispose();
    });

    testWidgets('white text on black background - Text Widget - direct style', (
      WidgetTester tester,
    ) async {
      final SemanticsHandle handle = tester.ensureSemantics();
      await tester.pumpWidget(
        _boilerplate(
          Container(
            width: 200.0,
            height: 200.0,
            color: Colors.black,
            child: const Text(
              'this is a test',
              style: TextStyle(fontSize: 14.0, color: Colors.white),
            ),
          ),
        ),
      );
      await expectLater(tester, meetsGuideline(textContrastGuideline));
      handle.dispose();
    });

    testWidgets('Material text field - amber on amber', (WidgetTester tester) async {
      final SemanticsHandle handle = tester.ensureSemantics();
      await tester.pumpWidget(
        _boilerplate(
          Container(
            width: 200.0,
            height: 200.0,
            color: Colors.amberAccent,
            child: TextField(
              style: const TextStyle(color: Colors.amber),
              controller: TextEditingController(text: 'this is a test'),
            ),
          ),
        ),
      );
      await expectLater(tester, doesNotMeetGuideline(textContrastGuideline));
      handle.dispose();
    });

    testWidgets('Correctly identify failures in complex transforms', (WidgetTester tester) async {
      final SemanticsHandle handle = tester.ensureSemantics();
      await tester.pumpWidget(
        _boilerplate(
          Padding(
            padding: const EdgeInsets.only(left: 100),
            child: Semantics(
              container: true,
              child: Padding(
                padding: const EdgeInsets.only(left: 100),
                child: Semantics(
                  container: true,
                  child: Container(
                    width: 100.0,
                    height: 200.0,
                    color: Colors.amberAccent,
                    child: const Text('this', style: TextStyle(color: Colors.amber)),
                  ),
                ),
              ),
            ),
          ),
        ),
      );
      await expectLater(tester, doesNotMeetGuideline(textContrastGuideline));
      handle.dispose();
    });

    testWidgets('Material text field - default style', (WidgetTester tester) async {
      final SemanticsHandle handle = tester.ensureSemantics();
      await tester.pumpWidget(
        _boilerplate(
          SizedBox(
            width: 100,
            child: TextField(controller: TextEditingController(text: 'this is a test')),
          ),
        ),
      );
      await tester.idle();
      await expectLater(tester, meetsGuideline(textContrastGuideline));
      handle.dispose();
    });

    testWidgets('Material2: yellow text on yellow background fails with correct message', (
      WidgetTester tester,
    ) async {
      final SemanticsHandle handle = tester.ensureSemantics();
      await tester.pumpWidget(
        _boilerplate(
          useMaterial3: false,
          Container(
            width: 200.0,
            height: 200.0,
            color: Colors.yellow,
            child: const Text(
              'this is a test',
              style: TextStyle(fontSize: 14.0, color: Colors.yellowAccent),
            ),
          ),
        ),
      );
      final Evaluation result = await textContrastGuideline.evaluate(tester);
      expect(result.passed, false);
      expect(
        result.reason,
        'SemanticsNode#4(Rect.fromLTRB(300.0, 200.0, 500.0, 400.0), '
        'label: "this is a test", textDirection: ltr):\n'
        'Expected contrast ratio of at least 4.5 but found 1.17 for a font '
        'size of 14.0.\n'
        'The computed colors was:\n'
        'light - ${const Color(0xfffafafa)}, dark - ${const Color(0xffffeb3b)}\n'
        'See also: https://www.w3.org/TR/UNDERSTANDING-WCAG20/visual-audio-contrast-contrast.html',
      );
      handle.dispose();
    });

    testWidgets('Material3: yellow text on yellow background fails with correct message', (
      WidgetTester tester,
    ) async {
      final SemanticsHandle handle = tester.ensureSemantics();
      await tester.pumpWidget(
        _boilerplate(
          useMaterial3: true,
          Container(
            width: 200.0,
            height: 200.0,
            color: Colors.yellow,
            child: const Text(
              'this is a test',
              style: TextStyle(fontSize: 14.0, color: Colors.yellowAccent),
            ),
          ),
        ),
      );
      final Evaluation result = await textContrastGuideline.evaluate(tester);
      expect(result.passed, false);
      expect(
        result.reason,
        'SemanticsNode#4(Rect.fromLTRB(300.0, 200.0, 500.0, 400.0), '
        'label: "this is a test", textDirection: ltr):\n'
        'Expected contrast ratio of at least 4.5 but found 1.16 for a font '
        'size of 14.0.\n'
        'The computed colors was:\n'
        'light - ${const Color(0xfffef7ff)}, dark - ${const Color(0xffffeb3b)}\n'
        'See also: https://www.w3.org/TR/UNDERSTANDING-WCAG20/visual-audio-contrast-contrast.html',
      );
      handle.dispose();
    });

    testWidgets('label without corresponding text is skipped', (WidgetTester tester) async {
      final SemanticsHandle handle = tester.ensureSemantics();
      await tester.pumpWidget(
        _boilerplate(
          Semantics(
            label: 'This is not text',
            container: true,
            child: const SizedBox(width: 200.0, height: 200.0, child: Placeholder()),
          ),
        ),
      );

      final Evaluation result = await textContrastGuideline.evaluate(tester);
      expect(result.passed, true);
      handle.dispose();
    });

    testWidgets('offscreen text is skipped', (WidgetTester tester) async {
      final SemanticsHandle handle = tester.ensureSemantics();
      await tester.pumpWidget(
        _boilerplate(
          Stack(
            children: <Widget>[
              Positioned(
                left: -300.0,
                child: Container(
                  width: 200.0,
                  height: 200.0,
                  color: Colors.yellow,
                  child: const Text(
                    'this is a test',
                    style: TextStyle(fontSize: 14.0, color: Colors.yellowAccent),
                  ),
                ),
              ),
            ],
          ),
        ),
      );

      final Evaluation result = await textContrastGuideline.evaluate(tester);
      expect(result.passed, true);
      handle.dispose();
    });

    testWidgets('Disabled button is excluded from text contrast guideline', (
      WidgetTester tester,
    ) async {
      // Regression test https://github.com/flutter/flutter/issues/94428
      final SemanticsHandle handle = tester.ensureSemantics();
      await tester.pumpWidget(
        _boilerplate(
          ElevatedButton(
            onPressed: null,
            child: Container(
              width: 200.0,
              height: 200.0,
              color: Colors.yellow,
              child: const Text(
                'this is a test',
                style: TextStyle(fontSize: 14.0, color: Colors.yellowAccent),
              ),
            ),
          ),
        ),
      );
      await expectLater(tester, meetsGuideline(textContrastGuideline));
      handle.dispose();
    });
  });

  group('custom minimum contrast guideline', () {
    Widget iconWidget({
      IconData icon = Icons.search,
      required Color color,
      required Color background,
    }) {
      return Container(
        padding: const EdgeInsets.all(8.0),
        color: background,
        child: Icon(icon, color: color),
      );
    }

    Widget textWidget({String text = 'Text', required Color color, required Color background}) {
      return Container(
        padding: const EdgeInsets.all(8.0),
        color: background,
        child: Text(text, style: TextStyle(color: color)),
      );
    }

    Widget rowWidget(List<Widget> widgets) => _boilerplate(Row(children: widgets));

    final Finder findIcons = find.byWidgetPredicate((Widget widget) => widget is Icon);
    final Finder findTexts = find.byWidgetPredicate((Widget widget) => widget is Text);
    final Finder findIconsAndTexts = find.byWidgetPredicate(
      (Widget widget) => widget is Icon || widget is Text,
    );

    testWidgets('Black icons on white background', (WidgetTester tester) async {
      await tester.pumpWidget(
        rowWidget(<Widget>[
          iconWidget(color: Colors.black, background: Colors.white),
          iconWidget(color: Colors.black, background: Colors.white),
        ]),
      );

      await expectLater(tester, meetsGuideline(CustomMinimumContrastGuideline(finder: findIcons)));
    });

    testWidgets('Black icons on black background', (WidgetTester tester) async {
      await tester.pumpWidget(
        rowWidget(<Widget>[
          iconWidget(color: Colors.black, background: Colors.black),
          iconWidget(color: Colors.black, background: Colors.black),
        ]),
      );

      await expectLater(
        tester,
        doesNotMeetGuideline(CustomMinimumContrastGuideline(finder: findIcons)),
      );
    });

    testWidgets('White icons on black background ("dark mode")', (WidgetTester tester) async {
      await tester.pumpWidget(
        rowWidget(<Widget>[
          iconWidget(color: Colors.white, background: Colors.black),
          iconWidget(color: Colors.white, background: Colors.black),
        ]),
      );

      await expectLater(tester, meetsGuideline(CustomMinimumContrastGuideline(finder: findIcons)));
    });

    testWidgets('Using different icons', (WidgetTester tester) async {
      await tester.pumpWidget(
        rowWidget(<Widget>[
          iconWidget(color: Colors.black, background: Colors.white, icon: Icons.more_horiz),
          iconWidget(color: Colors.black, background: Colors.white, icon: Icons.description),
          iconWidget(color: Colors.black, background: Colors.white, icon: Icons.image),
          iconWidget(color: Colors.black, background: Colors.white, icon: Icons.beach_access),
        ]),
      );

      await expectLater(tester, meetsGuideline(CustomMinimumContrastGuideline(finder: findIcons)));
    });

    testWidgets('One invalid instance fails entire test', (WidgetTester tester) async {
      await tester.pumpWidget(
        rowWidget(<Widget>[
          iconWidget(color: Colors.black, background: Colors.white),
          iconWidget(color: Colors.black, background: Colors.black),
        ]),
      );

      await expectLater(
        tester,
        doesNotMeetGuideline(CustomMinimumContrastGuideline(finder: findIcons)),
      );
    });

    testWidgets('White on different colors, passing', (WidgetTester tester) async {
      await tester.pumpWidget(
        rowWidget(<Widget>[
          iconWidget(color: Colors.white, background: Colors.red[800]!, icon: Icons.more_horiz),
          iconWidget(color: Colors.white, background: Colors.green[800]!, icon: Icons.description),
          iconWidget(color: Colors.white, background: Colors.blue[800]!, icon: Icons.image),
          iconWidget(
            color: Colors.white,
            background: Colors.purple[800]!,
            icon: Icons.beach_access,
          ),
        ]),
      );

      await expectLater(tester, meetsGuideline(CustomMinimumContrastGuideline(finder: findIcons)));
    });

    testWidgets('White on different colors, failing', (WidgetTester tester) async {
      await tester.pumpWidget(
        rowWidget(<Widget>[
          iconWidget(color: Colors.white, background: Colors.red[200]!, icon: Icons.more_horiz),
          iconWidget(color: Colors.white, background: Colors.green[400]!, icon: Icons.description),
          iconWidget(color: Colors.white, background: Colors.blue[600]!, icon: Icons.image),
          iconWidget(
            color: Colors.white,
            background: Colors.purple[800]!,
            icon: Icons.beach_access,
          ),
        ]),
      );

      await expectLater(
        tester,
        doesNotMeetGuideline(CustomMinimumContrastGuideline(finder: findIcons)),
      );
    });

    testWidgets('Absence of icons, passing', (WidgetTester tester) async {
      await tester.pumpWidget(rowWidget(<Widget>[]));

      await expectLater(tester, meetsGuideline(CustomMinimumContrastGuideline(finder: findIcons)));
    });

    testWidgets('Absence of icons, passing - 2nd test', (WidgetTester tester) async {
      await tester.pumpWidget(
        rowWidget(<Widget>[
          textWidget(color: Colors.black, background: Colors.white),
          textWidget(color: Colors.black, background: Colors.black),
        ]),
      );

      await expectLater(tester, meetsGuideline(CustomMinimumContrastGuideline(finder: findIcons)));
    });

    testWidgets('Guideline ignores widgets of other types', (WidgetTester tester) async {
      await tester.pumpWidget(
        rowWidget(<Widget>[
          iconWidget(color: Colors.black, background: Colors.white),
          iconWidget(color: Colors.black, background: Colors.white),
          textWidget(color: Colors.black, background: Colors.white),
          textWidget(color: Colors.black, background: Colors.black),
        ]),
      );

      await expectLater(tester, meetsGuideline(CustomMinimumContrastGuideline(finder: findIcons)));
      await expectLater(
        tester,
        doesNotMeetGuideline(CustomMinimumContrastGuideline(finder: findTexts)),
      );
      await expectLater(
        tester,
        doesNotMeetGuideline(CustomMinimumContrastGuideline(finder: findIconsAndTexts)),
      );
    });

    testWidgets('Custom minimum ratio - Icons', (WidgetTester tester) async {
      await tester.pumpWidget(
        rowWidget(<Widget>[
          iconWidget(color: Colors.blue, background: Colors.white),
          iconWidget(color: Colors.black, background: Colors.white),
        ]),
      );

      await expectLater(
        tester,
        doesNotMeetGuideline(CustomMinimumContrastGuideline(finder: findIcons)),
      );
      await expectLater(
        tester,
        meetsGuideline(CustomMinimumContrastGuideline(finder: findIcons, minimumRatio: 3.0)),
      );
    });

    testWidgets('Custom minimum ratio - Texts', (WidgetTester tester) async {
      await tester.pumpWidget(
        rowWidget(<Widget>[
          textWidget(color: Colors.blue, background: Colors.white),
          textWidget(color: Colors.black, background: Colors.white),
        ]),
      );

      await expectLater(
        tester,
        doesNotMeetGuideline(CustomMinimumContrastGuideline(finder: findTexts)),
      );
      await expectLater(
        tester,
        meetsGuideline(CustomMinimumContrastGuideline(finder: findTexts, minimumRatio: 3.0)),
      );
    });

    testWidgets('Custom minimum ratio - Different standards for icons and texts', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        rowWidget(<Widget>[
          iconWidget(color: Colors.blue, background: Colors.white),
          iconWidget(color: Colors.black, background: Colors.white),
          textWidget(color: Colors.blue, background: Colors.white),
          textWidget(color: Colors.black, background: Colors.white),
        ]),
      );

      await expectLater(
        tester,
        doesNotMeetGuideline(CustomMinimumContrastGuideline(finder: findIcons)),
      );
      await expectLater(
        tester,
        meetsGuideline(CustomMinimumContrastGuideline(finder: findTexts, minimumRatio: 3.0)),
      );
    });
  });

  group('tap target size guideline', () {
    testWidgets('Tappable box at 48 by 48', (WidgetTester tester) async {
      final SemanticsHandle handle = tester.ensureSemantics();
      await tester.pumpWidget(
        _boilerplate(SizedBox(width: 48.0, height: 48.0, child: GestureDetector(onTap: () {}))),
      );
      await expectLater(tester, meetsGuideline(androidTapTargetGuideline));
      handle.dispose();
    });

    testWidgets('Tappable box at 47 by 48', (WidgetTester tester) async {
      final SemanticsHandle handle = tester.ensureSemantics();
      await tester.pumpWidget(
        _boilerplate(SizedBox(width: 47.0, height: 48.0, child: GestureDetector(onTap: () {}))),
      );
      await expectLater(tester, doesNotMeetGuideline(androidTapTargetGuideline));
      handle.dispose();
    });

    testWidgets('Tappable box at 48 by 47', (WidgetTester tester) async {
      final SemanticsHandle handle = tester.ensureSemantics();
      await tester.pumpWidget(
        _boilerplate(SizedBox(width: 48.0, height: 47.0, child: GestureDetector(onTap: () {}))),
      );
      await expectLater(tester, doesNotMeetGuideline(androidTapTargetGuideline));
      handle.dispose();
    });

    testWidgets('Tappable box at 48 by 48 shrunk by transform', (WidgetTester tester) async {
      final SemanticsHandle handle = tester.ensureSemantics();
      await tester.pumpWidget(
        _boilerplate(
          Transform.scale(
            scale: 0.5, // should have new height of 24 by 24.
            child: SizedBox(width: 48.0, height: 48.0, child: GestureDetector(onTap: () {})),
          ),
        ),
      );
      await expectLater(tester, doesNotMeetGuideline(androidTapTargetGuideline));
      handle.dispose();
    });

    testWidgets('Too small tap target fails with the correct message', (WidgetTester tester) async {
      final SemanticsHandle handle = tester.ensureSemantics();
      await tester.pumpWidget(
        _boilerplate(SizedBox(width: 48.0, height: 47.0, child: GestureDetector(onTap: () {}))),
      );
      final Evaluation result = await androidTapTargetGuideline.evaluate(tester);
      expect(result.passed, false);
      expect(
        result.reason,
        'SemanticsNode#4(Rect.fromLTRB(376.0, 276.5, 424.0, 323.5), '
        'actions: [tap]): expected tap '
        'target size of at least Size(48.0, 48.0), '
        'but found Size(48.0, 47.0)\n'
        'See also: https://support.google.com/accessibility/android/answer/7101858?hl=en',
      );
      handle.dispose();
    });

    testWidgets('Box that overlaps edge of window is skipped', (WidgetTester tester) async {
      final SemanticsHandle handle = tester.ensureSemantics();
      final Widget smallBox = SizedBox(
        width: 48.0,
        height: 47.0,
        child: GestureDetector(onTap: () {}),
      );
      await tester.pumpWidget(
        MaterialApp(
          home: Stack(children: <Widget>[Positioned(left: 0.0, top: -1.0, child: smallBox)]),
        ),
      );

      final Evaluation overlappingTopResult = await androidTapTargetGuideline.evaluate(tester);
      expect(overlappingTopResult.passed, true);

      await tester.pumpWidget(
        MaterialApp(
          home: Stack(children: <Widget>[Positioned(left: -1.0, top: 0.0, child: smallBox)]),
        ),
      );

      final Evaluation overlappingLeftResult = await androidTapTargetGuideline.evaluate(tester);
      expect(overlappingLeftResult.passed, true);

      await tester.pumpWidget(
        MaterialApp(home: Stack(children: <Widget>[Positioned(bottom: -1.0, child: smallBox)])),
      );

      final Evaluation overlappingBottomResult = await androidTapTargetGuideline.evaluate(tester);
      expect(overlappingBottomResult.passed, true);

      await tester.pumpWidget(
        MaterialApp(home: Stack(children: <Widget>[Positioned(right: -1.0, child: smallBox)])),
      );

      final Evaluation overlappingRightResult = await androidTapTargetGuideline.evaluate(tester);
      expect(overlappingRightResult.passed, true);
      handle.dispose();
    });

    testWidgets('Does not fail on mergedIntoParent child', (WidgetTester tester) async {
      final SemanticsHandle handle = tester.ensureSemantics();
      await tester.pumpWidget(
        _boilerplate(
          MergeSemantics(
            child: Semantics(
              container: true,
              child: SizedBox(
                width: 50.0,
                height: 50.0,
                child: Semantics(
                  container: true,
                  child: GestureDetector(
                    onTap: () {},
                    child: const SizedBox(width: 4.0, height: 4.0),
                  ),
                ),
              ),
            ),
          ),
        ),
      );

      final Evaluation overlappingRightResult = await androidTapTargetGuideline.evaluate(tester);
      expect(overlappingRightResult.passed, true);
      handle.dispose();
    });

    testWidgets('Does not fail on links', (WidgetTester tester) async {
      Widget textWithLink() {
        return Builder(
          builder: (BuildContext context) {
            return RichText(
              text: TextSpan(
                children: <InlineSpan>[
                  const TextSpan(text: 'See examples at '),
                  TextSpan(text: 'flutter repo', recognizer: TapGestureRecognizer()..onTap = () {}),
                ],
              ),
            );
          },
        );
      }

      final SemanticsHandle handle = tester.ensureSemantics();
      await tester.pumpWidget(_boilerplate(textWithLink()));

      await expectLater(tester, meetsGuideline(androidTapTargetGuideline));
      handle.dispose();
    });

    testWidgets('Tap size test can handle partially off-screen items', (WidgetTester tester) async {
      final ScrollController controller = ScrollController();
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            appBar: AppBar(title: const Text('Foo')),
            body: ListView(
              controller: controller,
              children: <Widget>[
                Padding(
                  padding: const EdgeInsets.only(left: 10, right: 10),
                  child: SizedBox(
                    width: 100,
                    height: 100,
                    child: Semantics(container: true, onTap: () {}, child: const Text('hello')),
                  ),
                ),
                Container(height: 1000, color: Colors.red),
              ],
            ),
          ),
        ),
      );
      controller.jumpTo(90);
      await tester.pump();
      await expectLater(tester, meetsGuideline(iOSTapTargetGuideline));
    });
  });

  group('Labeled tappable node guideline', () {
    testWidgets('Passes when node is labeled', (WidgetTester tester) async {
      final SemanticsHandle handle = tester.ensureSemantics();
      await tester.pumpWidget(
        _boilerplate(
          Semantics(
            container: true,
            onTap: () {},
            label: 'test',
            child: const SizedBox(width: 10.0, height: 10.0),
          ),
        ),
      );
      final Evaluation result = await labeledTapTargetGuideline.evaluate(tester);
      expect(result.passed, true);
      handle.dispose();
    });
    testWidgets('Fails if long-press has no label', (WidgetTester tester) async {
      final SemanticsHandle handle = tester.ensureSemantics();
      await tester.pumpWidget(
        _boilerplate(
          Semantics(
            container: true,
            onLongPress: () {},
            label: '',
            child: const SizedBox(width: 10.0, height: 10.0),
          ),
        ),
      );
      final Evaluation result = await labeledTapTargetGuideline.evaluate(tester);
      expect(result.passed, false);
      handle.dispose();
    });

    testWidgets('Fails if tap has no label', (WidgetTester tester) async {
      final SemanticsHandle handle = tester.ensureSemantics();
      await tester.pumpWidget(
        _boilerplate(
          Semantics(
            container: true,
            onTap: () {},
            label: '',
            child: const SizedBox(width: 10.0, height: 10.0),
          ),
        ),
      );
      final Evaluation result = await labeledTapTargetGuideline.evaluate(tester);
      expect(result.passed, false);
      handle.dispose();
    });

    testWidgets('Passes if tap is merged into labeled node', (WidgetTester tester) async {
      final SemanticsHandle handle = tester.ensureSemantics();
      await tester.pumpWidget(
        _boilerplate(
          Semantics(
            container: true,
            onLongPress: () {},
            label: '',
            child: Semantics(label: 'test', child: const SizedBox(width: 10.0, height: 10.0)),
          ),
        ),
      );
      final Evaluation result = await labeledTapTargetGuideline.evaluate(tester);
      expect(result.passed, true);
      handle.dispose();
    });

    testWidgets('Passes if text field does not have label', (WidgetTester tester) async {
      final SemanticsHandle handle = tester.ensureSemantics();
      await tester.pumpWidget(_boilerplate(const TextField()));
      final Evaluation result = await labeledTapTargetGuideline.evaluate(tester);
      expect(result.passed, true);
      handle.dispose();
    });
  });

  testWidgets('regression test for material widget', (WidgetTester tester) async {
    final SemanticsHandle handle = tester.ensureSemantics();
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData.light(),
        home: Scaffold(
          backgroundColor: Colors.white,
          body: ElevatedButton(
            style: ElevatedButton.styleFrom(backgroundColor: const Color(0xFFFBBC04), elevation: 0),
            onPressed: () {},
            child: const Text('Button', style: TextStyle(color: Colors.black)),
          ),
        ),
      ),
    );
    await expectLater(tester, meetsGuideline(textContrastGuideline));
    handle.dispose();
  });
}

Widget _boilerplate(Widget child, {bool? useMaterial3}) {
  return MaterialApp(
    theme: ThemeData(useMaterial3: useMaterial3),
    home: Scaffold(body: Center(child: child)),
  );
}
