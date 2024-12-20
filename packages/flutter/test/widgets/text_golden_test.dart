// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file is run as part of a reduced test set in CI on Mac and Windows
// machines.
@Tags(<String>['reduced-test-set'])
@TestOn('!chrome')
library;

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Centered text', (WidgetTester tester) async {
    await tester.pumpWidget(
      Center(
        child: RepaintBoundary(
          child: Container(
            width: 200.0,
            height: 100.0,
            decoration: const BoxDecoration(color: Color(0xff00ff00)),
            child: const Text(
              'Hello',
              textDirection: TextDirection.ltr,
              textAlign: TextAlign.center,
              style: TextStyle(color: Color(0xffff0000)),
            ),
          ),
        ),
      ),
    );

    await expectLater(find.byType(Container), matchesGoldenFile('text_golden.Centered.png'));

    await tester.pumpWidget(
      Center(
        child: RepaintBoundary(
          child: Container(
            width: 200.0,
            height: 100.0,
            decoration: const BoxDecoration(color: Color(0xff00ff00)),
            child: const Text(
              'Hello world how are you today',
              textDirection: TextDirection.ltr,
              textAlign: TextAlign.center,
              style: TextStyle(color: Color(0xffff0000)),
            ),
          ),
        ),
      ),
    );

    await expectLater(find.byType(Container), matchesGoldenFile('text_golden.Centered.wrap.png'));
  });

  testWidgets('Text Foreground', (WidgetTester tester) async {
    const Color black = Color(0xFF000000);
    const Color red = Color(0xFFFF0000);
    const Color blue = Color(0xFF0000FF);
    final Shader linearGradient = const LinearGradient(
      colors: <Color>[red, blue],
    ).createShader(const Rect.fromLTWH(0.0, 0.0, 50.0, 20.0));

    await tester.pumpWidget(
      Align(
        alignment: Alignment.topLeft,
        child: RepaintBoundary(
          child: Text(
            'Hello',
            textDirection: TextDirection.ltr,
            style: TextStyle(
              foreground:
                  Paint()
                    ..color = black
                    ..shader = linearGradient,
            ),
          ),
        ),
      ),
    );

    await expectLater(
      find.byType(RepaintBoundary),
      matchesGoldenFile('text_golden.Foreground.gradient.png'),
    );

    await tester.pumpWidget(
      Align(
        alignment: Alignment.topLeft,
        child: RepaintBoundary(
          child: Text(
            'Hello',
            textDirection: TextDirection.ltr,
            style: TextStyle(
              foreground:
                  Paint()
                    ..color = black
                    ..style = PaintingStyle.stroke
                    ..strokeWidth = 2.0,
            ),
          ),
        ),
      ),
    );

    await expectLater(
      find.byType(RepaintBoundary),
      matchesGoldenFile('text_golden.Foreground.stroke.png'),
    );

    await tester.pumpWidget(
      Align(
        alignment: Alignment.topLeft,
        child: RepaintBoundary(
          child: Text(
            'Hello',
            textDirection: TextDirection.ltr,
            style: TextStyle(
              foreground:
                  Paint()
                    ..color = black
                    ..style = PaintingStyle.stroke
                    ..strokeWidth = 2.0
                    ..shader = linearGradient,
            ),
          ),
        ),
      ),
    );

    await expectLater(
      find.byType(RepaintBoundary),
      matchesGoldenFile('text_golden.Foreground.stroke_and_gradient.png'),
    );
  });

  // TODO(garyq): This test requires an update when the background
  // drawing from the beginning of the line bug is fixed. The current
  // tested version is not completely correct.
  testWidgets('Text Background', (WidgetTester tester) async {
    const Color red = Colors.red;
    const Color blue = Colors.blue;
    const Color translucentGreen = Color(0x5000F000);
    const Color translucentDarkRed = Color(0x500F0000);
    await tester.pumpWidget(
      Align(
        alignment: Alignment.topLeft,
        child: RepaintBoundary(
          child: Container(
            width: 200.0,
            height: 100.0,
            decoration: const BoxDecoration(color: Colors.green),
            child: Text.rich(
              TextSpan(
                text: 'text1 ',
                style: TextStyle(
                  color: translucentGreen,
                  background: Paint()..color = red.withOpacity(0.5),
                ),
                children: <InlineSpan>[
                  TextSpan(
                    text: 'text2',
                    style: TextStyle(
                      color: translucentDarkRed,
                      background: Paint()..color = blue.withOpacity(0.5),
                    ),
                  ),
                ],
              ),
              textDirection: TextDirection.ltr,
            ),
          ),
        ),
      ),
    );

    await expectLater(
      find.byType(RepaintBoundary),
      matchesGoldenFile('text_golden.Background.png'),
    );
  });

  testWidgets('Text Fade', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Scaffold(
          backgroundColor: Colors.transparent,
          body: RepaintBoundary(
            child: Center(
              child: Container(
                width: 200.0,
                height: 200.0,
                color: Colors.green,
                child: Center(
                  child: Container(
                    width: 100.0,
                    color: Colors.blue,
                    child: const Text(
                      'Pp PPp PPPp PPPPp PPPPpp PPPPppp PPPPppppp ',
                      style: TextStyle(color: Colors.black),
                      maxLines: 3,
                      overflow: TextOverflow.fade,
                    ),
                  ),
                ),
              ),
            ),
          ),
        ),
      ),
    );

    await expectLater(
      find.byType(RepaintBoundary).first,
      matchesGoldenFile('text_golden.Fade.png'),
    );
  });

  testWidgets('Default Strut text', (WidgetTester tester) async {
    await tester.pumpWidget(
      Center(
        child: RepaintBoundary(
          child: Container(
            width: 200.0,
            height: 100.0,
            decoration: const BoxDecoration(color: Color(0xff00ff00)),
            child: const Text(
              'Hello\nLine 2\nLine 3',
              textDirection: TextDirection.ltr,
              style: TextStyle(),
              strutStyle: StrutStyle(),
            ),
          ),
        ),
      ),
    );
    await expectLater(find.byType(Container), matchesGoldenFile('text_golden.StrutDefault.png'));
  });

  testWidgets('Strut text 1', (WidgetTester tester) async {
    await tester.pumpWidget(
      Center(
        child: RepaintBoundary(
          child: Container(
            width: 200.0,
            height: 100.0,
            decoration: const BoxDecoration(color: Color(0xff00ff00)),
            child: const Text(
              'Hello\nLine2\nLine3',
              textDirection: TextDirection.ltr,
              style: TextStyle(),
              strutStyle: StrutStyle(height: 1.5),
            ),
          ),
        ),
      ),
    );
    await expectLater(find.byType(Container), matchesGoldenFile('text_golden.Strut.1.png'));
  });

  testWidgets('Strut text 2', (WidgetTester tester) async {
    await tester.pumpWidget(
      Center(
        child: RepaintBoundary(
          child: Container(
            width: 200.0,
            height: 100.0,
            decoration: const BoxDecoration(color: Color(0xff00ff00)),
            child: const Text(
              'Hello\nLine 2\nLine 3',
              textDirection: TextDirection.ltr,
              style: TextStyle(),
              strutStyle: StrutStyle(height: 1.5, fontSize: 14),
            ),
          ),
        ),
      ),
    );
    await expectLater(find.byType(Container), matchesGoldenFile('text_golden.Strut.2.png'));
  });

  testWidgets('Strut text rich', (WidgetTester tester) async {
    await tester.pumpWidget(
      Center(
        child: RepaintBoundary(
          child: Container(
            width: 200.0,
            height: 150.0,
            decoration: const BoxDecoration(color: Color(0xff00ff00)),
            child: const Text.rich(
              TextSpan(
                text: 'Hello\n',
                style: TextStyle(color: Colors.red, fontSize: 30),
                children: <InlineSpan>[
                  TextSpan(
                    text: 'Second line!\n',
                    style: TextStyle(fontSize: 5, color: Colors.blue),
                  ),
                  TextSpan(
                    text: 'Third line!\n',
                    style: TextStyle(fontSize: 25, color: Colors.white),
                  ),
                ],
              ),
              textDirection: TextDirection.ltr,
              strutStyle: StrutStyle(fontSize: 14, height: 1.1, leading: 0.1),
            ),
          ),
        ),
      ),
    );
    await expectLater(find.byType(Container), matchesGoldenFile('text_golden.Strut.3.png'));
  });

  testWidgets('Strut text font fallback', (WidgetTester tester) async {
    // Font Fallback
    await tester.pumpWidget(
      Center(
        child: RepaintBoundary(
          child: Container(
            width: 200.0,
            height: 100.0,
            decoration: const BoxDecoration(color: Color(0xff00ff00)),
            child: const Text(
              'Hello\nLine 2\nLine 3',
              textDirection: TextDirection.ltr,
              style: TextStyle(),
              strutStyle: StrutStyle(
                fontFamily: 'FakeFont 1',
                fontFamilyFallback: <String>['FakeFont 2', 'EvilFont 3', 'Nice Font 4', 'ahem'],
                fontSize: 14,
              ),
            ),
          ),
        ),
      ),
    );
    await expectLater(find.byType(Container), matchesGoldenFile('text_golden.Strut.4.png'));
  });

  testWidgets('Strut text rich forceStrutHeight', (WidgetTester tester) async {
    await tester.pumpWidget(
      Center(
        child: RepaintBoundary(
          child: Container(
            width: 200.0,
            height: 100.0,
            decoration: const BoxDecoration(color: Color(0xff00ff00)),
            child: const Text.rich(
              TextSpan(
                text: 'Hello\n',
                style: TextStyle(color: Colors.red, fontSize: 30),
                children: <InlineSpan>[
                  TextSpan(
                    text: 'Second line!\n',
                    style: TextStyle(fontSize: 9, color: Colors.blue),
                  ),
                  TextSpan(
                    text: 'Third line!\n',
                    style: TextStyle(fontSize: 27, color: Colors.white),
                  ),
                ],
              ),
              textDirection: TextDirection.ltr,
              strutStyle: StrutStyle(fontSize: 14, height: 1.1, forceStrutHeight: true),
            ),
          ),
        ),
      ),
    );
    await expectLater(find.byType(Container), matchesGoldenFile('text_golden.StrutForce.1.png'));
  });

  testWidgets('Decoration thickness', (WidgetTester tester) async {
    final TextDecoration allDecorations = TextDecoration.combine(<TextDecoration>[
      TextDecoration.underline,
      TextDecoration.overline,
      TextDecoration.lineThrough,
    ]);

    await tester.pumpWidget(
      Center(
        child: RepaintBoundary(
          child: Container(
            width: 300.0,
            height: 100.0,
            decoration: const BoxDecoration(color: Color(0xff00ff00)),
            child: Text(
              'Hello, wor!\nabcd.',
              style: TextStyle(
                fontSize: 25,
                decoration: allDecorations,
                decorationColor: Colors.blue,
                decorationStyle: TextDecorationStyle.dashed,
              ),
              textDirection: TextDirection.rtl,
            ),
          ),
        ),
      ),
    );
    await expectLater(find.byType(Container), matchesGoldenFile('text_golden.Decoration.1.png'));
  });

  testWidgets('Decoration thickness', (WidgetTester tester) async {
    final TextDecoration allDecorations = TextDecoration.combine(<TextDecoration>[
      TextDecoration.underline,
      TextDecoration.overline,
      TextDecoration.lineThrough,
    ]);

    await tester.pumpWidget(
      Center(
        child: RepaintBoundary(
          child: Container(
            width: 300.0,
            height: 100.0,
            decoration: const BoxDecoration(color: Color(0xff00ff00)),
            child: Text(
              'Hello, wor!\nabcd.',
              style: TextStyle(
                fontSize: 25,
                decoration: allDecorations,
                decorationColor: Colors.blue,
                decorationStyle: TextDecorationStyle.wavy,
                decorationThickness: 4,
              ),
              textDirection: TextDirection.rtl,
            ),
          ),
        ),
      ),
    );
    await expectLater(
      find.byType(Container),
      matchesGoldenFile('text_golden.DecorationThickness.1.png'),
    );
  });

  testWidgets('Text Inline widget', (WidgetTester tester) async {
    await tester.pumpWidget(
      Theme(
        data: ThemeData(useMaterial3: false),
        child: Center(
          child: RepaintBoundary(
            child: Material(
              child: Directionality(
                textDirection: TextDirection.ltr,
                child: Container(
                  width: 400.0,
                  height: 200.0,
                  decoration: const BoxDecoration(color: Color(0xff00ff00)),
                  child: ConstrainedBox(
                    constraints: const BoxConstraints(maxWidth: 200, maxHeight: 100),
                    child: const Text.rich(
                      TextSpan(
                        text: 'C ',
                        style: TextStyle(fontSize: 16),
                        children: <InlineSpan>[
                          WidgetSpan(child: Checkbox(value: true, onChanged: null)),
                          WidgetSpan(child: Checkbox(value: false, onChanged: null)),
                          TextSpan(text: 'He ', style: TextStyle(fontSize: 20)),
                          WidgetSpan(
                            child: SizedBox(
                              width: 50.0,
                              height: 55.0,
                              child: DecoratedBox(
                                decoration: BoxDecoration(color: Color(0xffffff00)),
                                child: Center(
                                  child: SizedBox(
                                    width: 10.0,
                                    height: 15.0,
                                    child: DecoratedBox(
                                      decoration: BoxDecoration(color: Color(0xffff0000)),
                                    ),
                                  ),
                                ),
                              ),
                            ),
                          ),
                          TextSpan(text: 'hello world! seize the day!'),
                          WidgetSpan(child: Checkbox(value: false, onChanged: null)),
                          WidgetSpan(
                            child: SizedBox(
                              width: 20,
                              height: 20,
                              child: Checkbox(value: true, onChanged: null),
                            ),
                          ),
                          WidgetSpan(
                            child: Checkbox(value: false, onChanged: null),
                            alignment: PlaceholderAlignment.baseline,
                            baseline: TextBaseline.alphabetic,
                          ),
                          WidgetSpan(
                            child: SizedBox(
                              width: 20,
                              height: 20,
                              child: Checkbox(value: true, onChanged: null),
                            ),
                          ),
                          WidgetSpan(child: Text('embedded')),
                        ],
                      ),
                      textDirection: TextDirection.ltr,
                    ),
                  ),
                ),
              ),
            ),
          ),
        ),
      ),
    );
    await expectLater(
      find.byType(Container),
      matchesGoldenFile('text_golden.TextInlineWidget.1.png'),
    );
  });

  testWidgets('Text Inline widget textfield', (WidgetTester tester) async {
    await tester.pumpWidget(
      Center(
        child: MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: RepaintBoundary(
            child: Material(
              child: Container(
                width: 400.0,
                height: 200.0,
                decoration: const BoxDecoration(color: Color(0xff00ff00)),
                child: ConstrainedBox(
                  constraints: const BoxConstraints(maxWidth: 200, maxHeight: 100),
                  child: const Text.rich(
                    TextSpan(
                      text: 'My name is: ',
                      style: TextStyle(fontSize: 20),
                      children: <InlineSpan>[
                        WidgetSpan(child: SizedBox(width: 70, height: 25, child: TextField())),
                        TextSpan(
                          text: ', and my favorite city is: ',
                          style: TextStyle(fontSize: 20),
                        ),
                        WidgetSpan(child: SizedBox(width: 70, height: 25, child: TextField())),
                      ],
                    ),
                    textDirection: TextDirection.ltr,
                  ),
                ),
              ),
            ),
          ),
        ),
      ),
    );
    await expectLater(
      find.byType(Container),
      matchesGoldenFile('text_golden.TextInlineWidget.2.png'),
    );
  });

  // This tests if multiple Text.rich widgets are able to inline nest within each other.
  testWidgets('Text Inline widget nesting', (WidgetTester tester) async {
    await tester.pumpWidget(
      Center(
        child: MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: RepaintBoundary(
            child: Material(
              child: Container(
                width: 400.0,
                height: 200.0,
                decoration: const BoxDecoration(color: Color(0xff00ff00)),
                child: ConstrainedBox(
                  constraints: const BoxConstraints(maxWidth: 200, maxHeight: 100),
                  child: const Text.rich(
                    TextSpan(
                      text: 'outer',
                      style: TextStyle(fontSize: 20),
                      children: <InlineSpan>[
                        WidgetSpan(
                          child: Text.rich(
                            TextSpan(
                              text: 'inner',
                              style: TextStyle(color: Color(0xf402f4ff)),
                              children: <InlineSpan>[
                                WidgetSpan(
                                  child: Text.rich(
                                    TextSpan(
                                      text: 'inner2',
                                      style: TextStyle(color: Color(0xf003ffff)),
                                      children: <InlineSpan>[
                                        WidgetSpan(
                                          child: SizedBox(
                                            width: 50.0,
                                            height: 55.0,
                                            child: DecoratedBox(
                                              decoration: BoxDecoration(color: Color(0xffffff30)),
                                              child: Center(
                                                child: SizedBox(
                                                  width: 10.0,
                                                  height: 15.0,
                                                  child: DecoratedBox(
                                                    decoration: BoxDecoration(
                                                      color: Color(0xff5f00f0),
                                                    ),
                                                  ),
                                                ),
                                              ),
                                            ),
                                          ),
                                        ),
                                      ],
                                    ),
                                  ),
                                ),
                                WidgetSpan(
                                  child: SizedBox(
                                    width: 50.0,
                                    height: 55.0,
                                    child: DecoratedBox(
                                      decoration: BoxDecoration(color: Color(0xff5fff00)),
                                      child: Center(
                                        child: SizedBox(
                                          width: 10.0,
                                          height: 15.0,
                                          child: DecoratedBox(
                                            decoration: BoxDecoration(color: Color(0xff5f0000)),
                                          ),
                                        ),
                                      ),
                                    ),
                                  ),
                                ),
                              ],
                            ),
                          ),
                        ),
                        TextSpan(text: 'outer', style: TextStyle(fontSize: 20)),
                        WidgetSpan(child: SizedBox(width: 70, height: 25, child: TextField())),
                        WidgetSpan(
                          child: SizedBox(
                            width: 50.0,
                            height: 55.0,
                            child: DecoratedBox(
                              decoration: BoxDecoration(color: Color(0xffff00ff)),
                              child: Center(
                                child: SizedBox(
                                  width: 10.0,
                                  height: 15.0,
                                  child: DecoratedBox(
                                    decoration: BoxDecoration(color: Color(0xff0000ff)),
                                  ),
                                ),
                              ),
                            ),
                          ),
                        ),
                      ],
                    ),
                    textDirection: TextDirection.ltr,
                  ),
                ),
              ),
            ),
          ),
        ),
      ),
    );
    await expectLater(
      find.byType(Container),
      matchesGoldenFile('text_golden.TextInlineWidgetNest.1.png'),
    );
  });

  testWidgets('Text Inline widget baseline', (WidgetTester tester) async {
    await tester.pumpWidget(
      Theme(
        data: ThemeData(useMaterial3: false),
        child: Center(
          child: RepaintBoundary(
            child: Material(
              child: Directionality(
                textDirection: TextDirection.ltr,
                child: Container(
                  width: 400.0,
                  height: 200.0,
                  decoration: const BoxDecoration(color: Color(0xff00ff00)),
                  child: ConstrainedBox(
                    constraints: const BoxConstraints(maxWidth: 200, maxHeight: 100),
                    child: const Text.rich(
                      TextSpan(
                        text: 'C ',
                        style: TextStyle(fontSize: 16),
                        children: <InlineSpan>[
                          WidgetSpan(
                            alignment: PlaceholderAlignment.baseline,
                            baseline: TextBaseline.alphabetic,
                            child: Checkbox(value: true, onChanged: null),
                          ),
                          WidgetSpan(child: Checkbox(value: false, onChanged: null)),
                          TextSpan(text: 'He ', style: TextStyle(fontSize: 20)),
                          WidgetSpan(
                            alignment: PlaceholderAlignment.baseline,
                            baseline: TextBaseline.alphabetic,
                            child: SizedBox(
                              width: 50.0,
                              height: 55.0,
                              child: DecoratedBox(
                                decoration: BoxDecoration(color: Color(0xffffff00)),
                                child: Center(
                                  child: SizedBox(
                                    width: 10.0,
                                    height: 15.0,
                                    child: DecoratedBox(
                                      decoration: BoxDecoration(color: Color(0xffff0000)),
                                    ),
                                  ),
                                ),
                              ),
                            ),
                          ),
                          TextSpan(text: 'hello world! seize the day!'),
                          WidgetSpan(
                            alignment: PlaceholderAlignment.baseline,
                            baseline: TextBaseline.alphabetic,
                            child: Checkbox(value: false, onChanged: null),
                          ),
                          WidgetSpan(
                            alignment: PlaceholderAlignment.baseline,
                            baseline: TextBaseline.alphabetic,
                            child: SizedBox(
                              width: 20,
                              height: 20,
                              child: Checkbox(value: true, onChanged: null),
                            ),
                          ),
                          WidgetSpan(
                            alignment: PlaceholderAlignment.baseline,
                            baseline: TextBaseline.alphabetic,
                            child: Checkbox(value: false, onChanged: null),
                          ),
                          WidgetSpan(
                            alignment: PlaceholderAlignment.baseline,
                            baseline: TextBaseline.alphabetic,
                            child: SizedBox(
                              width: 20,
                              height: 20,
                              child: Checkbox(value: true, onChanged: null),
                            ),
                          ),
                          WidgetSpan(
                            alignment: PlaceholderAlignment.baseline,
                            baseline: TextBaseline.alphabetic,
                            child: Text('embedded'),
                          ),
                          TextSpan(text: 'ref'),
                        ],
                      ),
                      textDirection: TextDirection.ltr,
                    ),
                  ),
                ),
              ),
            ),
          ),
        ),
      ),
    );
    await expectLater(
      find.byType(Container),
      matchesGoldenFile('text_golden.TextInlineWidgetBaseline.1.png'),
    );
  });

  testWidgets('Text Inline widget aboveBaseline', (WidgetTester tester) async {
    await tester.pumpWidget(
      Theme(
        data: ThemeData(useMaterial3: false),
        child: Center(
          child: RepaintBoundary(
            child: Material(
              child: Directionality(
                textDirection: TextDirection.ltr,
                child: Container(
                  width: 400.0,
                  height: 200.0,
                  decoration: const BoxDecoration(color: Color(0xff00ff00)),
                  child: ConstrainedBox(
                    constraints: const BoxConstraints(maxWidth: 200, maxHeight: 100),
                    child: const Text.rich(
                      TextSpan(
                        text: 'C ',
                        style: TextStyle(fontSize: 16),
                        children: <InlineSpan>[
                          WidgetSpan(
                            alignment: PlaceholderAlignment.aboveBaseline,
                            baseline: TextBaseline.alphabetic,
                            child: Checkbox(value: true, onChanged: null),
                          ),
                          WidgetSpan(child: Checkbox(value: false, onChanged: null)),
                          TextSpan(text: 'He ', style: TextStyle(fontSize: 20)),
                          WidgetSpan(
                            alignment: PlaceholderAlignment.aboveBaseline,
                            baseline: TextBaseline.alphabetic,
                            child: SizedBox(
                              width: 50.0,
                              height: 55.0,
                              child: DecoratedBox(
                                decoration: BoxDecoration(color: Color(0xffffff00)),
                                child: Center(
                                  child: SizedBox(
                                    width: 10.0,
                                    height: 15.0,
                                    child: DecoratedBox(
                                      decoration: BoxDecoration(color: Color(0xffff0000)),
                                    ),
                                  ),
                                ),
                              ),
                            ),
                          ),
                          TextSpan(text: 'hello world! seize the day!'),
                          WidgetSpan(
                            alignment: PlaceholderAlignment.aboveBaseline,
                            baseline: TextBaseline.alphabetic,
                            child: Checkbox(value: false, onChanged: null),
                          ),
                          WidgetSpan(
                            alignment: PlaceholderAlignment.aboveBaseline,
                            baseline: TextBaseline.alphabetic,
                            child: SizedBox(
                              width: 20,
                              height: 20,
                              child: Checkbox(value: true, onChanged: null),
                            ),
                          ),
                          WidgetSpan(
                            alignment: PlaceholderAlignment.aboveBaseline,
                            baseline: TextBaseline.alphabetic,
                            child: Checkbox(value: false, onChanged: null),
                          ),
                          WidgetSpan(
                            alignment: PlaceholderAlignment.aboveBaseline,
                            baseline: TextBaseline.alphabetic,
                            child: SizedBox(
                              width: 20,
                              height: 20,
                              child: Checkbox(value: true, onChanged: null),
                            ),
                          ),
                          WidgetSpan(
                            alignment: PlaceholderAlignment.aboveBaseline,
                            baseline: TextBaseline.alphabetic,
                            child: Text('embedded'),
                          ),
                          TextSpan(text: 'ref'),
                        ],
                      ),
                      textDirection: TextDirection.ltr,
                    ),
                  ),
                ),
              ),
            ),
          ),
        ),
      ),
    );
    await expectLater(
      find.byType(Container),
      matchesGoldenFile('text_golden.TextInlineWidgetAboveBaseline.1.png'),
    );
  });

  testWidgets('Text Inline widget belowBaseline', (WidgetTester tester) async {
    await tester.pumpWidget(
      Theme(
        data: ThemeData(useMaterial3: false),
        child: Center(
          child: RepaintBoundary(
            child: Material(
              child: Directionality(
                textDirection: TextDirection.ltr,
                child: Container(
                  width: 400.0,
                  height: 200.0,
                  decoration: const BoxDecoration(color: Color(0xff00ff00)),
                  child: ConstrainedBox(
                    constraints: const BoxConstraints(maxWidth: 200, maxHeight: 100),
                    child: const Text.rich(
                      TextSpan(
                        text: 'C ',
                        style: TextStyle(fontSize: 16),
                        children: <InlineSpan>[
                          WidgetSpan(
                            alignment: PlaceholderAlignment.belowBaseline,
                            baseline: TextBaseline.alphabetic,
                            child: Checkbox(value: true, onChanged: null),
                          ),
                          WidgetSpan(child: Checkbox(value: false, onChanged: null)),
                          TextSpan(text: 'He ', style: TextStyle(fontSize: 20)),
                          WidgetSpan(
                            alignment: PlaceholderAlignment.belowBaseline,
                            baseline: TextBaseline.alphabetic,
                            child: SizedBox(
                              width: 50.0,
                              height: 55.0,
                              child: DecoratedBox(
                                decoration: BoxDecoration(color: Color(0xffffff00)),
                                child: Center(
                                  child: SizedBox(
                                    width: 10.0,
                                    height: 15.0,
                                    child: DecoratedBox(
                                      decoration: BoxDecoration(color: Color(0xffff0000)),
                                    ),
                                  ),
                                ),
                              ),
                            ),
                          ),
                          TextSpan(text: 'hello world! seize the day!'),
                          WidgetSpan(
                            alignment: PlaceholderAlignment.belowBaseline,
                            baseline: TextBaseline.alphabetic,
                            child: Checkbox(value: false, onChanged: null),
                          ),
                          WidgetSpan(
                            alignment: PlaceholderAlignment.belowBaseline,
                            baseline: TextBaseline.alphabetic,
                            child: SizedBox(
                              width: 20,
                              height: 20,
                              child: Checkbox(value: true, onChanged: null),
                            ),
                          ),
                          WidgetSpan(
                            alignment: PlaceholderAlignment.belowBaseline,
                            baseline: TextBaseline.alphabetic,
                            child: Checkbox(value: false, onChanged: null),
                          ),
                          WidgetSpan(
                            alignment: PlaceholderAlignment.belowBaseline,
                            baseline: TextBaseline.alphabetic,
                            child: SizedBox(
                              width: 20,
                              height: 20,
                              child: Checkbox(value: true, onChanged: null),
                            ),
                          ),
                          WidgetSpan(
                            alignment: PlaceholderAlignment.belowBaseline,
                            baseline: TextBaseline.alphabetic,
                            child: Text('embedded'),
                          ),
                          TextSpan(text: 'ref'),
                        ],
                      ),
                      textDirection: TextDirection.ltr,
                    ),
                  ),
                ),
              ),
            ),
          ),
        ),
      ),
    );
    await expectLater(
      find.byType(Container),
      matchesGoldenFile('text_golden.TextInlineWidgetBelowBaseline.1.png'),
    );
  });

  testWidgets('Text Inline widget top', (WidgetTester tester) async {
    await tester.pumpWidget(
      Theme(
        data: ThemeData(useMaterial3: false),
        child: Center(
          child: RepaintBoundary(
            child: Material(
              child: Directionality(
                textDirection: TextDirection.ltr,
                child: Container(
                  width: 400.0,
                  height: 200.0,
                  decoration: const BoxDecoration(color: Color(0xff00ff00)),
                  child: ConstrainedBox(
                    constraints: const BoxConstraints(maxWidth: 200, maxHeight: 100),
                    child: const Text.rich(
                      TextSpan(
                        text: 'C ',
                        style: TextStyle(fontSize: 16),
                        children: <InlineSpan>[
                          WidgetSpan(
                            alignment: PlaceholderAlignment.top,
                            baseline: TextBaseline.alphabetic,
                            child: Checkbox(value: true, onChanged: null),
                          ),
                          WidgetSpan(child: Checkbox(value: false, onChanged: null)),
                          TextSpan(text: 'He ', style: TextStyle(fontSize: 20)),
                          WidgetSpan(
                            alignment: PlaceholderAlignment.top,
                            baseline: TextBaseline.alphabetic,
                            child: SizedBox(
                              width: 50.0,
                              height: 55.0,
                              child: DecoratedBox(
                                decoration: BoxDecoration(color: Color(0xffffff00)),
                                child: Center(
                                  child: SizedBox(
                                    width: 10.0,
                                    height: 15.0,
                                    child: DecoratedBox(
                                      decoration: BoxDecoration(color: Color(0xffff0000)),
                                    ),
                                  ),
                                ),
                              ),
                            ),
                          ),
                          TextSpan(text: 'hello world! seize the day!'),
                          WidgetSpan(
                            alignment: PlaceholderAlignment.top,
                            baseline: TextBaseline.alphabetic,
                            child: Checkbox(value: false, onChanged: null),
                          ),
                          WidgetSpan(
                            alignment: PlaceholderAlignment.top,
                            baseline: TextBaseline.alphabetic,
                            child: SizedBox(
                              width: 20,
                              height: 20,
                              child: Checkbox(value: true, onChanged: null),
                            ),
                          ),
                          WidgetSpan(
                            alignment: PlaceholderAlignment.top,
                            baseline: TextBaseline.alphabetic,
                            child: Checkbox(value: false, onChanged: null),
                          ),
                          WidgetSpan(
                            alignment: PlaceholderAlignment.top,
                            baseline: TextBaseline.alphabetic,
                            child: SizedBox(
                              width: 20,
                              height: 20,
                              child: Checkbox(value: true, onChanged: null),
                            ),
                          ),
                          WidgetSpan(
                            alignment: PlaceholderAlignment.top,
                            baseline: TextBaseline.alphabetic,
                            child: Text('embedded'),
                          ),
                          TextSpan(text: 'ref'),
                        ],
                      ),
                      textDirection: TextDirection.ltr,
                    ),
                  ),
                ),
              ),
            ),
          ),
        ),
      ),
    );
    await expectLater(
      find.byType(Container),
      matchesGoldenFile('text_golden.TextInlineWidgetTop.1.png'),
    );
  });

  testWidgets('Text Inline widget middle', (WidgetTester tester) async {
    await tester.pumpWidget(
      Theme(
        data: ThemeData(useMaterial3: false),
        child: Center(
          child: RepaintBoundary(
            child: Material(
              child: Directionality(
                textDirection: TextDirection.ltr,
                child: Container(
                  width: 400.0,
                  height: 200.0,
                  decoration: const BoxDecoration(color: Color(0xff00ff00)),
                  child: ConstrainedBox(
                    constraints: const BoxConstraints(maxWidth: 200, maxHeight: 100),
                    child: const Text.rich(
                      TextSpan(
                        text: 'C ',
                        style: TextStyle(fontSize: 16),
                        children: <InlineSpan>[
                          WidgetSpan(
                            alignment: PlaceholderAlignment.middle,
                            baseline: TextBaseline.alphabetic,
                            child: Checkbox(value: true, onChanged: null),
                          ),
                          WidgetSpan(child: Checkbox(value: false, onChanged: null)),
                          TextSpan(text: 'He ', style: TextStyle(fontSize: 20)),
                          WidgetSpan(
                            alignment: PlaceholderAlignment.middle,
                            baseline: TextBaseline.alphabetic,
                            child: SizedBox(
                              width: 50.0,
                              height: 55.0,
                              child: DecoratedBox(
                                decoration: BoxDecoration(color: Color(0xffffff00)),
                                child: Center(
                                  child: SizedBox(
                                    width: 10.0,
                                    height: 15.0,
                                    child: DecoratedBox(
                                      decoration: BoxDecoration(color: Color(0xffff0000)),
                                    ),
                                  ),
                                ),
                              ),
                            ),
                          ),
                          TextSpan(text: 'hello world! seize the day!'),
                          WidgetSpan(
                            alignment: PlaceholderAlignment.middle,
                            baseline: TextBaseline.alphabetic,
                            child: Checkbox(value: false, onChanged: null),
                          ),
                          WidgetSpan(
                            alignment: PlaceholderAlignment.middle,
                            baseline: TextBaseline.alphabetic,
                            child: SizedBox(
                              width: 20,
                              height: 20,
                              child: Checkbox(value: true, onChanged: null),
                            ),
                          ),
                          WidgetSpan(
                            alignment: PlaceholderAlignment.middle,
                            baseline: TextBaseline.alphabetic,
                            child: Checkbox(value: false, onChanged: null),
                          ),
                          WidgetSpan(
                            alignment: PlaceholderAlignment.middle,
                            baseline: TextBaseline.alphabetic,
                            child: SizedBox(
                              width: 20,
                              height: 20,
                              child: Checkbox(value: true, onChanged: null),
                            ),
                          ),
                          WidgetSpan(
                            alignment: PlaceholderAlignment.middle,
                            baseline: TextBaseline.alphabetic,
                            child: Text('embedded'),
                          ),
                          TextSpan(text: 'ref'),
                        ],
                      ),
                      textDirection: TextDirection.ltr,
                    ),
                  ),
                ),
              ),
            ),
          ),
        ),
      ),
    );
    await expectLater(
      find.byType(Container),
      matchesGoldenFile('text_golden.TextInlineWidgetMiddle.1.png'),
    );
  });

  testWidgets('Text TextHeightBehavior', (WidgetTester tester) async {
    await tester.pumpWidget(
      Center(
        child: RepaintBoundary(
          child: Container(
            width: 200.0,
            height: 700.0,
            decoration: const BoxDecoration(color: Color(0xff00ff00)),
            child: const Column(
              children: <Widget>[
                Text(
                  'Hello\nLine 2\nLine 3',
                  textDirection: TextDirection.ltr,
                  style: TextStyle(height: 5),
                ),
                Text(
                  'Hello\nLine 2\nLine 3',
                  textDirection: TextDirection.ltr,
                  style: TextStyle(height: 5),
                  textHeightBehavior: TextHeightBehavior(
                    applyHeightToFirstAscent: false,
                    applyHeightToLastDescent: false,
                  ),
                ),
                Text(
                  'Hello',
                  textDirection: TextDirection.ltr,
                  style: TextStyle(height: 5),
                  textHeightBehavior: TextHeightBehavior(applyHeightToFirstAscent: false),
                ),
              ],
            ),
          ),
        ),
      ),
    );
    await expectLater(
      find.byType(Container),
      matchesGoldenFile('text_golden.TextHeightBehavior.1.png'),
    );
  });
}
