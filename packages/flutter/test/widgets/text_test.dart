// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' as ui;

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:leak_tracker_flutter_testing/leak_tracker_flutter_testing.dart';

import 'semantics_tester.dart';

void main() {
  testWidgets('DefaultTextStyle.merge correctly merges arguments', (WidgetTester tester) async {
    DefaultTextStyle defaultTextStyle = const DefaultTextStyle.fallback();

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: DefaultTextStyle(
          style: const TextStyle(color: Colors.black, fontSize: 20),
          textAlign: TextAlign.left,
          softWrap: false,
          overflow: TextOverflow.ellipsis,
          maxLines: 2,
          textWidthBasis: TextWidthBasis.longestLine,
          textHeightBehavior: const TextHeightBehavior(applyHeightToFirstAscent: false),
          child: DefaultTextStyle.merge(
            style: const TextStyle(color: Colors.red, fontWeight: FontWeight.bold),
            textAlign: TextAlign.center,
            softWrap: true,
            overflow: TextOverflow.fade,
            maxLines: 3,
            textWidthBasis: TextWidthBasis.parent,
            textHeightBehavior: const TextHeightBehavior(applyHeightToLastDescent: false),
            child: Builder(
              builder: (BuildContext context) {
                defaultTextStyle = DefaultTextStyle.of(context);
                return const Text('Text');
              },
            ),
          ),
        ),
      ),
    );

    expect(
      defaultTextStyle.style,
      const TextStyle(color: Colors.red, fontSize: 20, fontWeight: FontWeight.bold),
    );
    expect(defaultTextStyle.textAlign, TextAlign.center);
    expect(defaultTextStyle.softWrap, true);
    expect(defaultTextStyle.overflow, TextOverflow.fade);
    expect(defaultTextStyle.maxLines, 3);
    expect(defaultTextStyle.textWidthBasis, TextWidthBasis.parent);
    expect(
      defaultTextStyle.textHeightBehavior,
      const TextHeightBehavior(applyHeightToLastDescent: false),
    );
  });

  testWidgets('Text respects media query', (WidgetTester tester) async {
    await tester.pumpWidget(
      MediaQuery.withClampedTextScaling(
        minScaleFactor: 1.3,
        maxScaleFactor: 1.3,
        child: const Center(child: Text('Hello', textDirection: TextDirection.ltr)),
      ),
    );

    RichText text = tester.firstWidget(find.byType(RichText));
    expect(text, isNotNull);
    expect(text.textScaler, const TextScaler.linear(1.3));

    await tester.pumpWidget(const Center(child: Text('Hello', textDirection: TextDirection.ltr)));

    text = tester.firstWidget(find.byType(RichText));
    expect(text, isNotNull);
    expect(text.textScaler, TextScaler.noScaling);
  });

  testWidgets('Text respects textScaleFactor with default font size', (WidgetTester tester) async {
    await tester.pumpWidget(const Center(child: Text('Hello', textDirection: TextDirection.ltr)));

    RichText text = tester.firstWidget(find.byType(RichText));
    expect(text, isNotNull);
    expect(text.textScaler, TextScaler.noScaling);
    final Size baseSize = tester.getSize(find.byType(RichText));
    expect(baseSize.width, equals(70.0));
    expect(baseSize.height, equals(14.0));

    await tester.pumpWidget(
      const Center(child: Text('Hello', textScaleFactor: 1.5, textDirection: TextDirection.ltr)),
    );

    text = tester.firstWidget(find.byType(RichText));
    expect(text, isNotNull);
    expect(text.textScaler, const TextScaler.linear(1.5));
    final Size largeSize = tester.getSize(find.byType(RichText));
    expect(largeSize.width, 105.0);
    expect(largeSize.height, equals(21.0));
  });

  testWidgets('Text respects textScaleFactor with explicit font size', (WidgetTester tester) async {
    await tester.pumpWidget(
      const Center(
        child: Text('Hello', style: TextStyle(fontSize: 20.0), textDirection: TextDirection.ltr),
      ),
    );

    RichText text = tester.firstWidget(find.byType(RichText));
    expect(text, isNotNull);
    expect(text.textScaler, TextScaler.noScaling);
    final Size baseSize = tester.getSize(find.byType(RichText));
    expect(baseSize.width, equals(100.0));
    expect(baseSize.height, equals(20.0));

    await tester.pumpWidget(
      const Center(
        child: Text(
          'Hello',
          style: TextStyle(fontSize: 20.0),
          textScaleFactor: 1.3,
          textDirection: TextDirection.ltr,
        ),
      ),
    );

    text = tester.firstWidget(find.byType(RichText));
    expect(text, isNotNull);
    expect(text.textScaler, const TextScaler.linear(1.3));
    final Size largeSize = tester.getSize(find.byType(RichText));
    expect(largeSize.width, 130.0);
    expect(largeSize.height, equals(26.0));
  });

  testWidgets(
    "Text throws a nice error message if there's no Directionality",
    experimentalLeakTesting:
        LeakTesting.settings.withIgnoredAll(), // leaking by design because of exception
    (WidgetTester tester) async {
      await tester.pumpWidget(const Text('Hello'));
      final String message = tester.takeException().toString();
      expect(message, contains('Directionality'));
      expect(message, contains(' Text '));
    },
  );

  testWidgets('Text can be created from TextSpans and uses defaultTextStyle', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      const DefaultTextStyle(
        style: TextStyle(fontSize: 20.0),
        child: Text.rich(
          TextSpan(
            text: 'Hello',
            children: <TextSpan>[
              TextSpan(text: ' beautiful ', style: TextStyle(fontStyle: FontStyle.italic)),
              TextSpan(text: 'world', style: TextStyle(fontWeight: FontWeight.bold)),
            ],
          ),
          textDirection: TextDirection.ltr,
        ),
      ),
    );

    final RichText text = tester.firstWidget(find.byType(RichText));
    expect(text, isNotNull);
    expect(text.text.style!.fontSize, 20.0);
  });

  testWidgets('inline widgets works with ellipsis', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/35869
    const TextStyle textStyle = TextStyle();
    await tester.pumpWidget(
      Text.rich(
        TextSpan(
          children: <InlineSpan>[
            const TextSpan(text: 'a very very very very very very very very very very long line'),
            WidgetSpan(
              child: SizedBox(
                width: 20,
                height: 40,
                child: Card(
                  child: RichText(
                    text: const TextSpan(text: 'widget should be truncated'),
                    textDirection: TextDirection.rtl,
                  ),
                ),
              ),
            ),
          ],
          style: textStyle,
        ),
        textDirection: TextDirection.ltr,
        maxLines: 1,
        overflow: TextOverflow.ellipsis,
      ),
    );
    expect(tester.takeException(), null);
  });

  testWidgets('inline widgets hitTest works with ellipsis', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/68559
    const TextStyle textStyle = TextStyle();
    await tester.pumpWidget(
      Text.rich(
        TextSpan(
          children: <InlineSpan>[
            const TextSpan(text: 'a very very very very very very very very very very long line'),
            WidgetSpan(
              child: SizedBox(
                width: 20,
                height: 40,
                child: Card(
                  child: RichText(
                    text: const TextSpan(text: 'widget should be truncated'),
                    textDirection: TextDirection.rtl,
                  ),
                ),
              ),
            ),
          ],
          style: textStyle,
        ),
        textDirection: TextDirection.ltr,
        maxLines: 1,
        overflow: TextOverflow.ellipsis,
      ),
    );

    await tester.tap(find.byType(Text));

    expect(tester.takeException(), null);
  });

  testWidgets('inline widgets works with textScaleFactor', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/59316
    final UniqueKey key = UniqueKey();
    double textScaleFactor = 1.0;
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Scaffold(
          appBar: AppBar(title: const Text('title')),
          body: Center(
            child: Text.rich(
              TextSpan(
                children: <InlineSpan>[
                  WidgetSpan(
                    child: RichText(
                      text: const TextSpan(text: 'widget should be truncated'),
                      textDirection: TextDirection.ltr,
                    ),
                  ),
                ],
              ),
              key: key,
              textDirection: TextDirection.ltr,
              textScaleFactor: textScaleFactor,
            ),
          ),
        ),
      ),
    );
    RenderBox renderText = tester.renderObject(find.byKey(key));
    final double singleLineHeight = renderText.size.height;
    // Now, increases the text scale factor by 5 times.
    textScaleFactor = textScaleFactor * 5;
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Scaffold(
          appBar: AppBar(title: const Text('title')),
          body: Center(
            child: Text.rich(
              TextSpan(
                children: <InlineSpan>[
                  WidgetSpan(
                    child: RichText(
                      text: const TextSpan(text: 'widget should be truncated'),
                      textDirection: TextDirection.ltr,
                    ),
                  ),
                ],
              ),
              key: key,
              textDirection: TextDirection.ltr,
              textScaleFactor: textScaleFactor,
            ),
          ),
        ),
      ),
    );

    renderText = tester.renderObject(find.byKey(key));
    // The RichText in the widget span should wrap into three lines.
    expect(renderText.size.height, singleLineHeight * textScaleFactor * 3);
  });

  testWidgets("Inline widgets' scaled sizes are constrained", (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/130588
    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: SizedBox(
            width: 502.5454545454545,
            child: Text.rich(WidgetSpan(child: Row()), textScaleFactor: 0.95),
          ),
        ),
      ),
    );

    expect(tester.takeException(), isNull);
  });

  testWidgets('semanticsLabel can override text label', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    await tester.pumpWidget(
      const Text(r'$$', semanticsLabel: 'Double dollars', textDirection: TextDirection.ltr),
    );
    final TestSemantics expectedSemantics = TestSemantics.root(
      children: <TestSemantics>[
        TestSemantics.rootChild(label: 'Double dollars', textDirection: TextDirection.ltr),
      ],
    );
    expect(
      semantics,
      hasSemantics(expectedSemantics, ignoreTransform: true, ignoreId: true, ignoreRect: true),
    );

    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: Text(r'$$', semanticsLabel: 'Double dollars'),
      ),
    );

    expect(
      semantics,
      hasSemantics(expectedSemantics, ignoreTransform: true, ignoreId: true, ignoreRect: true),
    );
    semantics.dispose();
  });

  testWidgets('semantics label is in order when uses widget span', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Text.rich(
          TextSpan(
            children: <InlineSpan>[
              const TextSpan(text: 'before '),
              WidgetSpan(
                alignment: PlaceholderAlignment.baseline,
                baseline: TextBaseline.alphabetic,
                child: Semantics(label: 'foo'),
              ),
              const TextSpan(text: ' after'),
            ],
          ),
        ),
      ),
    );
    expect(tester.getSemantics(find.byType(Text)), matchesSemantics(label: 'before \nfoo\n after'));

    // If the Paragraph is not dirty it should use the cache correctly.
    final RenderObject parent = tester.renderObject<RenderObject>(find.byType(Directionality));
    parent.markNeedsSemanticsUpdate();
    await tester.pumpAndSettle();

    expect(tester.getSemantics(find.byType(Text)), matchesSemantics(label: 'before \nfoo\n after'));
  });

  testWidgets('semantics can handle some widget spans without semantics', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Text.rich(
          TextSpan(
            children: <InlineSpan>[
              const TextSpan(text: 'before '),
              const WidgetSpan(
                alignment: PlaceholderAlignment.baseline,
                baseline: TextBaseline.alphabetic,
                child: SizedBox(width: 10.0),
              ),
              const TextSpan(text: ' mid'),
              WidgetSpan(
                alignment: PlaceholderAlignment.baseline,
                baseline: TextBaseline.alphabetic,
                child: Semantics(label: 'foo'),
              ),
              const TextSpan(text: ' after'),
              const WidgetSpan(
                alignment: PlaceholderAlignment.baseline,
                baseline: TextBaseline.alphabetic,
                child: SizedBox(width: 10.0),
              ),
            ],
          ),
        ),
      ),
    );
    expect(
      tester.getSemantics(find.byType(Text)),
      matchesSemantics(label: 'before \n mid\nfoo\n after'),
    );

    // If the Paragraph is not dirty it should use the cache correctly.
    final RenderObject parent = tester.renderObject<RenderObject>(find.byType(Directionality));
    parent.markNeedsSemanticsUpdate();
    await tester.pumpAndSettle();

    expect(
      tester.getSemantics(find.byType(Text)),
      matchesSemantics(label: 'before \n mid\nfoo\n after'),
    );
  });

  testWidgets('semantics can handle all widget spans without semantics', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: Text.rich(
          TextSpan(
            children: <InlineSpan>[
              TextSpan(text: 'before '),
              WidgetSpan(
                alignment: PlaceholderAlignment.baseline,
                baseline: TextBaseline.alphabetic,
                child: SizedBox(width: 10.0),
              ),
              TextSpan(text: ' mid'),
              WidgetSpan(
                alignment: PlaceholderAlignment.baseline,
                baseline: TextBaseline.alphabetic,
                child: SizedBox(width: 10.0),
              ),
              TextSpan(text: ' after'),
              WidgetSpan(
                alignment: PlaceholderAlignment.baseline,
                baseline: TextBaseline.alphabetic,
                child: SizedBox(width: 10.0),
              ),
            ],
          ),
        ),
      ),
    );
    expect(
      tester.getSemantics(find.byType(Text)),
      matchesSemantics(label: 'before \n mid\n after'),
    );

    // If the Paragraph is not dirty it should use the cache correctly.
    final RenderObject parent = tester.renderObject<RenderObject>(find.byType(Directionality));
    parent.markNeedsSemanticsUpdate();
    await tester.pumpAndSettle();

    expect(
      tester.getSemantics(find.byType(Text)),
      matchesSemantics(label: 'before \n mid\n after'),
    );
  });

  testWidgets('semantics can handle widget spans with explicit semantics node', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Text.rich(
          TextSpan(
            children: <InlineSpan>[
              const TextSpan(text: 'before '),
              WidgetSpan(
                alignment: PlaceholderAlignment.baseline,
                baseline: TextBaseline.alphabetic,
                child: Semantics(label: 'inner', container: true),
              ),
              const TextSpan(text: ' after'),
            ],
          ),
        ),
      ),
    );
    expect(
      tester.getSemantics(find.byType(Text)),
      matchesSemantics(
        label: 'before \n after',
        children: <Matcher>[matchesSemantics(label: 'inner')],
      ),
    );

    // If the Paragraph is not dirty it should use the cache correctly.
    final RenderObject parent = tester.renderObject<RenderObject>(find.byType(Directionality));
    parent.markNeedsSemanticsUpdate();
    await tester.pumpAndSettle();

    expect(
      tester.getSemantics(find.byType(Text)),
      matchesSemantics(
        label: 'before \n after',
        children: <Matcher>[matchesSemantics(label: 'inner')],
      ),
    );
  });

  testWidgets('semanticsLabel can be shorter than text', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    final TapGestureRecognizer recognizer = TapGestureRecognizer();
    addTearDown(recognizer.dispose);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: RichText(
          text: TextSpan(
            children: <InlineSpan>[
              const TextSpan(text: 'Some Text', semanticsLabel: ''),
              TextSpan(text: 'Clickable', recognizer: recognizer..onTap = () {}),
            ],
          ),
        ),
      ),
    );
    final TestSemantics expectedSemantics = TestSemantics.root(
      children: <TestSemantics>[
        TestSemantics(
          children: <TestSemantics>[
            TestSemantics(textDirection: TextDirection.ltr),
            TestSemantics(
              label: 'Clickable',
              actions: <SemanticsAction>[SemanticsAction.tap],
              flags: <SemanticsFlag>[SemanticsFlag.isLink],
              textDirection: TextDirection.ltr,
            ),
          ],
        ),
      ],
    );
    expect(
      semantics,
      hasSemantics(expectedSemantics, ignoreTransform: true, ignoreId: true, ignoreRect: true),
    );
    semantics.dispose();
  });

  testWidgets('recognizers split semantic node', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    const TextStyle textStyle = TextStyle();
    final TapGestureRecognizer recognizer = TapGestureRecognizer();
    addTearDown(recognizer.dispose);

    await tester.pumpWidget(
      Text.rich(
        TextSpan(
          children: <TextSpan>[
            const TextSpan(text: 'hello '),
            TextSpan(text: 'world', recognizer: recognizer..onTap = () {}),
            const TextSpan(text: ' this is a '),
            const TextSpan(text: 'cat-astrophe'),
          ],
          style: textStyle,
        ),
        textDirection: TextDirection.ltr,
      ),
    );
    final TestSemantics expectedSemantics = TestSemantics.root(
      children: <TestSemantics>[
        TestSemantics.rootChild(
          children: <TestSemantics>[
            TestSemantics(label: 'hello ', textDirection: TextDirection.ltr),
            TestSemantics(
              label: 'world',
              textDirection: TextDirection.ltr,
              actions: <SemanticsAction>[SemanticsAction.tap],
              flags: <SemanticsFlag>[SemanticsFlag.isLink],
            ),
            TestSemantics(label: ' this is a cat-astrophe', textDirection: TextDirection.ltr),
          ],
        ),
      ],
    );
    expect(
      semantics,
      hasSemantics(expectedSemantics, ignoreTransform: true, ignoreId: true, ignoreRect: true),
    );
    semantics.dispose();
  });

  testWidgets('semantic nodes of offscreen recognizers are marked hidden', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/100395.
    final SemanticsTester semantics = SemanticsTester(tester);
    const TextStyle textStyle = TextStyle(fontSize: 200);
    const String onScreenText = 'onscreen\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n';
    const String offScreenText = 'off screen';
    final ScrollController controller = ScrollController();
    addTearDown(controller.dispose);
    final TapGestureRecognizer recognizer = TapGestureRecognizer();
    addTearDown(recognizer.dispose);

    await tester.pumpWidget(
      SingleChildScrollView(
        controller: controller,
        child: Text.rich(
          TextSpan(
            children: <TextSpan>[
              const TextSpan(text: onScreenText),
              TextSpan(text: offScreenText, recognizer: recognizer..onTap = () {}),
            ],
            style: textStyle,
          ),
          textDirection: TextDirection.ltr,
        ),
      ),
    );

    final TestSemantics expectedSemantics = TestSemantics.root(
      children: <TestSemantics>[
        TestSemantics(
          flags: <SemanticsFlag>[SemanticsFlag.hasImplicitScrolling],
          actions: <SemanticsAction>[SemanticsAction.scrollUp, SemanticsAction.scrollToOffset],
          children: <TestSemantics>[
            TestSemantics(
              children: <TestSemantics>[
                TestSemantics(label: onScreenText, textDirection: TextDirection.ltr),
                TestSemantics(
                  label: offScreenText,
                  textDirection: TextDirection.ltr,
                  actions: <SemanticsAction>[SemanticsAction.tap],
                  flags: <SemanticsFlag>[SemanticsFlag.isLink, SemanticsFlag.isHidden],
                ),
              ],
            ),
          ],
        ),
      ],
    );
    expect(
      semantics,
      hasSemantics(expectedSemantics, ignoreTransform: true, ignoreId: true, ignoreRect: true),
    );

    // Test show on screen.
    expect(controller.offset, 0.0);
    tester.binding.pipelineOwner.semanticsOwner!.performAction(4, SemanticsAction.showOnScreen);
    await tester.pumpAndSettle();
    expect(controller.offset != 0.0, isTrue);

    semantics.dispose();
  });

  testWidgets('recognizers split semantic node when TextSpan overflows', (
    WidgetTester tester,
  ) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    const TextStyle textStyle = TextStyle();
    final TapGestureRecognizer recognizer = TapGestureRecognizer();
    addTearDown(recognizer.dispose);

    await tester.pumpWidget(
      SizedBox(
        height: 10,
        child: Text.rich(
          TextSpan(
            children: <TextSpan>[
              const TextSpan(text: '\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n'),
              TextSpan(text: 'world', recognizer: recognizer..onTap = () {}),
            ],
            style: textStyle,
          ),
          textDirection: TextDirection.ltr,
        ),
      ),
    );
    final TestSemantics expectedSemantics = TestSemantics.root(
      children: <TestSemantics>[
        TestSemantics.rootChild(
          children: <TestSemantics>[
            TestSemantics(
              label: '\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n',
              textDirection: TextDirection.ltr,
            ),
            TestSemantics(
              label: 'world',
              textDirection: TextDirection.ltr,
              actions: <SemanticsAction>[SemanticsAction.tap],
              flags: <SemanticsFlag>[SemanticsFlag.isLink],
            ),
          ],
        ),
      ],
    );
    expect(
      semantics,
      hasSemantics(expectedSemantics, ignoreTransform: true, ignoreId: true, ignoreRect: true),
    );
    semantics.dispose();
  });

  testWidgets('recognizers split semantic nodes with text span labels', (
    WidgetTester tester,
  ) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    const TextStyle textStyle = TextStyle();
    final TapGestureRecognizer recognizer = TapGestureRecognizer();
    addTearDown(recognizer.dispose);

    await tester.pumpWidget(
      Text.rich(
        TextSpan(
          children: <TextSpan>[
            const TextSpan(text: 'hello '),
            TextSpan(text: 'world', recognizer: recognizer..onTap = () {}),
            const TextSpan(text: ' this is a '),
            const TextSpan(text: 'cat-astrophe', semanticsLabel: 'regrettable event'),
          ],
          style: textStyle,
        ),
        textDirection: TextDirection.ltr,
      ),
    );
    final TestSemantics expectedSemantics = TestSemantics.root(
      children: <TestSemantics>[
        TestSemantics.rootChild(
          children: <TestSemantics>[
            TestSemantics(label: 'hello ', textDirection: TextDirection.ltr),
            TestSemantics(
              label: 'world',
              textDirection: TextDirection.ltr,
              actions: <SemanticsAction>[SemanticsAction.tap],
              flags: <SemanticsFlag>[SemanticsFlag.isLink],
            ),
            TestSemantics(label: ' this is a regrettable event', textDirection: TextDirection.ltr),
          ],
        ),
      ],
    );
    expect(
      semantics,
      hasSemantics(expectedSemantics, ignoreTransform: true, ignoreId: true, ignoreRect: true),
    );
    semantics.dispose();
  });

  testWidgets('recognizers split semantic node - bidi', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    const TextStyle textStyle = TextStyle();
    final LongPressGestureRecognizer recognizer1 = LongPressGestureRecognizer();
    addTearDown(recognizer1.dispose);
    final TapGestureRecognizer recognizer2 = TapGestureRecognizer();
    addTearDown(recognizer2.dispose);

    await tester.pumpWidget(
      RichText(
        text: TextSpan(
          style: textStyle,
          children: <TextSpan>[
            const TextSpan(text: 'hello world${Unicode.RLE}${Unicode.RLO} '),
            TextSpan(text: 'BOY', recognizer: recognizer1..onLongPress = () {}),
            const TextSpan(text: ' HOW DO${Unicode.PDF} you ${Unicode.RLO} DO '),
            TextSpan(text: 'SIR', recognizer: recognizer2..onTap = () {}),
            const TextSpan(text: '${Unicode.PDF}${Unicode.PDF} good bye'),
          ],
        ),
        textDirection: TextDirection.ltr,
      ),
    );
    // The expected visual order of the text is:
    //   hello world RIS OD you OD WOH YOB good bye
    // There are five unique text areas, they are, in visual order but
    // showing the logical text:
    //   [hello world][SIR][HOW DO you DO][BOY][good bye]
    // The direction of each varies based on the first bit of that area.
    // The presence of the bidi formatting characters in the text is a
    // bit dubious, but that's what we do currently, and it's not really
    // clear what the perfect behavior would be...
    final TestSemantics expectedSemantics = TestSemantics.root(
      children: <TestSemantics>[
        TestSemantics.rootChild(
          rect: const Rect.fromLTRB(0.0, 0.0, 800.0, 600.0),
          children: <TestSemantics>[
            TestSemantics(
              rect: const Rect.fromLTRB(-4.0, -4.0, 480.0, 18.0),
              label: 'hello world${Unicode.RLE}${Unicode.RLO} ',
              textDirection: TextDirection.ltr,
            ),
            TestSemantics(
              rect: const Rect.fromLTRB(416.0, -4.0, 466.0, 18.0),
              label: 'BOY',
              textDirection: TextDirection.rtl,
              actions: <SemanticsAction>[SemanticsAction.longPress],
            ),
            TestSemantics(
              rect: const Rect.fromLTRB(192.0, -4.0, 424.0, 18.0),
              label: ' HOW DO${Unicode.PDF} you ${Unicode.RLO} DO ',
              textDirection: TextDirection.rtl,
            ),
            TestSemantics(
              rect: const Rect.fromLTRB(150.0, -4.0, 200.0, 18.0),
              label: 'SIR',
              textDirection: TextDirection.rtl,
              actions: <SemanticsAction>[SemanticsAction.tap],
              flags: <SemanticsFlag>[SemanticsFlag.isLink],
            ),
            TestSemantics(
              rect: const Rect.fromLTRB(472.0, -4.0, 606.0, 18.0),
              label: '${Unicode.PDF}${Unicode.PDF} good bye',
              textDirection: TextDirection.rtl,
            ),
          ],
        ),
      ],
    );
    expect(semantics, hasSemantics(expectedSemantics, ignoreTransform: true, ignoreId: true));
    semantics.dispose();
  }, skip: isBrowser); // https://github.com/flutter/flutter/issues/62945

  testWidgets('TapGesture recognizers contribute link semantics', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    const TextStyle textStyle = TextStyle();
    final TapGestureRecognizer recognizer = TapGestureRecognizer();
    addTearDown(recognizer.dispose);

    await tester.pumpWidget(
      Text.rich(
        TextSpan(
          children: <TextSpan>[TextSpan(text: 'click me', recognizer: recognizer..onTap = () {})],
          style: textStyle,
        ),
        textDirection: TextDirection.ltr,
      ),
    );
    final TestSemantics expectedSemantics = TestSemantics.root(
      children: <TestSemantics>[
        TestSemantics.rootChild(
          children: <TestSemantics>[
            TestSemantics(
              label: 'click me',
              textDirection: TextDirection.ltr,
              actions: <SemanticsAction>[SemanticsAction.tap],
              flags: <SemanticsFlag>[SemanticsFlag.isLink],
            ),
          ],
        ),
      ],
    );
    expect(
      semantics,
      hasSemantics(expectedSemantics, ignoreTransform: true, ignoreId: true, ignoreRect: true),
    );
    semantics.dispose();
  });

  testWidgets('inline widgets generate semantic nodes', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    const TextStyle textStyle = TextStyle();
    final TapGestureRecognizer recognizer = TapGestureRecognizer();
    addTearDown(recognizer.dispose);

    await tester.pumpWidget(
      Text.rich(
        TextSpan(
          children: <InlineSpan>[
            const TextSpan(text: 'a '),
            TextSpan(text: 'pebble', recognizer: recognizer..onTap = () {}),
            const TextSpan(text: ' in the '),
            WidgetSpan(
              child: SizedBox(
                width: 20,
                height: 40,
                child: Card(
                  child: RichText(
                    text: const TextSpan(text: 'INTERRUPTION'),
                    textDirection: TextDirection.rtl,
                  ),
                ),
              ),
            ),
            const TextSpan(text: 'sky'),
          ],
          style: textStyle,
        ),
        textDirection: TextDirection.ltr,
      ),
    );
    final TestSemantics expectedSemantics = TestSemantics.root(
      children: <TestSemantics>[
        TestSemantics.rootChild(
          children: <TestSemantics>[
            TestSemantics(label: 'a ', textDirection: TextDirection.ltr),
            TestSemantics(
              label: 'pebble',
              textDirection: TextDirection.ltr,
              actions: <SemanticsAction>[SemanticsAction.tap],
              flags: <SemanticsFlag>[SemanticsFlag.isLink],
            ),
            TestSemantics(label: ' in the ', textDirection: TextDirection.ltr),
            TestSemantics(label: 'INTERRUPTION', textDirection: TextDirection.rtl),
            TestSemantics(label: 'sky', textDirection: TextDirection.ltr),
          ],
        ),
      ],
    );
    expect(
      semantics,
      hasSemantics(expectedSemantics, ignoreTransform: true, ignoreId: true, ignoreRect: true),
    );
    semantics.dispose();
  }, skip: isBrowser); // https://github.com/flutter/flutter/issues/62945

  testWidgets('inline widgets semantic nodes scale', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    const TextStyle textStyle = TextStyle();
    final TapGestureRecognizer recognizer = TapGestureRecognizer();
    addTearDown(recognizer.dispose);

    await tester.pumpWidget(
      Text.rich(
        TextSpan(
          children: <InlineSpan>[
            const TextSpan(text: 'a '),
            TextSpan(text: 'pebble', recognizer: recognizer..onTap = () {}),
            const TextSpan(text: ' in the '),
            WidgetSpan(
              child: SizedBox(
                width: 20,
                height: 40,
                child: Card(
                  child: RichText(
                    text: const TextSpan(text: 'INTERRUPTION'),
                    textDirection: TextDirection.rtl,
                  ),
                ),
              ),
            ),
            const TextSpan(text: 'sky'),
          ],
          style: textStyle,
        ),
        textDirection: TextDirection.ltr,
        textScaleFactor: 2,
      ),
    );
    final TestSemantics expectedSemantics = TestSemantics.root(
      children: <TestSemantics>[
        TestSemantics.rootChild(
          rect: const Rect.fromLTRB(0.0, 0.0, 800.0, 600.0),
          children: <TestSemantics>[
            TestSemantics(
              label: 'a ',
              textDirection: TextDirection.ltr,
              rect: const Rect.fromLTRB(-4.0, 48.0, 60.0, 84.0),
            ),
            TestSemantics(
              label: 'pebble',
              textDirection: TextDirection.ltr,
              actions: <SemanticsAction>[SemanticsAction.tap],
              flags: <SemanticsFlag>[SemanticsFlag.isLink],
              rect: const Rect.fromLTRB(52.0, 48.0, 228.0, 84.0),
            ),
            TestSemantics(
              label: ' in the ',
              textDirection: TextDirection.ltr,
              rect: const Rect.fromLTRB(220.0, 48.0, 452.0, 84.0),
            ),
            TestSemantics(
              label: 'INTERRUPTION',
              textDirection: TextDirection.rtl,
              rect: const Rect.fromLTRB(0.0, 0.0, 20.0, 40.0),
            ),
            TestSemantics(
              label: 'sky',
              textDirection: TextDirection.ltr,
              rect: const Rect.fromLTRB(484.0, 48.0, 576.0, 84.0),
            ),
          ],
        ),
      ],
    );
    expect(semantics, hasSemantics(expectedSemantics, ignoreTransform: true, ignoreId: true));
    semantics.dispose();
  }, skip: isBrowser); // https://github.com/flutter/flutter/issues/62945

  testWidgets('receives fontFamilyFallback and package from root ThemeData', (
    WidgetTester tester,
  ) async {
    const String fontFamily = 'fontFamily';
    const String package = 'package_name';
    final List<String> fontFamilyFallback = <String>['font', 'family', 'fallback'];
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(
          fontFamily: fontFamily,
          fontFamilyFallback: fontFamilyFallback,
          package: package,
          primarySwatch: Colors.blue,
        ),
        home: const Scaffold(body: Center(child: Text('foo'))),
      ),
    );

    expect(find.byType(RichText), findsOneWidget);
    final RichText richText = tester.widget(find.byType(RichText));
    final InlineSpan text = richText.text;
    final TextStyle? style = text.style;
    expect(style?.fontFamily, equals('packages/$package/$fontFamily'));
    for (int i = 0; i < fontFamilyFallback.length; i++) {
      final String fallback = fontFamilyFallback[i];
      expect(style?.fontFamilyFallback?[i], equals('packages/$package/$fallback'));
    }
  });

  testWidgets('Overflow is clipping correctly - short text with overflow: clip', (
    WidgetTester tester,
  ) async {
    await _pumpTextWidget(tester: tester, overflow: TextOverflow.clip, text: 'Hi');

    expect(find.byType(Text), isNot(paints..clipRect()));
  });

  testWidgets('Overflow is clipping correctly - long text with overflow: ellipsis', (
    WidgetTester tester,
  ) async {
    await _pumpTextWidget(
      tester: tester,
      overflow: TextOverflow.ellipsis,
      text: 'a long long long long text, should be clip',
    );

    expect(find.byType(Text), paints..clipRect(rect: const Rect.fromLTWH(0, 0, 50, 50)));
  }, skip: isBrowser); // https://github.com/flutter/flutter/issues/87878

  testWidgets('Overflow is clipping correctly - short text with overflow: ellipsis', (
    WidgetTester tester,
  ) async {
    await _pumpTextWidget(tester: tester, overflow: TextOverflow.ellipsis, text: 'Hi');

    expect(find.byType(Text), isNot(paints..clipRect()));
  });

  testWidgets('Overflow is clipping correctly - long text with overflow: fade', (
    WidgetTester tester,
  ) async {
    await _pumpTextWidget(
      tester: tester,
      overflow: TextOverflow.fade,
      text: 'a long long long long text, should be clip',
    );

    expect(find.byType(Text), paints..clipRect(rect: const Rect.fromLTWH(0, 0, 50, 50)));
  });

  testWidgets('Overflow is clipping correctly - short text with overflow: fade', (
    WidgetTester tester,
  ) async {
    await _pumpTextWidget(tester: tester, overflow: TextOverflow.fade, text: 'Hi');

    expect(find.byType(Text), isNot(paints..clipRect()));
  });

  testWidgets('Overflow is clipping correctly - long text with overflow: visible', (
    WidgetTester tester,
  ) async {
    await _pumpTextWidget(
      tester: tester,
      overflow: TextOverflow.visible,
      text: 'a long long long long text, should be clip',
    );

    expect(find.byType(Text), isNot(paints..clipRect()));
  });

  testWidgets('Overflow is clipping correctly - short text with overflow: visible', (
    WidgetTester tester,
  ) async {
    await _pumpTextWidget(tester: tester, overflow: TextOverflow.visible, text: 'Hi');

    expect(find.byType(Text), isNot(paints..clipRect()));
  });

  testWidgets('textWidthBasis affects the width of a Text widget', (WidgetTester tester) async {
    Future<void> createText(TextWidthBasis textWidthBasis) {
      return tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: Scaffold(
            body: Center(
              // Each word takes up more than a half of a line. Together they
              // wrap onto two lines, but leave a lot of extra space.
              child: Text(
                'twowordsthateachtakeupmorethanhalfof alineoftextsothattheywrapwithlotsofextraspace',
                textDirection: TextDirection.ltr,
                textWidthBasis: textWidthBasis,
              ),
            ),
          ),
        ),
      );
    }

    const double fontHeight = 14.0;
    const double screenWidth = 800.0;

    // When textWidthBasis is parent, takes up full screen width.
    await createText(TextWidthBasis.parent);
    final Size textSizeParent = tester.getSize(find.byType(Text));
    expect(textSizeParent.width, equals(screenWidth));
    expect(textSizeParent.height, equals(fontHeight * 2));

    // When textWidthBasis is longestLine, sets the width to as small as
    // possible for the two lines.
    await createText(TextWidthBasis.longestLine);
    final Size textSizeLongestLine = tester.getSize(find.byType(Text));
    expect(textSizeLongestLine.width, equals(630.0));
    expect(textSizeLongestLine.height, equals(fontHeight * 2));
  }, skip: isBrowser); // https://github.com/flutter/flutter/issues/44020

  testWidgets('textWidthBasis with textAlign still obeys parent alignment', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Scaffold(
          body: Center(
            child: Column(
              mainAxisSize: MainAxisSize.min,
              crossAxisAlignment: CrossAxisAlignment.start,
              children: <Widget>[
                Text(
                  'LEFT ALIGNED, PARENT',
                  textAlign: TextAlign.left,
                  textWidthBasis: TextWidthBasis.parent,
                ),
                Text(
                  'RIGHT ALIGNED, PARENT',
                  textAlign: TextAlign.right,
                  textWidthBasis: TextWidthBasis.parent,
                ),
                Text(
                  'LEFT ALIGNED, LONGEST LINE',
                  textAlign: TextAlign.left,
                  textWidthBasis: TextWidthBasis.longestLine,
                ),
                Text(
                  'RIGHT ALIGNED, LONGEST LINE',
                  textAlign: TextAlign.right,
                  textWidthBasis: TextWidthBasis.longestLine,
                ),
              ],
            ),
          ),
        ),
      ),
    );

    // All Texts have the same horizontal alignment.
    final double offsetX = tester.getTopLeft(find.text('LEFT ALIGNED, PARENT')).dx;
    expect(tester.getTopLeft(find.text('RIGHT ALIGNED, PARENT')).dx, equals(offsetX));
    expect(tester.getTopLeft(find.text('LEFT ALIGNED, LONGEST LINE')).dx, equals(offsetX));
    expect(tester.getTopLeft(find.text('RIGHT ALIGNED, LONGEST LINE')).dx, equals(offsetX));

    // All Texts are less than or equal to the width of the Column.
    final double width = tester.getSize(find.byType(Column)).width;
    expect(tester.getSize(find.text('LEFT ALIGNED, PARENT')).width, lessThan(width));
    expect(tester.getSize(find.text('RIGHT ALIGNED, PARENT')).width, lessThan(width));
    expect(tester.getSize(find.text('LEFT ALIGNED, LONGEST LINE')).width, lessThan(width));
    expect(tester.getSize(find.text('RIGHT ALIGNED, LONGEST LINE')).width, equals(width));
  }, skip: isBrowser); // https://github.com/flutter/flutter/issues/44020

  testWidgets(
    'textWidthBasis.longestLine confines the width of the paragraph '
    'when given loose constraints',
    (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/62550.
      await tester.pumpWidget(
        Center(
          child: SizedBox(
            width: 400,
            child: Center(
              child: RichText(
                // 400 is not wide enough for this string. The part after the
                // whitespace is going to be broken into a 2nd line.
                text: const TextSpan(
                  text: 'fwefwefwewfefewfwe fwfwfwefweabcdefghijklmnopqrstuvwxyz',
                ),
                textWidthBasis: TextWidthBasis.longestLine,
                textDirection: TextDirection.rtl,
              ),
            ),
          ),
        ),
      );

      expect(
        find.byType(RichText),
        paints..something((Symbol method, List<dynamic> arguments) {
          if (method != #drawParagraph) {
            return false;
          }
          final ui.Paragraph paragraph = arguments[0] as ui.Paragraph;
          final Offset offset = arguments[1] as Offset;
          final List<ui.LineMetrics> lines = paragraph.computeLineMetrics();
          for (final ui.LineMetrics line in lines) {
            if (line.left + offset.dx + line.width >= 400) {
              throw 'line $line is greater than the max width constraints';
            }
          }
          return true;
        }),
      );
    },
    skip: isBrowser, // https://github.com/flutter/flutter/issues/44020
  );

  testWidgets('Paragraph.getBoxesForRange returns nothing when selection range is zero length', (
    WidgetTester tester,
  ) async {
    final ui.ParagraphBuilder builder = ui.ParagraphBuilder(ui.ParagraphStyle());
    builder.addText('hello');
    final ui.Paragraph paragraph = builder.build();
    paragraph.layout(const ui.ParagraphConstraints(width: 1000));
    expect(paragraph.getBoxesForRange(2, 2), isEmpty);
    paragraph.dispose();
  });

  // Regression test for https://github.com/flutter/flutter/issues/65818
  testWidgets('WidgetSpans with no semantic information are elided from semantics', (
    WidgetTester tester,
  ) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    final TapGestureRecognizer recognizer = TapGestureRecognizer();
    addTearDown(recognizer.dispose);
    // Without the fix for this bug the pump widget will throw a RangeError.
    await tester.pumpWidget(
      RichText(
        textDirection: TextDirection.ltr,
        text: TextSpan(
          children: <InlineSpan>[
            const WidgetSpan(child: SizedBox.shrink()),
            TextSpan(
              text: 'HELLO',
              style: const TextStyle(color: Colors.black),
              recognizer: recognizer..onTap = () {},
            ),
          ],
        ),
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics(
              id: 1,
              rect: const Rect.fromLTRB(0.0, 0.0, 800.0, 600.0),
              transform: Matrix4(
                3.0,
                0.0,
                0.0,
                0.0,
                0.0,
                3.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
              ),
              children: <TestSemantics>[
                TestSemantics(
                  rect: const Rect.fromLTRB(-4.0, -4.0, 74.0, 18.0),
                  id: 2,
                  label: 'HELLO',
                  actions: <SemanticsAction>[SemanticsAction.tap],
                  flags: <SemanticsFlag>[SemanticsFlag.isLink],
                ),
              ],
            ),
          ],
        ),
      ),
    );
    semantics.dispose();
  }, skip: isBrowser); // https://github.com/flutter/flutter/issues/87877

  // Regression test for https://github.com/flutter/flutter/issues/69787
  testWidgets('WidgetSpans with no semantic information are elided from semantics - case 2', (
    WidgetTester tester,
  ) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    final TapGestureRecognizer recognizer = TapGestureRecognizer();
    addTearDown(recognizer.dispose);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: RichText(
          text: TextSpan(
            children: <InlineSpan>[
              const WidgetSpan(child: SizedBox.shrink()),
              const WidgetSpan(child: Text('included')),
              TextSpan(
                text: 'HELLO',
                style: const TextStyle(color: Colors.black),
                recognizer: recognizer..onTap = () {},
              ),
              const WidgetSpan(child: Text('included2')),
            ],
          ),
        ),
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics(
              children: <TestSemantics>[
                TestSemantics(label: 'included'),
                TestSemantics(
                  label: 'HELLO',
                  actions: <SemanticsAction>[SemanticsAction.tap],
                  flags: <SemanticsFlag>[SemanticsFlag.isLink],
                ),
                TestSemantics(label: 'included2'),
              ],
            ),
          ],
        ),
        ignoreId: true,
        ignoreRect: true,
        ignoreTransform: true,
      ),
    );
    semantics.dispose();
  }, skip: isBrowser); // https://github.com/flutter/flutter/issues/87877

  // Regression test for https://github.com/flutter/flutter/issues/69787
  testWidgets('WidgetSpans with no semantic information are elided from semantics - case 3', (
    WidgetTester tester,
  ) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    final TapGestureRecognizer recognizer = TapGestureRecognizer();
    addTearDown(recognizer.dispose);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: RichText(
          text: TextSpan(
            children: <InlineSpan>[
              const WidgetSpan(child: SizedBox.shrink()),
              WidgetSpan(
                child: Row(
                  children: <Widget>[
                    Semantics(container: true, child: const Text('foo')),
                    Semantics(container: true, child: const Text('bar')),
                  ],
                ),
              ),
              TextSpan(
                text: 'HELLO',
                style: const TextStyle(color: Colors.black),
                recognizer: recognizer..onTap = () {},
              ),
            ],
          ),
        ),
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics(
              children: <TestSemantics>[
                TestSemantics(label: 'foo'),
                TestSemantics(label: 'bar'),
                TestSemantics(
                  label: 'HELLO',
                  actions: <SemanticsAction>[SemanticsAction.tap],
                  flags: <SemanticsFlag>[SemanticsFlag.isLink],
                ),
              ],
            ),
          ],
        ),
        ignoreId: true,
        ignoreRect: true,
        ignoreTransform: true,
      ),
    );
    semantics.dispose();
  }, skip: isBrowser); // https://github.com/flutter/flutter/issues/87877

  // Regression test for https://github.com/flutter/flutter/issues/69787
  testWidgets('WidgetSpans with no semantic information are elided from semantics - case 4', (
    WidgetTester tester,
  ) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    final TapGestureRecognizer recognizer = TapGestureRecognizer();
    addTearDown(recognizer.dispose);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: ClipRect(
            child: Container(
              color: Colors.green,
              height: 100,
              width: 100,
              child: OverflowBox(
                alignment: Alignment.topLeft,
                maxWidth: double.infinity,
                child: RichText(
                  text: TextSpan(
                    children: <InlineSpan>[
                      const WidgetSpan(
                        child: Icon(Icons.edit, size: 16, semanticLabel: 'not clipped'),
                      ),
                      TextSpan(text: 'next WS is clipped', recognizer: recognizer..onTap = () {}),
                      const WidgetSpan(child: Icon(Icons.edit, size: 16, semanticLabel: 'clipped')),
                    ],
                  ),
                ),
              ),
            ),
          ),
        ),
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics(
              children: <TestSemantics>[
                TestSemantics(label: 'not clipped'),
                TestSemantics(
                  label: 'next WS is clipped',
                  flags: <SemanticsFlag>[SemanticsFlag.isLink],
                  actions: <SemanticsAction>[SemanticsAction.tap],
                ),
              ],
            ),
          ],
        ),
        ignoreId: true,
        ignoreRect: true,
        ignoreTransform: true,
      ),
    );
    semantics.dispose();
  }, skip: isBrowser); // https://github.com/flutter/flutter/issues/87877

  testWidgets('RenderParagraph intrinsic width', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: SizedBox(
            height: 100,
            child: IntrinsicWidth(
              child: RichText(
                text: const TextSpan(
                  style: TextStyle(fontSize: 16, height: 1),
                  children: <InlineSpan>[
                    TextSpan(text: 'S '),
                    WidgetSpan(
                      alignment: PlaceholderAlignment.top,
                      child: Wrap(
                        direction: Axis.vertical,
                        children: <Widget>[
                          SizedBox(width: 200, height: 100),
                          SizedBox(width: 200, height: 30),
                        ],
                      ),
                    ),
                    TextSpan(text: ' E'),
                  ],
                ),
              ),
            ),
          ),
        ),
      ),
    );

    expect(tester.getSize(find.byType(RichText)).width, 200 + 4 * 16.0);
    final RenderParagraph paragraph = tester.renderObject<RenderParagraph>(find.byType(RichText));
    // The inline spans are rendered on one (horizontal) line, the sum of the widths is the max intrinsic width.
    expect(paragraph.getMaxIntrinsicWidth(0.0), 200 + 4 * 16.0);
    // The inline spans are rendered in one vertical run, the widest one determines the min intrinsic width.
    expect(paragraph.getMinIntrinsicWidth(0.0), 200);
  });

  testWidgets('can compute intrinsic width and height for widget span with text scaling', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/59316
    const Key textKey = Key('RichText');
    Widget textWithNestedInlineSpans({
      required double textScaleFactor,
      required double screenWidth,
    }) {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: OverflowBox(
            alignment: Alignment.topLeft,
            maxWidth: screenWidth,
            child: RichText(
              key: textKey,
              textScaleFactor: textScaleFactor,
              text: const TextSpan(children: <InlineSpan>[WidgetSpan(child: Text('one two'))]),
            ),
          ),
        ),
      );
    }

    // The render object is going to be reused across widget tree rebuilds.
    late final RenderParagraph outerParagraph = tester.renderObject(find.byKey(textKey));

    await tester.pumpWidget(textWithNestedInlineSpans(textScaleFactor: 1.0, screenWidth: 100.0));
    expect(outerParagraph.getMaxIntrinsicHeight(100.0), 14.0, reason: 'singleLineHeight = 14.0');

    await tester.pumpWidget(textWithNestedInlineSpans(textScaleFactor: 2.0, screenWidth: 100.0));
    expect(
      outerParagraph.getMinIntrinsicHeight(100.0),
      14.0 * 2.0 * 2,
      reason: 'intrinsicHeight = singleLineHeight * textScaleFactor * two lines.',
    );

    await tester.pumpWidget(textWithNestedInlineSpans(textScaleFactor: 1.0, screenWidth: 1000.0));
    expect(
      outerParagraph.getMaxIntrinsicWidth(1000.0),
      14.0 * 7,
      reason: 'intrinsic width = 14.0 * 7',
    );

    await tester.pumpWidget(textWithNestedInlineSpans(textScaleFactor: 2.0, screenWidth: 1000.0));
    expect(
      outerParagraph.getMaxIntrinsicWidth(1000.0),
      14.0 * 2.0 * 7,
      reason: 'intrinsic width = glyph advance * textScaleFactor * num of glyphs',
    );
  });

  testWidgets('Text uses TextStyle.overflow', (WidgetTester tester) async {
    const TextOverflow overflow = TextOverflow.fade;

    await tester.pumpWidget(
      const Text(
        'Hello World',
        textDirection: TextDirection.ltr,
        style: TextStyle(overflow: overflow),
      ),
    );

    final RichText richText = tester.firstWidget(find.byType(RichText));

    expect(richText.overflow, overflow);
    expect(richText.text.style!.overflow, overflow);
  });

  testWidgets('Text can be hit-tested without layout or paint being called in a frame', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/85108.
    await tester.pumpWidget(
      const Opacity(
        opacity: 1.0,
        child: Text(
          'Hello World',
          textDirection: TextDirection.ltr,
          style: TextStyle(color: Color(0xFF123456)),
        ),
      ),
    );

    // The color changed and the opacity is set to 0:
    //  * 0 opacity will prevent RenderParagraph.paint from being called.
    //  * Only changing the color will prevent RenderParagraph.performLayout
    //    from being called.
    //  The underlying TextPainter should not evict its layout cache in this
    //  case, for hit-testing.
    await tester.pumpWidget(
      const Opacity(
        opacity: 0.0,
        child: Text(
          'Hello World',
          textDirection: TextDirection.ltr,
          style: TextStyle(color: Color(0x87654321)),
        ),
      ),
    );

    await tester.tap(find.text('Hello World'));
    expect(tester.takeException(), isNull);
  });

  testWidgets('Mouse hovering over selectable Text uses SystemMouseCursor.text', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const MaterialApp(home: SelectionArea(child: Text('Flutter'))));

    final TestGesture gesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );
    await gesture.addPointer(location: tester.getCenter(find.byType(Text)));

    await tester.pump();

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.text,
    );
  });

  testWidgets('Mouse hovering over selectable Text uses default selection style mouse cursor', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: SelectionArea(
          child: DefaultSelectionStyle.merge(
            mouseCursor: SystemMouseCursors.click,
            child: const Text('Flutter'),
          ),
        ),
      ),
    );

    final TestGesture gesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );
    await gesture.addPointer(location: tester.getCenter(find.byType(Text)));

    await tester.pump();

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.click,
    );
  });

  testWidgets('can set heading level', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    for (int level = 1; level <= 6; level++) {
      await tester.pumpWidget(
        Semantics(
          headingLevel: 1,
          child: Text('Heading level $level', textDirection: TextDirection.ltr),
        ),
      );
      final TestSemantics expectedSemantics = TestSemantics.root(
        children: <TestSemantics>[
          TestSemantics.rootChild(
            label: 'Heading level $level',
            headingLevel: 1,
            textDirection: TextDirection.ltr,
          ),
        ],
      );
      expect(
        semantics,
        hasSemantics(expectedSemantics, ignoreTransform: true, ignoreId: true, ignoreRect: true),
      );
    }

    semantics.dispose();
  });
}

Future<void> _pumpTextWidget({
  required WidgetTester tester,
  required String text,
  required TextOverflow overflow,
}) {
  return tester.pumpWidget(
    Directionality(
      textDirection: TextDirection.ltr,
      child: Center(
        child: SizedBox(width: 50.0, height: 50.0, child: Text(text, overflow: overflow)),
      ),
    ),
  );
}
