// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_api_samples/material/selection_area/selection_area.2.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('SelectionArea Color Text Red Example Smoke Test', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SelectionAreaColorTextRedExampleApp());
    expect(find.widgetWithIcon(FloatingActionButton, Icons.undo), findsOneWidget);
    expect(find.byType(Column), findsNWidgets(2));
    expect(find.textContaining('This is some bulleted list:\n'), findsOneWidget);
    for (int i = 1; i <= 7; i += 1) {
      expect(find.widgetWithText(Text, '• Bullet $i'), findsOneWidget);
    }
    expect(find.textContaining('This is some text in a text widget.'), findsOneWidget);
    expect(find.textContaining(' This is some more text in the same text widget.'), findsOneWidget);
    expect(find.textContaining('This is some text in another text widget.'), findsOneWidget);
  });

  testWidgets('SelectionArea Color Text Red Example - colors selected range red', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.SelectionAreaColorTextRedExampleApp());
    await tester.pumpAndSettle();
    final Finder paragraph1Finder = find.descendant(
      of: find.textContaining('This is some bulleted list').first,
      matching: find.byType(RichText).first,
    );
    final Finder paragraph3Finder = find.descendant(
      of: find.textContaining('This is some text in another text widget.'),
      matching: find.byType(RichText),
    );
    final RenderParagraph paragraph1 = tester.renderObject<RenderParagraph>(paragraph1Finder);
    final List<RenderParagraph> bullets =
        tester
            .renderObjectList<RenderParagraph>(
              find.descendant(of: find.textContaining('• Bullet'), matching: find.byType(RichText)),
            )
            .toList();
    expect(bullets.length, 7);
    final RenderParagraph paragraph2 = tester.renderObject<RenderParagraph>(
      find.descendant(
        of: find.textContaining('This is some text in a text widget.'),
        matching: find.byType(RichText),
      ),
    );
    final RenderParagraph paragraph3 = tester.renderObject<RenderParagraph>(paragraph3Finder);
    // Drag to select from paragraph 1 position 4 to paragraph 3 position 25.
    final TestGesture gesture = await tester.startGesture(
      tester.getRect(paragraph1Finder).topLeft + const Offset(50.0, 10.0),
      kind: PointerDeviceKind.mouse,
    );
    addTearDown(gesture.removePointer);
    await tester.pump();
    await gesture.moveTo(tester.getRect(paragraph3Finder).centerLeft + const Offset(360.0, 0.0));
    await tester.pump();
    await gesture.up();
    await tester.pumpAndSettle();

    // Verify selection.
    // Bulleted list title.
    expect(paragraph1.selections.length, 1);
    expect(paragraph1.selections[0], const TextSelection(baseOffset: 4, extentOffset: 27));
    // Bulleted list.
    for (final RenderParagraph paragraphBullet in bullets) {
      expect(paragraphBullet.selections.length, 1);
      expect(paragraphBullet.selections[0], const TextSelection(baseOffset: 0, extentOffset: 10));
    }
    // Second text widget.
    expect(paragraph2.selections.length, 1);
    expect(paragraph2.selections[0], const TextSelection(baseOffset: 0, extentOffset: 83));
    // Third text widget.
    expect(paragraph3.selections.length, 1);
    expect(paragraph3.selections[0], const TextSelection(baseOffset: 0, extentOffset: 25));

    // Color selection red.
    expect(find.textContaining('Color Text Red'), findsOneWidget);
    await tester.tap(find.textContaining('Color Text Red'));
    await tester.pumpAndSettle();

    // Verify selection is red.
    final TextSpan paragraph1ResultingSpan = paragraph1.text as TextSpan;
    final TextSpan paragraph2ResultingSpan = paragraph2.text as TextSpan;
    final TextSpan paragraph3ResultingSpan = paragraph3.text as TextSpan;
    // Title of bulleted list is partially red.
    expect(paragraph1ResultingSpan.children, isNotNull);
    expect(paragraph1ResultingSpan.children!.length, 1);
    expect((paragraph1ResultingSpan.children![0] as TextSpan).children, isNotNull);
    expect((paragraph1ResultingSpan.children![0] as TextSpan).children!.length, 3);
    expect((paragraph1ResultingSpan.children![0] as TextSpan).children![0].style, isNull);
    expect((paragraph1ResultingSpan.children![0] as TextSpan).children![1], isA<TextSpan>());
    expect(
      ((paragraph1ResultingSpan.children![0] as TextSpan).children![1] as TextSpan).text,
      isNotNull,
    );
    expect(
      ((paragraph1ResultingSpan.children![0] as TextSpan).children![1] as TextSpan).text,
      ' is some bulleted list:\n',
    );
    expect((paragraph1ResultingSpan.children![0] as TextSpan).children![1].style, isNotNull);
    expect((paragraph1ResultingSpan.children![0] as TextSpan).children![1].style!.color, isNotNull);
    expect(
      (paragraph1ResultingSpan.children![0] as TextSpan).children![1].style!.color,
      Colors.red,
    );
    expect((paragraph1ResultingSpan.children![0] as TextSpan).children![2], isA<WidgetSpan>());
    // Bullets are red.
    for (final RenderParagraph paragraphBullet in bullets) {
      final TextSpan resultingBulletSpan = paragraphBullet.text as TextSpan;
      expect(resultingBulletSpan.children, isNotNull);
      expect(resultingBulletSpan.children!.length, 1);
      expect(resultingBulletSpan.children![0], isA<TextSpan>());
      expect((resultingBulletSpan.children![0] as TextSpan).children, isNotNull);
      expect((resultingBulletSpan.children![0] as TextSpan).children!.length, 1);
      expect((resultingBulletSpan.children![0] as TextSpan).children![0], isA<TextSpan>());
      expect(
        ((resultingBulletSpan.children![0] as TextSpan).children![0] as TextSpan).style,
        isNotNull,
      );
      expect(
        ((resultingBulletSpan.children![0] as TextSpan).children![0] as TextSpan).style!.color,
        isNotNull,
      );
      expect(
        ((resultingBulletSpan.children![0] as TextSpan).children![0] as TextSpan).style!.color,
        Colors.red,
      );
    }
    // Second text widget is red.
    expect(paragraph2ResultingSpan.children, isNotNull);
    expect(paragraph2ResultingSpan.children!.length, 1);
    expect(paragraph2ResultingSpan.children![0], isA<TextSpan>());
    expect((paragraph2ResultingSpan.children![0] as TextSpan).children, isNotNull);
    for (final InlineSpan span in (paragraph2ResultingSpan.children![0] as TextSpan).children!) {
      if (span is TextSpan) {
        expect(span.style, isNotNull);
        expect(span.style!.color, isNotNull);
        expect(span.style!.color, Colors.red);
      }
    }
    // Part of third text widget is red.
    expect(paragraph3ResultingSpan.children, isNotNull);
    expect(paragraph3ResultingSpan.children!.length, 1);
    expect(paragraph3ResultingSpan.children![0], isA<TextSpan>());
    expect((paragraph3ResultingSpan.children![0] as TextSpan).children, isNotNull);
    expect((paragraph3ResultingSpan.children![0] as TextSpan).children!.length, 2);
    expect((paragraph3ResultingSpan.children![0] as TextSpan).children![0], isA<TextSpan>());
    expect(
      ((paragraph3ResultingSpan.children![0] as TextSpan).children![0] as TextSpan).text,
      isNotNull,
    );
    expect(
      ((paragraph3ResultingSpan.children![0] as TextSpan).children![0] as TextSpan).text,
      'This is some text in ano',
    );
    expect((paragraph3ResultingSpan.children![0] as TextSpan).children![0].style, isNotNull);
    expect((paragraph3ResultingSpan.children![0] as TextSpan).children![0].style!.color, isNotNull);
    expect(
      (paragraph3ResultingSpan.children![0] as TextSpan).children![0].style!.color,
      Colors.red,
    );
    expect((paragraph3ResultingSpan.children![0] as TextSpan).children![1].style, isNull);
  });
}
