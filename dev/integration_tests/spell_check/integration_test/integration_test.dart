// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// TODO(camsim99): Revert this timeout change after effects are investigated.
@Timeout(Duration(seconds: 60))
library;

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:integration_test/integration_test.dart';
import 'package:spell_check/main.dart';

late DefaultSpellCheckService defaultSpellCheckService;
late Locale locale;

/// Waits to find [EditableText] that displays text with misspelled
/// words marked the same as the [TextSpan] provided and returns
/// true if it is found before timing out at 20 seconds.
Future<bool> findTextSpanTree(WidgetTester tester, TextSpan inlineSpan) async {
  final RenderObject root = tester.renderObject(find.byType(EditableText));
  expect(root, isNotNull);

  RenderEditable? renderEditable;
  void recursiveFinder(RenderObject child) {
    if (child is RenderEditable && child.text == inlineSpan) {
      renderEditable = child;
      return;
    }
    child.visitChildren(recursiveFinder);
  }

  final DateTime endTime = tester.binding.clock.now().add(const Duration(seconds: 20));
  do {
    if (tester.binding.clock.now().isAfter(endTime)) {
      return false;
    }
    await tester.pump(const Duration(seconds: 1));
    root.visitChildren(recursiveFinder);
  } while (renderEditable == null);

  return true;
}

void main() {
  IntegrationTestWidgetsFlutterBinding.ensureInitialized();

  setUp(() {
    defaultSpellCheckService = DefaultSpellCheckService();
    locale = const Locale('en', 'us');
  });

  test('fetchSpellCheckSuggestions returns null with no misspelled words', () async {
    const String text = 'Hello, world!';

    final List<SuggestionSpan>? spellCheckSuggestionSpans = await defaultSpellCheckService
        .fetchSpellCheckSuggestions(locale, text);

    expect(spellCheckSuggestionSpans!.length, equals(0));
    expect(defaultSpellCheckService.lastSavedResults!.spellCheckedText, equals(text));
    expect(
      defaultSpellCheckService.lastSavedResults!.suggestionSpans,
      equals(spellCheckSuggestionSpans),
    );
  });

  test('fetchSpellCheckSuggestions returns correct ranges with misspelled words', () async {
    const String text = 'Hlelo, world! Yuou are magnificente';
    const List<TextRange> misspelledWordRanges = <TextRange>[
      TextRange(start: 0, end: 5),
      TextRange(start: 14, end: 18),
      TextRange(start: 23, end: 35),
    ];

    final List<SuggestionSpan>? spellCheckSuggestionSpans = await defaultSpellCheckService
        .fetchSpellCheckSuggestions(locale, text);

    expect(spellCheckSuggestionSpans, isNotNull);
    expect(spellCheckSuggestionSpans!.length, equals(misspelledWordRanges.length));

    for (int i = 0; i < misspelledWordRanges.length; i += 1) {
      expect(spellCheckSuggestionSpans[i].range, equals(misspelledWordRanges[i]));
    }

    expect(defaultSpellCheckService.lastSavedResults!.spellCheckedText, equals(text));
    expect(
      defaultSpellCheckService.lastSavedResults!.suggestionSpans,
      equals(spellCheckSuggestionSpans),
    );
  });

  test(
    'fetchSpellCheckSuggestions does not correct results when Gboard not ignoring composing region',
    () async {
      const String text = 'Wwow, whaaett a beautiful day it is!';

      final List<SuggestionSpan>? spellCheckSpansWithComposingRegion =
          await defaultSpellCheckService.fetchSpellCheckSuggestions(locale, text);

      expect(spellCheckSpansWithComposingRegion, isNotNull);
      expect(spellCheckSpansWithComposingRegion!.length, equals(2));

      final List<SuggestionSpan>? spellCheckSuggestionSpans = await defaultSpellCheckService
          .fetchSpellCheckSuggestions(locale, text);

      expect(spellCheckSuggestionSpans, equals(spellCheckSpansWithComposingRegion));
    },
  );

  test('fetchSpellCheckSuggestions merges results when Gboard ignoring composing region', () async {
    const String text = 'Wooahha it is an amazzinng dayyebf!';

    final List<SuggestionSpan>? modifiedSpellCheckSuggestionSpans = await defaultSpellCheckService
        .fetchSpellCheckSuggestions(locale, text);
    final List<SuggestionSpan> expectedSpellCheckSuggestionSpans = List<SuggestionSpan>.from(
      modifiedSpellCheckSuggestionSpans!,
    );
    expect(modifiedSpellCheckSuggestionSpans, isNotNull);
    expect(modifiedSpellCheckSuggestionSpans.length, equals(3));

    // Remove one span to simulate Gboard attempting to un-ignore the composing region, after tapping away from "Yuou".
    modifiedSpellCheckSuggestionSpans.removeAt(1);

    defaultSpellCheckService.lastSavedResults = SpellCheckResults(
      text,
      modifiedSpellCheckSuggestionSpans,
    );

    final List<SuggestionSpan>? spellCheckSuggestionSpans = await defaultSpellCheckService
        .fetchSpellCheckSuggestions(locale, text);

    expect(spellCheckSuggestionSpans, isNotNull);
    expect(spellCheckSuggestionSpans, equals(expectedSpellCheckSuggestionSpans));
  });

  testWidgets('EditableText spell checks when text is entered and spell check enabled', (
    WidgetTester tester,
  ) async {
    const TextStyle style = TextStyle();
    const TextStyle misspelledTextStyle = TextField.materialMisspelledTextStyle;

    await tester.pumpWidget(const MyApp());

    await tester.enterText(find.byType(EditableText), 'Hey cfabiueq qocnakoef! Hey!');

    const TextSpan expectedTextSpanTree = TextSpan(
      style: style,
      children: <TextSpan>[
        TextSpan(style: style, text: 'Hey '),
        TextSpan(style: misspelledTextStyle, text: 'cfabiueq'),
        TextSpan(style: style, text: ' '),
        TextSpan(style: misspelledTextStyle, text: 'qocnakoef'),
        TextSpan(style: style, text: '! Hey!'),
      ],
    );

    final bool expectedTextSpanTreeFound = await findTextSpanTree(tester, expectedTextSpanTree);

    expect(expectedTextSpanTreeFound, isTrue);
  });
}
