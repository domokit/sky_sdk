// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

late TextStyle composingStyle;
late TextStyle misspelledTextStyle;

void main() {
  setUp(() {
    composingStyle = const TextStyle(decoration: TextDecoration.underline);

    // Using Android handling for testing.
    misspelledTextStyle = TextField.materialMisspelledTextStyle;
  });

  testWidgets(
    'buildTextSpanWithSpellCheckSuggestions ignores composing region when composing region out of range',
    (WidgetTester tester) async {
      const String text = 'Hello, wrold! Hey';
      const TextEditingValue value = TextEditingValue(text: text);
      const bool composingRegionOutOfRange = true;
      const SpellCheckResults spellCheckResults = SpellCheckResults(text, <SuggestionSpan>[
        SuggestionSpan(TextRange(start: 7, end: 12), <String>['world', 'word', 'old']),
      ]);

      final TextSpan expectedTextSpanTree = TextSpan(
        children: <TextSpan>[
          const TextSpan(text: 'Hello, '),
          TextSpan(style: misspelledTextStyle, text: 'wrold'),
          const TextSpan(text: '! Hey'),
        ],
      );
      final TextSpan textSpanTree = buildTextSpanWithSpellCheckSuggestions(
        value,
        composingRegionOutOfRange,
        null,
        misspelledTextStyle,
        spellCheckResults,
      );

      expect(textSpanTree, equals(expectedTextSpanTree));
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.android,
      TargetPlatform.iOS,
    }),
  );

  testWidgets(
    'buildTextSpanWithSpellCheckSuggestions, isolated misspelled word with separate composing region example',
    (WidgetTester tester) async {
      const String text = 'Hello, wrold! Hey';
      const TextEditingValue value = TextEditingValue(
        text: text,
        composing: TextRange(start: 14, end: 17),
      );
      const bool composingRegionOutOfRange = false;
      const SpellCheckResults spellCheckResults = SpellCheckResults(text, <SuggestionSpan>[
        SuggestionSpan(TextRange(start: 7, end: 12), <String>['world', 'word', 'old']),
      ]);

      final TextSpan expectedTextSpanTree = TextSpan(
        children: <TextSpan>[
          const TextSpan(text: 'Hello, '),
          TextSpan(style: misspelledTextStyle, text: 'wrold'),
          const TextSpan(text: '! '),
          TextSpan(style: composingStyle, text: 'Hey'),
        ],
      );
      final TextSpan textSpanTree = buildTextSpanWithSpellCheckSuggestions(
        value,
        composingRegionOutOfRange,
        null,
        misspelledTextStyle,
        spellCheckResults,
      );

      expect(textSpanTree, equals(expectedTextSpanTree));
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.android}),
  );

  testWidgets(
    'buildTextSpanWithSpellCheckSuggestions, composing region and misspelled words overlap example',
    (WidgetTester tester) async {
      const String text = 'Right worng worng right';
      const TextEditingValue value = TextEditingValue(
        text: text,
        composing: TextRange(start: 12, end: 17),
      );
      const bool composingRegionOutOfRange = false;
      const SpellCheckResults spellCheckResults = SpellCheckResults(text, <SuggestionSpan>[
        SuggestionSpan(TextRange(start: 6, end: 11), <String>['wrong', 'worn', 'wrung']),
        SuggestionSpan(TextRange(start: 12, end: 17), <String>['wrong', 'worn', 'wrung']),
      ]);

      final TextSpan expectedTextSpanTree = TextSpan(
        children: <TextSpan>[
          const TextSpan(text: 'Right '),
          TextSpan(style: misspelledTextStyle, text: 'worng'),
          const TextSpan(text: ' '),
          TextSpan(style: composingStyle, text: 'worng'),
          const TextSpan(text: ' right'),
        ],
      );
      final TextSpan textSpanTree = buildTextSpanWithSpellCheckSuggestions(
        value,
        composingRegionOutOfRange,
        null,
        misspelledTextStyle,
        spellCheckResults,
      );

      expect(textSpanTree, equals(expectedTextSpanTree));
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.android}),
  );

  testWidgets(
    'buildTextSpanWithSpellCheckSuggestions, consecutive misspelled words example',
    (WidgetTester tester) async {
      const String text = 'Right worng worng right';
      const TextEditingValue value = TextEditingValue(text: text);
      const bool composingRegionOutOfRange = true;
      const SpellCheckResults spellCheckResults = SpellCheckResults(text, <SuggestionSpan>[
        SuggestionSpan(TextRange(start: 6, end: 11), <String>['wrong', 'worn', 'wrung']),
        SuggestionSpan(TextRange(start: 12, end: 17), <String>['wrong', 'worn', 'wrung']),
      ]);

      final TextSpan expectedTextSpanTree = TextSpan(
        children: <TextSpan>[
          const TextSpan(text: 'Right '),
          TextSpan(style: misspelledTextStyle, text: 'worng'),
          const TextSpan(text: ' '),
          TextSpan(style: misspelledTextStyle, text: 'worng'),
          const TextSpan(text: ' right'),
        ],
      );
      final TextSpan textSpanTree = buildTextSpanWithSpellCheckSuggestions(
        value,
        composingRegionOutOfRange,
        null,
        misspelledTextStyle,
        spellCheckResults,
      );

      expect(textSpanTree, equals(expectedTextSpanTree));
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.android,
      TargetPlatform.iOS,
    }),
  );

  testWidgets(
    'buildTextSpanWithSpellCheckSuggestions corrects results when they lag, results text shorter than actual text example',
    (WidgetTester tester) async {
      const String text = 'Hello, wrold! Hey';
      const String resultsText = 'Hello, wrold!';
      const TextEditingValue value = TextEditingValue(text: text);
      const bool composingRegionOutOfRange = false;
      const SpellCheckResults spellCheckResults = SpellCheckResults(resultsText, <SuggestionSpan>[
        SuggestionSpan(TextRange(start: 7, end: 12), <String>['world', 'word', 'old']),
      ]);

      final TextSpan expectedTextSpanTree = TextSpan(
        children: <TextSpan>[
          const TextSpan(text: 'Hello, '),
          TextSpan(style: misspelledTextStyle, text: 'wrold'),
          const TextSpan(text: '! Hey'),
        ],
      );
      final TextSpan textSpanTree = buildTextSpanWithSpellCheckSuggestions(
        value,
        composingRegionOutOfRange,
        null,
        misspelledTextStyle,
        spellCheckResults,
      );

      expect(textSpanTree, equals(expectedTextSpanTree));
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.android,
      TargetPlatform.iOS,
    }),
  );

  testWidgets(
    'buildTextSpanWithSpellCheckSuggestions corrects results when they lag, results text longer with more misspelled words than actual text example',
    (WidgetTester tester) async {
      const String text = 'Hello, wrold! Hey';
      const String resultsText = 'Hello, wrold Hey feirnd!';
      const TextEditingValue value = TextEditingValue(text: text);
      const bool composingRegionOutOfRange = false;
      const SpellCheckResults spellCheckResults = SpellCheckResults(resultsText, <SuggestionSpan>[
        SuggestionSpan(TextRange(start: 7, end: 12), <String>['world', 'word', 'old']),
        SuggestionSpan(TextRange(start: 17, end: 23), <String>['friend', 'fiend', 'fern']),
      ]);

      final TextSpan expectedTextSpanTree = TextSpan(
        children: <TextSpan>[
          const TextSpan(text: 'Hello, '),
          TextSpan(style: misspelledTextStyle, text: 'wrold'),
          const TextSpan(text: '! Hey'),
        ],
      );
      final TextSpan textSpanTree = buildTextSpanWithSpellCheckSuggestions(
        value,
        composingRegionOutOfRange,
        null,
        misspelledTextStyle,
        spellCheckResults,
      );

      expect(textSpanTree, equals(expectedTextSpanTree));
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.android,
      TargetPlatform.iOS,
    }),
  );

  testWidgets(
    'buildTextSpanWithSpellCheckSuggestions corrects results when they lag, results text mismatched example',
    (WidgetTester tester) async {
      const String text = 'Hello, wrold! Hey';
      const String resultsText = 'Hello, wrild! Hey';
      const TextEditingValue value = TextEditingValue(text: text);
      const bool composingRegionOutOfRange = false;
      const SpellCheckResults spellCheckResults = SpellCheckResults(resultsText, <SuggestionSpan>[
        SuggestionSpan(TextRange(start: 7, end: 12), <String>['wild', 'world']),
      ]);

      const TextSpan expectedTextSpanTree = TextSpan(
        children: <TextSpan>[TextSpan(text: 'Hello, wrold! Hey')],
      );
      final TextSpan textSpanTree = buildTextSpanWithSpellCheckSuggestions(
        value,
        composingRegionOutOfRange,
        null,
        misspelledTextStyle,
        spellCheckResults,
      );

      expect(textSpanTree, equals(expectedTextSpanTree));
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.android,
      TargetPlatform.iOS,
    }),
  );

  testWidgets(
    'buildTextSpanWithSpellCheckSuggestions corrects results when they lag, results shifted forward example',
    (WidgetTester tester) async {
      const String text = 'Hello, there wrold! Hey';
      const String resultsText = 'Hello, wrold! Hey';
      const TextEditingValue value = TextEditingValue(text: text);
      const bool composingRegionOutOfRange = false;
      const SpellCheckResults spellCheckResults = SpellCheckResults(resultsText, <SuggestionSpan>[
        SuggestionSpan(TextRange(start: 7, end: 12), <String>['world', 'word', 'old']),
      ]);

      final TextSpan expectedTextSpanTree = TextSpan(
        children: <TextSpan>[
          const TextSpan(text: 'Hello, there '),
          TextSpan(style: misspelledTextStyle, text: 'wrold'),
          const TextSpan(text: '! Hey'),
        ],
      );
      final TextSpan textSpanTree = buildTextSpanWithSpellCheckSuggestions(
        value,
        composingRegionOutOfRange,
        null,
        misspelledTextStyle,
        spellCheckResults,
      );

      expect(textSpanTree, equals(expectedTextSpanTree));
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.android,
      TargetPlatform.iOS,
    }),
  );

  testWidgets(
    'buildTextSpanWithSpellCheckSuggestions corrects results when they lag, results shifted backwards example',
    (WidgetTester tester) async {
      const String text = 'Hello, wrold! Hey';
      const String resultsText = 'Hello, great wrold! Hey';
      const TextEditingValue value = TextEditingValue(text: text);
      const bool composingRegionOutOfRange = false;
      const SpellCheckResults spellCheckResults = SpellCheckResults(resultsText, <SuggestionSpan>[
        SuggestionSpan(TextRange(start: 13, end: 18), <String>['world', 'word', 'old']),
      ]);

      final TextSpan expectedTextSpanTree = TextSpan(
        children: <TextSpan>[
          const TextSpan(text: 'Hello, '),
          TextSpan(style: misspelledTextStyle, text: 'wrold'),
          const TextSpan(text: '! Hey'),
        ],
      );
      final TextSpan textSpanTree = buildTextSpanWithSpellCheckSuggestions(
        value,
        composingRegionOutOfRange,
        null,
        misspelledTextStyle,
        spellCheckResults,
      );

      expect(textSpanTree, equals(expectedTextSpanTree));
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.android,
      TargetPlatform.iOS,
    }),
  );

  testWidgets(
    'buildTextSpanWithSpellCheckSuggestions corrects results when they lag, results shifted backwards and forwards example',
    (WidgetTester tester) async {
      const String text = 'Hello, wrold! And Hye!';
      const String resultsText = 'Hello, great wrold! Hye!';
      const TextEditingValue value = TextEditingValue(text: text);
      const bool composingRegionOutOfRange = false;
      const SpellCheckResults spellCheckResults = SpellCheckResults(resultsText, <SuggestionSpan>[
        SuggestionSpan(TextRange(start: 13, end: 18), <String>['world', 'word', 'old']),
        SuggestionSpan(TextRange(start: 20, end: 23), <String>['Hey', 'He']),
      ]);

      final TextSpan expectedTextSpanTree = TextSpan(
        children: <TextSpan>[
          const TextSpan(text: 'Hello, '),
          TextSpan(style: misspelledTextStyle, text: 'wrold'),
          const TextSpan(text: '! And '),
          TextSpan(style: misspelledTextStyle, text: 'Hye'),
          const TextSpan(text: '!'),
        ],
      );
      final TextSpan textSpanTree = buildTextSpanWithSpellCheckSuggestions(
        value,
        composingRegionOutOfRange,
        null,
        misspelledTextStyle,
        spellCheckResults,
      );

      expect(textSpanTree, equals(expectedTextSpanTree));
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.android,
      TargetPlatform.iOS,
    }),
  );

  testWidgets(
    'buildTextSpanWithSpellCheckSuggestions discards result when additions are made to misspelled word example',
    (WidgetTester tester) async {
      const String text = 'Hello, wroldd!';
      const String resultsText = 'Hello, wrold!';
      const TextEditingValue value = TextEditingValue(text: text);
      const bool composingRegionOutOfRange = false;
      const SpellCheckResults spellCheckResults = SpellCheckResults(resultsText, <SuggestionSpan>[
        SuggestionSpan(TextRange(start: 7, end: 12), <String>['world', 'word', 'old']),
      ]);

      const TextSpan expectedTextSpanTree = TextSpan(
        children: <TextSpan>[TextSpan(text: 'Hello, wroldd!')],
      );
      final TextSpan textSpanTree = buildTextSpanWithSpellCheckSuggestions(
        value,
        composingRegionOutOfRange,
        null,
        misspelledTextStyle,
        spellCheckResults,
      );

      expect(textSpanTree, equals(expectedTextSpanTree));
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.android,
      TargetPlatform.iOS,
    }),
  );

  testWidgets(
    'buildTextSpanWithSpellCheckSuggestions does not throw when text contains regex reserved characters',
    (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/136032.
      const String text = 'Hello, *ãresaa';
      const String resultsText = 'Hello, *ãresa';
      const TextEditingValue value = TextEditingValue(text: text);
      const bool composingRegionOutOfRange = false;
      const SpellCheckResults spellCheckResults = SpellCheckResults(resultsText, <SuggestionSpan>[
        SuggestionSpan(TextRange(start: 7, end: 12), <String>['*rangesa']),
      ]);

      const TextSpan expectedTextSpanTree = TextSpan(
        children: <TextSpan>[TextSpan(text: 'Hello, *ãresaa')],
      );
      final TextSpan textSpanTree = buildTextSpanWithSpellCheckSuggestions(
        value,
        composingRegionOutOfRange,
        null,
        misspelledTextStyle,
        spellCheckResults,
      );

      expect(tester.takeException(), null);
      expect(textSpanTree, equals(expectedTextSpanTree));
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.android,
      TargetPlatform.iOS,
    }),
  );

  testWidgets(
    'buildTextSpanWithSpellCheckSuggestions does not throw when text is being deleted',
    (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/152272.
      const String text = 'Lorem ipsum dolor ';
      const String resultsText = 'Lorem ipsum dolor sit ';
      const TextEditingValue value = TextEditingValue(
        text: text,
        selection: TextSelection.collapsed(offset: text.length),
      );
      const bool composingRegionOutOfRange = false;
      const SpellCheckResults spellCheckResults = SpellCheckResults(resultsText, <SuggestionSpan>[
        SuggestionSpan(TextRange(start: 0, end: 5), <String>['suggestion1', 'suggestion2']),
        SuggestionSpan(TextRange(start: 6, end: 11), <String>['suggestion1', 'suggestion2']),
        SuggestionSpan(TextRange(start: 12, end: 17), <String>['suggestion1', 'suggestion2']),
        SuggestionSpan(TextRange(start: 18, end: 21), <String>['suggestion1', 'suggestion2']),
      ]);

      final TextSpan expectedTextSpanTree = TextSpan(
        children: <TextSpan>[
          TextSpan(text: 'Lorem', style: misspelledTextStyle),
          const TextSpan(text: ' '),
          TextSpan(text: 'ipsum', style: misspelledTextStyle),
          const TextSpan(text: ' '),
          TextSpan(text: 'dolor', style: misspelledTextStyle),
          const TextSpan(text: ' '),
        ],
      );
      final TextSpan textSpanTree = buildTextSpanWithSpellCheckSuggestions(
        value,
        composingRegionOutOfRange,
        null,
        misspelledTextStyle,
        spellCheckResults,
      );

      expect(tester.takeException(), null);
      expect(textSpanTree, equals(expectedTextSpanTree));
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.android,
      TargetPlatform.iOS,
    }),
  );
}
