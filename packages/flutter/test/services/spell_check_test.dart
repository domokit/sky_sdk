// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('SuggestionSpan.toString', () {
    const SuggestionSpan suggestionSpan = SuggestionSpan(TextRange(start: 12, end: 17), <String>[
      'weird',
    ]);

    expect(
      suggestionSpan.toString(),
      'SuggestionSpan(range: TextRange(start: 12, end: 17), suggestions: [weird])',
    );
  });

  test('SpellCheckResults.toString', () {
    const SuggestionSpan suggestionSpan = SuggestionSpan(TextRange(start: 12, end: 17), <String>[
      'weird',
    ]);
    const SpellCheckResults spellCheckResults = SpellCheckResults(
      'i before e except after c is so wierd.',
      <SuggestionSpan>[suggestionSpan],
    );

    expect(
      spellCheckResults.toString(),
      'SpellCheckResults(spellCheckText: i before e except after c is so wierd., suggestionSpans: [SuggestionSpan(range: TextRange(start: 12, end: 17), suggestions: [weird])])',
    );
  });
}
