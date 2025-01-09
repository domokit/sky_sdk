// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';

import 'package:ui/src/engine.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  group('$WordBreaker', () {
    test('Does not go beyond the ends of a string', () {
      expect(WordBreaker.prevBreakIndex('foo', -1), 0);
      expect(WordBreaker.prevBreakIndex('foo', 0), 0);
      expect(WordBreaker.prevBreakIndex('foo', 'foo'.length + 1), 'foo'.length);

      expect(WordBreaker.nextBreakIndex('foo', -1), 0);
      expect(WordBreaker.nextBreakIndex('foo', 'foo'.length), 'foo'.length);
      expect(WordBreaker.nextBreakIndex('foo', 'foo'.length + 1), 'foo'.length);
    });

    test('Words and spaces', () {
      expectWords('foo bar', <String>['foo', ' ', 'bar']);
      expectWords('foo bar', <String>['foo', ' ', 'bar']);
    });

    test('Single-letter words', () {
      expectWords('foo a bar', <String>['foo', ' ', 'a', ' ', 'bar']);
      expectWords('a b c', <String>['a', ' ', 'b', ' ', 'c']);
      expectWords(' a b ', <String>[' ', 'a', ' ', 'b', ' ']);
    });

    test('Multiple consecutive spaces', () {
      // Different types of whitespace:
      final String oghamSpace = String.fromCharCode(0x1680);
      final String punctuationSpace = String.fromCharCode(0x2008);
      final String mathSpace = String.fromCharCode(0x205F);
      final String ideographicSpace = String.fromCharCode(0x3000);
      expectWords('foo$oghamSpace ${punctuationSpace}bar', <String>[
        'foo',
        '$oghamSpace $punctuationSpace',
        'bar',
      ]);
      expectWords('$mathSpace$ideographicSpace${oghamSpace}foo', <String>[
        '$mathSpace$ideographicSpace$oghamSpace',
        'foo',
      ]);
      expectWords('foo$punctuationSpace$mathSpace ', <String>[
        'foo',
        '$punctuationSpace$mathSpace ',
      ]);
      expectWords('$oghamSpace $punctuationSpace$mathSpace$ideographicSpace', <String>[
        '$oghamSpace $punctuationSpace$mathSpace$ideographicSpace',
      ]);
    });

    test('Punctuation', () {
      expectWords('foo,bar.baz', <String>['foo', ',', 'bar.baz']);
      expectWords('foo_bar', <String>['foo_bar']);
      expectWords('foo-bar', <String>['foo', '-', 'bar']);
    });

    test('Numeric', () {
      expectWords('12ab ab12', <String>['12ab', ' ', 'ab12']);
      expectWords('ab12,34cd', <String>['ab12,34cd']);
      expectWords('123,456_789.0', <String>['123,456_789.0']);
    });

    test('Quotes', () {
      expectWords("Mike's bike", <String>["Mike's", ' ', 'bike']);
      expectWords("Students' grades", <String>['Students', "'", ' ', 'grades']);
      expectWords('Joe said: "I\'m here"', <String>[
        'Joe',
        ' ',
        'said',
        ':',
        ' ',
        '"',
        "I'm",
        ' ',
        'here',
        '"',
      ]);
    });

    // Hebrew letters have the same rules as other letters, except
    // when they are around quotes. So we need to test those rules separately.
    test('Hebrew with quotes', () {
      // A few hebrew letters:
      const String h1 = 'א';
      const String h2 = 'ל';
      const String h3 = 'ט';

      // Test the single quote behavior that should be the same as other letters.
      expectWords("$h1$h2'$h3", <String>["$h1$h2'$h3"]);

      // Single quote following a hebrew shouldn't break.
      expectWords("$h1$h2'", <String>["$h1$h2'"]);
      expectWords("$h1$h2' $h3", <String>["$h1$h2'", ' ', h3]);

      // Double quotes around hebrew words should break.
      expectWords('"$h1$h3"', <String>['"', '$h1$h3', '"']);

      // Double quotes within hebrew words shouldn't break.
      expectWords('$h3"$h2', <String>['$h3"$h2']);
    });

    test('Newline, CR and LF', () {
      final String newline = String.fromCharCode(0x000B);
      // The only sequence of new lines that isn't a word boundary is CR×LF.
      expectWords('foo\r\nbar', <String>['foo', '\r\n', 'bar']);

      // All other sequences are considered word boundaries.

      expectWords('foo\n\nbar', <String>['foo', '\n', '\n', 'bar']);
      expectWords('foo\r\rbar', <String>['foo', '\r', '\r', 'bar']);
      expectWords('foo$newline${newline}bar', <String>['foo', newline, newline, 'bar']);

      expectWords('foo\n\rbar', <String>['foo', '\n', '\r', 'bar']);
      expectWords('foo$newline\rbar', <String>['foo', newline, '\r', 'bar']);
      expectWords('foo\r${newline}bar', <String>['foo', '\r', newline, 'bar']);
      expectWords('foo$newline\nbar', <String>['foo', newline, '\n', 'bar']);
      expectWords('foo\n${newline}bar', <String>['foo', '\n', newline, 'bar']);
    });

    test('katakana', () {
      // Katakana letters should stick together but not with other letters.
      expectWords('ナヌ', <String>['ナヌ']);
      expectWords('ナabcヌ', <String>['ナ', 'abc', 'ヌ']);

      // Katakana sometimes behaves the same as other letters.
      expectWords('ナ,ヌ_イ', <String>['ナ', ',', 'ヌ_イ']);

      // Katakana sometimes behaves differently from other letters.
      expectWords('ナ.ヌ', <String>['ナ', '.', 'ヌ']);
      expectWords("ナ'ヌ", <String>['ナ', "'", 'ヌ']);
      expectWords('ナ12ヌ', <String>['ナ', '12', 'ヌ']);
    });
  });
}

void expectWords(String text, List<String> expectedWords) {
  int strIndex = 0;

  // Forward word break lookup.
  for (final String word in expectedWords) {
    final int nextBreak = WordBreaker.nextBreakIndex(text, strIndex);
    expect(
      nextBreak,
      strIndex + word.length,
      reason: 'Forward word break lookup: expecting to move to the end of "$word" in "$text".',
    );
    strIndex += word.length;
  }

  // Backward word break lookup.
  strIndex = text.length;
  for (final String word in expectedWords.reversed) {
    final int prevBreak = WordBreaker.prevBreakIndex(text, strIndex);
    expect(
      prevBreak,
      strIndex - word.length,
      reason: 'Backward word break lookup: expecting to move to the start of "$word" in "$text".',
    );
    strIndex -= word.length;
  }
}
