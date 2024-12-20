// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

class _ConsistentTextRangeImplementationMatcher extends Matcher {
  _ConsistentTextRangeImplementationMatcher(int length)
    : range = TextRange(start: -1, end: length + 1),
      assert(length >= 0);

  final TextRange range;
  @override
  Description describe(Description description) {
    return description.add(
      'The implementation of TextBoundary.getTextBoundaryAt is consistent with its other methods.',
    );
  }

  @override
  Description describeMismatch(
    dynamic item,
    Description mismatchDescription,
    Map<dynamic, dynamic> matchState,
    bool verbose,
  ) {
    final TextBoundary boundary = matchState['textBoundary'] as TextBoundary;
    final int position = matchState['position'] as int;
    final int leading = boundary.getLeadingTextBoundaryAt(position) ?? -1;
    final int trailing = boundary.getTrailingTextBoundaryAt(position) ?? -1;

    return mismatchDescription.add(
      'at position $position, expected ${TextRange(start: leading, end: trailing)} but got ${boundary.getTextBoundaryAt(position)}',
    );
  }

  @override
  bool matches(dynamic item, Map<dynamic, dynamic> matchState) {
    for (int i = range.start; i <= range.end; i++) {
      final int? leading = (item as TextBoundary).getLeadingTextBoundaryAt(i);
      final int? trailing = item.getTrailingTextBoundaryAt(i);
      final TextRange boundary = item.getTextBoundaryAt(i);
      final bool consistent = boundary.start == (leading ?? -1) && boundary.end == (trailing ?? -1);
      if (!consistent) {
        matchState['textBoundary'] = item;
        matchState['position'] = i;
        return false;
      }
    }
    return true;
  }
}

Matcher _hasConsistentTextRangeImplementationWithinRange(int length) =>
    _ConsistentTextRangeImplementationMatcher(length);

void main() {
  test('Character boundary works', () {
    const CharacterBoundary boundary = CharacterBoundary('abc');
    expect(boundary, _hasConsistentTextRangeImplementationWithinRange(3));

    expect(boundary.getLeadingTextBoundaryAt(-1), null);
    expect(boundary.getTrailingTextBoundaryAt(-1), 0);

    expect(boundary.getLeadingTextBoundaryAt(0), 0);
    expect(boundary.getTrailingTextBoundaryAt(0), 1);

    expect(boundary.getLeadingTextBoundaryAt(1), 1);
    expect(boundary.getTrailingTextBoundaryAt(1), 2);

    expect(boundary.getLeadingTextBoundaryAt(2), 2);
    expect(boundary.getTrailingTextBoundaryAt(2), 3);

    expect(boundary.getLeadingTextBoundaryAt(3), 3);
    expect(boundary.getTrailingTextBoundaryAt(3), null);

    expect(boundary.getLeadingTextBoundaryAt(4), 3);
    expect(boundary.getTrailingTextBoundaryAt(4), null);
  });

  test('Character boundary works with grapheme', () {
    const String text = 'a❄︎c';
    const CharacterBoundary boundary = CharacterBoundary(text);
    expect(boundary, _hasConsistentTextRangeImplementationWithinRange(text.length));

    expect(boundary.getLeadingTextBoundaryAt(-1), null);
    expect(boundary.getTrailingTextBoundaryAt(-1), 0);

    expect(boundary.getLeadingTextBoundaryAt(0), 0);
    expect(boundary.getTrailingTextBoundaryAt(0), 1);

    // The `❄` takes two character length.
    expect(boundary.getLeadingTextBoundaryAt(1), 1);
    expect(boundary.getTrailingTextBoundaryAt(1), 3);

    expect(boundary.getLeadingTextBoundaryAt(2), 1);
    expect(boundary.getTrailingTextBoundaryAt(2), 3);

    expect(boundary.getLeadingTextBoundaryAt(3), 3);
    expect(boundary.getTrailingTextBoundaryAt(3), 4);

    expect(boundary.getLeadingTextBoundaryAt(text.length), text.length);
    expect(boundary.getTrailingTextBoundaryAt(text.length), null);
  });

  test('wordBoundary.moveByWordBoundary', () {
    const String text =
        'ABC   ABC\n' // [0, 10)
        'AÁ    Á\n' // [10, 20)
        '         \n' // [20, 30)
        'ABC!!!ABC\n' // [30, 40)
        '  !ABC !!\n' // [40, 50)
        'A  𑗋𑗋 A\n'; // [50, 60)

    final TextPainter textPainter =
        TextPainter()
          ..textDirection = TextDirection.ltr
          ..text = const TextSpan(text: text)
          ..layout();

    final TextBoundary boundary = textPainter.wordBoundaries.moveByWordBoundary;

    // 4 points to the 2nd whitespace in the first line.
    // Don't break between horizontal spaces and letters/numbers.
    expect(boundary.getLeadingTextBoundaryAt(4), 0);
    expect(boundary.getTrailingTextBoundaryAt(4), 9);

    // Works when words are starting/ending with a combining diacritical mark.
    expect(boundary.getLeadingTextBoundaryAt(14), 10);
    expect(boundary.getTrailingTextBoundaryAt(14), 19);

    // Do break before and after newlines.
    expect(boundary.getLeadingTextBoundaryAt(24), 20);
    expect(boundary.getTrailingTextBoundaryAt(24), 29);

    // Do not break on punctuations.
    expect(boundary.getLeadingTextBoundaryAt(34), 30);
    expect(boundary.getTrailingTextBoundaryAt(34), 39);

    // Ok to break if next to punctuations or separating spaces.
    expect(boundary.getLeadingTextBoundaryAt(44), 43);
    expect(boundary.getTrailingTextBoundaryAt(44), 46);

    // 44 points to a low surrogate of a punctuation.
    expect(boundary.getLeadingTextBoundaryAt(54), 50);
    expect(boundary.getTrailingTextBoundaryAt(54), 59);
  });

  test('line boundary works', () {
    final LineBoundary boundary = LineBoundary(TestTextLayoutMetrics());
    expect(boundary.getLeadingTextBoundaryAt(3), TestTextLayoutMetrics.lineAt3.start);
    expect(boundary.getTrailingTextBoundaryAt(3), TestTextLayoutMetrics.lineAt3.end);
    expect(boundary.getTextBoundaryAt(3), TestTextLayoutMetrics.lineAt3);
  });

  group('paragraph boundary', () {
    test('works for simple cases', () {
      const String textA = 'abcd efg hi\njklmno\npqrstuv';
      const ParagraphBoundary boundaryA = ParagraphBoundary(textA);

      // Position enclosed inside of paragraph, 'abcd efg h|i\n'.
      const int position = 10;

      // The range includes the line terminator.
      expect(boundaryA.getLeadingTextBoundaryAt(position), 0);
      expect(boundaryA.getTrailingTextBoundaryAt(position), 12);

      // This text includes a carriage return followed by a line feed.
      const String textB = 'abcd efg hi\r\njklmno\npqrstuv';
      const ParagraphBoundary boundaryB = ParagraphBoundary(textB);
      expect(boundaryB.getLeadingTextBoundaryAt(position), 0);
      expect(boundaryB.getTrailingTextBoundaryAt(position), 13);

      const String textF =
          'Now is the time for\n' // 20
          'all good people\n' // 20 + 16 => 36
          'to come to the aid\n' // 36 + 19 => 55
          'of their country.'; // 55 + 17 => 72
      const ParagraphBoundary boundaryF = ParagraphBoundary(textF);
      const int positionF = 11;
      expect(boundaryF.getLeadingTextBoundaryAt(positionF), 0);
      expect(boundaryF.getTrailingTextBoundaryAt(positionF), 20);
    });

    test('works for consecutive line terminators involving CRLF', () {
      const String textI =
          'Now is the time for\n' // 20
          'all good people\n\r\n' // 20 + 16 => 38
          'to come to the aid\n' // 38 + 19 => 57
          'of their country.'; // 57 + 17 => 74
      const ParagraphBoundary boundaryI = ParagraphBoundary(textI);
      const int positionI = 56; // \n at the end of the third line.
      const int positionJ = 38; // t at beginning of third line.
      const int positionK = 37; // \n at end of second line.
      expect(boundaryI.getLeadingTextBoundaryAt(positionI), 38);
      expect(boundaryI.getTrailingTextBoundaryAt(positionI), 57);
      expect(boundaryI.getLeadingTextBoundaryAt(positionJ), 38);
      expect(boundaryI.getTrailingTextBoundaryAt(positionJ), 57);
      expect(boundaryI.getLeadingTextBoundaryAt(positionK), 36);
      expect(boundaryI.getTrailingTextBoundaryAt(positionK), 38);
    });

    test('works for consecutive line terminators', () {
      const String textI =
          'Now is the time for\n' // 20
          'all good people\n\n' // 20 + 16 => 37
          'to come to the aid\n' // 37 + 19 => 56
          'of their country.'; // 56 + 17 => 73
      const ParagraphBoundary boundaryI = ParagraphBoundary(textI);
      const int positionI = 55; // \n at the end of the third line.
      const int positionJ = 37; // t at beginning of third line.
      const int positionK = 36; // \n at end of second line.
      expect(boundaryI.getLeadingTextBoundaryAt(positionI), 37);
      expect(boundaryI.getTrailingTextBoundaryAt(positionI), 56);
      expect(boundaryI.getLeadingTextBoundaryAt(positionJ), 37);
      expect(boundaryI.getTrailingTextBoundaryAt(positionJ), 56);
      expect(boundaryI.getLeadingTextBoundaryAt(positionK), 36);
      expect(boundaryI.getTrailingTextBoundaryAt(positionK), 37);
    });

    test('leading boundary works for consecutive CRLF', () {
      // This text includes multiple consecutive carriage returns followed by line feeds (CRLF).
      const String textH = 'abcd efg hi\r\n\r\n\r\n\r\n\r\n\r\n\r\n\n\n\n\n\njklmno\npqrstuv';
      const ParagraphBoundary boundaryH = ParagraphBoundary(textH);
      const int positionH = 18;
      expect(boundaryH.getLeadingTextBoundaryAt(positionH), 17);
      expect(boundaryH.getTrailingTextBoundaryAt(positionH), 19);
    });

    test('trailing boundary works for consecutive CRLF', () {
      // This text includes multiple consecutive carriage returns followed by line feeds (CRLF).
      const String textG = 'abcd efg hi\r\n\n\n\n\n\n\r\n\r\n\r\n\r\n\n\n\n\n\njklmno\npqrstuv';
      const ParagraphBoundary boundaryG = ParagraphBoundary(textG);
      const int positionG = 18;
      expect(boundaryG.getLeadingTextBoundaryAt(positionG), 18);
      expect(boundaryG.getTrailingTextBoundaryAt(positionG), 20);
    });

    test('works when position is between two CRLF', () {
      const String textE = 'abcd efg hi\r\nhello\r\n\n';
      const ParagraphBoundary boundaryE = ParagraphBoundary(textE);
      // Position enclosed inside of paragraph, 'abcd efg hi\r\nhello\r\n\n'.
      const int positionE = 16;
      expect(boundaryE.getLeadingTextBoundaryAt(positionE), 13);
      expect(boundaryE.getTrailingTextBoundaryAt(positionE), 20);
    });

    test('works for multiple consecutive line terminators', () {
      // This text includes multiple consecutive line terminators.
      const String textC = 'abcd efg hi\r\n\n\n\n\n\n\n\n\n\n\n\njklmno\npqrstuv';
      const ParagraphBoundary boundaryC = ParagraphBoundary(textC);
      // Position enclosed inside of paragraph, 'abcd efg hi\r\n\n\n\n\n\n|\n\n\n\n\n\njklmno\npqrstuv'.
      const int positionC = 18;
      expect(boundaryC.getLeadingTextBoundaryAt(positionC), 18);
      expect(boundaryC.getTrailingTextBoundaryAt(positionC), 19);

      const String textD = 'abcd efg hi\r\n\n\n\n';
      const ParagraphBoundary boundaryD = ParagraphBoundary(textD);
      // Position enclosed inside of paragraph, 'abcd efg hi\r\n\n|\n\n'.
      const int positionD = 14;
      expect(boundaryD.getLeadingTextBoundaryAt(positionD), 14);
      expect(boundaryD.getTrailingTextBoundaryAt(positionD), 15);
    });
  });

  test('document boundary works', () {
    const String text = 'abcd efg hi\njklmno\npqrstuv';
    const DocumentBoundary boundary = DocumentBoundary(text);
    expect(boundary, _hasConsistentTextRangeImplementationWithinRange(text.length));

    expect(boundary.getLeadingTextBoundaryAt(-1), null);
    expect(boundary.getTrailingTextBoundaryAt(-1), text.length);

    expect(boundary.getLeadingTextBoundaryAt(0), 0);
    expect(boundary.getTrailingTextBoundaryAt(0), text.length);

    expect(boundary.getLeadingTextBoundaryAt(10), 0);
    expect(boundary.getTrailingTextBoundaryAt(10), text.length);

    expect(boundary.getLeadingTextBoundaryAt(text.length), 0);
    expect(boundary.getTrailingTextBoundaryAt(text.length), null);

    expect(boundary.getLeadingTextBoundaryAt(text.length + 1), 0);
    expect(boundary.getTrailingTextBoundaryAt(text.length + 1), null);
  });
}

class TestTextLayoutMetrics extends TextLayoutMetrics {
  static const TextSelection lineAt3 = TextSelection(baseOffset: 0, extentOffset: 10);
  static const TextRange wordBoundaryAt3 = TextRange(start: 4, end: 7);

  @override
  TextSelection getLineAtOffset(TextPosition position) {
    if (position.offset == 3) {
      return lineAt3;
    }
    throw UnimplementedError();
  }

  @override
  TextPosition getTextPositionAbove(TextPosition position) {
    throw UnimplementedError();
  }

  @override
  TextPosition getTextPositionBelow(TextPosition position) {
    throw UnimplementedError();
  }

  @override
  TextRange getWordBoundary(TextPosition position) {
    if (position.offset == 3) {
      return wordBoundaryAt3;
    }
    throw UnimplementedError();
  }
}
