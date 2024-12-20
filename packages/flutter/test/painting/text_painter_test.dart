// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' as ui;

import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:leak_tracker_flutter_testing/leak_tracker_flutter_testing.dart';

void _checkCaretOffsetsLtrAt(String text, List<int> boundaries) {
  expect(boundaries.first, 0);
  expect(boundaries.last, text.length);

  final TextPainter painter = TextPainter()..textDirection = TextDirection.ltr;

  // Lay out the string up to each boundary, and record the width.
  final List<double> prefixWidths = <double>[];
  for (final int boundary in boundaries) {
    painter.text = TextSpan(text: text.substring(0, boundary));
    painter.layout();
    prefixWidths.add(painter.width);
  }

  // The painter has the full text laid out.  Check the caret offsets.
  double caretOffset(int offset) {
    final TextPosition position = ui.TextPosition(offset: offset);
    return painter.getOffsetForCaret(position, ui.Rect.zero).dx;
  }

  expect(boundaries.map(caretOffset).toList(), prefixWidths);
  double lastOffset = caretOffset(0);
  for (int i = 1; i <= text.length; i++) {
    final double offset = caretOffset(i);
    expect(offset, greaterThanOrEqualTo(lastOffset));
    lastOffset = offset;
  }
  painter.dispose();
}

/// Check the caret offsets are accurate for the given single line of LTR text.
///
/// This lays out the given text as a single line with [TextDirection.ltr]
/// and checks the following invariants, which should always hold if the text
/// is made up of LTR characters:
///  * The caret offsets go monotonically from 0.0 to the width of the text.
///  * At each character (that is, grapheme cluster) boundary, the caret
///    offset equals the width that the text up to that point would have
///    if laid out on its own.
///
/// If you have a [TextSpan] instead of a plain [String],
/// see [caretOffsetsForTextSpan].
void checkCaretOffsetsLtr(String text) {
  final List<int> characterBoundaries = <int>[];
  final CharacterRange range = CharacterRange.at(text, 0);
  while (true) {
    characterBoundaries.add(range.current.length);
    if (range.stringAfterLength <= 0) {
      break;
    }
    range.expandNext();
  }
  _checkCaretOffsetsLtrAt(text, characterBoundaries);
}

/// Check the caret offsets are accurate for the given single line of LTR text,
/// ignoring character boundaries within each given cluster.
///
/// This concatenates [clusters] into a string and then performs the same
/// checks as [checkCaretOffsetsLtr], except that instead of checking the
/// offset-equals-prefix-width invariant at every character boundary,
/// it does so only at the boundaries between the elements of [clusters].
///
/// The elements of [clusters] should be composed of whole characters: each
/// element should be a valid character range in the concatenated string.
///
/// Consider using [checkCaretOffsetsLtr] instead of this function.  If that
/// doesn't pass, you may have an instance of <https://github.com/flutter/flutter/issues/122478>.
void checkCaretOffsetsLtrFromPieces(List<String> clusters) {
  final StringBuffer buffer = StringBuffer();
  final List<int> boundaries = <int>[];
  boundaries.add(buffer.length);
  for (final String cluster in clusters) {
    buffer.write(cluster);
    boundaries.add(buffer.length);
  }
  _checkCaretOffsetsLtrAt(buffer.toString(), boundaries);
}

/// Compute the caret offsets for the given single line of text, a [TextSpan].
///
/// This lays out the given text as a single line with the given [textDirection]
/// and returns a full list of caret offsets, one at each code unit boundary.
///
/// This also checks that the offset at the very start or very end, if the text
/// direction is RTL or LTR respectively, equals the line's width.
///
/// If you have a [String] instead of a nontrivial [TextSpan],
/// consider using [checkCaretOffsetsLtr] instead.
List<double> caretOffsetsForTextSpan(TextDirection textDirection, TextSpan text) {
  final TextPainter painter =
      TextPainter()
        ..textDirection = textDirection
        ..text = text
        ..layout();
  final int length = text.toPlainText().length;
  final List<double> result = List<double>.generate(length + 1, (int offset) {
    final TextPosition position = ui.TextPosition(offset: offset);
    return painter.getOffsetForCaret(position, ui.Rect.zero).dx;
  });
  switch (textDirection) {
    case TextDirection.ltr:
      expect(result[length], painter.width);
    case TextDirection.rtl:
      expect(result[0], painter.width);
  }
  painter.dispose();
  return result;
}

void main() {
  group('caret', () {
    test('TextPainter caret test', () {
      final TextPainter painter = TextPainter()..textDirection = TextDirection.ltr;

      String text = 'A';
      checkCaretOffsetsLtr(text);

      painter.text = TextSpan(text: text);
      painter.layout();

      Offset caretOffset = painter.getOffsetForCaret(
        const ui.TextPosition(offset: 0),
        ui.Rect.zero,
      );
      expect(caretOffset.dx, 0);
      caretOffset = painter.getOffsetForCaret(ui.TextPosition(offset: text.length), ui.Rect.zero);
      expect(caretOffset.dx, painter.width);

      // Check that getOffsetForCaret handles a character that is encoded as a
      // surrogate pair.
      text = 'A\u{1F600}';
      checkCaretOffsetsLtr(text);
      painter.text = TextSpan(text: text);
      painter.layout();
      caretOffset = painter.getOffsetForCaret(ui.TextPosition(offset: text.length), ui.Rect.zero);
      expect(caretOffset.dx, painter.width);

      /// Verify the handling of spaces by SkParagraph and TextPainter.
      ///
      /// Test characters that are in the Unicode-Zs category but are not treated as whitespace characters by SkParagraph.
      /// The following character codes are intentionally excluded from the test target.
      ///   * '\u{00A0}' (no-break space)
      ///   * '\u{2007}' (figure space)
      ///   * '\u{202F}' (narrow no-break space)
      void verifyCharacterIsConsideredTrailingSpace(String character) {
        final String reason = 'character: ${character.codeUnitAt(0).toRadixString(16)}';

        text = 'A$character';
        checkCaretOffsetsLtr(text);
        painter.text = TextSpan(text: text);
        painter.layout();
        caretOffset = painter.getOffsetForCaret(const ui.TextPosition(offset: 0), ui.Rect.zero);
        expect(caretOffset.dx, 0.0, reason: reason);
        caretOffset = painter.getOffsetForCaret(const ui.TextPosition(offset: 1), ui.Rect.zero);
        expect(caretOffset.dx, 14.0, reason: reason);
        caretOffset = painter.getOffsetForCaret(ui.TextPosition(offset: text.length), ui.Rect.zero);
        expect(caretOffset.dx, painter.width, reason: reason);

        painter.layout(maxWidth: 14.0);
        final List<ui.LineMetrics> lines = painter.computeLineMetrics();
        expect(lines.length, 1, reason: reason);
        expect(lines.first.width, 14.0, reason: reason);
      }

      // Test with trailing space.
      verifyCharacterIsConsideredTrailingSpace('\u{0020}');

      // Test with trailing full-width space.
      verifyCharacterIsConsideredTrailingSpace('\u{3000}');

      // Test with trailing ogham space mark.
      verifyCharacterIsConsideredTrailingSpace('\u{1680}');

      // Test with trailing en quad.
      verifyCharacterIsConsideredTrailingSpace('\u{2000}');

      // Test with trailing em quad.
      verifyCharacterIsConsideredTrailingSpace('\u{2001}');

      // Test with trailing en space.
      verifyCharacterIsConsideredTrailingSpace('\u{2002}');

      // Test with trailing em space.
      verifyCharacterIsConsideredTrailingSpace('\u{2003}');

      // Test with trailing three-per-em space.
      verifyCharacterIsConsideredTrailingSpace('\u{2004}');

      // Test with trailing four-per-em space.
      verifyCharacterIsConsideredTrailingSpace('\u{2005}');

      // Test with trailing six-per-em space.
      verifyCharacterIsConsideredTrailingSpace('\u{2006}');

      // Test with trailing punctuation space.
      verifyCharacterIsConsideredTrailingSpace('\u{2008}');

      // Test with trailing thin space.
      verifyCharacterIsConsideredTrailingSpace('\u{2009}');

      // Test with trailing hair space.
      verifyCharacterIsConsideredTrailingSpace('\u{200A}');

      // Test with trailing medium mathematical space(MMSP).
      verifyCharacterIsConsideredTrailingSpace('\u{205F}');

      painter.dispose();
    }, skip: isBrowser && !isSkiaWeb); // https://github.com/flutter/flutter/issues/56308

    test('TextPainter caret test with WidgetSpan', () {
      // Regression test for https://github.com/flutter/flutter/issues/98458.
      final TextPainter painter = TextPainter()..textDirection = TextDirection.ltr;

      painter.text = const TextSpan(
        children: <InlineSpan>[
          TextSpan(text: 'before'),
          WidgetSpan(child: Text('widget')),
          TextSpan(text: 'after'),
        ],
      );
      painter.setPlaceholderDimensions(const <PlaceholderDimensions>[
        PlaceholderDimensions(
          size: Size(50, 30),
          baselineOffset: 25,
          alignment: ui.PlaceholderAlignment.bottom,
        ),
      ]);
      painter.layout();
      final Offset caretOffset = painter.getOffsetForCaret(
        ui.TextPosition(offset: painter.text!.toPlainText().length),
        ui.Rect.zero,
      );
      expect(caretOffset.dx, painter.width);
      painter.dispose();
    }, skip: isBrowser && !isSkiaWeb); // https://github.com/flutter/flutter/issues/56308

    test('TextPainter null text test', () {
      final TextPainter painter = TextPainter()..textDirection = TextDirection.ltr;

      List<TextSpan> children = <TextSpan>[const TextSpan(text: 'B'), const TextSpan(text: 'C')];
      painter.text = TextSpan(children: children);
      painter.layout();

      Offset caretOffset = painter.getOffsetForCaret(
        const ui.TextPosition(offset: 0),
        ui.Rect.zero,
      );
      expect(caretOffset.dx, 0);
      caretOffset = painter.getOffsetForCaret(const ui.TextPosition(offset: 1), ui.Rect.zero);
      expect(caretOffset.dx, painter.width / 2);
      caretOffset = painter.getOffsetForCaret(const ui.TextPosition(offset: 2), ui.Rect.zero);
      expect(caretOffset.dx, painter.width);

      children = <TextSpan>[];
      painter.text = TextSpan(children: children);
      painter.layout();

      caretOffset = painter.getOffsetForCaret(const ui.TextPosition(offset: 0), ui.Rect.zero);
      expect(caretOffset.dx, 0);
      caretOffset = painter.getOffsetForCaret(const ui.TextPosition(offset: 1), ui.Rect.zero);
      expect(caretOffset.dx, 0);
      painter.dispose();
    });

    test('TextPainter caret emoji test', () {
      final TextPainter painter = TextPainter()..textDirection = TextDirection.ltr;

      // Format: '👩‍<zwj>👩‍<zwj>👦👩‍<zwj>👩‍<zwj>👧‍<zwj>👧👏<modifier>'
      // One three-person family, one four-person family, one clapping hands (medium skin tone).
      const String text = '👩‍👩‍👦👩‍👩‍👧‍👧👏🏽';
      checkCaretOffsetsLtr(text);

      painter.text = const TextSpan(text: text);
      painter.layout(maxWidth: 10000);

      expect(text.length, 23);

      Offset caretOffset = painter.getOffsetForCaret(
        const ui.TextPosition(offset: 0),
        ui.Rect.zero,
      );
      expect(caretOffset.dx, 0); // 👩‍
      caretOffset = painter.getOffsetForCaret(
        const ui.TextPosition(offset: text.length),
        ui.Rect.zero,
      );
      expect(caretOffset.dx, painter.width);

      // Two UTF-16 codepoints per emoji, one codepoint per zwj
      caretOffset = painter.getOffsetForCaret(const ui.TextPosition(offset: 1), ui.Rect.zero);
      expect(caretOffset.dx, 42); // 👩‍
      caretOffset = painter.getOffsetForCaret(const ui.TextPosition(offset: 2), ui.Rect.zero);
      expect(caretOffset.dx, 42); // <zwj>
      caretOffset = painter.getOffsetForCaret(const ui.TextPosition(offset: 3), ui.Rect.zero);
      expect(caretOffset.dx, 42); // 👩‍
      caretOffset = painter.getOffsetForCaret(const ui.TextPosition(offset: 4), ui.Rect.zero);
      expect(caretOffset.dx, 42); // 👩‍
      caretOffset = painter.getOffsetForCaret(const ui.TextPosition(offset: 5), ui.Rect.zero);
      expect(caretOffset.dx, 42); // <zwj>
      caretOffset = painter.getOffsetForCaret(const ui.TextPosition(offset: 6), ui.Rect.zero);
      expect(caretOffset.dx, 42); // 👦
      caretOffset = painter.getOffsetForCaret(const ui.TextPosition(offset: 7), ui.Rect.zero);
      expect(caretOffset.dx, 42); // 👦
      caretOffset = painter.getOffsetForCaret(const ui.TextPosition(offset: 8), ui.Rect.zero);
      expect(caretOffset.dx, 42); // 👩‍
      caretOffset = painter.getOffsetForCaret(const ui.TextPosition(offset: 9), ui.Rect.zero);
      expect(caretOffset.dx, 98); // 👩‍
      caretOffset = painter.getOffsetForCaret(const ui.TextPosition(offset: 10), ui.Rect.zero);
      expect(caretOffset.dx, 98); // <zwj>
      caretOffset = painter.getOffsetForCaret(const ui.TextPosition(offset: 11), ui.Rect.zero);
      expect(caretOffset.dx, 98); // 👩‍
      caretOffset = painter.getOffsetForCaret(const ui.TextPosition(offset: 12), ui.Rect.zero);
      expect(caretOffset.dx, 98); // 👩‍
      caretOffset = painter.getOffsetForCaret(const ui.TextPosition(offset: 13), ui.Rect.zero);
      expect(caretOffset.dx, 98); // <zwj>
      caretOffset = painter.getOffsetForCaret(const ui.TextPosition(offset: 14), ui.Rect.zero);
      expect(caretOffset.dx, 98); // 👧‍
      caretOffset = painter.getOffsetForCaret(const ui.TextPosition(offset: 15), ui.Rect.zero);
      expect(caretOffset.dx, 98); // 👧‍
      caretOffset = painter.getOffsetForCaret(const ui.TextPosition(offset: 16), ui.Rect.zero);
      expect(caretOffset.dx, 98); // <zwj>
      caretOffset = painter.getOffsetForCaret(const ui.TextPosition(offset: 17), ui.Rect.zero);
      expect(caretOffset.dx, 98); // 👧
      caretOffset = painter.getOffsetForCaret(const ui.TextPosition(offset: 18), ui.Rect.zero);
      expect(caretOffset.dx, 98); // 👧
      caretOffset = painter.getOffsetForCaret(const ui.TextPosition(offset: 19), ui.Rect.zero);
      expect(caretOffset.dx, 98); // 👏
      caretOffset = painter.getOffsetForCaret(const ui.TextPosition(offset: 20), ui.Rect.zero);
      expect(caretOffset.dx, 126); // 👏
      caretOffset = painter.getOffsetForCaret(const ui.TextPosition(offset: 21), ui.Rect.zero);
      expect(caretOffset.dx, 126); // <medium skin tone modifier>
      caretOffset = painter.getOffsetForCaret(const ui.TextPosition(offset: 22), ui.Rect.zero);
      expect(caretOffset.dx, 126); // <medium skin tone modifier>
      caretOffset = painter.getOffsetForCaret(const ui.TextPosition(offset: 23), ui.Rect.zero);
      expect(caretOffset.dx, 126); // end of string
      painter.dispose();
    }, skip: isBrowser && !isSkiaWeb); // https://github.com/flutter/flutter/issues/56308

    test('TextPainter caret emoji tests: single, long emoji', () {
      // Regression test for https://github.com/flutter/flutter/issues/50563
      checkCaretOffsetsLtr('👩‍🚀');
      checkCaretOffsetsLtr('👩‍❤️‍💋‍👩');
      checkCaretOffsetsLtr('👨‍👩‍👦‍👦');
      checkCaretOffsetsLtr('👨🏾‍🤝‍👨🏻');
      checkCaretOffsetsLtr('👨‍👦');
      checkCaretOffsetsLtr('👩‍👦');
      checkCaretOffsetsLtr('🏌🏿‍♀️');
      checkCaretOffsetsLtr('🏊‍♀️');
      checkCaretOffsetsLtr('🏄🏻‍♂️');

      // These actually worked even before #50563 was fixed (because
      // their lengths in code units are powers of 2, namely 4 and 8).
      checkCaretOffsetsLtr('🇺🇳');
      checkCaretOffsetsLtr('👩‍❤️‍👨');
    }, skip: isBrowser && !isSkiaWeb); // https://github.com/flutter/flutter/issues/56308

    test(
      'TextPainter caret emoji test: letters, then 1 emoji of 5 code units',
      () {
        // Regression test for https://github.com/flutter/flutter/issues/50563
        checkCaretOffsetsLtr('a👩‍🚀');
        checkCaretOffsetsLtr('ab👩‍🚀');
        checkCaretOffsetsLtr('abc👩‍🚀');
        checkCaretOffsetsLtr('abcd👩‍🚀');
      },
      skip: isBrowser && !isSkiaWeb, // https://github.com/flutter/flutter/issues/56308
    );

    test('TextPainter caret zalgo test', () {
      // Regression test for https://github.com/flutter/flutter/issues/98516
      checkCaretOffsetsLtr('Z͉̳̺ͥͬ̾a̴͕̲̒̒͌̋ͪl̨͎̰̘͉̟ͤ̀̈̚͜g͕͔̤͖̟̒͝ͅo̵̡̡̼͚̐ͯ̅ͪ̆ͣ̚');
    }, skip: isBrowser && !isSkiaWeb); // https://github.com/flutter/flutter/issues/56308

    test('TextPainter caret Devanagari test', () {
      // Regression test for https://github.com/flutter/flutter/issues/118403
      checkCaretOffsetsLtrFromPieces(<String>[
        'प्रा',
        'प्त',
        ' ',
        'व',
        'र्ण',
        'न',
        ' ',
        'प्र',
        'व्रु',
        'ति',
      ]);
    }, skip: isBrowser && !isSkiaWeb); // https://github.com/flutter/flutter/issues/56308

    test('TextPainter caret Devanagari test, full strength', () {
      // Regression test for https://github.com/flutter/flutter/issues/118403
      checkCaretOffsetsLtr('प्राप्त वर्णन प्रव्रुति');
    }, skip: true); // https://github.com/flutter/flutter/issues/122478

    test(
      'TextPainter caret emoji test LTR: letters next to emoji, as separate TextBoxes',
      () {
        // Regression test for https://github.com/flutter/flutter/issues/122477
        // The trigger for this bug was to have SkParagraph report separate
        // TextBoxes for the emoji and for the characters next to it.
        // In normal usage on a real device, this can happen by simply typing
        // letters and then an emoji, presumably because they get different fonts.
        // In these tests, our single test font covers both letters and emoji,
        // so we provoke the same effect by adding styles.
        expect(
          caretOffsetsForTextSpan(
            TextDirection.ltr,
            const TextSpan(
              children: <TextSpan>[
                TextSpan(text: '👩‍🚀', style: TextStyle()),
                TextSpan(text: ' words', style: TextStyle(fontWeight: FontWeight.bold)),
              ],
            ),
          ),
          <double>[0, 28, 28, 28, 28, 28, 42, 56, 70, 84, 98, 112],
        );
        expect(
          caretOffsetsForTextSpan(
            TextDirection.ltr,
            const TextSpan(
              children: <TextSpan>[
                TextSpan(text: 'words ', style: TextStyle(fontWeight: FontWeight.bold)),
                TextSpan(text: '👩‍🚀', style: TextStyle()),
              ],
            ),
          ),
          <double>[0, 14, 28, 42, 56, 70, 84, 112, 112, 112, 112, 112],
        );
      },
      skip: isBrowser && !isSkiaWeb, // https://github.com/flutter/flutter/issues/56308
    );

    test(
      'TextPainter caret emoji test RTL: letters next to emoji, as separate TextBoxes',
      () {
        // Regression test for https://github.com/flutter/flutter/issues/122477
        expect(
          caretOffsetsForTextSpan(
            TextDirection.rtl,
            const TextSpan(
              children: <TextSpan>[
                TextSpan(text: '👩‍🚀', style: TextStyle()),
                TextSpan(text: ' מילים', style: TextStyle(fontWeight: FontWeight.bold)),
              ],
            ),
          ),
          <double>[112, 84, 84, 84, 84, 84, 70, 56, 42, 28, 14, 0],
        );
        expect(
          caretOffsetsForTextSpan(
            TextDirection.rtl,
            const TextSpan(
              children: <TextSpan>[
                TextSpan(text: 'מילים ', style: TextStyle(fontWeight: FontWeight.bold)),
                TextSpan(text: '👩‍🚀', style: TextStyle()),
              ],
            ),
          ),
          <double>[112, 98, 84, 70, 56, 42, 28, 0, 0, 0, 0, 0],
        );
      },
      skip: isBrowser && !isSkiaWeb, // https://github.com/flutter/flutter/issues/56308
    );

    test('TextPainter caret center space test', () {
      final TextPainter painter = TextPainter()..textDirection = TextDirection.ltr;

      const String text = 'test text with space at end   ';
      painter.text = const TextSpan(text: text);
      painter.textAlign = TextAlign.center;
      painter.layout();

      Offset caretOffset = painter.getOffsetForCaret(
        const ui.TextPosition(offset: 0),
        ui.Rect.zero,
      );
      expect(caretOffset.dx, 21);
      caretOffset = painter.getOffsetForCaret(
        const ui.TextPosition(offset: text.length),
        ui.Rect.zero,
      );
      // The end of the line is 441, but the width is only 420, so the cursor is
      // stopped there without overflowing.
      expect(caretOffset.dx, painter.width);

      caretOffset = painter.getOffsetForCaret(const ui.TextPosition(offset: 1), ui.Rect.zero);
      expect(caretOffset.dx, 35);
      caretOffset = painter.getOffsetForCaret(const ui.TextPosition(offset: 2), ui.Rect.zero);
      expect(caretOffset.dx, 49);
      painter.dispose();
    }, skip: isBrowser && !isSkiaWeb); // https://github.com/flutter/flutter/issues/56308

    test('TextPainter caret height and line height', () {
      final TextPainter painter =
          TextPainter()
            ..textDirection = TextDirection.ltr
            ..strutStyle = const StrutStyle(fontSize: 50.0);

      const String text = 'A';
      painter.text = const TextSpan(text: text, style: TextStyle(height: 1.0));
      painter.layout();

      final double caretHeight = painter.getFullHeightForCaret(
        const ui.TextPosition(offset: 0),
        ui.Rect.zero,
      );
      expect(caretHeight, 50.0);
      painter.dispose();
    }, skip: isBrowser && !isSkiaWeb); // https://github.com/flutter/flutter/issues/56308

    test('upstream downstream makes no difference in the same line within the same bidi run', () {
      final TextPainter painter =
          TextPainter(textDirection: TextDirection.ltr)
            ..text = const TextSpan(text: 'aa')
            ..layout();

      final Rect largeRect = Offset.zero & const Size.square(5);
      expect(
        painter.getOffsetForCaret(const TextPosition(offset: 1), largeRect),
        painter.getOffsetForCaret(
          const TextPosition(offset: 1, affinity: TextAffinity.upstream),
          largeRect,
        ),
      );
    });

    test('trailing newlines', () {
      const double fontSize = 14.0;
      final TextPainter painter = TextPainter();
      final Rect largeRect = Offset.zero & const Size.square(5);
      String text = 'a    ';
      painter
        ..text = TextSpan(text: text)
        ..textDirection = TextDirection.ltr
        ..layout(minWidth: 1000.0, maxWidth: 1000.0);
      expect(
        painter.getOffsetForCaret(TextPosition(offset: text.length), largeRect).dx,
        text.length * fontSize,
      );

      text = 'ل    ';
      painter
        ..text = TextSpan(text: text)
        ..textDirection = TextDirection.rtl
        ..layout(minWidth: 1000.0, maxWidth: 1000.0);
      expect(
        painter.getOffsetForCaret(TextPosition(offset: text.length), largeRect).dx,
        1000 - text.length * fontSize - largeRect.width,
      );
    }, skip: isBrowser && !isSkiaWeb); // https://github.com/flutter/flutter/issues/56308

    test('End of text caret when the text ends with +1 bidi level', () {
      const double fontSize = 14.0;
      final TextPainter painter = TextPainter();
      final Rect largeRect = Offset.zero & const Size.square(5);
      const String text = 'aل';
      painter
        ..text = const TextSpan(text: text)
        ..textDirection = TextDirection.ltr
        ..layout(minWidth: 1000.0, maxWidth: 1000.0);

      expect(painter.getOffsetForCaret(const TextPosition(offset: 0), largeRect).dx, 0.0);
      expect(
        painter.getOffsetForCaret(const TextPosition(offset: 1), largeRect).dx,
        fontSize * 2 - largeRect.width,
      );
      expect(painter.getOffsetForCaret(const TextPosition(offset: 2), largeRect).dx, fontSize * 2);
    }, skip: isBrowser && !isSkiaWeb); // https://github.com/flutter/flutter/issues/56308

    test('handles newlines properly', () {
      final TextPainter painter = TextPainter()..textDirection = TextDirection.ltr;

      const double SIZE_OF_A = 14.0; // square size of "a" character
      String text = 'aaa';
      painter.text = TextSpan(text: text);
      painter.layout();

      // getOffsetForCaret in a plain one-line string is the same for either affinity.
      int offset = 0;
      painter.text = TextSpan(text: text);
      painter.layout();
      Offset caretOffset = painter.getOffsetForCaret(ui.TextPosition(offset: offset), ui.Rect.zero);
      expect(caretOffset.dx, SIZE_OF_A * offset);
      expect(caretOffset.dy, 0.0);
      caretOffset = painter.getOffsetForCaret(
        ui.TextPosition(offset: offset, affinity: ui.TextAffinity.upstream),
        ui.Rect.zero,
      );
      expect(caretOffset.dx, SIZE_OF_A * offset);
      expect(caretOffset.dy, 0.0);
      offset = 1;
      caretOffset = painter.getOffsetForCaret(ui.TextPosition(offset: offset), ui.Rect.zero);
      expect(caretOffset.dx, SIZE_OF_A * offset);
      expect(caretOffset.dy, 0.0);
      caretOffset = painter.getOffsetForCaret(
        ui.TextPosition(offset: offset, affinity: ui.TextAffinity.upstream),
        ui.Rect.zero,
      );
      expect(caretOffset.dx, SIZE_OF_A * offset);
      expect(caretOffset.dy, 0.0);
      offset = 2;
      caretOffset = painter.getOffsetForCaret(ui.TextPosition(offset: offset), ui.Rect.zero);
      expect(caretOffset.dx, SIZE_OF_A * offset);
      expect(caretOffset.dy, 0.0);
      caretOffset = painter.getOffsetForCaret(
        ui.TextPosition(offset: offset, affinity: ui.TextAffinity.upstream),
        ui.Rect.zero,
      );
      expect(caretOffset.dx, SIZE_OF_A * offset);
      expect(caretOffset.dy, 0.0);
      offset = 3;
      caretOffset = painter.getOffsetForCaret(ui.TextPosition(offset: offset), ui.Rect.zero);
      expect(caretOffset.dx, SIZE_OF_A * offset);
      expect(caretOffset.dy, 0.0);
      caretOffset = painter.getOffsetForCaret(
        ui.TextPosition(offset: offset, affinity: ui.TextAffinity.upstream),
        ui.Rect.zero,
      );
      expect(caretOffset.dx, SIZE_OF_A * offset);
      expect(caretOffset.dy, 0.0);

      // For explicit newlines, getOffsetForCaret places the caret at the location
      // indicated by offset regardless of affinity.
      text = '\n\n';
      painter.text = TextSpan(text: text);
      painter.layout();
      offset = 0;
      caretOffset = painter.getOffsetForCaret(ui.TextPosition(offset: offset), ui.Rect.zero);
      expect(caretOffset.dx, 0.0);
      expect(caretOffset.dy, 0.0);
      caretOffset = painter.getOffsetForCaret(
        ui.TextPosition(offset: offset, affinity: ui.TextAffinity.upstream),
        ui.Rect.zero,
      );
      expect(caretOffset.dx, 0.0);
      expect(caretOffset.dy, 0.0);
      offset = 1;
      caretOffset = painter.getOffsetForCaret(ui.TextPosition(offset: offset), ui.Rect.zero);
      expect(caretOffset.dx, 0.0);
      expect(caretOffset.dy, SIZE_OF_A);
      caretOffset = painter.getOffsetForCaret(
        ui.TextPosition(offset: offset, affinity: ui.TextAffinity.upstream),
        ui.Rect.zero,
      );
      expect(caretOffset.dx, 0.0);
      expect(caretOffset.dy, SIZE_OF_A);
      offset = 2;
      caretOffset = painter.getOffsetForCaret(ui.TextPosition(offset: offset), ui.Rect.zero);
      expect(caretOffset.dx, 0.0);
      expect(caretOffset.dy, SIZE_OF_A * 2);
      caretOffset = painter.getOffsetForCaret(
        ui.TextPosition(offset: offset, affinity: ui.TextAffinity.upstream),
        ui.Rect.zero,
      );
      expect(caretOffset.dx, 0.0);
      expect(caretOffset.dy, SIZE_OF_A * 2);

      // getOffsetForCaret in an unwrapped string with explicit newlines is the
      // same for either affinity.
      text = '\naaa';
      painter.text = TextSpan(text: text);
      painter.layout();
      offset = 0;
      caretOffset = painter.getOffsetForCaret(ui.TextPosition(offset: offset), ui.Rect.zero);
      expect(caretOffset.dx, 0.0);
      expect(caretOffset.dy, 0.0);
      caretOffset = painter.getOffsetForCaret(
        ui.TextPosition(offset: offset, affinity: ui.TextAffinity.upstream),
        ui.Rect.zero,
      );
      expect(caretOffset.dx, 0.0);
      expect(caretOffset.dy, 0.0);
      offset = 1;
      caretOffset = painter.getOffsetForCaret(ui.TextPosition(offset: offset), ui.Rect.zero);
      expect(caretOffset.dx, 0.0);
      expect(caretOffset.dy, SIZE_OF_A);
      caretOffset = painter.getOffsetForCaret(
        ui.TextPosition(offset: offset, affinity: ui.TextAffinity.upstream),
        ui.Rect.zero,
      );
      expect(caretOffset.dx, 0.0);
      expect(caretOffset.dy, SIZE_OF_A);

      // When text wraps on its own, getOffsetForCaret disambiguates between the
      // end of one line and start of next using affinity.
      text = 'aaaaaaaa'; // Just enough to wrap one character down to second line
      painter.text = TextSpan(text: text);
      painter.layout(maxWidth: 100); // SIZE_OF_A * text.length > 100, so it wraps
      caretOffset = painter.getOffsetForCaret(
        ui.TextPosition(offset: text.length - 1),
        ui.Rect.zero,
      );
      // When affinity is downstream, cursor is at beginning of second line
      expect(caretOffset.dx, 0.0);
      expect(caretOffset.dy, SIZE_OF_A);
      caretOffset = painter.getOffsetForCaret(
        ui.TextPosition(offset: text.length - 1, affinity: ui.TextAffinity.upstream),
        ui.Rect.zero,
      );
      // When affinity is upstream, cursor is at end of first line
      expect(caretOffset.dx, 98.0);
      expect(caretOffset.dy, 0.0);

      // When given a string with a newline at the end, getOffsetForCaret puts
      // the cursor at the start of the next line regardless of affinity
      text = 'aaa\n';
      painter.text = TextSpan(text: text);
      painter.layout();
      caretOffset = painter.getOffsetForCaret(ui.TextPosition(offset: text.length), ui.Rect.zero);
      expect(caretOffset.dx, 0.0);
      expect(caretOffset.dy, SIZE_OF_A);
      offset = text.length;
      caretOffset = painter.getOffsetForCaret(
        ui.TextPosition(offset: offset, affinity: TextAffinity.upstream),
        ui.Rect.zero,
      );
      expect(caretOffset.dx, 0.0);
      expect(caretOffset.dy, SIZE_OF_A);

      // Given a one-line right aligned string, positioning the cursor at offset 0
      // means that it appears at the "end" of the string, after the character
      // that was typed first, at x=0.
      painter.textAlign = TextAlign.right;
      text = 'aaa';
      painter.text = TextSpan(text: text);
      painter.layout();
      offset = 0;
      caretOffset = painter.getOffsetForCaret(ui.TextPosition(offset: offset), ui.Rect.zero);
      expect(caretOffset.dx, 0.0);
      expect(caretOffset.dy, 0.0);
      painter.textAlign = TextAlign.left;

      // When given an offset after a newline in the middle of a string,
      // getOffsetForCaret returns the start of the next line regardless of
      // affinity.
      text = 'aaa\naaa';
      painter.text = TextSpan(text: text);
      painter.layout();
      offset = 4;
      caretOffset = painter.getOffsetForCaret(ui.TextPosition(offset: offset), ui.Rect.zero);
      expect(caretOffset.dx, 0.0);
      expect(caretOffset.dy, SIZE_OF_A);
      caretOffset = painter.getOffsetForCaret(
        ui.TextPosition(offset: offset, affinity: TextAffinity.upstream),
        ui.Rect.zero,
      );
      expect(caretOffset.dx, 0.0);
      expect(caretOffset.dy, SIZE_OF_A);

      // When given a string with multiple trailing newlines, places the caret
      // in the position given by offset regardless of affinity.
      text = 'aaa\n\n\n';
      offset = 3;
      caretOffset = painter.getOffsetForCaret(ui.TextPosition(offset: offset), ui.Rect.zero);
      expect(caretOffset.dx, SIZE_OF_A * 3);
      expect(caretOffset.dy, 0.0);
      caretOffset = painter.getOffsetForCaret(
        ui.TextPosition(offset: offset, affinity: TextAffinity.upstream),
        ui.Rect.zero,
      );
      expect(caretOffset.dx, SIZE_OF_A * 3);
      expect(caretOffset.dy, 0.0);

      offset = 4;
      painter.text = TextSpan(text: text);
      painter.layout();
      caretOffset = painter.getOffsetForCaret(ui.TextPosition(offset: offset), ui.Rect.zero);
      expect(caretOffset.dx, 0.0);
      expect(caretOffset.dy, SIZE_OF_A);
      caretOffset = painter.getOffsetForCaret(
        ui.TextPosition(offset: offset, affinity: TextAffinity.upstream),
        ui.Rect.zero,
      );
      expect(caretOffset.dx, 0.0);
      expect(caretOffset.dy, SIZE_OF_A);

      offset = 5;
      caretOffset = painter.getOffsetForCaret(ui.TextPosition(offset: offset), ui.Rect.zero);
      expect(caretOffset.dx, 0.0);
      expect(caretOffset.dy, SIZE_OF_A * 2);
      caretOffset = painter.getOffsetForCaret(
        ui.TextPosition(offset: offset, affinity: TextAffinity.upstream),
        ui.Rect.zero,
      );
      expect(caretOffset.dx, 0.0);
      expect(caretOffset.dy, SIZE_OF_A * 2);

      offset = 6;
      caretOffset = painter.getOffsetForCaret(ui.TextPosition(offset: offset), ui.Rect.zero);
      expect(caretOffset.dx, 0.0);
      expect(caretOffset.dy, SIZE_OF_A * 3);

      caretOffset = painter.getOffsetForCaret(
        ui.TextPosition(offset: offset, affinity: TextAffinity.upstream),
        ui.Rect.zero,
      );
      expect(caretOffset.dx, 0.0);
      expect(caretOffset.dy, SIZE_OF_A * 3);

      // When given a string with multiple leading newlines, places the caret in
      // the position given by offset regardless of affinity.
      text = '\n\n\naaa';
      offset = 3;
      painter.text = TextSpan(text: text);
      painter.layout();
      caretOffset = painter.getOffsetForCaret(ui.TextPosition(offset: offset), ui.Rect.zero);
      expect(caretOffset.dx, 0.0);
      expect(caretOffset.dy, SIZE_OF_A * 3);
      caretOffset = painter.getOffsetForCaret(
        ui.TextPosition(offset: offset, affinity: TextAffinity.upstream),
        ui.Rect.zero,
      );
      expect(caretOffset.dx, 0.0);
      expect(caretOffset.dy, SIZE_OF_A * 3);

      offset = 2;
      caretOffset = painter.getOffsetForCaret(ui.TextPosition(offset: offset), ui.Rect.zero);
      expect(caretOffset.dx, 0.0);
      expect(caretOffset.dy, SIZE_OF_A * 2);
      caretOffset = painter.getOffsetForCaret(
        ui.TextPosition(offset: offset, affinity: TextAffinity.upstream),
        ui.Rect.zero,
      );
      expect(caretOffset.dx, 0.0);
      expect(caretOffset.dy, SIZE_OF_A * 2);

      offset = 1;
      caretOffset = painter.getOffsetForCaret(ui.TextPosition(offset: offset), ui.Rect.zero);
      expect(caretOffset.dx, 0.0);
      expect(caretOffset.dy, SIZE_OF_A);
      caretOffset = painter.getOffsetForCaret(
        ui.TextPosition(offset: offset, affinity: TextAffinity.upstream),
        ui.Rect.zero,
      );
      expect(caretOffset.dx, 0.0);
      expect(caretOffset.dy, SIZE_OF_A);

      offset = 0;
      caretOffset = painter.getOffsetForCaret(ui.TextPosition(offset: offset), ui.Rect.zero);
      expect(caretOffset.dx, 0.0);
      expect(caretOffset.dy, 0.0);
      caretOffset = painter.getOffsetForCaret(
        ui.TextPosition(offset: offset, affinity: TextAffinity.upstream),
        ui.Rect.zero,
      );
      expect(caretOffset.dx, 0.0);
      expect(caretOffset.dy, 0.0);
      painter.dispose();
    });

    test('caret height reflects run height if strut is disabled', () {
      const TextSpan span = TextSpan(
        text: 'M',
        style: TextStyle(fontSize: 128),
        children: <InlineSpan>[
          TextSpan(text: 'M', style: TextStyle(fontSize: 32)),
          TextSpan(text: 'M', style: TextStyle(fontSize: 64)),
        ],
      );
      final TextPainter painter =
          TextPainter()
            ..textDirection = TextDirection.ltr
            ..text = span
            ..layout();

      expect(
        painter.getFullHeightForCaret(
          const TextPosition(offset: 0, affinity: ui.TextAffinity.upstream),
          Rect.zero,
        ),
        128.0,
      );
      expect(painter.getFullHeightForCaret(const TextPosition(offset: 0), Rect.zero), 128.0);
      expect(
        painter.getFullHeightForCaret(
          const TextPosition(offset: 1, affinity: ui.TextAffinity.upstream),
          Rect.zero,
        ),
        128.0,
      );
      expect(painter.getFullHeightForCaret(const TextPosition(offset: 1), Rect.zero), 32.0);
      expect(
        painter.getFullHeightForCaret(
          const TextPosition(offset: 2, affinity: ui.TextAffinity.upstream),
          Rect.zero,
        ),
        32.0,
      );
      expect(painter.getFullHeightForCaret(const TextPosition(offset: 2), Rect.zero), 64.0);
      expect(
        painter.getFullHeightForCaret(
          const TextPosition(offset: 3, affinity: ui.TextAffinity.upstream),
          Rect.zero,
        ),
        64.0,
      );
      expect(painter.getFullHeightForCaret(const TextPosition(offset: 3), Rect.zero), 128.0);

      painter.dispose();
    });
  });

  test('TextPainter error test', () {
    final TextPainter painter = TextPainter(textDirection: TextDirection.ltr);

    expect(
      () => painter.paint(MockCanvas(), Offset.zero),
      throwsA(
        isA<StateError>().having(
          (StateError error) => error.message,
          'message',
          contains('TextPainter.paint called when text geometry was not yet calculated'),
        ),
      ),
    );
    painter.dispose();
  });

  test('TextPainter requires textDirection', () {
    final TextPainter painter1 = TextPainter(text: const TextSpan(text: ''));
    expect(painter1.layout, throwsStateError);
    final TextPainter painter2 = TextPainter(
      text: const TextSpan(text: ''),
      textDirection: TextDirection.rtl,
    );
    expect(painter2.layout, isNot(throwsStateError));
  });

  test('TextPainter size test', () {
    final TextPainter painter = TextPainter(
      text: const TextSpan(text: 'X', style: TextStyle(inherit: false, fontSize: 123.0)),
      textDirection: TextDirection.ltr,
    );
    painter.layout();
    expect(painter.size, const Size(123.0, 123.0));
    painter.dispose();
  });

  test('TextPainter textScaler test', () {
    final TextPainter painter = TextPainter(
      text: const TextSpan(text: 'X', style: TextStyle(inherit: false, fontSize: 10.0)),
      textDirection: TextDirection.ltr,
      textScaler: const TextScaler.linear(2.0),
    );
    painter.layout();
    expect(painter.size, const Size(20.0, 20.0));
    painter.dispose();
  });

  test('TextPainter textScaler null style test', () {
    final TextPainter painter = TextPainter(
      text: const TextSpan(text: 'X'),
      textDirection: TextDirection.ltr,
      textScaler: const TextScaler.linear(2.0),
    );
    painter.layout();
    expect(painter.size, const Size(28.0, 28.0));
    painter.dispose();
  });

  test('TextPainter default text height is 14 pixels', () {
    final TextPainter painter = TextPainter(
      text: const TextSpan(text: 'x'),
      textDirection: TextDirection.ltr,
    );
    painter.layout();
    expect(painter.preferredLineHeight, 14.0);
    expect(painter.size, const Size(14.0, 14.0));
    painter.dispose();
  });

  test('TextPainter sets paragraph size from root', () {
    final TextPainter painter = TextPainter(
      text: const TextSpan(text: 'x', style: TextStyle(fontSize: 100.0)),
      textDirection: TextDirection.ltr,
    );
    painter.layout();
    expect(painter.preferredLineHeight, 100.0);
    expect(painter.size, const Size(100.0, 100.0));
    painter.dispose();
  });

  test('TextPainter intrinsic dimensions', () {
    const TextStyle style = TextStyle(inherit: false, fontSize: 10.0);
    TextPainter painter;

    painter = TextPainter(
      text: const TextSpan(text: 'X X X', style: style),
      textDirection: TextDirection.ltr,
    );
    painter.layout();
    expect(painter.size, const Size(50.0, 10.0));
    expect(painter.minIntrinsicWidth, 10.0);
    expect(painter.maxIntrinsicWidth, 50.0);
    painter.dispose();

    painter = TextPainter(
      text: const TextSpan(text: 'X X X', style: style),
      textDirection: TextDirection.ltr,
      ellipsis: 'e',
    );
    painter.layout();
    expect(painter.size, const Size(50.0, 10.0));
    expect(painter.minIntrinsicWidth, 50.0);
    expect(painter.maxIntrinsicWidth, 50.0);
    painter.dispose();

    painter = TextPainter(
      text: const TextSpan(text: 'X X XXXX', style: style),
      textDirection: TextDirection.ltr,
      maxLines: 2,
    );
    painter.layout();
    expect(painter.size, const Size(80.0, 10.0));
    expect(painter.minIntrinsicWidth, 40.0);
    expect(painter.maxIntrinsicWidth, 80.0);
    painter.dispose();

    painter = TextPainter(
      text: const TextSpan(text: 'X X XXXX XX', style: style),
      textDirection: TextDirection.ltr,
      maxLines: 2,
    );
    painter.layout();
    expect(painter.size, const Size(110.0, 10.0));
    expect(painter.minIntrinsicWidth, 70.0);
    expect(painter.maxIntrinsicWidth, 110.0);
    painter.dispose();

    painter = TextPainter(
      text: const TextSpan(text: 'XXXXXXXX XXXX XX X', style: style),
      textDirection: TextDirection.ltr,
      maxLines: 2,
    );
    painter.layout();
    expect(painter.size, const Size(180.0, 10.0));
    expect(painter.minIntrinsicWidth, 90.0);
    expect(painter.maxIntrinsicWidth, 180.0);
    painter.dispose();

    painter = TextPainter(
      text: const TextSpan(text: 'X XX XXXX XXXXXXXX', style: style),
      textDirection: TextDirection.ltr,
      maxLines: 2,
    );
    painter.layout();
    expect(painter.size, const Size(180.0, 10.0));
    expect(painter.minIntrinsicWidth, 90.0);
    expect(painter.maxIntrinsicWidth, 180.0);
    painter.dispose();
  }, skip: true); // https://github.com/flutter/flutter/issues/13512

  test('TextPainter widget span', () {
    final TextPainter painter = TextPainter()..textDirection = TextDirection.ltr;

    const String text = 'test';
    painter.text = const TextSpan(
      text: text,
      children: <InlineSpan>[
        WidgetSpan(child: SizedBox(width: 50, height: 30)),
        TextSpan(text: text),
        WidgetSpan(child: SizedBox(width: 50, height: 30)),
        WidgetSpan(child: SizedBox(width: 50, height: 30)),
        TextSpan(text: text),
        WidgetSpan(child: SizedBox(width: 50, height: 30)),
        WidgetSpan(child: SizedBox(width: 50, height: 30)),
        WidgetSpan(child: SizedBox(width: 50, height: 30)),
        WidgetSpan(child: SizedBox(width: 50, height: 30)),
        WidgetSpan(child: SizedBox(width: 50, height: 30)),
        WidgetSpan(child: SizedBox(width: 50, height: 30)),
        WidgetSpan(child: SizedBox(width: 50, height: 30)),
        WidgetSpan(child: SizedBox(width: 50, height: 30)),
        WidgetSpan(child: SizedBox(width: 50, height: 30)),
        WidgetSpan(child: SizedBox(width: 50, height: 30)),
        WidgetSpan(child: SizedBox(width: 50, height: 30)),
      ],
    );

    // We provide dimensions for the widgets
    painter.setPlaceholderDimensions(const <PlaceholderDimensions>[
      PlaceholderDimensions(
        size: Size(50, 30),
        baselineOffset: 25,
        alignment: ui.PlaceholderAlignment.bottom,
      ),
      PlaceholderDimensions(
        size: Size(50, 30),
        baselineOffset: 25,
        alignment: ui.PlaceholderAlignment.bottom,
      ),
      PlaceholderDimensions(
        size: Size(50, 30),
        baselineOffset: 25,
        alignment: ui.PlaceholderAlignment.bottom,
      ),
      PlaceholderDimensions(
        size: Size(50, 30),
        baselineOffset: 25,
        alignment: ui.PlaceholderAlignment.bottom,
      ),
      PlaceholderDimensions(
        size: Size(50, 30),
        baselineOffset: 25,
        alignment: ui.PlaceholderAlignment.bottom,
      ),
      PlaceholderDimensions(
        size: Size(50, 30),
        baselineOffset: 25,
        alignment: ui.PlaceholderAlignment.bottom,
      ),
      PlaceholderDimensions(
        size: Size(50, 30),
        baselineOffset: 25,
        alignment: ui.PlaceholderAlignment.bottom,
      ),
      PlaceholderDimensions(
        size: Size(50, 30),
        baselineOffset: 25,
        alignment: ui.PlaceholderAlignment.bottom,
      ),
      PlaceholderDimensions(
        size: Size(50, 30),
        baselineOffset: 25,
        alignment: ui.PlaceholderAlignment.bottom,
      ),
      PlaceholderDimensions(
        size: Size(50, 30),
        baselineOffset: 25,
        alignment: ui.PlaceholderAlignment.bottom,
      ),
      PlaceholderDimensions(
        size: Size(50, 30),
        baselineOffset: 25,
        alignment: ui.PlaceholderAlignment.bottom,
      ),
      PlaceholderDimensions(
        size: Size(50, 30),
        baselineOffset: 25,
        alignment: ui.PlaceholderAlignment.bottom,
      ),
      PlaceholderDimensions(
        size: Size(51, 30),
        baselineOffset: 25,
        alignment: ui.PlaceholderAlignment.bottom,
      ),
      PlaceholderDimensions(
        size: Size(50, 30),
        baselineOffset: 25,
        alignment: ui.PlaceholderAlignment.bottom,
      ),
    ]);

    painter.layout(maxWidth: 500);

    // Now, each of the WidgetSpans will have their own placeholder 'hole'.
    Offset caretOffset = painter.getOffsetForCaret(const ui.TextPosition(offset: 1), ui.Rect.zero);
    expect(caretOffset.dx, 14);
    caretOffset = painter.getOffsetForCaret(const ui.TextPosition(offset: 4), ui.Rect.zero);
    expect(caretOffset.dx, 56);
    caretOffset = painter.getOffsetForCaret(const ui.TextPosition(offset: 5), ui.Rect.zero);
    expect(caretOffset.dx, 106);
    caretOffset = painter.getOffsetForCaret(const ui.TextPosition(offset: 6), ui.Rect.zero);
    expect(caretOffset.dx, 120);
    caretOffset = painter.getOffsetForCaret(const ui.TextPosition(offset: 10), ui.Rect.zero);
    expect(caretOffset.dx, 212);
    caretOffset = painter.getOffsetForCaret(const ui.TextPosition(offset: 11), ui.Rect.zero);
    expect(caretOffset.dx, 262);
    caretOffset = painter.getOffsetForCaret(const ui.TextPosition(offset: 12), ui.Rect.zero);
    expect(caretOffset.dx, 276);
    caretOffset = painter.getOffsetForCaret(const ui.TextPosition(offset: 13), ui.Rect.zero);
    expect(caretOffset.dx, 290);
    caretOffset = painter.getOffsetForCaret(const ui.TextPosition(offset: 14), ui.Rect.zero);
    expect(caretOffset.dx, 304);
    caretOffset = painter.getOffsetForCaret(const ui.TextPosition(offset: 15), ui.Rect.zero);
    expect(caretOffset.dx, 318);
    caretOffset = painter.getOffsetForCaret(const ui.TextPosition(offset: 16), ui.Rect.zero);
    expect(caretOffset.dx, 368);
    caretOffset = painter.getOffsetForCaret(const ui.TextPosition(offset: 17), ui.Rect.zero);
    expect(caretOffset.dx, 418);
    caretOffset = painter.getOffsetForCaret(const ui.TextPosition(offset: 18), ui.Rect.zero);
    expect(caretOffset.dx, 0);
    caretOffset = painter.getOffsetForCaret(const ui.TextPosition(offset: 19), ui.Rect.zero);
    expect(caretOffset.dx, 50);
    caretOffset = painter.getOffsetForCaret(const ui.TextPosition(offset: 23), ui.Rect.zero);
    expect(caretOffset.dx, 250);

    expect(painter.inlinePlaceholderBoxes!.length, 14);
    expect(
      painter.inlinePlaceholderBoxes![0],
      const TextBox.fromLTRBD(56, 0, 106, 30, TextDirection.ltr),
    );
    expect(
      painter.inlinePlaceholderBoxes![2],
      const TextBox.fromLTRBD(212, 0, 262, 30, TextDirection.ltr),
    );
    expect(
      painter.inlinePlaceholderBoxes![3],
      const TextBox.fromLTRBD(318, 0, 368, 30, TextDirection.ltr),
    );
    expect(
      painter.inlinePlaceholderBoxes![4],
      const TextBox.fromLTRBD(368, 0, 418, 30, TextDirection.ltr),
    );
    expect(
      painter.inlinePlaceholderBoxes![5],
      const TextBox.fromLTRBD(418, 0, 468, 30, TextDirection.ltr),
    );
    // line should break here
    expect(
      painter.inlinePlaceholderBoxes![6],
      const TextBox.fromLTRBD(0, 30, 50, 60, TextDirection.ltr),
    );
    expect(
      painter.inlinePlaceholderBoxes![7],
      const TextBox.fromLTRBD(50, 30, 100, 60, TextDirection.ltr),
    );
    expect(
      painter.inlinePlaceholderBoxes![10],
      const TextBox.fromLTRBD(200, 30, 250, 60, TextDirection.ltr),
    );
    expect(
      painter.inlinePlaceholderBoxes![11],
      const TextBox.fromLTRBD(250, 30, 300, 60, TextDirection.ltr),
    );
    expect(
      painter.inlinePlaceholderBoxes![12],
      const TextBox.fromLTRBD(300, 30, 351, 60, TextDirection.ltr),
    );
    expect(
      painter.inlinePlaceholderBoxes![13],
      const TextBox.fromLTRBD(351, 30, 401, 60, TextDirection.ltr),
    );
    painter.dispose();
  }, skip: isBrowser && !isSkiaWeb); // https://github.com/flutter/flutter/issues/87540

  // Null values are valid. See https://github.com/flutter/flutter/pull/48346#issuecomment-584839221
  test('TextPainter set TextHeightBehavior null test', () {
    final TextPainter painter = TextPainter()..textDirection = TextDirection.ltr;

    painter.textHeightBehavior = const TextHeightBehavior();
    painter.textHeightBehavior = null;
    painter.dispose();
  });

  test('TextPainter line metrics', () {
    final TextPainter painter = TextPainter()..textDirection = TextDirection.ltr;

    const String text = 'test1\nhello line two really long for soft break\nfinal line 4';
    painter.text = const TextSpan(text: text);

    painter.layout(maxWidth: 300);

    expect(painter.text, const TextSpan(text: text));
    expect(painter.preferredLineHeight, 14);

    final List<ui.LineMetrics> lines = painter.computeLineMetrics();

    expect(lines.length, 4);

    expect(lines[0].hardBreak, true);
    expect(lines[1].hardBreak, false);
    expect(lines[2].hardBreak, true);
    expect(lines[3].hardBreak, true);

    expect(lines[0].ascent, 10.5);
    expect(lines[1].ascent, 10.5);
    expect(lines[2].ascent, 10.5);
    expect(lines[3].ascent, 10.5);

    expect(lines[0].descent, 3.5);
    expect(lines[1].descent, 3.5);
    expect(lines[2].descent, 3.5);
    expect(lines[3].descent, 3.5);

    expect(lines[0].unscaledAscent, 10.5);
    expect(lines[1].unscaledAscent, 10.5);
    expect(lines[2].unscaledAscent, 10.5);
    expect(lines[3].unscaledAscent, 10.5);

    expect(lines[0].baseline, 10.5);
    expect(lines[1].baseline, 24.5);
    expect(lines[2].baseline, 38.5);
    expect(lines[3].baseline, 52.5);

    expect(lines[0].height, 14);
    expect(lines[1].height, 14);
    expect(lines[2].height, 14);
    expect(lines[3].height, 14);

    expect(lines[0].width, 70);
    expect(lines[1].width, 294);
    expect(lines[2].width, 266);
    expect(lines[3].width, 168);

    expect(lines[0].left, 0);
    expect(lines[1].left, 0);
    expect(lines[2].left, 0);
    expect(lines[3].left, 0);

    expect(lines[0].lineNumber, 0);
    expect(lines[1].lineNumber, 1);
    expect(lines[2].lineNumber, 2);
    expect(lines[3].lineNumber, 3);
    painter.dispose();
  }, skip: kIsWeb && !isSkiaWeb); // https://github.com/flutter/flutter/issues/122066

  group('TextPainter line-height', () {
    test('half-leading', () {
      const TextStyle style = TextStyle(
        height: 20,
        fontSize: 1,
        leadingDistribution: TextLeadingDistribution.even,
      );

      final TextPainter painter =
          TextPainter()
            ..textDirection = TextDirection.ltr
            ..text = const TextSpan(text: 'A', style: style)
            ..layout();

      final Rect glyphBox =
          painter
              .getBoxesForSelection(const TextSelection(baseOffset: 0, extentOffset: 1))
              .first
              .toRect();

      final RelativeRect insets = RelativeRect.fromSize(glyphBox, painter.size);
      // The glyph box is centered.
      expect(insets.top, insets.bottom);
      // The glyph box is exactly 1 logical pixel high.
      expect(insets.top, (20 - 1) / 2);
      painter.dispose();
    });

    test('half-leading with small height', () {
      const TextStyle style = TextStyle(
        height: 0.1,
        fontSize: 10,
        leadingDistribution: TextLeadingDistribution.even,
      );

      final TextPainter painter =
          TextPainter()
            ..textDirection = TextDirection.ltr
            ..text = const TextSpan(text: 'A', style: style)
            ..layout();

      final Rect glyphBox =
          painter
              .getBoxesForSelection(const TextSelection(baseOffset: 0, extentOffset: 1))
              .first
              .toRect();

      final RelativeRect insets = RelativeRect.fromSize(glyphBox, painter.size);
      // The glyph box is still centered.
      expect(insets.top, insets.bottom);
      // The glyph box is exactly 10 logical pixel high (the height multiplier
      // does not scale the glyph). Negative leading.
      expect(insets.top, (1 - 10) / 2);
      painter.dispose();
    });

    test('half-leading with leading trim', () {
      const TextStyle style = TextStyle(
        height: 0.1,
        fontSize: 10,
        leadingDistribution: TextLeadingDistribution.even,
      );

      final TextPainter painter =
          TextPainter()
            ..textDirection = TextDirection.ltr
            ..text = const TextSpan(text: 'A', style: style)
            ..textHeightBehavior = const TextHeightBehavior(
              applyHeightToFirstAscent: false,
              applyHeightToLastDescent: false,
            )
            ..layout();

      final Rect glyphBox =
          painter
              .getBoxesForSelection(const TextSelection(baseOffset: 0, extentOffset: 1))
              .first
              .toRect();

      expect(painter.size, glyphBox.size);
      // The glyph box is still centered.
      expect(glyphBox.topLeft, Offset.zero);
      painter.dispose();
    });

    test('TextLeadingDistribution falls back to paragraph style', () {
      const TextStyle style = TextStyle(height: 20, fontSize: 1);
      final TextPainter painter =
          TextPainter()
            ..textDirection = TextDirection.ltr
            ..text = const TextSpan(text: 'A', style: style)
            ..textHeightBehavior = const TextHeightBehavior(
              leadingDistribution: TextLeadingDistribution.even,
            )
            ..layout();

      final Rect glyphBox =
          painter
              .getBoxesForSelection(const TextSelection(baseOffset: 0, extentOffset: 1))
              .first
              .toRect();

      // Still uses half-leading.
      final RelativeRect insets = RelativeRect.fromSize(glyphBox, painter.size);
      expect(insets.top, insets.bottom);
      expect(insets.top, (20 - 1) / 2);
      painter.dispose();
    });

    test('TextLeadingDistribution does nothing if height multiplier is null', () {
      const TextStyle style = TextStyle(fontSize: 1);
      final TextPainter painter =
          TextPainter()
            ..textDirection = TextDirection.ltr
            ..text = const TextSpan(text: 'A', style: style)
            ..textHeightBehavior = const TextHeightBehavior(
              leadingDistribution: TextLeadingDistribution.even,
            )
            ..layout();

      final Rect glyphBox =
          painter
              .getBoxesForSelection(const TextSelection(baseOffset: 0, extentOffset: 1))
              .first
              .toRect();

      painter.textHeightBehavior = const TextHeightBehavior();
      painter.layout();

      final Rect newGlyphBox =
          painter
              .getBoxesForSelection(const TextSelection(baseOffset: 0, extentOffset: 1))
              .first
              .toRect();
      expect(glyphBox, newGlyphBox);
      painter.dispose();
    });
  }, skip: isBrowser && !isSkiaWeb); // https://github.com/flutter/flutter/issues/87543

  test('TextPainter handles invalid UTF-16', () {
    FlutterErrorDetails? error;
    FlutterError.onError = (FlutterErrorDetails details) {
      error = details;
    };

    final TextPainter painter = TextPainter()..textDirection = TextDirection.ltr;

    const String text = 'Hello\uD83DWorld';
    const double fontSize = 20.0;
    painter.text = const TextSpan(text: text, style: TextStyle(fontSize: fontSize));
    painter.layout();
    // The layout should include one replacement character.
    expect(painter.width, equals(fontSize));
    expect(error!.exception, isNotNull);
    expect(error!.silent, isTrue);
    painter.dispose();
  }, skip: kIsWeb); // https://github.com/flutter/flutter/issues/87544

  test('Diacritic', () {
    final TextPainter painter = TextPainter()..textDirection = TextDirection.ltr;

    // Two letters followed by a diacritic
    const String text = 'ฟห้';
    painter.text = const TextSpan(text: text);
    painter.layout();

    final ui.Offset caretOffset = painter.getOffsetForCaret(
      const ui.TextPosition(offset: text.length, affinity: TextAffinity.upstream),
      ui.Rect.zero,
    );
    expect(caretOffset.dx, painter.width);
    painter.dispose();
  }, skip: kIsWeb && !isSkiaWeb); // https://github.com/flutter/flutter/issues/87545

  test('TextPainter line metrics update after layout', () {
    final TextPainter painter = TextPainter()..textDirection = TextDirection.ltr;

    const String text = 'word1 word2 word3';
    painter.text = const TextSpan(text: text);

    painter.layout(maxWidth: 80);

    List<ui.LineMetrics> lines = painter.computeLineMetrics();
    expect(lines.length, 3);

    painter.layout(maxWidth: 1000);

    lines = painter.computeLineMetrics();
    expect(lines.length, 1);
    painter.dispose();
  }, skip: kIsWeb && !isSkiaWeb); // https://github.com/flutter/flutter/issues/62819

  test('TextPainter throws with stack trace when accessing text layout', () {
    final TextPainter painter =
        TextPainter()
          ..text = const TextSpan(text: 'TEXT')
          ..textDirection = TextDirection.ltr;

    expect(
      () => painter.getPositionForOffset(Offset.zero),
      throwsA(
        isA<FlutterError>().having(
          (FlutterError error) => error.message,
          'message',
          contains('The TextPainter has never been laid out.'),
        ),
      ),
    );

    expect(() {
      painter.layout();
      painter.getPositionForOffset(Offset.zero);
    }, returnsNormally);

    expect(
      () {
        painter.markNeedsLayout();
        painter.getPositionForOffset(Offset.zero);
      },
      throwsA(
        isA<FlutterError>().having(
          (FlutterError error) => error.message,
          'message',
          contains('The calls that first invalidated the text layout were:'),
        ),
      ),
    );
    painter.dispose();
  });

  test(
    'TextPainter requires layout after providing different placeholder dimensions',
    () {
      final TextPainter painter = TextPainter()..textDirection = TextDirection.ltr;

      painter.text = const TextSpan(
        children: <InlineSpan>[
          TextSpan(text: 'before'),
          WidgetSpan(child: Text('widget1')),
          WidgetSpan(child: Text('widget2')),
          WidgetSpan(child: Text('widget3')),
          TextSpan(text: 'after'),
        ],
      );

      painter.setPlaceholderDimensions(const <PlaceholderDimensions>[
        PlaceholderDimensions(size: Size(30, 30), alignment: ui.PlaceholderAlignment.bottom),
        PlaceholderDimensions(size: Size(40, 30), alignment: ui.PlaceholderAlignment.bottom),
        PlaceholderDimensions(size: Size(50, 30), alignment: ui.PlaceholderAlignment.bottom),
      ]);
      painter.layout();

      painter.setPlaceholderDimensions(const <PlaceholderDimensions>[
        PlaceholderDimensions(size: Size(30, 30), alignment: ui.PlaceholderAlignment.bottom),
        PlaceholderDimensions(size: Size(40, 20), alignment: ui.PlaceholderAlignment.bottom),
        PlaceholderDimensions(size: Size(50, 30), alignment: ui.PlaceholderAlignment.bottom),
      ]);

      expect(
        () => painter.paint(MockCanvas(), Offset.zero),
        throwsA(
          isA<StateError>().having(
            (StateError error) => error.message,
            'message',
            contains('TextPainter.paint called when text geometry was not yet calculated'),
          ),
        ),
      );
      painter.dispose();
    },
    skip: isBrowser && !isSkiaWeb, // https://github.com/flutter/flutter/issues/56308
  );

  test(
    'TextPainter does not require layout after providing identical placeholder dimensions',
    () {
      final TextPainter painter = TextPainter()..textDirection = TextDirection.ltr;

      painter.text = const TextSpan(
        children: <InlineSpan>[
          TextSpan(text: 'before'),
          WidgetSpan(child: Text('widget1')),
          WidgetSpan(child: Text('widget2')),
          WidgetSpan(child: Text('widget3')),
          TextSpan(text: 'after'),
        ],
      );

      painter.setPlaceholderDimensions(const <PlaceholderDimensions>[
        PlaceholderDimensions(size: Size(30, 30), alignment: ui.PlaceholderAlignment.bottom),
        PlaceholderDimensions(size: Size(40, 30), alignment: ui.PlaceholderAlignment.bottom),
        PlaceholderDimensions(size: Size(50, 30), alignment: ui.PlaceholderAlignment.bottom),
      ]);
      painter.layout();

      painter.setPlaceholderDimensions(const <PlaceholderDimensions>[
        PlaceholderDimensions(size: Size(30, 30), alignment: ui.PlaceholderAlignment.bottom),
        PlaceholderDimensions(size: Size(40, 30), alignment: ui.PlaceholderAlignment.bottom),
        PlaceholderDimensions(size: Size(50, 30), alignment: ui.PlaceholderAlignment.bottom),
      ]);

      // In tests, paint() will throw an UnimplementedError due to missing drawParagraph method.
      expect(
        () => painter.paint(MockCanvas(), Offset.zero),
        isNot(
          throwsA(
            isA<StateError>().having(
              (StateError error) => error.message,
              'message',
              contains('TextPainter.paint called when text geometry was not yet calculated'),
            ),
          ),
        ),
      );
      painter.dispose();
    },
    skip: isBrowser && !isSkiaWeb, // https://github.com/flutter/flutter/issues/56308
  );

  test('TextPainter - debugDisposed', () {
    final TextPainter painter = TextPainter();
    expect(painter.debugDisposed, false);
    painter.dispose();
    expect(painter.debugDisposed, true);
  });

  test('TextPainter - asserts if disposed more than once', () {
    final TextPainter painter = TextPainter()..dispose();
    expect(painter.debugDisposed, isTrue);
    expect(painter.dispose, throwsAssertionError);
  });

  test('TextPainter computeWidth', () {
    const InlineSpan text = TextSpan(text: 'foobar');
    final TextPainter painter = TextPainter(text: text, textDirection: TextDirection.ltr);
    painter.layout();
    expect(painter.width, TextPainter.computeWidth(text: text, textDirection: TextDirection.ltr));

    painter.layout(minWidth: 500);
    expect(
      painter.width,
      TextPainter.computeWidth(text: text, textDirection: TextDirection.ltr, minWidth: 500),
    );

    painter.dispose();
  });

  test('TextPainter computeMaxIntrinsicWidth', () {
    const InlineSpan text = TextSpan(text: 'foobar');
    final TextPainter painter = TextPainter(text: text, textDirection: TextDirection.ltr);
    painter.layout();
    expect(
      painter.maxIntrinsicWidth,
      TextPainter.computeMaxIntrinsicWidth(text: text, textDirection: TextDirection.ltr),
    );

    painter.layout(minWidth: 500);
    expect(
      painter.maxIntrinsicWidth,
      TextPainter.computeMaxIntrinsicWidth(
        text: text,
        textDirection: TextDirection.ltr,
        minWidth: 500,
      ),
    );

    painter.dispose();
  });

  test('TextPainter.getWordBoundary works', () {
    // Regression test for https://github.com/flutter/flutter/issues/93493 .
    const String testCluster = '👨‍👩‍👦👨‍👩‍👦👨‍👩‍👦'; // 8 * 3
    final TextPainter textPainter = TextPainter(
      text: const TextSpan(text: testCluster),
      textDirection: TextDirection.ltr,
    );

    textPainter.layout();
    expect(
      textPainter.getWordBoundary(const TextPosition(offset: 8)),
      const TextRange(start: 8, end: 16),
    );
  }, skip: isBrowser); // https://github.com/flutter/flutter/issues/61017

  test('TextHeightBehavior with strut on empty paragraph', () {
    // Regression test for https://github.com/flutter/flutter/issues/112123
    const TextStyle style = TextStyle(height: 11, fontSize: 7);
    const TextSpan simple = TextSpan(text: 'x', style: style);
    const TextSpan emptyString = TextSpan(text: '', style: style);
    const TextSpan emptyParagraph = TextSpan(style: style);

    final TextPainter painter = TextPainter(
      textDirection: TextDirection.ltr,
      strutStyle: StrutStyle.fromTextStyle(style, forceStrutHeight: true),
      textHeightBehavior: const TextHeightBehavior(
        applyHeightToFirstAscent: false,
        applyHeightToLastDescent: false,
      ),
    );

    painter.text = simple;
    painter.layout();
    final double height = painter.height;
    for (final TextSpan span in <TextSpan>[simple, emptyString, emptyParagraph]) {
      painter.text = span;
      painter.layout();
      expect(painter.height, height, reason: '$span is expected to have a height of $height');
      expect(
        painter.preferredLineHeight,
        height,
        reason: '$span is expected to have a height of $height',
      );
    }
  });

  test('TextPainter plainText getter', () {
    final TextPainter painter = TextPainter()..textDirection = TextDirection.ltr;

    expect(painter.plainText, '');

    painter.text = const TextSpan(
      children: <InlineSpan>[
        TextSpan(text: 'before\n'),
        WidgetSpan(child: Text('widget')),
        TextSpan(text: 'after'),
      ],
    );
    expect(painter.plainText, 'before\n\uFFFCafter');

    painter.setPlaceholderDimensions(const <PlaceholderDimensions>[
      PlaceholderDimensions(size: Size(50, 30), alignment: ui.PlaceholderAlignment.bottom),
    ]);
    painter.layout();
    expect(painter.plainText, 'before\n\uFFFCafter');

    painter.text = const TextSpan(
      children: <InlineSpan>[
        TextSpan(text: 'be\nfo\nre\n'),
        WidgetSpan(child: Text('widget')),
        TextSpan(text: 'af\nter'),
      ],
    );
    expect(painter.plainText, 'be\nfo\nre\n\uFFFCaf\nter');
    painter.layout();
    expect(painter.plainText, 'be\nfo\nre\n\uFFFCaf\nter');

    painter.dispose();
  });

  test('TextPainter infinite width - centered', () {
    final TextPainter painter =
        TextPainter()
          ..textAlign = TextAlign.center
          ..textDirection = TextDirection.ltr;
    painter.text = const TextSpan(text: 'A', style: TextStyle(fontSize: 10));
    MockCanvasWithDrawParagraph mockCanvas = MockCanvasWithDrawParagraph();

    painter.layout(minWidth: double.infinity);
    expect(painter.width, double.infinity);
    expect(
      () => painter.paint(mockCanvas = MockCanvasWithDrawParagraph(), Offset.zero),
      returnsNormally,
    );
    expect(mockCanvas.centerX, isNull);

    painter.layout();
    expect(painter.width, 10);
    expect(
      () => painter.paint(mockCanvas = MockCanvasWithDrawParagraph(), Offset.zero),
      returnsNormally,
    );
    expect(mockCanvas.centerX, 5);

    painter.layout(minWidth: 100);
    expect(painter.width, 100);
    expect(
      () => painter.paint(mockCanvas = MockCanvasWithDrawParagraph(), Offset.zero),
      returnsNormally,
    );
    expect(mockCanvas.centerX, 50);

    painter.dispose();
  });

  test('TextPainter infinite width - LTR justified', () {
    final TextPainter painter =
        TextPainter()
          ..textAlign = TextAlign.justify
          ..textDirection = TextDirection.ltr;
    painter.text = const TextSpan(text: 'A', style: TextStyle(fontSize: 10));
    MockCanvasWithDrawParagraph mockCanvas = MockCanvasWithDrawParagraph();

    painter.layout(minWidth: double.infinity);
    expect(painter.width, double.infinity);
    expect(
      () => painter.paint(mockCanvas = MockCanvasWithDrawParagraph(), Offset.zero),
      returnsNormally,
    );
    expect(mockCanvas.offsetX, 0);

    painter.layout();
    expect(painter.width, 10);
    expect(
      () => painter.paint(mockCanvas = MockCanvasWithDrawParagraph(), Offset.zero),
      returnsNormally,
    );
    expect(mockCanvas.offsetX, 0);

    painter.layout(minWidth: 100);
    expect(painter.width, 100);
    expect(
      () => painter.paint(mockCanvas = MockCanvasWithDrawParagraph(), Offset.zero),
      returnsNormally,
    );
    expect(mockCanvas.offsetX, 0);

    painter.dispose();
  });

  test('LongestLine TextPainter properly relayout when maxWidth changes.', () {
    // Regression test for https://github.com/flutter/flutter/issues/142309.
    final TextPainter painter =
        TextPainter()
          ..textAlign = TextAlign.justify
          ..textWidthBasis = TextWidthBasis.longestLine
          ..textDirection = TextDirection.ltr
          ..text = TextSpan(text: 'A' * 100, style: const TextStyle(fontSize: 10));

    painter.layout(maxWidth: 1000);
    expect(painter.width, 1000);

    painter.layout(maxWidth: 100);
    expect(painter.width, 100);

    painter.layout(maxWidth: 1000);
    expect(painter.width, 1000);
  });

  test('TextPainter line breaking does not round to integers', () {
    const double fontSize = 1.25;
    const String text = '12345';
    assert((fontSize * text.length).truncate() != fontSize * text.length);
    final TextPainter painter = TextPainter(
      textDirection: TextDirection.ltr,
      text: const TextSpan(text: text, style: TextStyle(fontSize: fontSize)),
    )..layout(maxWidth: text.length * fontSize);

    expect(painter.maxIntrinsicWidth, text.length * fontSize);
    switch (painter.computeLineMetrics()) {
      case [ui.LineMetrics(width: final double width)]:
        expect(width, text.length * fontSize);
      case final List<ui.LineMetrics> metrics:
        expect(metrics, hasLength(1));
    }
  }, skip: kIsWeb && !isSkiaWeb); // [intended] Browsers seem to always round font/glyph metrics.

  group(
    'strut style',
    () {
      test('strut style applies when the span has no style', () {
        const StrutStyle strut = StrutStyle(height: 10, fontSize: 10);
        final TextPainter painter = TextPainter(
          textDirection: TextDirection.ltr,
          text: const TextSpan(),
          strutStyle: strut,
        )..layout();
        expect(painter.height, 100);
      });

      test('strut style leading is a fontSize multiplier', () {
        const StrutStyle strut = StrutStyle(height: 10, fontSize: 10, leading: 2);
        final TextPainter painter = TextPainter(
          textDirection: TextDirection.ltr,
          text: const TextSpan(),
          strutStyle: strut,
        )..layout();
        expect(painter.height, 100 + 20);
        // Top leading + scaled ascent.
        expect(painter.computeDistanceToActualBaseline(TextBaseline.alphabetic), 10 + 10 * 7.5);
      });

      test('strut no half leading + force strut height', () {
        const StrutStyle strut = StrutStyle(height: 10, fontSize: 10, forceStrutHeight: true);
        final TextPainter painter = TextPainter(
          textDirection: TextDirection.ltr,
          text: const TextSpan(text: 'A', style: TextStyle(fontSize: 20)),
          strutStyle: strut,
        )..layout();
        expect(painter.height, 100);
        const double baseline = 75;
        expect(
          painter.getBoxesForSelection(const TextSelection(baseOffset: 0, extentOffset: 1)),
          const <ui.TextBox>[
            TextBox.fromLTRBD(0, baseline - 15, 20, baseline + 5, TextDirection.ltr),
          ],
        );
      });

      test('strut half leading + force strut height', () {
        const StrutStyle strut = StrutStyle(
          height: 10,
          fontSize: 10,
          forceStrutHeight: true,
          leadingDistribution: TextLeadingDistribution.even,
        );
        final TextPainter painter = TextPainter(
          textDirection: TextDirection.ltr,
          text: const TextSpan(text: 'A', style: TextStyle(fontSize: 20)),
          strutStyle: strut,
        )..layout();
        expect(painter.height, 100);
        const double baseline = 45 + 7.5;
        expect(
          painter.getBoxesForSelection(const TextSelection(baseOffset: 0, extentOffset: 1)),
          const <ui.TextBox>[
            TextBox.fromLTRBD(0, baseline - 15, 20, baseline + 5, TextDirection.ltr),
          ],
        );
      });

      test('force strut height applies to widget spans', () {
        const Size placeholderSize = Size(1000, 1000);
        const StrutStyle strut = StrutStyle(height: 10, fontSize: 10, forceStrutHeight: true);
        final TextPainter painter =
            TextPainter(
                textDirection: TextDirection.ltr,
                text: const WidgetSpan(child: SizedBox()),
                strutStyle: strut,
              )
              ..setPlaceholderDimensions(const <PlaceholderDimensions>[
                PlaceholderDimensions(
                  size: placeholderSize,
                  alignment: PlaceholderAlignment.bottom,
                ),
              ])
              ..layout();
        expect(painter.height, 100);
      });
    },
    // [intended] strut support for HTML renderer https://github.com/flutter/flutter/issues/32243.
    skip: kIsWeb && !isSkiaWeb,
  );

  test('getOffsetForCaret does not crash on decomposed characters', () {
    final TextPainter painter = TextPainter(
      textDirection: TextDirection.ltr,
      text: const TextSpan(text: '각', style: TextStyle(fontSize: 10)),
    )..layout(maxWidth: 1); // Force the jamo characters to soft wrap.
    expect(
      () => painter.getOffsetForCaret(const TextPosition(offset: 0), Rect.zero),
      returnsNormally,
    );
  });

  test('kTextHeightNone unsets the text height multiplier', () {
    final TextPainter painter = TextPainter(
      textDirection: TextDirection.ltr,
      text: const TextSpan(
        style: TextStyle(fontSize: 10, height: 1000),
        children: <TextSpan>[TextSpan(text: 'A', style: TextStyle(height: kTextHeightNone))],
      ),
    )..layout();
    expect(painter.height, 10);
  });

  test('TextPainter dispatches memory events', () async {
    await expectLater(
      await memoryEvents(() => TextPainter().dispose(), TextPainter),
      areCreateAndDispose,
    );
  });
}

class MockCanvas extends Fake implements Canvas {}

class MockCanvasWithDrawParagraph extends Fake implements Canvas {
  double? centerX;
  double? offsetX;
  @override
  void drawParagraph(ui.Paragraph paragraph, Offset offset) {
    offsetX = offset.dx;
    centerX = offset.dx + paragraph.width / 2;
  }
}
