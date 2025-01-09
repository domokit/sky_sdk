// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/painting.dart';
import 'package:flutter_test/flutter_test.dart';

const bool skipTestsWithKnownBugs = true;
const bool skipExpectsWithKnownBugs = false;

void main() {
  test('TextPainter - basic words', () {
    final TextPainter painter = TextPainter()..textDirection = TextDirection.ltr;

    painter.text = const TextSpan(text: 'ABC DEF\nGHI', style: TextStyle(fontSize: 10.0));
    painter.layout();

    expect(
      painter.getWordBoundary(const TextPosition(offset: 1)),
      const TextRange(start: 0, end: 3),
    );
    expect(
      painter.getWordBoundary(const TextPosition(offset: 5)),
      const TextRange(start: 4, end: 7),
    );
    expect(
      painter.getWordBoundary(const TextPosition(offset: 9)),
      const TextRange(start: 8, end: 11),
    );
    painter.dispose();
  });

  test('TextPainter - bidi overrides in LTR', () {
    final TextPainter painter = TextPainter()..textDirection = TextDirection.ltr;

    painter.text = const TextSpan(
      text: '${Unicode.RLO}HEBREW1 ${Unicode.LRO}english2${Unicode.PDF} HEBREW3${Unicode.PDF}',
      //      0       12345678      9      101234567       18     90123456       27
      style: TextStyle(fontSize: 10.0),
    );
    TextSpan textSpan = painter.text! as TextSpan;
    expect(textSpan.text!.length, 28);
    painter.layout();

    // The skips here are because the old rendering code considers the bidi formatting characters
    // to be part of the word sometimes and not others, which is fine, but we'd mildly prefer if
    // we were consistently considering them part of words always.
    final TextRange hebrew1 = painter.getWordBoundary(const TextPosition(offset: 4));
    expect(
      hebrew1,
      const TextRange(start: 0, end: 8),
      skip: skipExpectsWithKnownBugs, // https://github.com/flutter/flutter/issues/87536
    );
    final TextRange english2 = painter.getWordBoundary(const TextPosition(offset: 14));
    expect(
      english2,
      const TextRange(start: 9, end: 19),
      skip: skipExpectsWithKnownBugs, // https://github.com/flutter/flutter/issues/87536
    );
    final TextRange hebrew3 = painter.getWordBoundary(const TextPosition(offset: 24));
    expect(hebrew3, const TextRange(start: 20, end: 28));

    //                              >>>>>>>>>>>>>>>                       embedding level 2
    //              <==============================================       embedding level 1
    //             ------------------------------------------------>      embedding level 0
    //            0 2 2 2 2 2 2 2 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 2
    //            0 6 5 4 3 2 1 0 9 0 1 2 3 4 5 6 7 8 7 6 5 4 3 2 1 7  <- index of character in string
    // Paints as:   3 W E R B E H   e n g l i s h 2   1 W E R B E H
    //             0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 2 2 2 2 2   <- pixel offset at boundary
    //             0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4
    //             0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0

    expect(
      painter.getOffsetForCaret(
        const TextPosition(offset: 0, affinity: TextAffinity.upstream),
        Rect.zero,
      ),
      Offset.zero,
    );
    expect(painter.getOffsetForCaret(const TextPosition(offset: 0), Rect.zero), Offset.zero);
    expect(
      painter.getOffsetForCaret(
        const TextPosition(offset: 1, affinity: TextAffinity.upstream),
        Rect.zero,
      ),
      const Offset(240.0, 0.0),
    );
    expect(
      painter.getOffsetForCaret(const TextPosition(offset: 1), Rect.zero),
      const Offset(240.0, 0.0),
    );
    expect(
      painter.getOffsetForCaret(
        const TextPosition(offset: 7, affinity: TextAffinity.upstream),
        Rect.zero,
      ),
      const Offset(180.0, 0.0),
    );
    expect(
      painter.getOffsetForCaret(const TextPosition(offset: 7), Rect.zero),
      const Offset(180.0, 0.0),
    );
    expect(
      painter.getOffsetForCaret(
        const TextPosition(offset: 8, affinity: TextAffinity.upstream),
        Rect.zero,
      ),
      const Offset(170.0, 0.0),
    );
    expect(
      painter.getOffsetForCaret(const TextPosition(offset: 8), Rect.zero),
      const Offset(170.0, 0.0),
    );
    expect(
      painter.getOffsetForCaret(
        const TextPosition(offset: 9, affinity: TextAffinity.upstream),
        Rect.zero,
      ),
      const Offset(160.0, 0.0),
    );
    expect(
      painter.getOffsetForCaret(const TextPosition(offset: 9), Rect.zero),
      const Offset(160.0, 0.0),
    );
    expect(
      painter.getOffsetForCaret(
        const TextPosition(offset: 10, affinity: TextAffinity.upstream),
        Rect.zero,
      ),
      const Offset(80.0, 0.0),
    );
    expect(
      painter.getOffsetForCaret(const TextPosition(offset: 10), Rect.zero),
      const Offset(80.0, 0.0),
    );

    expect(
      painter.getBoxesForSelection(const TextSelection(baseOffset: 0, extentOffset: 27)),
      const <TextBox>[
        TextBox.fromLTRBD(160.0, 0.0, 240.0, 10.0, TextDirection.rtl), // HEBREW1
        TextBox.fromLTRBD(80.0, 0.0, 160.0, 10.0, TextDirection.ltr), // english2
        TextBox.fromLTRBD(0.0, 0.0, 80.0, 10.0, TextDirection.rtl), // HEBREW3
      ],
      // Horizontal offsets are currently one pixel off in places; vertical offsets are good.
      // The list is currently in the wrong order (so selection boxes will paint in the wrong order).
    );

    textSpan = painter.text! as TextSpan;
    final List<List<TextBox>> list = <List<TextBox>>[
      for (int index = 0; index < textSpan.text!.length; index += 1)
        painter.getBoxesForSelection(TextSelection(baseOffset: index, extentOffset: index + 1)),
    ];
    expect(list, const <List<TextBox>>[
      <TextBox>[], // U+202E, non-printing Unicode bidi formatting character
      <TextBox>[TextBox.fromLTRBD(230.0, 0.0, 240.0, 10.0, TextDirection.rtl)],
      <TextBox>[TextBox.fromLTRBD(220.0, 0.0, 230.0, 10.0, TextDirection.rtl)],
      <TextBox>[TextBox.fromLTRBD(210.0, 0.0, 220.0, 10.0, TextDirection.rtl)],
      <TextBox>[TextBox.fromLTRBD(200.0, 0.0, 210.0, 10.0, TextDirection.rtl)],
      <TextBox>[TextBox.fromLTRBD(190.0, 0.0, 200.0, 10.0, TextDirection.rtl)],
      <TextBox>[TextBox.fromLTRBD(180.0, 0.0, 190.0, 10.0, TextDirection.rtl)],
      <TextBox>[TextBox.fromLTRBD(170.0, 0.0, 180.0, 10.0, TextDirection.rtl)],
      <TextBox>[TextBox.fromLTRBD(160.0, 0.0, 170.0, 10.0, TextDirection.rtl)],
      <TextBox>[], // U+202D, non-printing Unicode bidi formatting character
      <TextBox>[TextBox.fromLTRBD(80.0, 0.0, 90.0, 10.0, TextDirection.ltr)],
      <TextBox>[TextBox.fromLTRBD(90.0, 0.0, 100.0, 10.0, TextDirection.ltr)],
      <TextBox>[TextBox.fromLTRBD(100.0, 0.0, 110.0, 10.0, TextDirection.ltr)],
      <TextBox>[TextBox.fromLTRBD(110.0, 0.0, 120.0, 10.0, TextDirection.ltr)],
      <TextBox>[TextBox.fromLTRBD(120.0, 0.0, 130.0, 10.0, TextDirection.ltr)],
      <TextBox>[TextBox.fromLTRBD(130.0, 0.0, 140.0, 10.0, TextDirection.ltr)],
      <TextBox>[TextBox.fromLTRBD(140.0, 0.0, 150.0, 10.0, TextDirection.ltr)],
      <TextBox>[TextBox.fromLTRBD(150.0, 0.0, 160.0, 10.0, TextDirection.ltr)],
      <TextBox>[], // U+202C, non-printing Unicode bidi formatting character
      <TextBox>[TextBox.fromLTRBD(70.0, 0.0, 80.0, 10.0, TextDirection.rtl)],
      <TextBox>[TextBox.fromLTRBD(60.0, 0.0, 70.0, 10.0, TextDirection.rtl)],
      <TextBox>[TextBox.fromLTRBD(50.0, 0.0, 60.0, 10.0, TextDirection.rtl)],
      <TextBox>[TextBox.fromLTRBD(40.0, 0.0, 50.0, 10.0, TextDirection.rtl)],
      <TextBox>[TextBox.fromLTRBD(30.0, 0.0, 40.0, 10.0, TextDirection.rtl)],
      <TextBox>[TextBox.fromLTRBD(20.0, 0.0, 30.0, 10.0, TextDirection.rtl)],
      <TextBox>[TextBox.fromLTRBD(10.0, 0.0, 20.0, 10.0, TextDirection.rtl)],
      <TextBox>[TextBox.fromLTRBD(0.0, 0.0, 10.0, 10.0, TextDirection.rtl)],
      <TextBox>[], // U+202C, non-printing Unicode bidi formatting character
      // The list currently has one extra bogus entry (the last entry, for the
      // trailing U+202C PDF, should be empty but is one-pixel-wide instead).
    ], skip: skipExpectsWithKnownBugs); // https://github.com/flutter/flutter/issues/87536
    painter.dispose();
  }, skip: skipTestsWithKnownBugs); // https://github.com/flutter/flutter/issues/87536

  test('TextPainter - bidi overrides in RTL', () {
    final TextPainter painter = TextPainter()..textDirection = TextDirection.rtl;

    painter.text = const TextSpan(
      text: '${Unicode.RLO}HEBREW1 ${Unicode.LRO}english2${Unicode.PDF} HEBREW3${Unicode.PDF}',
      //      0       12345678      9      101234567       18     90123456       27
      style: TextStyle(fontSize: 10.0),
    );
    final TextSpan textSpan = painter.text! as TextSpan;
    expect(textSpan.text!.length, 28);
    painter.layout();

    final TextRange hebrew1 = painter.getWordBoundary(const TextPosition(offset: 4));
    expect(
      hebrew1,
      const TextRange(start: 0, end: 8),
      skip: skipExpectsWithKnownBugs, // https://github.com/flutter/flutter/issues/87536
    );
    final TextRange english2 = painter.getWordBoundary(const TextPosition(offset: 14));
    expect(
      english2,
      const TextRange(start: 9, end: 19),
      skip: skipExpectsWithKnownBugs, // https://github.com/flutter/flutter/issues/87536
    );
    final TextRange hebrew3 = painter.getWordBoundary(const TextPosition(offset: 24));
    expect(hebrew3, const TextRange(start: 20, end: 28));

    //                              >>>>>>>>>>>>>>>                       embedding level 2
    //            <==================================================     embedding level 1
    //            2 2 2 2 2 2 2 2 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0
    //            7 6 5 4 3 2 1 0 9 0 1 2 3 4 5 6 7 8 7 6 5 4 3 2 1 0  <- index of character in string
    // Paints as:   3 W E R B E H   e n g l i s h 2   1 W E R B E H
    //             0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 2 2 2 2 2   <- pixel offset at boundary
    //             0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4
    //             0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0

    expect(
      painter.getOffsetForCaret(
        const TextPosition(offset: 0, affinity: TextAffinity.upstream),
        Rect.zero,
      ),
      const Offset(240.0, 0.0),
    );
    expect(
      painter.getOffsetForCaret(const TextPosition(offset: 0), Rect.zero),
      const Offset(240.0, 0.0),
    );
    expect(
      painter.getOffsetForCaret(
        const TextPosition(offset: 1, affinity: TextAffinity.upstream),
        Rect.zero,
      ),
      const Offset(240.0, 0.0),
    );
    expect(
      painter.getOffsetForCaret(const TextPosition(offset: 1), Rect.zero),
      const Offset(240.0, 0.0),
    );
    expect(
      painter.getOffsetForCaret(
        const TextPosition(offset: 7, affinity: TextAffinity.upstream),
        Rect.zero,
      ),
      const Offset(180.0, 0.0),
    );
    expect(
      painter.getOffsetForCaret(const TextPosition(offset: 7), Rect.zero),
      const Offset(180.0, 0.0),
    );
    expect(
      painter.getOffsetForCaret(
        const TextPosition(offset: 8, affinity: TextAffinity.upstream),
        Rect.zero,
      ),
      const Offset(170.0, 0.0),
    );
    expect(
      painter.getOffsetForCaret(const TextPosition(offset: 8), Rect.zero),
      const Offset(170.0, 0.0),
    );
    expect(
      painter.getOffsetForCaret(
        const TextPosition(offset: 9, affinity: TextAffinity.upstream),
        Rect.zero,
      ),
      const Offset(160.0, 0.0),
    );
    expect(
      painter.getOffsetForCaret(const TextPosition(offset: 9), Rect.zero),
      const Offset(160.0, 0.0),
    );
    expect(
      painter.getOffsetForCaret(
        const TextPosition(offset: 10, affinity: TextAffinity.upstream),
        Rect.zero,
      ),
      const Offset(80.0, 0.0),
    );
    expect(
      painter.getOffsetForCaret(const TextPosition(offset: 10), Rect.zero),
      const Offset(80.0, 0.0),
    );

    expect(
      painter.getBoxesForSelection(const TextSelection(baseOffset: 0, extentOffset: 27)),
      const <TextBox>[
        TextBox.fromLTRBD(160.0, 0.0, 240.0, 10.0, TextDirection.rtl), // HEBREW1
        TextBox.fromLTRBD(80.0, 0.0, 160.0, 10.0, TextDirection.ltr), // english2
        TextBox.fromLTRBD(0.0, 0.0, 80.0, 10.0, TextDirection.rtl), // HEBREW3
      ],
      // Horizontal offsets are currently one pixel off in places; vertical offsets are good.
      // The list is currently in the wrong order (so selection boxes will paint in the wrong order).
      skip: skipExpectsWithKnownBugs, // https://github.com/flutter/flutter/issues/87536
    );
    painter.dispose();
  }, skip: skipTestsWithKnownBugs); // https://github.com/flutter/flutter/issues/87536

  test('TextPainter - forced line-wrapping with bidi', () {
    final TextPainter painter = TextPainter()..textDirection = TextDirection.ltr;

    painter.text = const TextSpan(
      text: 'A\u05D0', // A, Alef
      style: TextStyle(fontSize: 10.0),
    );
    final TextSpan textSpan = painter.text! as TextSpan;
    expect(textSpan.text!.length, 2);
    painter.layout(maxWidth: 10.0);

    for (int index = 0; index <= 2; index += 1) {
      expect(
        painter.getWordBoundary(const TextPosition(offset: 0)),
        const TextRange(start: 0, end: 2),
      );
    }

    expect(
      // before the A
      painter.getOffsetForCaret(
        const TextPosition(offset: 0, affinity: TextAffinity.upstream),
        Rect.zero,
      ),
      Offset.zero,
    );
    expect(
      // before the A
      painter.getOffsetForCaret(const TextPosition(offset: 0), Rect.zero),
      Offset.zero,
    );

    expect(
      // between A and Alef, after the A
      painter.getOffsetForCaret(
        const TextPosition(offset: 1, affinity: TextAffinity.upstream),
        Rect.zero,
      ),
      const Offset(10.0, 0.0),
    );
    expect(
      // between A and Alef, before the Alef
      painter.getOffsetForCaret(const TextPosition(offset: 1), Rect.zero),
      const Offset(10.0, 10.0),
    );

    expect(
      // after the Alef
      painter.getOffsetForCaret(
        const TextPosition(offset: 2, affinity: TextAffinity.upstream),
        Rect.zero,
      ),
      const Offset(0.0, 10.0),
    );
    expect(
      // To the right of the Alef
      painter.getOffsetForCaret(const TextPosition(offset: 2), Rect.zero),
      const Offset(10.0, 10.0),
    );

    expect(
      painter.getBoxesForSelection(const TextSelection(baseOffset: 0, extentOffset: 2)),
      const <TextBox>[
        TextBox.fromLTRBD(0.0, 0.0, 10.0, 10.0, TextDirection.ltr), // A
        TextBox.fromLTRBD(0.0, 10.0, 10.0, 20.0, TextDirection.rtl), // Alef
      ],
    );
    expect(
      painter.getBoxesForSelection(const TextSelection(baseOffset: 0, extentOffset: 1)),
      const <TextBox>[
        TextBox.fromLTRBD(0.0, 0.0, 10.0, 10.0, TextDirection.ltr), // A
      ],
    );
    expect(
      painter.getBoxesForSelection(const TextSelection(baseOffset: 1, extentOffset: 2)),
      const <TextBox>[
        TextBox.fromLTRBD(0.0, 10.0, 10.0, 20.0, TextDirection.rtl), // Alef
      ],
    );
    painter.dispose();
  }, skip: isBrowser); // https://github.com/flutter/flutter/issues/32238

  test('TextPainter - line wrap mid-word', () {
    final TextPainter painter = TextPainter()..textDirection = TextDirection.ltr;

    painter.text = const TextSpan(
      style: TextStyle(fontSize: 10.0),
      children: <TextSpan>[
        TextSpan(
          text: 'hello', // width 50
        ),
        TextSpan(
          text: 'lovely', // width 120
          style: TextStyle(fontSize: 20.0),
        ),
        TextSpan(
          text: 'world', // width 50
        ),
      ],
    );
    painter.layout(maxWidth: 110.0); // half-way through "lovely"

    expect(
      painter.getBoxesForSelection(const TextSelection(baseOffset: 0, extentOffset: 16)),
      const <TextBox>[
        TextBox.fromLTRBD(0.0, 8.0, 50.0, 18.0, TextDirection.ltr),
        TextBox.fromLTRBD(50.0, 0.0, 110.0, 20.0, TextDirection.ltr),
        TextBox.fromLTRBD(0.0, 20.0, 60.0, 40.0, TextDirection.ltr),
        TextBox.fromLTRBD(60.0, 28.0, 110.0, 38.0, TextDirection.ltr),
      ],
      // horizontal offsets are one pixel off in places; vertical offsets are good
      skip: skipExpectsWithKnownBugs, // https://github.com/flutter/flutter/issues/87536
    );
    painter.dispose();
  }, skip: skipTestsWithKnownBugs); // https://github.com/flutter/flutter/issues/87536

  test('TextPainter - line wrap mid-word, bidi - LTR base', () {
    final TextPainter painter = TextPainter()..textDirection = TextDirection.ltr;

    painter.text = const TextSpan(
      style: TextStyle(fontSize: 10.0),
      children: <TextSpan>[
        TextSpan(
          text: 'hello', // width 50
        ),
        TextSpan(
          text: '\u062C\u0645\u064A\u0644', // width 80
          style: TextStyle(fontSize: 20.0),
        ),
        TextSpan(
          text: 'world', // width 50
        ),
      ],
    );
    painter.layout(maxWidth: 90.0); // half-way through the Arabic word

    expect(
      painter.getBoxesForSelection(const TextSelection(baseOffset: 0, extentOffset: 16)),
      const <TextBox>[
        TextBox.fromLTRBD(0.0, 8.0, 50.0, 18.0, TextDirection.ltr),
        TextBox.fromLTRBD(50.0, 0.0, 90.0, 20.0, TextDirection.rtl),
        TextBox.fromLTRBD(0.0, 20.0, 40.0, 40.0, TextDirection.rtl),
        TextBox.fromLTRBD(40.0, 28.0, 90.0, 38.0, TextDirection.ltr),
      ],
      // horizontal offsets are one pixel off in places; vertical offsets are good
      skip: skipExpectsWithKnownBugs, // https://github.com/flutter/flutter/issues/87536
    );

    final List<List<TextBox>> list = <List<TextBox>>[
      for (int index = 0; index < 5 + 4 + 5; index += 1)
        painter.getBoxesForSelection(TextSelection(baseOffset: index, extentOffset: index + 1)),
    ];

    expect(list, const <List<TextBox>>[
      <TextBox>[TextBox.fromLTRBD(0.0, 8.0, 10.0, 18.0, TextDirection.ltr)],
      <TextBox>[TextBox.fromLTRBD(10.0, 8.0, 20.0, 18.0, TextDirection.ltr)],
      <TextBox>[TextBox.fromLTRBD(20.0, 8.0, 30.0, 18.0, TextDirection.ltr)],
      <TextBox>[TextBox.fromLTRBD(30.0, 8.0, 40.0, 18.0, TextDirection.ltr)],
      <TextBox>[TextBox.fromLTRBD(40.0, 8.0, 50.0, 18.0, TextDirection.ltr)],
      <TextBox>[TextBox.fromLTRBD(70.0, 0.0, 90.0, 20.0, TextDirection.rtl)],
      <TextBox>[TextBox.fromLTRBD(50.0, 0.0, 70.0, 20.0, TextDirection.rtl)],
      <TextBox>[TextBox.fromLTRBD(20.0, 20.0, 40.0, 40.0, TextDirection.rtl)],
      <TextBox>[TextBox.fromLTRBD(0.0, 20.0, 20.0, 40.0, TextDirection.rtl)],
      <TextBox>[TextBox.fromLTRBD(40.0, 28.0, 50.0, 38.0, TextDirection.ltr)],
      <TextBox>[TextBox.fromLTRBD(50.0, 28.0, 60.0, 38.0, TextDirection.ltr)],
      <TextBox>[TextBox.fromLTRBD(60.0, 28.0, 70.0, 38.0, TextDirection.ltr)],
      <TextBox>[TextBox.fromLTRBD(70.0, 28.0, 80.0, 38.0, TextDirection.ltr)],
      <TextBox>[TextBox.fromLTRBD(80.0, 28.0, 90.0, 38.0, TextDirection.ltr)],
    ]);
    painter.dispose();
  }, skip: skipTestsWithKnownBugs); // https://github.com/flutter/flutter/issues/87536

  test('TextPainter - line wrap mid-word, bidi - RTL base', () {
    final TextPainter painter = TextPainter()..textDirection = TextDirection.rtl;

    painter.text = const TextSpan(
      style: TextStyle(fontSize: 10.0),
      children: <TextSpan>[
        TextSpan(
          text: 'hello', // width 50
        ),
        TextSpan(
          text: '\u062C\u0645\u064A\u0644', // width 80
          style: TextStyle(fontSize: 20.0),
        ),
        TextSpan(
          text: 'world', // width 50
        ),
      ],
    );
    painter.layout(maxWidth: 90.0); // half-way through the Arabic word

    expect(
      painter.getBoxesForSelection(const TextSelection(baseOffset: 0, extentOffset: 16)),
      const <TextBox>[
        TextBox.fromLTRBD(40.0, 8.0, 90.0, 18.0, TextDirection.ltr),
        TextBox.fromLTRBD(0.0, 0.0, 40.0, 20.0, TextDirection.rtl),
        TextBox.fromLTRBD(50.0, 20.0, 90.0, 40.0, TextDirection.rtl),
        TextBox.fromLTRBD(0.0, 28.0, 50.0, 38.0, TextDirection.ltr),
      ],
      // Horizontal offsets are currently one pixel off in places; vertical offsets are good.
      // The list is currently in the wrong order (so selection boxes will paint in the wrong order).
      skip: skipExpectsWithKnownBugs, // https://github.com/flutter/flutter/issues/87536
    );
    painter.dispose();
  }, skip: skipTestsWithKnownBugs); // https://github.com/flutter/flutter/issues/87536

  test('TextPainter - multiple levels', () {
    final TextPainter painter = TextPainter()..textDirection = TextDirection.rtl;

    final String pyramid = rlo(lro(rlo(lro(rlo('')))));
    painter.text = TextSpan(text: pyramid, style: const TextStyle(fontSize: 10.0));
    painter.layout();

    expect(
      painter.getBoxesForSelection(TextSelection(baseOffset: 0, extentOffset: pyramid.length)),
      const <TextBox>[
        TextBox.fromLTRBD(90.0, 0.0, 100.0, 10.0, TextDirection.rtl), // outer R, start (right)
        TextBox.fromLTRBD(10.0, 0.0, 20.0, 10.0, TextDirection.ltr), // level 1 L, start (left)
        TextBox.fromLTRBD(70.0, 0.0, 80.0, 10.0, TextDirection.rtl), // level 2 R, start (right)
        TextBox.fromLTRBD(30.0, 0.0, 40.0, 10.0, TextDirection.ltr), // level 3 L, start (left)
        TextBox.fromLTRBD(40.0, 0.0, 60.0, 10.0, TextDirection.rtl), // inner-most RR
        TextBox.fromLTRBD(60.0, 0.0, 70.0, 10.0, TextDirection.ltr), // lever 3 L, end (right)
        TextBox.fromLTRBD(20.0, 0.0, 30.0, 10.0, TextDirection.rtl), // level 2 R, end (left)
        TextBox.fromLTRBD(80.0, 0.0, 90.0, 10.0, TextDirection.ltr), // level 1 L, end (right)
        TextBox.fromLTRBD(0.0, 0.0, 10.0, 10.0, TextDirection.rtl), // outer R, end (left)
      ],
      // Horizontal offsets are currently one pixel off in places; vertical offsets are good.
      // The list is currently in the wrong order (so selection boxes will paint in the wrong order).
      // Also currently there's an extraneous box at the start of the list.
      skip: skipExpectsWithKnownBugs, // https://github.com/flutter/flutter/issues/87536
    );
    painter.dispose();
  }, skip: skipTestsWithKnownBugs); // https://github.com/flutter/flutter/issues/87536

  test('TextPainter - getPositionForOffset - RTL in LTR', () {
    final TextPainter painter = TextPainter()..textDirection = TextDirection.ltr;

    painter.text = const TextSpan(
      text:
          'ABC\u05D0\u05D1\u05D2DEF', // A B C Alef Bet Gimel D E F -- but the Hebrew letters are RTL
      style: TextStyle(fontSize: 10.0),
    );
    painter.layout();

    // TODO(ianh): Remove the toString()s once https://github.com/flutter/engine/pull/4283 lands
    expect(
      //  Aaa  Bbb  Ccc  Gimel  Bet  Alef  Ddd  Eee  Fff
      // ^
      painter.getPositionForOffset(const Offset(0.0, 5.0)).toString(),
      const TextPosition(offset: 0).toString(),
    );
    expect(
      //                     Aaa  Bbb  Ccc  Gimel  Bet  Alef  Ddd  Eee  Fff
      // ^
      painter.getPositionForOffset(const Offset(-100.0, 5.0)).toString(),
      const TextPosition(offset: 0).toString(),
    );
    expect(
      //  Aaa  Bbb  Ccc  Gimel  Bet  Alef  Ddd  Eee  Fff
      //  ^
      painter.getPositionForOffset(const Offset(4.0, 5.0)).toString(),
      const TextPosition(offset: 0).toString(),
    );
    expect(
      //  Aaa  Bbb  Ccc  Gimel  Bet  Alef  Ddd  Eee  Fff
      //    ^
      painter.getPositionForOffset(const Offset(8.0, 5.0)).toString(),
      const TextPosition(offset: 1, affinity: TextAffinity.upstream).toString(),
    );
    expect(
      //  Aaa  Bbb  Ccc  Gimel  Bet  Alef  Ddd  Eee  Fff
      //       ^
      painter.getPositionForOffset(const Offset(12.0, 5.0)).toString(),
      const TextPosition(offset: 1).toString(),
      // currently we say upstream instead of downstream
      skip: skipExpectsWithKnownBugs, // https://github.com/flutter/flutter/issues/87536
    );
    expect(
      //  Aaa  Bbb  Ccc  Gimel  Bet  Alef  Ddd  Eee  Fff
      //              ^
      painter.getPositionForOffset(const Offset(28.0, 5.0)).toString(),
      const TextPosition(offset: 3, affinity: TextAffinity.upstream).toString(),
    );
    expect(
      //  Aaa  Bbb  Ccc  Gimel  Bet  Alef  Ddd  Eee  Fff
      //                 ^
      painter.getPositionForOffset(const Offset(32.0, 5.0)).toString(),
      const TextPosition(offset: 6, affinity: TextAffinity.upstream).toString(),
      // this is part of https://github.com/flutter/flutter/issues/11375
      skip: skipExpectsWithKnownBugs,
    );
    expect(
      //  Aaa  Bbb  Ccc  Gimel  Bet  Alef  Ddd  Eee  Fff
      //                                ^
      painter.getPositionForOffset(const Offset(58.0, 5.0)).toString(),
      const TextPosition(offset: 3).toString(),
      // this is part of https://github.com/flutter/flutter/issues/11375
      skip: skipExpectsWithKnownBugs,
    );
    expect(
      //  Aaa  Bbb  Ccc  Gimel  Bet  Alef  Ddd  Eee  Fff
      //                                   ^
      painter.getPositionForOffset(const Offset(62.0, 5.0)).toString(),
      const TextPosition(offset: 6).toString(),
    );
    expect(
      //  Aaa  Bbb  Ccc  Gimel  Bet  Alef  Ddd  Eee  Fff
      //                                               ^
      painter.getPositionForOffset(const Offset(88.0, 5.0)).toString(),
      const TextPosition(offset: 9, affinity: TextAffinity.upstream).toString(),
    );
    expect(
      //  Aaa  Bbb  Ccc  Gimel  Bet  Alef  Ddd  Eee  Fff
      //                                                     ^
      painter.getPositionForOffset(const Offset(100.0, 5.0)).toString(),
      const TextPosition(offset: 9, affinity: TextAffinity.upstream).toString(),
    );
    painter.dispose();
  }, skip: skipTestsWithKnownBugs); // https://github.com/flutter/flutter/issues/87536

  test('TextPainter - getPositionForOffset - LTR in RTL', () {
    final TextPainter painter = TextPainter()..textDirection = TextDirection.rtl;

    painter.text = const TextSpan(
      text: '\u05D0\u05D1\u05D2ABC\u05D3\u05D4\u05D5',
      style: TextStyle(fontSize: 10.0),
    );
    painter.layout();

    // TODO(ianh): Remove the toString()s once https://github.com/flutter/engine/pull/4283 lands
    expect(
      //   Vav He Dalet Aaa Bbb Ccc Gimel Bet Alef
      // ^
      painter.getPositionForOffset(const Offset(-4.0, 5.0)).toString(),
      const TextPosition(offset: 9, affinity: TextAffinity.upstream).toString(),
    );
    expect(
      // Vav He Dalet Aaa Bbb Ccc Gimel Bet Alef
      //            ^
      painter.getPositionForOffset(const Offset(28.0, 5.0)).toString(),
      const TextPosition(offset: 6).toString(),
    );
    expect(
      // Vav He Dalet Aaa Bbb Ccc Gimel Bet Alef
      //              ^
      painter.getPositionForOffset(const Offset(32.0, 5.0)).toString(),
      const TextPosition(offset: 3).toString(),
      // this is part of https://github.com/flutter/flutter/issues/11375
      skip: skipExpectsWithKnownBugs,
    );
    expect(
      // Vav He Dalet Aaa Bbb Ccc Gimel Bet Alef
      //                        ^
      painter.getPositionForOffset(const Offset(58.0, 5.0)).toString(),
      const TextPosition(offset: 6, affinity: TextAffinity.upstream).toString(),
      // this is part of https://github.com/flutter/flutter/issues/11375
      skip: skipExpectsWithKnownBugs,
    );
    expect(
      // Vav He Dalet Aaa Bbb Ccc Gimel Bet Alef
      //                          ^
      painter.getPositionForOffset(const Offset(62.0, 5.0)).toString(),
      const TextPosition(offset: 3, affinity: TextAffinity.upstream).toString(),
    );
    painter.dispose();
  }, skip: skipTestsWithKnownBugs); // https://github.com/flutter/flutter/issues/87536

  test('TextPainter - Spaces', () {
    final TextPainter painter = TextPainter()..textDirection = TextDirection.ltr;

    painter.text = const TextSpan(
      text: ' ',
      style: TextStyle(fontSize: 100.0),
      children: <TextSpan>[
        TextSpan(text: ' ', style: TextStyle(fontSize: 10.0)),
        TextSpan(text: ' ', style: TextStyle(fontSize: 200.0)),
        // Add a non-whitespace character because the renderer's line breaker
        // may strip trailing whitespace on a line.
        TextSpan(text: 'A'),
      ],
    );
    painter.layout();

    // This renders as three (invisible) boxes:
    //
    //                 |<--------200------->|
    //                  ____________________
    //                 |        ^           |
    //                 |        :           |
    //                 |        :           |
    //                 |        :           |
    //                 |        :           |
    //   ___________   |        : 160       |
    //  |  ^        |  |        :           |
    //  |<-+-100--->|10|        :           |
    //  |  :        |__|        :           |
    //  |  : 80     |  |8       :           |
    // _|__v________|__|________v___________| BASELINE
    //  |     ^20   |__|2       ^           |
    //  |_____v_____|  |        |           |
    //                 |        | 40        |
    //                 |        |           |
    //                 |________v___________|

    expect(painter.width, 410.0);
    expect(painter.height, 200.0);
    expect(
      painter.computeDistanceToActualBaseline(TextBaseline.alphabetic),
      moreOrLessEquals(160.0, epsilon: 0.001),
    );
    expect(painter.preferredLineHeight, 100.0);

    expect(
      painter.getBoxesForSelection(const TextSelection(baseOffset: 0, extentOffset: 3)),
      const <TextBox>[
        TextBox.fromLTRBD(0.0, 80.0, 100.0, 180.0, TextDirection.ltr),
        TextBox.fromLTRBD(100.0, 152.0, 110.0, 162.0, TextDirection.ltr),
        TextBox.fromLTRBD(110.0, 0.0, 310.0, 200.0, TextDirection.ltr),
      ],
      // Horizontal offsets are currently one pixel off in places; vertical offsets are good.
      skip: skipExpectsWithKnownBugs, // https://github.com/flutter/flutter/issues/87536
    );
    painter.dispose();
  }, skip: skipTestsWithKnownBugs); // https://github.com/flutter/flutter/issues/87536

  test('TextPainter - empty text baseline', () {
    final TextPainter painter = TextPainter()..textDirection = TextDirection.ltr;
    painter.text = const TextSpan(
      text: '',
      style: TextStyle(fontFamily: 'FlutterTest', fontSize: 100.0, height: 1.0),
    );
    painter.layout();
    expect(painter.computeDistanceToActualBaseline(TextBaseline.alphabetic), 75.0);
    painter.dispose();
  });
}

String lro(String s) => '${Unicode.LRO}L${s}L${Unicode.PDF}';
String rlo(String s) => '${Unicode.RLO}R${s}R${Unicode.PDF}';
