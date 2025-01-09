// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../util.dart';
import 'word_break_properties.dart';

enum _FindBreakDirection {
  forward(step: 1),
  backward(step: -1);

  const _FindBreakDirection({required this.step});

  final int step;
}

/// [WordBreaker] exposes static methods to identify word boundaries.
abstract final class WordBreaker {
  /// It starts from [index] and tries to find the next word boundary in [text].
  static int nextBreakIndex(String text, int index) =>
      _findBreakIndex(_FindBreakDirection.forward, text, index);

  /// It starts from [index] and tries to find the previous word boundary in
  /// [text].
  static int prevBreakIndex(String text, int index) =>
      _findBreakIndex(_FindBreakDirection.backward, text, index);

  static int _findBreakIndex(_FindBreakDirection direction, String text, int index) {
    int i = index;
    while (i >= 0 && i <= text.length) {
      i += direction.step;
      if (_isBreak(text, i)) {
        break;
      }
    }
    return clampInt(i, 0, text.length);
  }

  /// Find out if there's a word break between [index - 1] and [index].
  /// http://unicode.org/reports/tr29/#Word_Boundary_Rules
  static bool _isBreak(String? text, int index) {
    // Break at the start and end of text.
    // WB1: sot ÷ Any
    // WB2: Any ÷ eot
    if (index <= 0 || index >= text!.length) {
      return true;
    }

    // Do not break inside surrogate pair
    if (_isUtf16Surrogate(text.codeUnitAt(index - 1))) {
      return false;
    }

    final WordCharProperty immediateRight = wordLookup.find(text, index);
    WordCharProperty immediateLeft = wordLookup.find(text, index - 1);

    // Do not break within CRLF.
    // WB3: CR × LF
    if (immediateLeft == WordCharProperty.CR && immediateRight == WordCharProperty.LF) {
      return false;
    }

    // Otherwise break before and after Newlines (including CR and LF)
    // WB3a: (Newline | CR | LF) ÷
    if (_oneOf(immediateLeft, WordCharProperty.Newline, WordCharProperty.CR, WordCharProperty.LF)) {
      return true;
    }

    // WB3b: ÷ (Newline | CR | LF)
    if (_oneOf(
      immediateRight,
      WordCharProperty.Newline,
      WordCharProperty.CR,
      WordCharProperty.LF,
    )) {
      return true;
    }

    // WB3c: ZWJ	×	\p{Extended_Pictographic}
    // TODO(mdebbar): What's the right way to implement this?

    // Keep horizontal whitespace together.
    // WB3d: WSegSpace × WSegSpace
    if (immediateLeft == WordCharProperty.WSegSpace &&
        immediateRight == WordCharProperty.WSegSpace) {
      return false;
    }

    // Ignore Format and Extend characters, except after sot, CR, LF, and
    // Newline.
    // WB4: X (Extend | Format | ZWJ)* → X
    if (_oneOf(
      immediateRight,
      WordCharProperty.Extend,
      WordCharProperty.Format,
      WordCharProperty.ZWJ,
    )) {
      // The Extend|Format|ZWJ character is to the right, so it is attached
      // to a character to the left, don't split here
      return false;
    }

    // We've reached the end of an Extend|Format|ZWJ sequence, collapse it.
    int l = 0;
    while (_oneOf(
      immediateLeft,
      WordCharProperty.Extend,
      WordCharProperty.Format,
      WordCharProperty.ZWJ,
    )) {
      l++;
      if (index - l - 1 < 0) {
        // Reached the beginning of text.
        return true;
      }
      immediateLeft = wordLookup.find(text, index - l - 1);
    }

    // Do not break between most letters.
    // WB5: (ALetter | Hebrew_Letter) × (ALetter | Hebrew_Letter)
    if (_isAHLetter(immediateLeft) && _isAHLetter(immediateRight)) {
      return false;
    }

    // Some tests beyond this point require more context. We need to get that
    // context while also respecting rule WB4. So ignore Format, Extend and ZWJ.

    // Skip all Format, Extend and ZWJ to the right.
    int r = 0;
    WordCharProperty? nextRight;
    do {
      r++;
      nextRight = wordLookup.find(text, index + r);
    } while (_oneOf(
      nextRight,
      WordCharProperty.Extend,
      WordCharProperty.Format,
      WordCharProperty.ZWJ,
    ));

    // Skip all Format, Extend and ZWJ to the left.
    WordCharProperty? nextLeft;
    do {
      l++;
      nextLeft = wordLookup.find(text, index - l - 1);
    } while (_oneOf(
      nextLeft,
      WordCharProperty.Extend,
      WordCharProperty.Format,
      WordCharProperty.ZWJ,
    ));

    // Do not break letters across certain punctuation.
    // WB6: (AHLetter) × (MidLetter | MidNumLet | Single_Quote) (AHLetter)
    if (_isAHLetter(immediateLeft) &&
        _oneOf(
          immediateRight,
          WordCharProperty.MidLetter,
          WordCharProperty.MidNumLet,
          WordCharProperty.SingleQuote,
        ) &&
        _isAHLetter(nextRight)) {
      return false;
    }

    // WB7: (AHLetter) (MidLetter | MidNumLet | Single_Quote) × (AHLetter)
    if (_isAHLetter(nextLeft) &&
        _oneOf(
          immediateLeft,
          WordCharProperty.MidLetter,
          WordCharProperty.MidNumLet,
          WordCharProperty.SingleQuote,
        ) &&
        _isAHLetter(immediateRight)) {
      return false;
    }

    // WB7a: Hebrew_Letter × Single_Quote
    if (immediateLeft == WordCharProperty.HebrewLetter &&
        immediateRight == WordCharProperty.SingleQuote) {
      return false;
    }

    // WB7b: Hebrew_Letter × Double_Quote Hebrew_Letter
    if (immediateLeft == WordCharProperty.HebrewLetter &&
        immediateRight == WordCharProperty.DoubleQuote &&
        nextRight == WordCharProperty.HebrewLetter) {
      return false;
    }

    // WB7c: Hebrew_Letter Double_Quote × Hebrew_Letter
    if (nextLeft == WordCharProperty.HebrewLetter &&
        immediateLeft == WordCharProperty.DoubleQuote &&
        immediateRight == WordCharProperty.HebrewLetter) {
      return false;
    }

    // Do not break within sequences of digits, or digits adjacent to letters
    // (“3a”, or “A3”).
    // WB8: Numeric × Numeric
    if (immediateLeft == WordCharProperty.Numeric && immediateRight == WordCharProperty.Numeric) {
      return false;
    }

    // WB9: AHLetter × Numeric
    if (_isAHLetter(immediateLeft) && immediateRight == WordCharProperty.Numeric) {
      return false;
    }

    // WB10: Numeric × AHLetter
    if (immediateLeft == WordCharProperty.Numeric && _isAHLetter(immediateRight)) {
      return false;
    }

    // Do not break within sequences, such as “3.2” or “3,456.789”.
    // WB11: Numeric (MidNum | MidNumLet | Single_Quote) × Numeric
    if (nextLeft == WordCharProperty.Numeric &&
        _oneOf(
          immediateLeft,
          WordCharProperty.MidNum,
          WordCharProperty.MidNumLet,
          WordCharProperty.SingleQuote,
        ) &&
        immediateRight == WordCharProperty.Numeric) {
      return false;
    }

    // WB12: Numeric × (MidNum | MidNumLet | Single_Quote) Numeric
    if (immediateLeft == WordCharProperty.Numeric &&
        _oneOf(
          immediateRight,
          WordCharProperty.MidNum,
          WordCharProperty.MidNumLet,
          WordCharProperty.SingleQuote,
        ) &&
        nextRight == WordCharProperty.Numeric) {
      return false;
    }

    // Do not break between Katakana.
    // WB13: Katakana × Katakana
    if (immediateLeft == WordCharProperty.Katakana && immediateRight == WordCharProperty.Katakana) {
      return false;
    }

    // Do not break from extenders.
    // WB13a: (AHLetter | Numeric | Katakana | ExtendNumLet) × ExtendNumLet
    if (_oneOf(
          immediateLeft,
          WordCharProperty.ALetter,
          WordCharProperty.HebrewLetter,
          WordCharProperty.Numeric,
          WordCharProperty.Katakana,
          WordCharProperty.ExtendNumLet,
        ) &&
        immediateRight == WordCharProperty.ExtendNumLet) {
      return false;
    }

    // WB13b: ExtendNumLet × (AHLetter | Numeric | Katakana)
    if (immediateLeft == WordCharProperty.ExtendNumLet &&
        _oneOf(
          immediateRight,
          WordCharProperty.ALetter,
          WordCharProperty.HebrewLetter,
          WordCharProperty.Numeric,
          WordCharProperty.Katakana,
        )) {
      return false;
    }

    // Do not break within emoji flag sequences. That is, do not break between
    // regional indicator (RI) symbols if there is an odd number of RI
    // characters before the break point.
    // WB15: sot (RI RI)* RI × RI
    // TODO(mdebbar): implement this.

    // WB16: [^RI] (RI RI)* RI × RI
    // TODO(mdebbar): implement this.

    // Otherwise, break everywhere (including around ideographs).
    // WB999: Any ÷ Any
    return true;
  }

  static bool _isUtf16Surrogate(int value) {
    return value & 0xF800 == 0xD800;
  }

  static bool _oneOf(
    WordCharProperty? value,
    WordCharProperty choice1,
    WordCharProperty choice2, [
    WordCharProperty? choice3,
    WordCharProperty? choice4,
    WordCharProperty? choice5,
  ]) {
    if (value == choice1) {
      return true;
    }
    if (value == choice2) {
      return true;
    }
    if (choice3 != null && value == choice3) {
      return true;
    }
    if (choice4 != null && value == choice4) {
      return true;
    }
    if (choice5 != null && value == choice5) {
      return true;
    }
    return false;
  }

  static bool _isAHLetter(WordCharProperty? property) {
    return _oneOf(property, WordCharProperty.ALetter, WordCharProperty.HebrewLetter);
  }
}
