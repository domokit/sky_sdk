// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:js_interop';

import '../dom.dart';
import 'fragmenter.dart';
import 'line_break_properties.dart';
import 'unicode_range.dart';

const Set<int> _kNewlines = <int>{
  0x000A, // LF
  0x000B, // BK
  0x000C, // BK
  0x000D, // CR
  0x0085, // NL
  0x2028, // BK
  0x2029, // BK
};
const Set<int> _kSpaces = <int>{
  0x0020, // SP
  0x200B, // ZW
};

/// Various types of line breaks as defined by the Unicode spec.
enum LineBreakType {
  /// Indicates that a line break is possible but not mandatory.
  opportunity,

  /// Indicates that a line break isn't possible.
  prohibited,

  /// Indicates that this is a hard line break that can't be skipped.
  mandatory,

  /// Indicates the end of the text (which is also considered a line break in
  /// the Unicode spec). This is the same as [mandatory] but it's needed in our
  /// implementation to distinguish between the universal [endOfText] and the
  /// line break caused by "\n" at the end of the text.
  endOfText,
}

/// Splits [text] into fragments based on line breaks.
abstract class LineBreakFragmenter extends TextFragmenter {
  factory LineBreakFragmenter(String text) {
    if (domIntl.v8BreakIterator != null) {
      return V8LineBreakFragmenter(text);
    }
    return FWLineBreakFragmenter(text);
  }

  @override
  List<LineBreakFragment> fragment();
}

/// Flutter web's custom implementation of [LineBreakFragmenter].
class FWLineBreakFragmenter extends TextFragmenter implements LineBreakFragmenter {
  FWLineBreakFragmenter(super.text);

  @override
  List<LineBreakFragment> fragment() {
    return _computeLineBreakFragments(text);
  }
}

/// An implementation of [LineBreakFragmenter] that uses V8's
/// `v8BreakIterator` API to find line breaks in the given [text].
class V8LineBreakFragmenter extends TextFragmenter implements LineBreakFragmenter {
  V8LineBreakFragmenter(super.text) : assert(domIntl.v8BreakIterator != null);

  final DomV8BreakIterator _v8BreakIterator = createV8BreakIterator();

  @override
  List<LineBreakFragment> fragment() {
    return breakLinesUsingV8BreakIterator(text, text.toJS, _v8BreakIterator);
  }
}

List<LineBreakFragment> breakLinesUsingV8BreakIterator(
  String text,
  JSString jsText,
  DomV8BreakIterator iterator,
) {
  final List<LineBreakFragment> breaks = <LineBreakFragment>[];
  int fragmentStart = 0;

  iterator.adoptText(jsText);
  iterator.first();
  while (iterator.next() != -1) {
    final int fragmentEnd = iterator.current().toInt();
    int trailingNewlines = 0;
    int trailingSpaces = 0;

    // Calculate trailing newlines and spaces.
    for (int i = fragmentStart; i < fragmentEnd; i++) {
      final int codeUnit = text.codeUnitAt(i);
      if (_kNewlines.contains(codeUnit)) {
        trailingNewlines++;
        trailingSpaces++;
      } else if (_kSpaces.contains(codeUnit)) {
        trailingSpaces++;
      } else {
        // Always break after a sequence of spaces.
        if (trailingSpaces > 0) {
          breaks.add(
            LineBreakFragment(
              fragmentStart,
              i,
              LineBreakType.opportunity,
              trailingNewlines: trailingNewlines,
              trailingSpaces: trailingSpaces,
            ),
          );
          fragmentStart = i;
          trailingNewlines = 0;
          trailingSpaces = 0;
        }
      }
    }

    final LineBreakType type;
    if (trailingNewlines > 0) {
      type = LineBreakType.mandatory;
    } else if (fragmentEnd == text.length) {
      type = LineBreakType.endOfText;
    } else {
      type = LineBreakType.opportunity;
    }

    breaks.add(
      LineBreakFragment(
        fragmentStart,
        fragmentEnd,
        type,
        trailingNewlines: trailingNewlines,
        trailingSpaces: trailingSpaces,
      ),
    );
    fragmentStart = fragmentEnd;
  }

  if (breaks.isEmpty || breaks.last.type == LineBreakType.mandatory) {
    breaks.add(
      LineBreakFragment(
        text.length,
        text.length,
        LineBreakType.endOfText,
        trailingNewlines: 0,
        trailingSpaces: 0,
      ),
    );
  }

  return breaks;
}

class LineBreakFragment extends TextFragment {
  const LineBreakFragment(
    super.start,
    super.end,
    this.type, {
    required this.trailingNewlines,
    required this.trailingSpaces,
  });

  final LineBreakType type;
  final int trailingNewlines;
  final int trailingSpaces;

  @override
  int get hashCode => Object.hash(start, end, type, trailingNewlines, trailingSpaces);

  @override
  bool operator ==(Object other) {
    return other is LineBreakFragment &&
        other.start == start &&
        other.end == end &&
        other.type == type &&
        other.trailingNewlines == trailingNewlines &&
        other.trailingSpaces == trailingSpaces;
  }

  @override
  String toString() {
    return 'LineBreakFragment($start, $end, $type)';
  }
}

bool _isHardBreak(LineCharProperty? prop) {
  // No need to check for NL because it's already normalized to BK.
  return prop == LineCharProperty.LF || prop == LineCharProperty.BK;
}

bool _isALorHL(LineCharProperty? prop) {
  return prop == LineCharProperty.AL || prop == LineCharProperty.HL;
}

/// Whether the given property is part of a Korean Syllable block.
///
/// See:
/// - https://unicode.org/reports/tr14/tr14-45.html#LB27
bool _isKoreanSyllable(LineCharProperty? prop) {
  return prop == LineCharProperty.JL ||
      prop == LineCharProperty.JV ||
      prop == LineCharProperty.JT ||
      prop == LineCharProperty.H2 ||
      prop == LineCharProperty.H3;
}

/// Whether the given char code has an Eastern Asian width property of F, W or H.
///
/// See:
/// - https://www.unicode.org/reports/tr14/tr14-45.html#LB30
/// - https://www.unicode.org/Public/13.0.0/ucd/EastAsianWidth.txt
bool _hasEastAsianWidthFWH(int charCode) {
  return charCode == 0x2329 ||
      (charCode >= 0x3008 && charCode <= 0x301D) ||
      (charCode >= 0xFE17 && charCode <= 0xFF62);
}

bool _isSurrogatePair(int? codePoint) {
  return codePoint != null && codePoint > 0xFFFF;
}

/// Finds the next line break in the given [text] starting from [index].
///
/// We think about indices as pointing between characters, and they go all the
/// way from 0 to the string length. For example, here are the indices for the
/// string "foo bar":
///
/// ```none
///   f   o   o       b   a   r
/// ^   ^   ^   ^   ^   ^   ^   ^
/// 0   1   2   3   4   5   6   7
/// ```
///
/// This way the indices work well with [String.substring].
///
/// Useful resources:
///
/// * https://www.unicode.org/reports/tr14/tr14-45.html#Algorithm
/// * https://www.unicode.org/Public/11.0.0/ucd/LineBreak.txt
List<LineBreakFragment> _computeLineBreakFragments(String text) {
  final List<LineBreakFragment> fragments = <LineBreakFragment>[];

  // Keeps track of the character two positions behind.
  LineCharProperty? prev2;
  LineCharProperty? prev1;

  int? codePoint = getCodePoint(text, 0);
  LineCharProperty? curr = lineLookup.findForChar(codePoint);

  // When there's a sequence of combining marks, this variable contains the base
  // property i.e. the property of the character preceding the sequence.
  LineCharProperty baseOfCombiningMarks = LineCharProperty.AL;

  int index = 0;
  int trailingNewlines = 0;
  int trailingSpaces = 0;

  int fragmentStart = 0;

  void setBreak(LineBreakType type, int debugRuleNumber) {
    final int fragmentEnd = type == LineBreakType.endOfText ? text.length : index;
    assert(fragmentEnd >= fragmentStart);

    // Uncomment the following line to help debug line breaking.
    // print('{$fragmentStart:$fragmentEnd} [$debugRuleNumber] -- $type');

    if (prev1 == LineCharProperty.SP) {
      trailingSpaces++;
    } else if (_isHardBreak(prev1) || prev1 == LineCharProperty.CR) {
      trailingNewlines++;
      trailingSpaces++;
    }

    if (type == LineBreakType.prohibited) {
      // Don't create a fragment.
      return;
    }

    fragments.add(
      LineBreakFragment(
        fragmentStart,
        fragmentEnd,
        type,
        trailingNewlines: trailingNewlines,
        trailingSpaces: trailingSpaces,
      ),
    );

    fragmentStart = index;

    // Reset trailing spaces/newlines counter after a new fragment.
    trailingNewlines = 0;
    trailingSpaces = 0;

    prev1 = prev2 = null;
  }

  // Never break at the start of text.
  // LB2: sot ×
  setBreak(LineBreakType.prohibited, 2);

  // Never break at the start of text.
  // LB2: sot ×
  //
  // Skip index 0 because a line break can't exist at the start of text.
  index++;

  int regionalIndicatorCount = 0;

  // We need to go until `text.length` in order to handle the case where the
  // paragraph ends with a hard break. In this case, there will be an empty line
  // at the end.
  for (; index <= text.length; index++) {
    prev2 = prev1;
    prev1 = curr;

    if (_isSurrogatePair(codePoint)) {
      // Can't break in the middle of a surrogate pair.
      setBreak(LineBreakType.prohibited, -1);
      // Advance `index` one extra step to skip the tail of the surrogate pair.
      index++;
    }

    codePoint = getCodePoint(text, index);
    curr = lineLookup.findForChar(codePoint);

    // Keep count of the RI (regional indicator) sequence.
    if (prev1 == LineCharProperty.RI) {
      regionalIndicatorCount++;
    } else {
      regionalIndicatorCount = 0;
    }

    // Always break after hard line breaks.
    // LB4: BK !
    //
    // Treat CR followed by LF, as well as CR, LF, and NL as hard line breaks.
    // LB5: LF !
    //      NL !
    if (_isHardBreak(prev1)) {
      setBreak(LineBreakType.mandatory, 5);
      continue;
    }

    if (prev1 == LineCharProperty.CR) {
      if (curr == LineCharProperty.LF) {
        // LB5: CR × LF
        setBreak(LineBreakType.prohibited, 5);
      } else {
        // LB5: CR !
        setBreak(LineBreakType.mandatory, 5);
      }
      continue;
    }

    // Do not break before hard line breaks.
    // LB6: × ( BK | CR | LF | NL )
    if (_isHardBreak(curr) || curr == LineCharProperty.CR) {
      setBreak(LineBreakType.prohibited, 6);
      continue;
    }

    if (index >= text.length) {
      break;
    }

    // Do not break before spaces or zero width space.
    // LB7: × SP
    //      × ZW
    if (curr == LineCharProperty.SP || curr == LineCharProperty.ZW) {
      setBreak(LineBreakType.prohibited, 7);
      continue;
    }

    // Break after spaces.
    // LB18: SP ÷
    if (prev1 == LineCharProperty.SP) {
      setBreak(LineBreakType.opportunity, 18);
      continue;
    }

    // Break before any character following a zero-width space, even if one or
    // more spaces intervene.
    // LB8: ZW SP* ÷
    if (prev1 == LineCharProperty.ZW) {
      setBreak(LineBreakType.opportunity, 8);
      continue;
    }

    // Do not break after a zero width joiner.
    // LB8a: ZWJ ×
    if (prev1 == LineCharProperty.ZWJ) {
      setBreak(LineBreakType.prohibited, 8);
      continue;
    }

    // Establish the base for the sequences of combining marks.
    if (prev1 != LineCharProperty.CM && prev1 != LineCharProperty.ZWJ) {
      baseOfCombiningMarks = prev1 ?? LineCharProperty.AL;
    }

    // Do not break a combining character sequence; treat it as if it has the
    // line breaking class of the base character in all of the following rules.
    // Treat ZWJ as if it were CM.
    if (curr == LineCharProperty.CM || curr == LineCharProperty.ZWJ) {
      if (baseOfCombiningMarks == LineCharProperty.SP) {
        // LB10: Treat any remaining combining mark or ZWJ as AL.
        curr = LineCharProperty.AL;
      } else {
        // LB9: Treat X (CM | ZWJ)* as if it were X
        //      where X is any line break class except BK, NL, LF, CR, SP, or ZW.
        curr = baseOfCombiningMarks;
        if (curr == LineCharProperty.RI) {
          // Prevent the previous RI from being double-counted.
          regionalIndicatorCount--;
        }
        setBreak(LineBreakType.prohibited, 9);
        continue;
      }
    }
    // In certain situations (e.g. CM immediately following a hard break), we
    // need to also check if the previous character was CM/ZWJ. That's because
    // hard breaks caused the previous iteration to short-circuit, which leads
    // to `baseOfCombiningMarks` not being updated properly.
    if (prev1 == LineCharProperty.CM || prev1 == LineCharProperty.ZWJ) {
      prev1 = baseOfCombiningMarks;
    }

    // Do not break before or after Word joiner and related characters.
    // LB11: × WJ
    //       WJ ×
    if (curr == LineCharProperty.WJ || prev1 == LineCharProperty.WJ) {
      setBreak(LineBreakType.prohibited, 11);
      continue;
    }

    // Do not break after NBSP and related characters.
    // LB12: GL ×
    if (prev1 == LineCharProperty.GL) {
      setBreak(LineBreakType.prohibited, 12);
      continue;
    }

    // Do not break before NBSP and related characters, except after spaces and
    // hyphens.
    // LB12a: [^SP BA HY] × GL
    if (!(prev1 == LineCharProperty.SP ||
            prev1 == LineCharProperty.BA ||
            prev1 == LineCharProperty.HY) &&
        curr == LineCharProperty.GL) {
      setBreak(LineBreakType.prohibited, 12);
      continue;
    }

    // Do not break before ‘]’ or ‘!’ or ‘;’ or ‘/’, even after spaces.
    // LB13: × CL
    //       × CP
    //       × EX
    //       × IS
    //       × SY
    //
    // The above is a quote from unicode.org. In our implementation, we did the
    // following modification: When there are spaces present, we consider it a
    // line break opportunity.
    //
    // We made this modification to match the browser behavior.
    if (prev1 != LineCharProperty.SP &&
        (curr == LineCharProperty.CL ||
            curr == LineCharProperty.CP ||
            curr == LineCharProperty.EX ||
            curr == LineCharProperty.IS ||
            curr == LineCharProperty.SY)) {
      setBreak(LineBreakType.prohibited, 13);
      continue;
    }

    // Do not break after ‘[’, even after spaces.
    // LB14: OP SP* ×
    //
    // The above is a quote from unicode.org. In our implementation, we did the
    // following modification: Allow breaks when there are spaces.
    //
    // We made this modification to match the browser behavior.
    if (prev1 == LineCharProperty.OP) {
      setBreak(LineBreakType.prohibited, 14);
      continue;
    }

    // Do not break within ‘”[’, even with intervening spaces.
    // LB15: QU SP* × OP
    //
    // The above is a quote from unicode.org. In our implementation, we did the
    // following modification: Allow breaks when there are spaces.
    //
    // We made this modification to match the browser behavior.
    if (prev1 == LineCharProperty.QU && curr == LineCharProperty.OP) {
      setBreak(LineBreakType.prohibited, 15);
      continue;
    }

    // Do not break between closing punctuation and a nonstarter, even with
    // intervening spaces.
    // LB16: (CL | CP) SP* × NS
    //
    // The above is a quote from unicode.org. In our implementation, we did the
    // following modification: Allow breaks when there are spaces.
    //
    // We made this modification to match the browser behavior.
    if ((prev1 == LineCharProperty.CL || prev1 == LineCharProperty.CP) &&
        curr == LineCharProperty.NS) {
      setBreak(LineBreakType.prohibited, 16);
      continue;
    }

    // Do not break within ‘——’, even with intervening spaces.
    // LB17: B2 SP* × B2
    //
    // The above is a quote from unicode.org. In our implementation, we did the
    // following modification: Allow breaks when there are spaces.
    //
    // We made this modification to match the browser behavior.
    if (prev1 == LineCharProperty.B2 && curr == LineCharProperty.B2) {
      setBreak(LineBreakType.prohibited, 17);
      continue;
    }

    // Do not break before or after quotation marks, such as ‘”’.
    // LB19: × QU
    //       QU ×
    if (prev1 == LineCharProperty.QU || curr == LineCharProperty.QU) {
      setBreak(LineBreakType.prohibited, 19);
      continue;
    }

    // Break before and after unresolved CB.
    // LB20: ÷ CB
    //       CB ÷
    //
    // In flutter web, we use this as an object-replacement character for
    // placeholders.
    if (prev1 == LineCharProperty.CB || curr == LineCharProperty.CB) {
      setBreak(LineBreakType.opportunity, 20);
      continue;
    }

    // Do not break before hyphen-minus, other hyphens, fixed-width spaces,
    // small kana, and other non-starters, or after acute accents.
    // LB21: × BA
    //       × HY
    //       × NS
    //       BB ×
    if (curr == LineCharProperty.BA ||
        curr == LineCharProperty.HY ||
        curr == LineCharProperty.NS ||
        prev1 == LineCharProperty.BB) {
      setBreak(LineBreakType.prohibited, 21);
      continue;
    }

    // Don't break after Hebrew + Hyphen.
    // LB21a: HL (HY | BA) ×
    if (prev2 == LineCharProperty.HL &&
        (prev1 == LineCharProperty.HY || prev1 == LineCharProperty.BA)) {
      setBreak(LineBreakType.prohibited, 21);
      continue;
    }

    // Don’t break between Solidus and Hebrew letters.
    // LB21b: SY × HL
    if (prev1 == LineCharProperty.SY && curr == LineCharProperty.HL) {
      setBreak(LineBreakType.prohibited, 21);
      continue;
    }

    // Do not break before ellipses.
    // LB22: × IN
    if (curr == LineCharProperty.IN) {
      setBreak(LineBreakType.prohibited, 22);
      continue;
    }

    // Do not break between digits and letters.
    // LB23: (AL | HL) × NU
    //       NU × (AL | HL)
    if ((_isALorHL(prev1) && curr == LineCharProperty.NU) ||
        (prev1 == LineCharProperty.NU && _isALorHL(curr))) {
      setBreak(LineBreakType.prohibited, 23);
      continue;
    }

    // Do not break between numeric prefixes and ideographs, or between
    // ideographs and numeric postfixes.
    // LB23a: PR × (ID | EB | EM)
    if (prev1 == LineCharProperty.PR &&
        (curr == LineCharProperty.ID ||
            curr == LineCharProperty.EB ||
            curr == LineCharProperty.EM)) {
      setBreak(LineBreakType.prohibited, 23);
      continue;
    }
    // LB23a: (ID | EB | EM) × PO
    if ((prev1 == LineCharProperty.ID ||
            prev1 == LineCharProperty.EB ||
            prev1 == LineCharProperty.EM) &&
        curr == LineCharProperty.PO) {
      setBreak(LineBreakType.prohibited, 23);
      continue;
    }

    // Do not break between numeric prefix/postfix and letters, or between
    // letters and prefix/postfix.
    // LB24: (PR | PO) × (AL | HL)
    if ((prev1 == LineCharProperty.PR || prev1 == LineCharProperty.PO) && _isALorHL(curr)) {
      setBreak(LineBreakType.prohibited, 24);
      continue;
    }
    // LB24: (AL | HL) × (PR | PO)
    if (_isALorHL(prev1) && (curr == LineCharProperty.PR || curr == LineCharProperty.PO)) {
      setBreak(LineBreakType.prohibited, 24);
      continue;
    }

    // Do not break between the following pairs of classes relevant to numbers.
    // LB25: (CL | CP | NU) × (PO | PR)
    if ((prev1 == LineCharProperty.CL ||
            prev1 == LineCharProperty.CP ||
            prev1 == LineCharProperty.NU) &&
        (curr == LineCharProperty.PO || curr == LineCharProperty.PR)) {
      setBreak(LineBreakType.prohibited, 25);
      continue;
    }
    // LB25: (PO | PR) × OP
    if ((prev1 == LineCharProperty.PO || prev1 == LineCharProperty.PR) &&
        curr == LineCharProperty.OP) {
      setBreak(LineBreakType.prohibited, 25);
      continue;
    }
    // LB25: (PO | PR | HY | IS | NU | SY) × NU
    if ((prev1 == LineCharProperty.PO ||
            prev1 == LineCharProperty.PR ||
            prev1 == LineCharProperty.HY ||
            prev1 == LineCharProperty.IS ||
            prev1 == LineCharProperty.NU ||
            prev1 == LineCharProperty.SY) &&
        curr == LineCharProperty.NU) {
      setBreak(LineBreakType.prohibited, 25);
      continue;
    }

    // Do not break a Korean syllable.
    // LB26: JL × (JL | JV | H2 | H3)
    if (prev1 == LineCharProperty.JL &&
        (curr == LineCharProperty.JL ||
            curr == LineCharProperty.JV ||
            curr == LineCharProperty.H2 ||
            curr == LineCharProperty.H3)) {
      setBreak(LineBreakType.prohibited, 26);
      continue;
    }
    // LB26: (JV | H2) × (JV | JT)
    if ((prev1 == LineCharProperty.JV || prev1 == LineCharProperty.H2) &&
        (curr == LineCharProperty.JV || curr == LineCharProperty.JT)) {
      setBreak(LineBreakType.prohibited, 26);
      continue;
    }
    // LB26: (JT | H3) × JT
    if ((prev1 == LineCharProperty.JT || prev1 == LineCharProperty.H3) &&
        curr == LineCharProperty.JT) {
      setBreak(LineBreakType.prohibited, 26);
      continue;
    }

    // Treat a Korean Syllable Block the same as ID.
    // LB27: (JL | JV | JT | H2 | H3) × PO
    if (_isKoreanSyllable(prev1) && curr == LineCharProperty.PO) {
      setBreak(LineBreakType.prohibited, 27);
      continue;
    }
    // LB27: PR × (JL | JV | JT | H2 | H3)
    if (prev1 == LineCharProperty.PR && _isKoreanSyllable(curr)) {
      setBreak(LineBreakType.prohibited, 27);
      continue;
    }

    // Do not break between alphabetics.
    // LB28: (AL | HL) × (AL | HL)
    if (_isALorHL(prev1) && _isALorHL(curr)) {
      setBreak(LineBreakType.prohibited, 28);
      continue;
    }

    // Do not break between numeric punctuation and alphabetics (“e.g.”).
    // LB29: IS × (AL | HL)
    if (prev1 == LineCharProperty.IS && _isALorHL(curr)) {
      setBreak(LineBreakType.prohibited, 29);
      continue;
    }

    // Do not break between letters, numbers, or ordinary symbols and opening or
    // closing parentheses.
    // LB30: (AL | HL | NU) × OP
    //
    // LB30 requires that we exclude characters that have an Eastern Asian width
    // property of value F, W or H classes.
    if ((_isALorHL(prev1) || prev1 == LineCharProperty.NU) &&
        curr == LineCharProperty.OP &&
        !_hasEastAsianWidthFWH(text.codeUnitAt(index))) {
      setBreak(LineBreakType.prohibited, 30);
      continue;
    }
    // LB30: CP × (AL | HL | NU)
    if (prev1 == LineCharProperty.CP &&
        !_hasEastAsianWidthFWH(text.codeUnitAt(index - 1)) &&
        (_isALorHL(curr) || curr == LineCharProperty.NU)) {
      setBreak(LineBreakType.prohibited, 30);
      continue;
    }

    // Break between two regional indicator symbols if and only if there are an
    // even number of regional indicators preceding the position of the break.
    // LB30a: sot (RI RI)* RI × RI
    //        [^RI] (RI RI)* RI × RI
    if (curr == LineCharProperty.RI) {
      if (regionalIndicatorCount.isOdd) {
        setBreak(LineBreakType.prohibited, 30);
      } else {
        setBreak(LineBreakType.opportunity, 30);
      }
      continue;
    }

    // Do not break between an emoji base and an emoji modifier.
    // LB30b: EB × EM
    if (prev1 == LineCharProperty.EB && curr == LineCharProperty.EM) {
      setBreak(LineBreakType.prohibited, 30);
      continue;
    }

    // Break everywhere else.
    // LB31: ALL ÷
    //       ÷ ALL
    setBreak(LineBreakType.opportunity, 31);
  }

  // Always break at the end of text.
  // LB3: ! eot
  setBreak(LineBreakType.endOfText, 3);

  return fragments;
}
