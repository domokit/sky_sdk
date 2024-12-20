// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:ui/ui.dart' as ui;

import 'fragmenter.dart';
import 'unicode_range.dart';

enum FragmentFlow {
  /// The fragment flows from left to right regardless of its surroundings.
  ltr,

  /// The fragment flows from right to left regardless of its surroundings.
  rtl,

  /// The fragment flows the same as the previous fragment.
  ///
  /// If it's the first fragment in a line, then it flows the same as the
  /// paragraph direction.
  ///
  /// E.g. digits.
  previous,

  /// If the previous and next fragments flow in the same direction, then this
  /// fragment flows in that same direction. Otherwise, it flows the same as the
  /// paragraph direction.
  ///
  /// E.g. spaces, symbols.
  sandwich,
}

/// Splits [text] into fragments based on directionality.
class BidiFragmenter extends TextFragmenter {
  const BidiFragmenter(super.text);

  @override
  List<BidiFragment> fragment() {
    return _computeBidiFragments(text);
  }
}

class BidiFragment extends TextFragment {
  const BidiFragment(super.start, super.end, this.textDirection, this.fragmentFlow);

  final ui.TextDirection? textDirection;
  final FragmentFlow fragmentFlow;

  @override
  int get hashCode => Object.hash(start, end, textDirection, fragmentFlow);

  @override
  bool operator ==(Object other) {
    return other is BidiFragment &&
        other.start == start &&
        other.end == end &&
        other.textDirection == textDirection &&
        other.fragmentFlow == fragmentFlow;
  }

  @override
  String toString() {
    return 'BidiFragment($start, $end, $textDirection)';
  }
}

// This data was taken from the source code of the Closure library:
//
// - https://github.com/google/closure-library/blob/9d24a6c1809a671c2e54c328897ebeae15a6d172/closure/goog/i18n/bidi.js#L203-L234
final UnicodePropertyLookup<ui.TextDirection?> _textDirectionLookup =
    UnicodePropertyLookup<ui.TextDirection?>(<UnicodeRange<ui.TextDirection>>[
      // LTR
      const UnicodeRange<ui.TextDirection>(kChar_A, kChar_Z, ui.TextDirection.ltr),
      const UnicodeRange<ui.TextDirection>(kChar_a, kChar_z, ui.TextDirection.ltr),
      const UnicodeRange<ui.TextDirection>(0x00C0, 0x00D6, ui.TextDirection.ltr),
      const UnicodeRange<ui.TextDirection>(0x00D8, 0x00F6, ui.TextDirection.ltr),
      const UnicodeRange<ui.TextDirection>(0x00F8, 0x02B8, ui.TextDirection.ltr),
      const UnicodeRange<ui.TextDirection>(0x0300, 0x0590, ui.TextDirection.ltr),
      // RTL
      const UnicodeRange<ui.TextDirection>(0x0591, 0x06EF, ui.TextDirection.rtl),
      const UnicodeRange<ui.TextDirection>(0x06FA, 0x08FF, ui.TextDirection.rtl),
      // LTR
      const UnicodeRange<ui.TextDirection>(0x0900, 0x1FFF, ui.TextDirection.ltr),
      const UnicodeRange<ui.TextDirection>(0x200E, 0x200E, ui.TextDirection.ltr),
      // RTL
      const UnicodeRange<ui.TextDirection>(0x200F, 0x200F, ui.TextDirection.rtl),
      // LTR
      const UnicodeRange<ui.TextDirection>(0x2C00, 0xD801, ui.TextDirection.ltr),
      // RTL
      const UnicodeRange<ui.TextDirection>(0xD802, 0xD803, ui.TextDirection.rtl),
      // LTR
      const UnicodeRange<ui.TextDirection>(0xD804, 0xD839, ui.TextDirection.ltr),
      // RTL
      const UnicodeRange<ui.TextDirection>(0xD83A, 0xD83B, ui.TextDirection.rtl),
      // LTR
      const UnicodeRange<ui.TextDirection>(0xD83C, 0xDBFF, ui.TextDirection.ltr),
      const UnicodeRange<ui.TextDirection>(0xF900, 0xFB1C, ui.TextDirection.ltr),
      // RTL
      const UnicodeRange<ui.TextDirection>(0xFB1D, 0xFDFF, ui.TextDirection.rtl),
      // LTR
      const UnicodeRange<ui.TextDirection>(0xFE00, 0xFE6F, ui.TextDirection.ltr),
      // RTL
      const UnicodeRange<ui.TextDirection>(0xFE70, 0xFEFC, ui.TextDirection.rtl),
      // LTR
      const UnicodeRange<ui.TextDirection>(0xFEFD, 0xFFFF, ui.TextDirection.ltr),
    ], null);

List<BidiFragment> _computeBidiFragments(String text) {
  final List<BidiFragment> fragments = <BidiFragment>[];

  if (text.isEmpty) {
    fragments.add(const BidiFragment(0, 0, null, FragmentFlow.previous));
    return fragments;
  }

  int fragmentStart = 0;
  ui.TextDirection? textDirection = _getTextDirection(text, 0);
  FragmentFlow fragmentFlow = _getFragmentFlow(text, 0);

  for (int i = 1; i < text.length; i++) {
    final ui.TextDirection? charTextDirection = _getTextDirection(text, i);

    if (charTextDirection != textDirection) {
      // We've reached the end of a text direction fragment.
      fragments.add(BidiFragment(fragmentStart, i, textDirection, fragmentFlow));
      fragmentStart = i;
      textDirection = charTextDirection;

      fragmentFlow = _getFragmentFlow(text, i);
    } else {
      // This code handles the case of a sequence of digits followed by a sequence
      // of LTR characters with no space in between.
      if (fragmentFlow == FragmentFlow.previous) {
        fragmentFlow = _getFragmentFlow(text, i);
      }
    }
  }

  fragments.add(BidiFragment(fragmentStart, text.length, textDirection, fragmentFlow));
  return fragments;
}

ui.TextDirection? _getTextDirection(String text, int i) {
  final int codePoint = getCodePoint(text, i)!;
  if (_isDigit(codePoint) || _isMashriqiDigit(codePoint)) {
    // A sequence of regular digits or Mashriqi digits always goes from left to
    // regardless of their fragment flow direction.
    return ui.TextDirection.ltr;
  }

  final ui.TextDirection? textDirection = _textDirectionLookup.findForChar(codePoint);
  if (textDirection != null) {
    return textDirection;
  }

  return null;
}

FragmentFlow _getFragmentFlow(String text, int i) {
  final int codePoint = getCodePoint(text, i)!;
  if (_isDigit(codePoint)) {
    return FragmentFlow.previous;
  }
  if (_isMashriqiDigit(codePoint)) {
    return FragmentFlow.rtl;
  }

  final ui.TextDirection? textDirection = _textDirectionLookup.findForChar(codePoint);
  switch (textDirection) {
    case ui.TextDirection.ltr:
      return FragmentFlow.ltr;

    case ui.TextDirection.rtl:
      return FragmentFlow.rtl;

    case null:
      return FragmentFlow.sandwich;
  }
}

bool _isDigit(int codePoint) {
  return codePoint >= kChar_0 && codePoint <= kChar_9;
}

bool _isMashriqiDigit(int codePoint) {
  return codePoint >= kMashriqi_0 && codePoint <= kMashriqi_9;
}
