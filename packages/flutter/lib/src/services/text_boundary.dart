// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math';

import 'package:characters/characters.dart' show CharacterRange;

import 'text_layout_metrics.dart';

// Examples can assume:
// late TextLayoutMetrics textLayout;
// late TextSpan text;
// bool isWhitespace(int? codeUnit) => true;

/// Signature for a predicate that takes an offset into a UTF-16 string, and a
/// boolean that indicates the search direction.
typedef UntilPredicate = bool Function(int offset, bool forward);

/// An interface for retrieving the logical text boundary (as opposed to the
/// visual boundary) at a given code unit offset in a document.
///
/// Either the [getTextBoundaryAt] method, or both the
/// [getLeadingTextBoundaryAt] method and the [getTrailingTextBoundaryAt] method
/// must be implemented.
abstract class TextBoundary {
  /// A constant constructor to enable subclass override.
  const TextBoundary();

  /// Returns the offset of the closest text boundary before or at the given
  /// `position`, or null if no boundaries can be found.
  ///
  /// The return value, if not null, is usually less than or equal to `position`.
  ///
  /// The range of the return value is given by the closed interval
  /// `[0, string.length]`.
  int? getLeadingTextBoundaryAt(int position) {
    if (position < 0) {
      return null;
    }
    final int start = getTextBoundaryAt(position).start;
    return start >= 0 ? start : null;
  }

  /// Returns the offset of the closest text boundary after the given
  /// `position`, or null if there is no boundary can be found after `position`.
  ///
  /// The return value, if not null, is usually greater than `position`.
  ///
  /// The range of the return value is given by the closed interval
  /// `[0, string.length]`.
  int? getTrailingTextBoundaryAt(int position) {
    final int end = getTextBoundaryAt(max(0, position)).end;
    return end >= 0 ? end : null;
  }

  /// Returns the text boundary range that encloses the input position.
  ///
  /// The returned [TextRange] may contain `-1`, which indicates no boundaries
  /// can be found in that direction.
  TextRange getTextBoundaryAt(int position) {
    final int start = getLeadingTextBoundaryAt(position) ?? -1;
    final int end = getTrailingTextBoundaryAt(position) ?? -1;
    return TextRange(start: start, end: end);
  }
}

/// A [TextBoundary] subclass for retrieving the range of the grapheme the given
/// `position` is in.
///
/// The class is implemented using the
/// [characters](https://pub.dev/packages/characters) package.
class CharacterBoundary extends TextBoundary {
  /// Creates a [CharacterBoundary] with the text.
  const CharacterBoundary(this._text);

  final String _text;

  @override
  int? getLeadingTextBoundaryAt(int position) {
    if (position < 0) {
      return null;
    }
    final int graphemeStart =
        CharacterRange.at(_text, min(position, _text.length)).stringBeforeLength;
    assert(CharacterRange.at(_text, graphemeStart).isEmpty);
    return graphemeStart;
  }

  @override
  int? getTrailingTextBoundaryAt(int position) {
    if (position >= _text.length) {
      return null;
    }
    final CharacterRange rangeAtPosition = CharacterRange.at(_text, max(0, position + 1));
    final int nextBoundary = rangeAtPosition.stringBeforeLength + rangeAtPosition.current.length;
    assert(nextBoundary == _text.length || CharacterRange.at(_text, nextBoundary).isEmpty);
    return nextBoundary;
  }

  @override
  TextRange getTextBoundaryAt(int position) {
    if (position < 0) {
      return TextRange(start: -1, end: getTrailingTextBoundaryAt(position) ?? -1);
    } else if (position >= _text.length) {
      return TextRange(start: getLeadingTextBoundaryAt(position) ?? -1, end: -1);
    }
    final CharacterRange rangeAtPosition = CharacterRange.at(_text, position);
    return rangeAtPosition.isNotEmpty
        ? TextRange(
          start: rangeAtPosition.stringBeforeLength,
          end: rangeAtPosition.stringBeforeLength + rangeAtPosition.current.length,
        )
        // rangeAtPosition is empty means `position` is a grapheme boundary.
        : TextRange(
          start: rangeAtPosition.stringBeforeLength,
          end: getTrailingTextBoundaryAt(position) ?? -1,
        );
  }
}

/// A [TextBoundary] subclass for locating closest line breaks to a given
/// `position`.
///
/// When the given `position` points to a hard line break, the returned range
/// is the line's content range before the hard line break, and does not contain
/// the given `position`. For instance, the line breaks at `position = 1` for
/// "a\nb" is `[0, 1)`, which does not contain the position `1`.
class LineBoundary extends TextBoundary {
  /// Creates a [LineBoundary] with the text and layout information.
  const LineBoundary(this._textLayout);

  final TextLayoutMetrics _textLayout;

  @override
  TextRange getTextBoundaryAt(int position) =>
      _textLayout.getLineAtOffset(TextPosition(offset: max(position, 0)));
}

/// A text boundary that uses paragraphs as logical boundaries.
///
/// A paragraph is defined as the range between line terminators. If no
/// line terminators exist then the paragraph boundary is the entire document.
class ParagraphBoundary extends TextBoundary {
  /// Creates a [ParagraphBoundary] with the text.
  const ParagraphBoundary(this._text);

  final String _text;

  /// Returns the [int] representing the start position of the paragraph that
  /// bounds the given `position`. The returned [int] is the position of the code unit
  /// that follows the line terminator that encloses the desired paragraph.
  @override
  int? getLeadingTextBoundaryAt(int position) {
    if (position < 0 || _text.isEmpty) {
      return null;
    }

    if (position >= _text.length) {
      return _text.length;
    }

    if (position == 0) {
      return 0;
    }

    int index = position;

    if (index > 1 && _text.codeUnitAt(index) == 0x0A && _text.codeUnitAt(index - 1) == 0x0D) {
      index -= 2;
    } else if (TextLayoutMetrics.isLineTerminator(_text.codeUnitAt(index))) {
      index -= 1;
    }

    while (index > 0) {
      if (TextLayoutMetrics.isLineTerminator(_text.codeUnitAt(index))) {
        return index + 1;
      }
      index -= 1;
    }

    return max(index, 0);
  }

  /// Returns the [int] representing the end position of the paragraph that
  /// bounds the given `position`. The returned [int] is the position of the
  /// code unit representing the trailing line terminator that encloses the
  /// desired paragraph.
  @override
  int? getTrailingTextBoundaryAt(int position) {
    if (position >= _text.length || _text.isEmpty) {
      return null;
    }

    if (position < 0) {
      return 0;
    }

    int index = position;

    while (!TextLayoutMetrics.isLineTerminator(_text.codeUnitAt(index))) {
      index += 1;
      if (index == _text.length) {
        return index;
      }
    }

    return index < _text.length - 1 &&
            _text.codeUnitAt(index) == 0x0D &&
            _text.codeUnitAt(index + 1) == 0x0A
        ? index + 2
        : index + 1;
  }
}

/// A text boundary that uses the entire document as logical boundary.
class DocumentBoundary extends TextBoundary {
  /// Creates a [DocumentBoundary] with the text.
  const DocumentBoundary(this._text);

  final String _text;

  @override
  int? getLeadingTextBoundaryAt(int position) => position < 0 ? null : 0;
  @override
  int? getTrailingTextBoundaryAt(int position) => position >= _text.length ? null : _text.length;
}
