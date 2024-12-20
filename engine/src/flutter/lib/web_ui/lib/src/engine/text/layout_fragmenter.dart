// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;

import 'package:ui/ui.dart' as ui;

import '../util.dart';
import 'canvas_paragraph.dart';
import 'fragmenter.dart';
import 'layout_service.dart';
import 'line_breaker.dart';
import 'paragraph.dart';
import 'text_direction.dart';

/// Splits [text] into fragments that are ready to be laid out by
/// [TextLayoutService].
///
/// This fragmenter takes into account line breaks, directionality and styles.
class LayoutFragmenter extends TextFragmenter {
  const LayoutFragmenter(super.text, this.paragraphSpans);

  final List<ParagraphSpan> paragraphSpans;

  @override
  List<LayoutFragment> fragment() {
    final List<LayoutFragment> fragments = <LayoutFragment>[];

    int fragmentStart = 0;

    final Iterator<LineBreakFragment> lineBreakFragments =
        LineBreakFragmenter(text).fragment().iterator..moveNext();
    final Iterator<BidiFragment> bidiFragments =
        BidiFragmenter(text).fragment().iterator..moveNext();
    final Iterator<ParagraphSpan> spans = paragraphSpans.iterator..moveNext();

    LineBreakFragment currentLineBreakFragment = lineBreakFragments.current;
    BidiFragment currentBidiFragment = bidiFragments.current;
    ParagraphSpan currentSpan = spans.current;

    while (true) {
      final int fragmentEnd = math.min(
        currentLineBreakFragment.end,
        math.min(currentBidiFragment.end, currentSpan.end),
      );

      final int distanceFromLineBreak = currentLineBreakFragment.end - fragmentEnd;

      final LineBreakType lineBreakType =
          distanceFromLineBreak == 0 ? currentLineBreakFragment.type : LineBreakType.prohibited;

      final int trailingNewlines =
          currentLineBreakFragment.trailingNewlines - distanceFromLineBreak;
      final int trailingSpaces = currentLineBreakFragment.trailingSpaces - distanceFromLineBreak;

      final int fragmentLength = fragmentEnd - fragmentStart;
      fragments.add(
        LayoutFragment(
          fragmentStart,
          fragmentEnd,
          lineBreakType,
          currentBidiFragment.textDirection,
          currentBidiFragment.fragmentFlow,
          currentSpan,
          trailingNewlines: clampInt(trailingNewlines, 0, fragmentLength),
          trailingSpaces: clampInt(trailingSpaces, 0, fragmentLength),
        ),
      );

      fragmentStart = fragmentEnd;

      bool moved = false;
      if (currentLineBreakFragment.end == fragmentEnd) {
        if (lineBreakFragments.moveNext()) {
          moved = true;
          currentLineBreakFragment = lineBreakFragments.current;
        }
      }
      if (currentBidiFragment.end == fragmentEnd) {
        if (bidiFragments.moveNext()) {
          moved = true;
          currentBidiFragment = bidiFragments.current;
        }
      }
      if (currentSpan.end == fragmentEnd) {
        if (spans.moveNext()) {
          moved = true;
          currentSpan = spans.current;
        }
      }

      // Once we reached the end of all fragments, exit the loop.
      if (!moved) {
        break;
      }
    }

    return fragments;
  }
}

abstract class _CombinedFragment extends TextFragment {
  _CombinedFragment(
    super.start,
    super.end,
    this.type,
    this._textDirection,
    this.fragmentFlow,
    this.span, {
    required this.trailingNewlines,
    required this.trailingSpaces,
  }) : assert(trailingNewlines >= 0),
       assert(trailingSpaces >= trailingNewlines);

  final LineBreakType type;

  ui.TextDirection? get textDirection => _textDirection;
  ui.TextDirection? _textDirection;

  final FragmentFlow fragmentFlow;

  final ParagraphSpan span;

  final int trailingNewlines;

  final int trailingSpaces;

  @override
  int get hashCode => Object.hash(
    start,
    end,
    type,
    textDirection,
    fragmentFlow,
    span,
    trailingNewlines,
    trailingSpaces,
  );

  @override
  bool operator ==(Object other) {
    return other is LayoutFragment &&
        other.start == start &&
        other.end == end &&
        other.type == type &&
        other.textDirection == textDirection &&
        other.fragmentFlow == fragmentFlow &&
        other.span == span &&
        other.trailingNewlines == trailingNewlines &&
        other.trailingSpaces == trailingSpaces;
  }
}

class LayoutFragment extends _CombinedFragment
    with _FragmentMetrics, _FragmentPosition, _FragmentBox {
  LayoutFragment(
    super.start,
    super.end,
    super.type,
    super.textDirection,
    super.fragmentFlow,
    super.span, {
    required super.trailingNewlines,
    required super.trailingSpaces,
  });

  int get length => end - start;
  bool get isSpaceOnly => length == trailingSpaces;
  bool get isPlaceholder => span is PlaceholderSpan;
  bool get isBreak => type != LineBreakType.prohibited;
  bool get isHardBreak => type == LineBreakType.mandatory || type == LineBreakType.endOfText;
  EngineTextStyle get style => span.style;

  /// Returns the substring from [paragraph] that corresponds to this fragment,
  /// excluding new line characters.
  String getText(CanvasParagraph paragraph) {
    return paragraph.plainText.substring(start, end - trailingNewlines);
  }

  /// Splits this fragment into two fragments with the split point being the
  /// given [index].
  // TODO(mdebbar): If we ever get multiple return values in Dart, we should use it!
  //                See: https://github.com/dart-lang/language/issues/68
  List<LayoutFragment?> split(int index) {
    assert(start <= index);
    assert(index <= end);

    if (start == index) {
      return <LayoutFragment?>[null, this];
    }

    if (end == index) {
      return <LayoutFragment?>[this, null];
    }

    // The length of the second fragment after the split.
    final int secondLength = end - index;

    // Trailing spaces/new lines go to the second fragment. Any left over goes
    // to the first fragment.
    final int secondTrailingNewlines = math.min(trailingNewlines, secondLength);
    final int secondTrailingSpaces = math.min(trailingSpaces, secondLength);

    return <LayoutFragment>[
      LayoutFragment(
        start,
        index,
        LineBreakType.prohibited,
        textDirection,
        fragmentFlow,
        span,
        trailingNewlines: trailingNewlines - secondTrailingNewlines,
        trailingSpaces: trailingSpaces - secondTrailingSpaces,
      ),
      LayoutFragment(
        index,
        end,
        type,
        textDirection,
        fragmentFlow,
        span,
        trailingNewlines: secondTrailingNewlines,
        trailingSpaces: secondTrailingSpaces,
      ),
    ];
  }

  @override
  String toString() {
    return '$LayoutFragment($start, $end, $type, $textDirection)';
  }
}

mixin _FragmentMetrics on _CombinedFragment {
  late Spanometer _spanometer;

  /// The rise from the baseline as calculated from the font and style for this text.
  double get ascent => _ascent;
  late double _ascent;

  /// The drop from the baseline as calculated from the font and style for this text.
  double get descent => _descent;
  late double _descent;

  /// The width of the measured text, not including trailing spaces.
  double get widthExcludingTrailingSpaces => _widthExcludingTrailingSpaces;
  late double _widthExcludingTrailingSpaces;

  /// The width of the measured text, including any trailing spaces.
  double get widthIncludingTrailingSpaces =>
      _widthIncludingTrailingSpaces + _extraWidthForJustification;
  late double _widthIncludingTrailingSpaces;

  double _extraWidthForJustification = 0.0;

  /// The total height as calculated from the font and style for this text.
  double get height => ascent + descent;

  double get widthOfTrailingSpaces => widthIncludingTrailingSpaces - widthExcludingTrailingSpaces;

  /// Set measurement values for the fragment.
  void setMetrics(
    Spanometer spanometer, {
    required double ascent,
    required double descent,
    required double widthExcludingTrailingSpaces,
    required double widthIncludingTrailingSpaces,
  }) {
    _spanometer = spanometer;
    _ascent = ascent;
    _descent = descent;
    _widthExcludingTrailingSpaces = widthExcludingTrailingSpaces;
    _widthIncludingTrailingSpaces = widthIncludingTrailingSpaces;
  }
}

/// Encapsulates positioning of the fragment relative to the line.
///
/// The coordinates are all relative to the line it belongs to. For example,
/// [left] is the distance from the left edge of the line to the left edge of
/// the fragment.
///
/// This is what the various measurements/coordinates look like for a fragment
/// in an LTR paragraph:
///
///          *------------------------line.width-----------------*
///                            *---width----*
///          ┌─────────────────┬────────────┬────────────────────┐
///          │                 │--FRAGMENT--│                    │
///          └─────────────────┴────────────┴────────────────────┘
///          *---startOffset---*
///          *------left-------*
///          *--------endOffset-------------*
///          *----------right---------------*
///
///
/// And in an RTL paragraph, [startOffset] and [endOffset] are flipped because
/// the line starts from the right. Here's what they look like:
///
///          *------------------------line.width-----------------*
///                            *---width----*
///          ┌─────────────────┬────────────┬────────────────────┐
///          │                 │--FRAGMENT--│                    │
///          └─────────────────┴────────────┴────────────────────┘
///                                         *----startOffset-----*
///          *------left-------*
///                            *-----------endOffset-------------*
///          *----------right---------------*
///
mixin _FragmentPosition on _CombinedFragment, _FragmentMetrics {
  /// The distance from the beginning of the line to the beginning of the fragment.
  double get startOffset => _startOffset;
  late double _startOffset;

  /// The width of the line that contains this fragment.
  late ParagraphLine line;

  /// The distance from the beginning of the line to the end of the fragment.
  double get endOffset => startOffset + widthIncludingTrailingSpaces;

  /// The distance from the left edge of the line to the left edge of the fragment.
  double get left =>
      line.textDirection == ui.TextDirection.ltr ? startOffset : line.width - endOffset;

  /// The distance from the left edge of the line to the right edge of the fragment.
  double get right =>
      line.textDirection == ui.TextDirection.ltr ? endOffset : line.width - startOffset;

  /// Set the horizontal position of this fragment relative to the [line] that
  /// contains it.
  void setPosition({required double startOffset, required ui.TextDirection textDirection}) {
    _startOffset = startOffset;
    _textDirection ??= textDirection;
  }

  /// Adjust the width of this fragment for paragraph justification.
  void justifyTo({required double paragraphWidth}) {
    // Only justify this fragment if it's not a trailing space in the line.
    if (end > line.endIndex - line.trailingSpaces) {
      // Don't justify fragments that are part of trailing spaces of the line.
      return;
    }

    if (trailingSpaces == 0) {
      // If this fragment has no spaces, there's nothing to justify.
      return;
    }

    final double justificationTotal = paragraphWidth - line.width;
    final double justificationPerSpace = justificationTotal / line.nonTrailingSpaces;
    _extraWidthForJustification = justificationPerSpace * trailingSpaces;
  }
}

/// Encapsulates calculations related to the bounding box of the fragment
/// relative to the paragraph.
mixin _FragmentBox on _CombinedFragment, _FragmentMetrics, _FragmentPosition {
  double get top => line.baseline - ascent;
  double get bottom => line.baseline + descent;

  late final ui.TextBox _textBoxIncludingTrailingSpaces = ui.TextBox.fromLTRBD(
    line.left + left,
    top,
    line.left + right,
    bottom,
    textDirection!,
  );

  /// Whether or not the trailing spaces of this fragment are part of trailing
  /// spaces of the line containing the fragment.
  bool get _isPartOfTrailingSpacesInLine => end > line.endIndex - line.trailingSpaces;

  /// Returns a [ui.TextBox] for the purpose of painting this fragment.
  ///
  /// The coordinates of the resulting [ui.TextBox] are relative to the
  /// paragraph, not to the line.
  ///
  /// Trailing spaces in each line aren't painted on the screen, so they are
  /// excluded from the resulting text box.
  ui.TextBox toPaintingTextBox() {
    if (_isPartOfTrailingSpacesInLine) {
      // For painting, we exclude the width of trailing spaces from the box.
      return textDirection! == ui.TextDirection.ltr
          ? ui.TextBox.fromLTRBD(
            line.left + left,
            top,
            line.left + right - widthOfTrailingSpaces,
            bottom,
            textDirection!,
          )
          : ui.TextBox.fromLTRBD(
            line.left + left + widthOfTrailingSpaces,
            top,
            line.left + right,
            bottom,
            textDirection!,
          );
    }
    return _textBoxIncludingTrailingSpaces;
  }

  /// Returns a [ui.TextBox] representing this fragment.
  ///
  /// The coordinates of the resulting [ui.TextBox] are relative to the
  /// paragraph, not to the line.
  ///
  /// As opposed to [toPaintingTextBox], the resulting text box from this method
  /// includes trailing spaces of the fragment.
  ui.TextBox toTextBox({int? start, int? end}) {
    start ??= this.start;
    end ??= this.end;

    if (start <= this.start && end >= this.end - trailingNewlines) {
      return _textBoxIncludingTrailingSpaces;
    }
    return _intersect(start, end);
  }

  /// Performs the intersection of this fragment with the range given by [start] and
  /// [end] indices, and returns a [ui.TextBox] representing that intersection.
  ///
  /// The coordinates of the resulting [ui.TextBox] are relative to the
  /// paragraph, not to the line.
  ui.TextBox _intersect(int start, int end) {
    // `_intersect` should only be called when there's an actual intersection.
    assert(start > this.start || end < this.end);

    final double before;
    if (start <= this.start) {
      before = 0.0;
    } else {
      _spanometer.currentSpan = span;
      before = _spanometer.measureRange(this.start, start);
    }

    final double after;
    if (end >= this.end - trailingNewlines) {
      after = 0.0;
    } else {
      _spanometer.currentSpan = span;
      after = _spanometer.measureRange(end, this.end - trailingNewlines);
    }

    final double left, right;
    if (textDirection! == ui.TextDirection.ltr) {
      // Example: let's say the text is "Loremipsum" and we want to get the box
      // for "rem". In this case, `before` is the width of "Lo", and `after`
      // is the width of "ipsum".
      //
      // Here's how the measurements/coordinates look like:
      //
      //              before         after
      //              |----|     |----------|
      //              +---------------------+
      //              | L o r e m i p s u m |
      //              +---------------------+
      //    this.left ^                     ^ this.right
      left = this.left + before;
      right = this.right - after;
    } else {
      // Example: let's say the text is "txet_werbeH" ("Hebrew_text" flowing from
      // right to left). Say we want to get the box for "brew". The `before` is
      // the width of "He", and `after` is the width of "_text".
      //
      //                 after           before
      //              |----------|       |----|
      //              +-----------------------+
      //              | t x e t _ w e r b e H |
      //              +-----------------------+
      //    this.left ^                       ^ this.right
      //
      // Notice how `before` and `after` are reversed in the RTL example. That's
      // because the text flows from right to left.
      left = this.left + after;
      right = this.right - before;
    }

    // The fragment's left and right edges are relative to the line. In order
    // to make them relative to the paragraph, we need to add the left edge of
    // the line.
    return ui.TextBox.fromLTRBD(line.left + left, top, line.left + right, bottom, textDirection!);
  }

  /// Returns the text position within this fragment's range that's closest to
  /// the given [x] offset.
  ///
  /// The [x] offset is expected to be relative to the left edge of the fragment.
  ui.TextPosition getPositionForX(double x) {
    x = _makeXDirectionAgnostic(x);

    final int startIndex = start;
    final int endIndex = end - trailingNewlines;

    // Check some special cases to return the result quicker.

    final int length = endIndex - startIndex;
    if (length == 0) {
      return ui.TextPosition(offset: startIndex);
    }
    if (length == 1) {
      // Find out if `x` is closer to `startIndex` or `endIndex`.
      final double distanceFromStart = x;
      final double distanceFromEnd = widthIncludingTrailingSpaces - x;
      return distanceFromStart < distanceFromEnd
          ? ui.TextPosition(offset: startIndex)
          : ui.TextPosition(offset: endIndex, affinity: ui.TextAffinity.upstream);
    }

    _spanometer.currentSpan = span;
    // The resulting `cutoff` is the index of the character where the `x` offset
    // falls. We should return the text position of either `cutoff` or
    // `cutoff + 1` depending on which one `x` is closer to.
    //
    //   offset x
    //      ↓
    // "A B C D E F"
    //     ↑
    //   cutoff
    final int cutoff = _spanometer.forceBreak(
      startIndex,
      endIndex,
      availableWidth: x,
      allowEmpty: true,
    );

    if (cutoff == endIndex) {
      return ui.TextPosition(offset: cutoff, affinity: ui.TextAffinity.upstream);
    }

    final double lowWidth = _spanometer.measureRange(startIndex, cutoff);
    final double highWidth = _spanometer.measureRange(startIndex, cutoff + 1);

    // See if `x` is closer to `cutoff` or `cutoff + 1`.
    if (x - lowWidth < highWidth - x) {
      // The offset is closer to cutoff.
      return ui.TextPosition(offset: cutoff);
    } else {
      // The offset is closer to cutoff + 1.
      return ui.TextPosition(offset: cutoff + 1, affinity: ui.TextAffinity.upstream);
    }
  }

  /// Transforms the [x] coordinate to be direction-agnostic.
  ///
  /// The X (input) is relative to the [left] edge of the fragment, and this
  /// method returns an X' (output) that's relative to beginning of the text.
  ///
  /// Here's how it looks for a fragment with LTR content:
  ///
  ///          *------------------------line width------------------*
  ///                      *-----X (input)
  ///          ┌───────────┬────────────────────────┬───────────────┐
  ///          │           │ ---text-direction----> │               │
  ///          └───────────┴────────────────────────┴───────────────┘
  ///                      *-----X' (output)
  ///          *---left----*
  ///          *---------------right----------------*
  ///
  ///
  /// And here's how it looks for a fragment with RTL content:
  ///
  ///          *------------------------line width------------------*
  ///                      *-----X (input)
  ///          ┌───────────┬────────────────────────┬───────────────┐
  ///          │           │ <---text-direction---- │               │
  ///          └───────────┴────────────────────────┴───────────────┘
  ///                   (output) X'-----------------*
  ///          *---left----*
  ///          *---------------right----------------*
  ///
  double _makeXDirectionAgnostic(double x) {
    if (textDirection == ui.TextDirection.rtl) {
      return widthIncludingTrailingSpaces - x;
    }
    return x;
  }

  // [start, end).map((index) => line.graphemeStarts[index]) gives an ascending
  // list of UTF16 offsets of graphemes that start in this fragment.
  //
  // Returns null if this fragment contains no grapheme starts.
  late final (int, int)? graphemeStartIndexRange = _getBreaksRange();
  (int, int)? _getBreaksRange() {
    if (end == start) {
      return null;
    }
    final List<int> lineGraphemeBreaks = line.graphemeStarts;
    assert(end > start);
    assert(line.graphemeStarts.isNotEmpty);
    final int startIndex = line.graphemeStartIndexBefore(start, 0, lineGraphemeBreaks.length);
    final int endIndex =
        end == start + 1
            ? startIndex + 1
            : line.graphemeStartIndexBefore(end - 1, startIndex, lineGraphemeBreaks.length) + 1;
    final int firstGraphemeStart = lineGraphemeBreaks[startIndex];
    return firstGraphemeStart > start
        ? (endIndex == startIndex + 1 ? null : (startIndex + 1, endIndex))
        : (startIndex, endIndex);
  }

  /// Whether the first codepoints of this fragment is not a valid grapheme start,
  /// and belongs in the the previous fragment.
  ///
  /// This is the result of a known bug: in rare circumstances, a grapheme is
  /// split into different fragments. To workaround this we ignore the trailing
  /// part of the grapheme during hit-testing, by adjusting the leading offset of
  /// a fragment to the leading edge of the first grapheme start in that fragment.
  //
  // TODO(LongCatIsLooong): Grapheme clusters should not be separately even
  // when they are in different runs. Also document the recommendation to use
  // U+25CC or U+00A0 for showing nonspacing marks in isolation.
  bool get hasLeadingBrokenGrapheme {
    final int? graphemeStartIndexRangeStart = graphemeStartIndexRange?.$1;
    return graphemeStartIndexRangeStart == null ||
        line.graphemeStarts[graphemeStartIndexRangeStart] != start;
  }

  /// Returns the GlyphInfo within the range [line.graphemeStarts[startIndex], line.graphemeStarts[endIndex]),
  /// that's visually closeset to the given horizontal offset `x` (in the paragraph's coordinates).
  ui.GlyphInfo _getClosestCharacterInRange(double x, int startIndex, int endIndex) {
    final List<int> graphemeStartIndices = line.graphemeStarts;
    final ui.TextRange fullRange = ui.TextRange(
      start: graphemeStartIndices[startIndex],
      end: graphemeStartIndices[endIndex],
    );
    final ui.TextBox fullBox = toTextBox(start: fullRange.start, end: fullRange.end);
    if (startIndex + 1 == endIndex) {
      return ui.GlyphInfo(fullBox.toRect(), fullRange, fullBox.direction);
    }
    assert(startIndex + 1 < endIndex);
    final ui.TextBox(:double left, :double right) = fullBox;

    // The toTextBox call is potentially expensive so we'll try reducing the
    // search steps with a binary search.
    //
    // x ∈ (left, right),
    if (left < x && x < right) {
      final int midIndex = (startIndex + endIndex) ~/ 2;
      // endIndex >= startIndex + 2, so midIndex >= start + 1
      final ui.GlyphInfo firstHalf = _getClosestCharacterInRange(x, startIndex, midIndex);
      if (firstHalf.graphemeClusterLayoutBounds.left < x &&
          x < firstHalf.graphemeClusterLayoutBounds.right) {
        return firstHalf;
      }
      // startIndex <= endIndex - 2, so midIndex <= endIndex - 1
      final ui.GlyphInfo secondHalf = _getClosestCharacterInRange(x, midIndex, endIndex);
      if (secondHalf.graphemeClusterLayoutBounds.left < x &&
          x < secondHalf.graphemeClusterLayoutBounds.right) {
        return secondHalf;
      }
      // Neither box clips the given x. This is supposed to be rare.
      final double distanceToFirst =
          (x -
                  x.clamp(
                    firstHalf.graphemeClusterLayoutBounds.left,
                    firstHalf.graphemeClusterLayoutBounds.right,
                  ))
              .abs();
      final double distanceToSecond =
          (x -
                  x.clamp(
                    secondHalf.graphemeClusterLayoutBounds.left,
                    secondHalf.graphemeClusterLayoutBounds.right,
                  ))
              .abs();
      return distanceToFirst > distanceToSecond ? firstHalf : secondHalf;
    }

    // x ∉ (left, right), it's either the first character or the last, since
    // there can only be one writing direction in the fragment.
    final ui.TextRange range = switch ((fullBox.direction, x <= left)) {
      (ui.TextDirection.ltr, true) || (ui.TextDirection.rtl, false) => ui.TextRange(
        start: graphemeStartIndices[startIndex],
        end: graphemeStartIndices[startIndex + 1],
      ),
      (ui.TextDirection.ltr, false) || (ui.TextDirection.rtl, true) => ui.TextRange(
        start: graphemeStartIndices[endIndex - 1],
        end: graphemeStartIndices[endIndex],
      ),
    };
    assert(!range.isCollapsed);
    final ui.TextBox box = toTextBox(start: range.start, end: range.end);
    return ui.GlyphInfo(box.toRect(), range, box.direction);
  }

  /// Returns the GlyphInfo of the character in the fragment that is closest to
  /// the given offset x.
  ui.GlyphInfo getClosestCharacterBox(double x) {
    assert(end > start);
    assert(graphemeStartIndexRange != null);
    // The force ! is safe here because this method is only called by
    // LayoutService.getClosestGlyphInfo which checks this fragment has at least
    // one grapheme start before calling this method.
    final (int rangeStart, int rangeEnd) = graphemeStartIndexRange!;
    return _getClosestCharacterInRange(x, rangeStart, rangeEnd);
  }
}

class EllipsisFragment extends LayoutFragment {
  EllipsisFragment(int index, ParagraphSpan span)
    : super(
        index,
        index,
        LineBreakType.endOfText,
        null,
        // The ellipsis is always at the end of the line, so it can't be
        // sandwiched. This means it'll always follow the paragraph direction.
        FragmentFlow.sandwich,
        span,
        trailingNewlines: 0,
        trailingSpaces: 0,
      );

  @override
  bool get isSpaceOnly => false;

  @override
  bool get isPlaceholder => false;

  @override
  String getText(CanvasParagraph paragraph) {
    return paragraph.paragraphStyle.ellipsis!;
  }

  @override
  List<LayoutFragment> split(int index) {
    throw Exception('Cannot split an EllipsisFragment');
  }
}
