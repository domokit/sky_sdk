// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;

import 'package:ui/ui.dart' as ui;
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

import '../dom.dart';
import '../util.dart';
import '../view_embedder/style_manager.dart';
import 'canvas_paragraph.dart';
import 'layout_fragmenter.dart';
import 'ruler.dart';

class EngineLineMetrics implements ui.LineMetrics {
  const EngineLineMetrics({
    required this.hardBreak,
    required this.ascent,
    required this.descent,
    required this.unscaledAscent,
    required this.height,
    required this.width,
    required this.left,
    required this.baseline,
    required this.lineNumber,
  });

  @override
  final bool hardBreak;

  @override
  final double ascent;

  @override
  final double descent;

  @override
  final double unscaledAscent;

  @override
  final double height;

  @override
  final double width;

  @override
  final double left;

  @override
  final double baseline;

  @override
  final int lineNumber;

  @override
  int get hashCode => Object.hash(
    hardBreak,
    ascent,
    descent,
    unscaledAscent,
    height,
    width,
    left,
    baseline,
    lineNumber,
  );

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is EngineLineMetrics &&
        other.hardBreak == hardBreak &&
        other.ascent == ascent &&
        other.descent == descent &&
        other.unscaledAscent == unscaledAscent &&
        other.height == height &&
        other.width == width &&
        other.left == left &&
        other.baseline == baseline &&
        other.lineNumber == lineNumber;
  }

  @override
  String toString() {
    String result = super.toString();
    assert(() {
      result =
          'LineMetrics(hardBreak: $hardBreak, '
          'ascent: $ascent, '
          'descent: $descent, '
          'unscaledAscent: $unscaledAscent, '
          'height: $height, '
          'width: $width, '
          'left: $left, '
          'baseline: $baseline, '
          'lineNumber: $lineNumber)';
      return true;
    }());
    return result;
  }
}

class ParagraphLine {
  ParagraphLine({
    required bool hardBreak,
    required double ascent,
    required double descent,
    required double height,
    required double width,
    required double left,
    required double baseline,
    required int lineNumber,
    required this.startIndex,
    required this.endIndex,
    required this.trailingNewlines,
    required this.trailingSpaces,
    required this.spaceCount,
    required this.widthWithTrailingSpaces,
    required this.fragments,
    required this.textDirection,
    required this.paragraph,
    this.displayText,
  }) : assert(trailingNewlines <= endIndex - startIndex),
       lineMetrics = EngineLineMetrics(
         hardBreak: hardBreak,
         ascent: ascent,
         descent: descent,
         unscaledAscent: ascent,
         height: height,
         width: width,
         left: left,
         baseline: baseline,
         lineNumber: lineNumber,
       );

  /// Metrics for this line of the paragraph.
  final EngineLineMetrics lineMetrics;

  /// The index (inclusive) in the text where this line begins.
  final int startIndex;

  /// The index (exclusive) in the text where this line ends.
  ///
  /// When the line contains an overflow, then [endIndex] goes until the end of
  /// the text and doesn't stop at the overflow cutoff.
  final int endIndex;

  /// The largest visible index (exclusive) in this line.
  ///
  /// When the line contains an overflow, or is ellipsized at the end, this is
  /// the largest index that remains visible in this line. If the entire line is
  /// ellipsized, this returns [startIndex];
  late final int visibleEndIndex = switch (fragments) {
    [] => startIndex,
    [...final List<LayoutFragment> rest, EllipsisFragment()] || final List<LayoutFragment> rest =>
      rest.last.end,
  };

  /// The number of new line characters at the end of the line.
  final int trailingNewlines;

  /// The number of spaces at the end of the line.
  final int trailingSpaces;

  /// The number of space characters in the entire line.
  final int spaceCount;

  /// The full width of the line including all trailing space but not new lines.
  ///
  /// The difference between [width] and [widthWithTrailingSpaces] is that
  /// [widthWithTrailingSpaces] includes trailing spaces in the width
  /// calculation while [width] doesn't.
  ///
  /// For alignment purposes for example, the [width] property is the right one
  /// to use because trailing spaces shouldn't affect the centering of text.
  /// But for placing cursors in text fields, we do care about trailing
  /// spaces so [widthWithTrailingSpaces] is more suitable.
  final double widthWithTrailingSpaces;

  /// The fragments that make up this line.
  ///
  /// The fragments in the [List] are sorted by their logical order in within the
  /// line. In other words, a [LayoutFragment] in the [List] will have larger
  /// start and end indices than all [LayoutFragment]s that appear before it.
  final List<LayoutFragment> fragments;

  /// The text direction of this line, which is the same as the paragraph's.
  final ui.TextDirection textDirection;

  /// The text to be rendered on the screen representing this line.
  final String? displayText;

  /// The [CanvasParagraph] this line is part of.
  final CanvasParagraph paragraph;

  /// The number of space characters in the line excluding trailing spaces.
  int get nonTrailingSpaces => spaceCount - trailingSpaces;

  // Convenient getters for line metrics properties.

  bool get hardBreak => lineMetrics.hardBreak;
  double get ascent => lineMetrics.ascent;
  double get descent => lineMetrics.descent;
  double get unscaledAscent => lineMetrics.unscaledAscent;
  double get height => lineMetrics.height;
  double get width => lineMetrics.width;
  double get left => lineMetrics.left;
  double get baseline => lineMetrics.baseline;
  int get lineNumber => lineMetrics.lineNumber;

  bool overlapsWith(int startIndex, int endIndex) {
    return startIndex < this.endIndex && this.startIndex < endIndex;
  }

  String getText(CanvasParagraph paragraph) {
    final StringBuffer buffer = StringBuffer();
    for (final LayoutFragment fragment in fragments) {
      buffer.write(fragment.getText(paragraph));
    }
    return buffer.toString();
  }

  // This is the fallback graphme breaker that is only used if Intl.Segmenter()
  // is not supported so _fromDomSegmenter can't be called. This implementation
  // breaks the text into UTF-16 codepoints instead of graphme clusters.
  List<int> _fallbackGraphemeStartIterable(String lineText) {
    final List<int> graphemeStarts = <int>[];
    bool precededByHighSurrogate = false;
    for (int i = 0; i < lineText.length; i++) {
      final int maskedCodeUnit = lineText.codeUnitAt(i) & 0xFC00;
      // Only skip `i` if it points to a low surrogate in a valid surrogate pair.
      if (maskedCodeUnit != 0xDC00 || !precededByHighSurrogate) {
        graphemeStarts.add(startIndex + i);
      }
      precededByHighSurrogate = maskedCodeUnit == 0xD800;
    }
    return graphemeStarts;
  }

  // This will be called at most once to lazily populate _graphemeStarts.
  List<int> _fromDomSegmenter(String fragmentText) {
    final DomSegmenter domSegmenter = createIntlSegmenter(granularity: 'grapheme');
    final List<int> graphemeStarts = <int>[];
    final Iterator<DomSegment> segments = domSegmenter.segment(fragmentText).iterator();
    while (segments.moveNext()) {
      graphemeStarts.add(segments.current.index + startIndex);
    }
    assert(graphemeStarts.isEmpty || graphemeStarts.first == startIndex);
    return graphemeStarts;
  }

  List<int> _breakTextIntoGraphemes(String text) {
    final List<int> graphemeStarts =
        domIntl.Segmenter == null ? _fallbackGraphemeStartIterable(text) : _fromDomSegmenter(text);
    // Add the end index of the fragment to the list if the text is not empty.
    if (graphemeStarts.isNotEmpty) {
      graphemeStarts.add(visibleEndIndex);
    }
    return graphemeStarts;
  }

  /// This List contains an ascending sequence of UTF16 offsets that points to
  /// grapheme starts within the line. Each UTF16 offset is relative to the
  /// start of the paragraph, instead of the start of the line.
  ///
  /// For example, `graphemeStarts[n]` gives the UTF16 offset of the `n`-th
  /// grapheme in the line.
  late final List<int> graphemeStarts =
      visibleEndIndex == startIndex
          ? const <int>[]
          : _breakTextIntoGraphemes(paragraph.plainText.substring(startIndex, visibleEndIndex));

  /// Translate a UTF16 code unit in the paragaph (`offset`), to a grapheme
  /// offset with in the current line.
  ///
  /// The `start` and `end` parameters are both grapheme offsets within the
  /// current line. They are used to limit the search range (so the return value
  /// that corresponds to the code unit `offset` must be with in [start, end)).
  int graphemeStartIndexBefore(int offset, int start, int end) {
    int low = start;
    int high = end;
    assert(0 <= low);
    assert(low < high);

    final List<int> lineGraphemeBreaks = graphemeStarts;
    assert(offset >= lineGraphemeBreaks[start]);
    assert(offset < lineGraphemeBreaks.last, '$offset, $lineGraphemeBreaks');
    assert(end == lineGraphemeBreaks.length || offset < lineGraphemeBreaks[end]);
    while (low + 2 <= high) {
      // high >= low + 2, so low + 1 <= mid <= high - 1
      final int mid = (low + high) ~/ 2;
      switch (lineGraphemeBreaks[mid] - offset) {
        case > 0:
          high = mid;
        case < 0:
          low = mid;
        case == 0:
          return mid;
      }
    }

    assert(lineGraphemeBreaks[low] <= offset);
    assert(high == lineGraphemeBreaks.length || offset < lineGraphemeBreaks[high]);
    return low;
  }

  /// Returns the UTF-16 range of the character that encloses the code unit at
  /// the given offset.
  ui.TextRange? getCharacterRangeAt(int codeUnitOffset) {
    assert(codeUnitOffset >= this.startIndex);
    if (codeUnitOffset >= visibleEndIndex || graphemeStarts.isEmpty) {
      return null;
    }

    final int startIndex = graphemeStartIndexBefore(codeUnitOffset, 0, graphemeStarts.length);
    assert(startIndex < graphemeStarts.length - 1);
    return ui.TextRange(start: graphemeStarts[startIndex], end: graphemeStarts[startIndex + 1]);
  }

  LayoutFragment? closestFragmentTo(LayoutFragment targetFragment, bool searchLeft) {
    ({LayoutFragment fragment, double distance})? closestFragment;
    for (final LayoutFragment fragment in fragments) {
      assert(fragment is! EllipsisFragment);
      if (fragment.start >= visibleEndIndex) {
        break;
      }
      if (fragment.graphemeStartIndexRange == null) {
        continue;
      }
      final double distance =
          searchLeft ? targetFragment.left - fragment.right : fragment.left - targetFragment.right;
      final double? minDistance = closestFragment?.distance;
      switch (distance) {
        case > 0.0 when minDistance == null || minDistance > distance:
          closestFragment = (fragment: fragment, distance: distance);
        case == 0.0:
          return fragment;
        case _:
          continue;
      }
    }
    return closestFragment?.fragment;
  }

  /// Finds the closest [LayoutFragment] to the given horizontal offset `dx` in
  /// this line, that is not an [EllipsisFragment] and contains at least one
  /// grapheme start.
  LayoutFragment? closestFragmentAtOffset(double dx) {
    if (graphemeStarts.isEmpty) {
      return null;
    }
    assert(graphemeStarts.length >= 2);
    int graphemeIndex = 0;
    ({LayoutFragment fragment, double distance})? closestFragment;
    for (final LayoutFragment fragment in fragments) {
      assert(fragment is! EllipsisFragment);
      if (fragment.start >= visibleEndIndex) {
        break;
      }
      if (fragment.length == 0) {
        continue;
      }
      while (fragment.start > graphemeStarts[graphemeIndex]) {
        graphemeIndex += 1;
      }
      final int firstGraphemeStartInFragment = graphemeStarts[graphemeIndex];
      if (firstGraphemeStartInFragment >= fragment.end) {
        continue;
      }
      final double distance;
      if (dx < fragment.left) {
        distance = fragment.left - dx;
      } else if (dx > fragment.right) {
        distance = dx - fragment.right;
      } else {
        return fragment;
      }
      assert(distance > 0);

      final double? minDistance = closestFragment?.distance;
      if (minDistance == null || minDistance > distance) {
        closestFragment = (fragment: fragment, distance: distance);
      }
    }
    return closestFragment?.fragment;
  }

  @override
  int get hashCode => Object.hash(
    lineMetrics,
    startIndex,
    endIndex,
    trailingNewlines,
    trailingSpaces,
    spaceCount,
    widthWithTrailingSpaces,
    fragments,
    textDirection,
    displayText,
  );

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is ParagraphLine &&
        other.lineMetrics == lineMetrics &&
        other.startIndex == startIndex &&
        other.endIndex == endIndex &&
        other.trailingNewlines == trailingNewlines &&
        other.trailingSpaces == trailingSpaces &&
        other.spaceCount == spaceCount &&
        other.widthWithTrailingSpaces == widthWithTrailingSpaces &&
        other.fragments == fragments &&
        other.textDirection == textDirection &&
        other.displayText == displayText;
  }

  @override
  String toString() {
    return '$ParagraphLine($startIndex, $endIndex, $lineMetrics)';
  }
}

/// The web implementation of [ui.ParagraphStyle].
class EngineParagraphStyle implements ui.ParagraphStyle {
  /// Creates a new instance of [EngineParagraphStyle].
  EngineParagraphStyle({
    this.textAlign,
    this.textDirection,
    this.maxLines,
    this.fontFamily,
    this.fontSize,
    double? height,
    ui.TextHeightBehavior? textHeightBehavior,
    this.fontWeight,
    this.fontStyle,
    ui.StrutStyle? strutStyle,
    this.ellipsis,
    this.locale,
  }) : _textHeightBehavior = textHeightBehavior,
       // TODO(mdebbar): add support for strut style., b/128317744
       _strutStyle = strutStyle as EngineStrutStyle?,
       height = height == ui.kTextHeightNone ? null : height;

  final ui.TextAlign? textAlign;
  final ui.TextDirection? textDirection;
  final ui.FontWeight? fontWeight;
  final ui.FontStyle? fontStyle;
  final int? maxLines;
  final String? fontFamily;
  final double? fontSize;
  final double? height;
  final ui.TextHeightBehavior? _textHeightBehavior;
  final EngineStrutStyle? _strutStyle;
  final String? ellipsis;
  final ui.Locale? locale;

  // The effective style attributes should be consistent with paragraph_style.h.
  ui.TextAlign get effectiveTextAlign => textAlign ?? ui.TextAlign.start;
  ui.TextDirection get effectiveTextDirection => textDirection ?? ui.TextDirection.ltr;

  double? get lineHeight {
    // TODO(mdebbar): Implement proper support for strut styles.
    // https://github.com/flutter/flutter/issues/32243
    final EngineStrutStyle? strutStyle = _strutStyle;
    final double? strutHeight = strutStyle?._height;
    if (strutStyle == null || strutHeight == null || strutHeight == 0) {
      // When there's no strut height, always use paragraph style height.
      return height;
    }
    if (strutStyle._forceStrutHeight ?? false) {
      // When strut height is forced, ignore paragraph style height.
      return strutHeight;
    }
    // In this case, strut height acts as a minimum height for all parts of the
    // paragraph. So we take the max of strut height and paragraph style height.
    return math.max(strutHeight, height ?? 0.0);
  }

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is EngineParagraphStyle &&
        other.textAlign == textAlign &&
        other.textDirection == textDirection &&
        other.fontWeight == fontWeight &&
        other.fontStyle == fontStyle &&
        other.maxLines == maxLines &&
        other.fontFamily == fontFamily &&
        other.fontSize == fontSize &&
        other.height == height &&
        other._textHeightBehavior == _textHeightBehavior &&
        other._strutStyle == _strutStyle &&
        other.ellipsis == ellipsis &&
        other.locale == locale;
  }

  @override
  int get hashCode {
    return Object.hash(
      textAlign,
      textDirection,
      fontWeight,
      fontStyle,
      maxLines,
      fontFamily,
      fontSize,
      height,
      _textHeightBehavior,
      _strutStyle,
      ellipsis,
      locale,
    );
  }

  @override
  String toString() {
    String result = super.toString();
    assert(() {
      final double? fontSize = this.fontSize;
      final double? height = this.height;
      result =
          'ParagraphStyle('
          'textAlign: ${textAlign ?? "unspecified"}, '
          'textDirection: ${textDirection ?? "unspecified"}, '
          'fontWeight: ${fontWeight ?? "unspecified"}, '
          'fontStyle: ${fontStyle ?? "unspecified"}, '
          'maxLines: ${maxLines ?? "unspecified"}, '
          'textHeightBehavior: ${_textHeightBehavior ?? "unspecified"}, '
          'fontFamily: ${fontFamily ?? "unspecified"}, '
          'fontSize: ${fontSize != null ? fontSize.toStringAsFixed(1) : "unspecified"}, '
          'height: ${height != null ? "${height.toStringAsFixed(1)}x" : "unspecified"}, '
          'strutStyle: ${_strutStyle ?? "unspecified"}, '
          'ellipsis: ${ellipsis != null ? '"$ellipsis"' : "unspecified"}, '
          'locale: ${locale ?? "unspecified"}'
          ')';
      return true;
    }());
    return result;
  }
}

/// The web implementation of [ui.TextStyle].
class EngineTextStyle implements ui.TextStyle {
  /// Constructs an [EngineTextStyle] with all properties being required.
  ///
  /// This is good for call sites that need to be updated whenever a new
  /// property is added to [EngineTextStyle]. Non-updated call sites will fail
  /// the build otherwise.
  factory EngineTextStyle({
    required ui.Color? color,
    required ui.TextDecoration? decoration,
    required ui.Color? decorationColor,
    required ui.TextDecorationStyle? decorationStyle,
    required double? decorationThickness,
    required ui.FontWeight? fontWeight,
    required ui.FontStyle? fontStyle,
    required ui.TextBaseline? textBaseline,
    required String? fontFamily,
    required List<String>? fontFamilyFallback,
    required double? fontSize,
    required double? letterSpacing,
    required double? wordSpacing,
    required double? height,
    required ui.TextLeadingDistribution? leadingDistribution,
    required ui.Locale? locale,
    required ui.Paint? background,
    required ui.Paint? foreground,
    required List<ui.Shadow>? shadows,
    required List<ui.FontFeature>? fontFeatures,
    required List<ui.FontVariation>? fontVariations,
  }) = EngineTextStyle.only;

  /// Constructs an [EngineTextStyle] with only the given properties.
  ///
  /// This constructor should be used sparingly in tests, for example. Or when
  /// we know for sure that not all properties are needed.
  EngineTextStyle.only({
    this.color,
    this.decoration,
    this.decorationColor,
    this.decorationStyle,
    this.decorationThickness,
    this.fontWeight,
    this.fontStyle,
    this.textBaseline,
    String? fontFamily,
    this.fontFamilyFallback,
    this.fontSize,
    this.letterSpacing,
    this.wordSpacing,
    this.height,
    this.leadingDistribution,
    this.locale,
    this.background,
    this.foreground,
    this.shadows,
    this.fontFeatures,
    this.fontVariations,
  }) : assert(
         color == null || foreground == null,
         'Cannot provide both a color and a foreground\n'
         'The color argument is just a shorthand for "foreground: Paint()..color = color".',
       ),
       isFontFamilyProvided = fontFamily != null,
       fontFamily = fontFamily ?? '';

  /// Constructs an [EngineTextStyle] by reading properties from an
  /// [EngineParagraphStyle].
  factory EngineTextStyle.fromParagraphStyle(EngineParagraphStyle paragraphStyle) {
    return EngineTextStyle.only(
      fontWeight: paragraphStyle.fontWeight,
      fontStyle: paragraphStyle.fontStyle,
      fontFamily: paragraphStyle.fontFamily,
      fontSize: paragraphStyle.fontSize,
      height: paragraphStyle.height,
      locale: paragraphStyle.locale,
    );
  }

  final ui.Color? color;
  final ui.TextDecoration? decoration;
  final ui.Color? decorationColor;
  final ui.TextDecorationStyle? decorationStyle;
  final double? decorationThickness;
  final ui.FontWeight? fontWeight;
  final ui.FontStyle? fontStyle;
  final ui.TextBaseline? textBaseline;
  final bool isFontFamilyProvided;
  final String fontFamily;
  final List<String>? fontFamilyFallback;
  final List<ui.FontFeature>? fontFeatures;
  final List<ui.FontVariation>? fontVariations;
  final double? fontSize;
  final double? letterSpacing;
  final double? wordSpacing;
  final double? height;
  final ui.TextLeadingDistribution? leadingDistribution;
  final ui.Locale? locale;
  final ui.Paint? background;
  final ui.Paint? foreground;
  final List<ui.Shadow>? shadows;

  static final List<String> _testFonts = <String>['FlutterTest', 'Ahem'];
  String get effectiveFontFamily {
    final String fontFamily =
        this.fontFamily.isEmpty ? StyleManager.defaultFontFamily : this.fontFamily;
    // In the flutter tester environment, we use predictable-size test fonts.
    // This makes widget tests predictable and less flaky.
    String result = fontFamily;
    assert(() {
      if (ui_web.debugEmulateFlutterTesterEnvironment && !_testFonts.contains(fontFamily)) {
        result = _testFonts.first;
      }
      return true;
    }());
    return result;
  }

  String? _cssFontString;

  /// Font string to be used in CSS.
  ///
  /// See <https://developer.mozilla.org/en-US/docs/Web/CSS/font>.
  String get cssFontString {
    return _cssFontString ??= buildCssFontString(
      fontStyle: fontStyle,
      fontWeight: fontWeight,
      fontSize: fontSize,
      fontFamily: effectiveFontFamily,
    );
  }

  late final TextHeightStyle heightStyle = _createHeightStyle();

  TextHeightStyle _createHeightStyle() {
    return TextHeightStyle(
      fontFamily: effectiveFontFamily,
      fontSize: fontSize ?? StyleManager.defaultFontSize,
      height: height,
      // TODO(mdebbar): Pass the actual value when font features become supported
      //                https://github.com/flutter/flutter/issues/64595
      fontFeatures: null,
      fontVariations: null,
    );
  }

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    return other is EngineTextStyle &&
        other.color == color &&
        other.decoration == decoration &&
        other.decorationColor == decorationColor &&
        other.decorationStyle == decorationStyle &&
        other.fontWeight == fontWeight &&
        other.fontStyle == fontStyle &&
        other.textBaseline == textBaseline &&
        other.leadingDistribution == leadingDistribution &&
        other.fontFamily == fontFamily &&
        other.fontSize == fontSize &&
        other.letterSpacing == letterSpacing &&
        other.wordSpacing == wordSpacing &&
        other.height == height &&
        other.decorationThickness == decorationThickness &&
        other.locale == locale &&
        other.background == background &&
        other.foreground == foreground &&
        listEquals<ui.Shadow>(other.shadows, shadows) &&
        listEquals<String>(other.fontFamilyFallback, fontFamilyFallback) &&
        listEquals<ui.FontFeature>(other.fontFeatures, fontFeatures) &&
        listEquals<ui.FontVariation>(other.fontVariations, fontVariations);
  }

  @override
  int get hashCode {
    final List<ui.Shadow>? shadows = this.shadows;
    final List<ui.FontFeature>? fontFeatures = this.fontFeatures;
    final List<ui.FontVariation>? fontVariations = this.fontVariations;
    final List<String>? fontFamilyFallback = this.fontFamilyFallback;
    return Object.hash(
      color,
      decoration,
      decorationColor,
      decorationStyle,
      fontWeight,
      fontStyle,
      textBaseline,
      leadingDistribution,
      fontFamily,
      fontFamilyFallback == null ? null : Object.hashAll(fontFamilyFallback),
      fontSize,
      letterSpacing,
      wordSpacing,
      height,
      locale,
      background,
      foreground,
      shadows == null ? null : Object.hashAll(shadows),
      decorationThickness,
      // Object.hash goes up to 20 arguments, but we have 21
      Object.hash(
        fontFeatures == null ? null : Object.hashAll(fontFeatures),
        fontVariations == null ? null : Object.hashAll(fontVariations),
      ),
    );
  }

  @override
  String toString() {
    String result = super.toString();
    assert(() {
      final List<String>? fontFamilyFallback = this.fontFamilyFallback;
      final double? fontSize = this.fontSize;
      final double? height = this.height;
      result =
          'TextStyle('
          'color: ${color ?? "unspecified"}, '
          'decoration: ${decoration ?? "unspecified"}, '
          'decorationColor: ${decorationColor ?? "unspecified"}, '
          'decorationStyle: ${decorationStyle ?? "unspecified"}, '
          'decorationThickness: ${decorationThickness ?? "unspecified"}, '
          'fontWeight: ${fontWeight ?? "unspecified"}, '
          'fontStyle: ${fontStyle ?? "unspecified"}, '
          'textBaseline: ${textBaseline ?? "unspecified"}, '
          'fontFamily: ${isFontFamilyProvided && fontFamily != '' ? fontFamily : "unspecified"}, '
          'fontFamilyFallback: ${isFontFamilyProvided && fontFamilyFallback != null && fontFamilyFallback.isNotEmpty ? fontFamilyFallback : "unspecified"}, '
          'fontSize: ${fontSize != null ? fontSize.toStringAsFixed(1) : "unspecified"}, '
          'letterSpacing: ${letterSpacing != null ? "${letterSpacing}x" : "unspecified"}, '
          'wordSpacing: ${wordSpacing != null ? "${wordSpacing}x" : "unspecified"}, '
          'height: ${height != null ? "${height.toStringAsFixed(1)}x" : "unspecified"}, '
          'leadingDistribution: ${leadingDistribution ?? "unspecified"}, '
          'locale: ${locale ?? "unspecified"}, '
          'background: ${background ?? "unspecified"}, '
          'foreground: ${foreground ?? "unspecified"}, '
          'shadows: ${shadows ?? "unspecified"}, '
          'fontFeatures: ${fontFeatures ?? "unspecified"}, '
          'fontVariations: ${fontVariations ?? "unspecified"}'
          ')';
      return true;
    }());
    return result;
  }
}

/// The web implementation of [ui.StrutStyle].
class EngineStrutStyle implements ui.StrutStyle {
  EngineStrutStyle({
    String? fontFamily,
    List<String>? fontFamilyFallback,
    double? fontSize,
    double? height,
    ui.TextLeadingDistribution? leadingDistribution,
    double? leading,
    ui.FontWeight? fontWeight,
    ui.FontStyle? fontStyle,
    bool? forceStrutHeight,
  }) : _fontFamily = fontFamily,
       _fontFamilyFallback = fontFamilyFallback,
       _fontSize = fontSize,
       _height = height == ui.kTextHeightNone ? null : height,
       _leadingDistribution = leadingDistribution,
       _leading = leading,
       _fontWeight = fontWeight,
       _fontStyle = fontStyle,
       _forceStrutHeight = forceStrutHeight;

  final String? _fontFamily;
  final List<String>? _fontFamilyFallback;
  final double? _fontSize;
  final double? _height;
  final double? _leading;
  final ui.FontWeight? _fontWeight;
  final ui.FontStyle? _fontStyle;
  final bool? _forceStrutHeight;
  final ui.TextLeadingDistribution? _leadingDistribution;

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is EngineStrutStyle &&
        other._fontFamily == _fontFamily &&
        other._fontSize == _fontSize &&
        other._height == _height &&
        other._leading == _leading &&
        other._leadingDistribution == _leadingDistribution &&
        other._fontWeight == _fontWeight &&
        other._fontStyle == _fontStyle &&
        other._forceStrutHeight == _forceStrutHeight &&
        listEquals<String>(other._fontFamilyFallback, _fontFamilyFallback);
  }

  @override
  int get hashCode {
    final List<String>? fontFamilyFallback = _fontFamilyFallback;
    return Object.hash(
      _fontFamily,
      fontFamilyFallback != null ? Object.hashAll(fontFamilyFallback) : null,
      _fontSize,
      _height,
      _leading,
      _leadingDistribution,
      _fontWeight,
      _fontStyle,
      _forceStrutHeight,
    );
  }
}

/// Holds information for a placeholder in a paragraph.
///
/// [width], [height] and [baselineOffset] are expected to be already scaled.
class ParagraphPlaceholder {
  ParagraphPlaceholder(
    this.width,
    this.height,
    this.alignment, {
    required this.baselineOffset,
    required this.baseline,
  });

  /// The scaled width of the placeholder.
  final double width;

  /// The scaled height of the placeholder.
  final double height;

  /// Specifies how the placeholder rectangle will be vertically aligned with
  /// the surrounding text.
  final ui.PlaceholderAlignment alignment;

  /// When the [alignment] value is [ui.PlaceholderAlignment.baseline], the
  /// [baselineOffset] indicates the distance from the baseline to the top of
  /// the placeholder rectangle.
  final double baselineOffset;

  /// Dictates whether to use alphabetic or ideographic baseline.
  final ui.TextBaseline baseline;
}

extension FontStyleExtension on ui.FontStyle {
  /// Converts a [ui.FontStyle] value to its CSS equivalent.
  String toCssString() {
    return this == ui.FontStyle.normal ? 'normal' : 'italic';
  }
}

extension FontWeightExtension on ui.FontWeight {
  /// Converts a [ui.FontWeight] value to its CSS equivalent.
  String toCssString() {
    return fontWeightIndexToCss(fontWeightIndex: index);
  }
}

String fontWeightIndexToCss({int fontWeightIndex = 3}) {
  switch (fontWeightIndex) {
    case 0:
      return '100';
    case 1:
      return '200';
    case 2:
      return '300';
    case 3:
      return 'normal';
    case 4:
      return '500';
    case 5:
      return '600';
    case 6:
      return 'bold';
    case 7:
      return '800';
    case 8:
      return '900';
  }

  assert(() {
    throw AssertionError('Failed to convert font weight $fontWeightIndex to CSS.');
  }());

  return '';
}

/// Applies a text [style] to an [element], translating the properties to their
/// corresponding CSS equivalents.
void applyTextStyleToElement({required DomElement element, required EngineTextStyle style}) {
  bool updateDecoration = false;
  final DomCSSStyleDeclaration cssStyle = element.style;

  final ui.Color? color = style.foreground?.color ?? style.color;
  if (style.foreground?.style == ui.PaintingStyle.stroke) {
    // When comparing the outputs of the Bitmap Canvas and the DOM
    // implementation, we have found, that we need to set the background color
    // of the text to transparent to achieve the same effect as in the Bitmap
    // Canvas and the Skia Engine where only the text stroke is painted.
    // If we don't set it here to transparent, the text will inherit the color
    // of it's parent element.
    cssStyle.color = 'transparent';
    // Use hairline (device pixel when strokeWidth is not specified).
    final double? strokeWidth = style.foreground?.strokeWidth;
    final double adaptedWidth =
        strokeWidth != null && strokeWidth > 0 ? strokeWidth : 1.0 / ui.window.devicePixelRatio;
    cssStyle.textStroke = '${adaptedWidth}px ${color?.toCssString()}';
  } else if (color != null) {
    cssStyle.color = color.toCssString();
  }
  final ui.Color? background = style.background?.color;
  if (background != null) {
    cssStyle.backgroundColor = background.toCssString();
  }
  final double? fontSize = style.fontSize;
  if (fontSize != null) {
    cssStyle.fontSize = '${fontSize.floor()}px';
  }
  if (style.fontWeight != null) {
    cssStyle.fontWeight = style.fontWeight!.toCssString();
  }
  if (style.fontStyle != null) {
    cssStyle.fontStyle = style.fontStyle!.toCssString();
  }
  // For test environment use effectiveFontFamily since we need to
  // consistently use the correct test font.
  if (ui_web.debugEmulateFlutterTesterEnvironment) {
    cssStyle.fontFamily = canonicalizeFontFamily(style.effectiveFontFamily)!;
  } else {
    cssStyle.fontFamily = canonicalizeFontFamily(style.fontFamily)!;
  }
  if (style.letterSpacing != null) {
    cssStyle.letterSpacing = '${style.letterSpacing}px';
  }
  if (style.wordSpacing != null) {
    cssStyle.wordSpacing = '${style.wordSpacing}px';
  }
  if (style.decoration != null) {
    updateDecoration = true;
  }
  final List<ui.Shadow>? shadows = style.shadows;
  if (shadows != null) {
    cssStyle.textShadow = _shadowListToCss(shadows);
  }

  if (updateDecoration) {
    if (style.decoration != null) {
      final String? textDecoration = _textDecorationToCssString(
        style.decoration,
        style.decorationStyle,
      );
      if (textDecoration != null) {
        if (ui_web.browser.browserEngine == ui_web.BrowserEngine.webkit) {
          setElementStyle(element, '-webkit-text-decoration', textDecoration);
        } else {
          cssStyle.textDecoration = textDecoration;
        }
        final ui.Color? decorationColor = style.decorationColor;
        if (decorationColor != null) {
          cssStyle.textDecorationColor = decorationColor.toCssString();
        }
      }
    }
  }

  final List<ui.FontFeature>? fontFeatures = style.fontFeatures;
  if (fontFeatures != null && fontFeatures.isNotEmpty) {
    cssStyle.fontFeatureSettings = _fontFeatureListToCss(fontFeatures);
  }

  final List<ui.FontVariation>? fontVariations = style.fontVariations;
  if (fontVariations != null && fontVariations.isNotEmpty) {
    cssStyle.setProperty('font-variation-settings', _fontVariationListToCss(fontVariations));
  }
}

String _shadowListToCss(List<ui.Shadow> shadows) {
  if (shadows.isEmpty) {
    return '';
  }
  // CSS text-shadow is a comma separated list of shadows.
  // <offsetx> <offsety> <blur-radius> <color>.
  // Shadows are applied front-to-back with first shadow on top.
  // Color is optional. offsetx,y are required. blur-radius is optional as well
  // and defaults to 0.
  final StringBuffer sb = StringBuffer();
  final int len = shadows.length;
  for (int i = 0; i < len; i++) {
    if (i != 0) {
      sb.write(',');
    }
    final ui.Shadow shadow = shadows[i];
    sb.write(
      '${shadow.offset.dx}px ${shadow.offset.dy}px '
      '${shadow.blurRadius}px ${shadow.color.toCssString()}',
    );
  }
  return sb.toString();
}

String _fontFeatureListToCss(List<ui.FontFeature> fontFeatures) {
  assert(fontFeatures.isNotEmpty);

  // For more details, see:
  // * https://developer.mozilla.org/en-US/docs/Web/CSS/font-feature-settings
  final StringBuffer sb = StringBuffer();
  final int len = fontFeatures.length;
  for (int i = 0; i < len; i++) {
    if (i != 0) {
      sb.write(',');
    }
    final ui.FontFeature fontFeature = fontFeatures[i];
    sb.write('"${fontFeature.feature}" ${fontFeature.value}');
  }
  return sb.toString();
}

String _fontVariationListToCss(List<ui.FontVariation> fontVariations) {
  assert(fontVariations.isNotEmpty);

  final StringBuffer sb = StringBuffer();
  final int len = fontVariations.length;
  for (int i = 0; i < len; i++) {
    if (i != 0) {
      sb.write(',');
    }
    final ui.FontVariation fontVariation = fontVariations[i];
    sb.write('"${fontVariation.axis}" ${fontVariation.value}');
  }
  return sb.toString();
}

/// Converts text decoration style to CSS text-decoration-style value.
String? _textDecorationToCssString(
  ui.TextDecoration? decoration,
  ui.TextDecorationStyle? decorationStyle,
) {
  final StringBuffer decorations = StringBuffer();
  if (decoration != null) {
    if (decoration.contains(ui.TextDecoration.underline)) {
      decorations.write('underline ');
    }
    if (decoration.contains(ui.TextDecoration.overline)) {
      decorations.write('overline ');
    }
    if (decoration.contains(ui.TextDecoration.lineThrough)) {
      decorations.write('line-through ');
    }
  }
  if (decorationStyle != null) {
    decorations.write(_decorationStyleToCssString(decorationStyle));
  }
  return decorations.isEmpty ? null : decorations.toString();
}

String? _decorationStyleToCssString(ui.TextDecorationStyle decorationStyle) {
  switch (decorationStyle) {
    case ui.TextDecorationStyle.dashed:
      return 'dashed';
    case ui.TextDecorationStyle.dotted:
      return 'dotted';
    case ui.TextDecorationStyle.double:
      return 'double';
    case ui.TextDecorationStyle.solid:
      return 'solid';
    case ui.TextDecorationStyle.wavy:
      return 'wavy';
  }
}

/// Converts [align] to its corresponding CSS value.
///
/// This value is used as the "text-align" CSS property, e.g.:
///
/// ```css
/// text-align: right;
/// ```
String textAlignToCssValue(ui.TextAlign? align, ui.TextDirection textDirection) {
  switch (align) {
    case ui.TextAlign.left:
      return 'left';
    case ui.TextAlign.right:
      return 'right';
    case ui.TextAlign.center:
      return 'center';
    case ui.TextAlign.justify:
      return 'justify';
    case ui.TextAlign.end:
      switch (textDirection) {
        case ui.TextDirection.ltr:
          return 'end';
        case ui.TextDirection.rtl:
          return 'left';
      }
    case ui.TextAlign.start:
      switch (textDirection) {
        case ui.TextDirection.ltr:
          return ''; // it's the default
        case ui.TextDirection.rtl:
          return 'right';
      }
    case null:
      // If align is not specified return default.
      return '';
  }
}
