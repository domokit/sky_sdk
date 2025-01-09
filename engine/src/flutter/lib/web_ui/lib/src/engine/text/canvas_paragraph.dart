// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:ui/ui.dart' as ui;

import '../dom.dart';
import '../html/bitmap_canvas.dart';
import '../profiler.dart';
import '../view_embedder/style_manager.dart';
import 'layout_fragmenter.dart';
import 'layout_service.dart';
import 'paint_service.dart';
import 'paragraph.dart';
import 'word_breaker.dart';

final String placeholderChar = String.fromCharCode(0xFFFC);

/// A paragraph made up of a flat list of text spans and placeholders.
///
/// [CanvasParagraph] doesn't use a DOM element to represent the structure of
/// its spans and styles. Instead it uses a flat list of [ParagraphSpan]
/// objects.
class CanvasParagraph implements ui.Paragraph {
  /// This class is created by the engine, and should not be instantiated
  /// or extended directly.
  ///
  /// To create a [CanvasParagraph] object, use a [CanvasParagraphBuilder].
  CanvasParagraph(
    this.spans, {
    required this.paragraphStyle,
    required this.plainText,
    required this.canDrawOnCanvas,
  }) : assert(spans.isNotEmpty);

  /// The flat list of spans that make up this paragraph.
  final List<ParagraphSpan> spans;

  /// General styling information for this paragraph.
  final EngineParagraphStyle paragraphStyle;

  /// The full textual content of the paragraph.
  final String plainText;

  /// Whether this paragraph can be drawn on a bitmap canvas.
  ///
  /// Some text features cannot be rendered into a 2D canvas and must use HTML,
  /// such as font features and text decorations.
  final bool canDrawOnCanvas;

  @override
  double get width => _layoutService.width;

  @override
  double get height => _layoutService.height;

  @override
  double get longestLine => _layoutService.longestLine?.width ?? 0.0;

  @override
  double get minIntrinsicWidth => _layoutService.minIntrinsicWidth;

  @override
  double get maxIntrinsicWidth => _layoutService.maxIntrinsicWidth;

  @override
  double get alphabeticBaseline => _layoutService.alphabeticBaseline;

  @override
  double get ideographicBaseline => _layoutService.ideographicBaseline;

  @override
  bool get didExceedMaxLines => _layoutService.didExceedMaxLines;

  List<ParagraphLine> get lines => _layoutService.lines;

  /// The bounds that contain the text painted inside this paragraph.
  ui.Rect get paintBounds => _layoutService.paintBounds;

  /// Whether this paragraph has been laid out or not.
  bool isLaidOut = false;

  ui.ParagraphConstraints? _lastUsedConstraints;

  late final TextLayoutService _layoutService = TextLayoutService(this);
  late final TextPaintService _paintService = TextPaintService(this);

  @override
  void layout(ui.ParagraphConstraints constraints) {
    if (constraints == _lastUsedConstraints) {
      return;
    }

    late Stopwatch stopwatch;
    if (Profiler.isBenchmarkMode) {
      stopwatch = Stopwatch()..start();
    }
    _layoutService.performLayout(constraints);
    if (Profiler.isBenchmarkMode) {
      stopwatch.stop();
      Profiler.instance.benchmark('text_layout', stopwatch.elapsedMicroseconds.toDouble());
    }

    isLaidOut = true;
    _lastUsedConstraints = constraints;
    _cachedDomElement = null;
  }

  // TODO(mdebbar): Returning true means we always require a bitmap canvas. Revisit
  // this decision once `CanvasParagraph` is fully implemented.
  /// Whether this paragraph is doing arbitrary paint operations that require
  /// a bitmap canvas, and can't be expressed in a DOM canvas.
  bool get hasArbitraryPaint => true;

  /// Paints this paragraph instance on a [canvas] at the given [offset].
  void paint(BitmapCanvas canvas, ui.Offset offset) {
    _paintService.paint(canvas, offset);
  }

  DomElement? _cachedDomElement;

  /// Returns a DOM element that represents the entire paragraph and its
  /// children.
  ///
  /// Generates a new DOM element on every invocation.
  DomElement toDomElement() {
    assert(isLaidOut);
    final DomElement? domElement = _cachedDomElement;
    if (domElement == null) {
      return _cachedDomElement ??= _createDomElement();
    }
    return domElement.cloneNode(true) as DomElement;
  }

  DomElement _createDomElement() {
    final DomElement rootElement = domDocument.createElement('flt-paragraph');

    // 1. Set paragraph-level styles.

    final DomCSSStyleDeclaration cssStyle = rootElement.style;
    cssStyle
      ..position = 'absolute'
      // Prevent the browser from doing any line breaks in the paragraph. We want
      // to have full control of the paragraph layout.
      ..whiteSpace = 'pre';

    // 2. Append all spans to the paragraph.

    for (int i = 0; i < lines.length; i++) {
      final ParagraphLine line = lines[i];
      for (final LayoutFragment fragment in line.fragments) {
        if (fragment.isPlaceholder) {
          continue;
        }

        final String text = fragment.getText(this);
        if (text.isEmpty) {
          continue;
        }

        final DomElement spanElement = domDocument.createElement('flt-span');
        if (fragment.textDirection == ui.TextDirection.rtl) {
          spanElement.setAttribute('dir', 'rtl');
        }
        applyTextStyleToElement(element: spanElement, style: fragment.style);
        _positionSpanElement(spanElement, line, fragment);

        spanElement.appendText(text);
        rootElement.append(spanElement);
      }
    }

    return rootElement;
  }

  @override
  List<ui.TextBox> getBoxesForPlaceholders() {
    return _layoutService.getBoxesForPlaceholders();
  }

  @override
  List<ui.TextBox> getBoxesForRange(
    int start,
    int end, {
    ui.BoxHeightStyle boxHeightStyle = ui.BoxHeightStyle.tight,
    ui.BoxWidthStyle boxWidthStyle = ui.BoxWidthStyle.tight,
  }) {
    return _layoutService.getBoxesForRange(start, end, boxHeightStyle, boxWidthStyle);
  }

  @override
  ui.TextPosition getPositionForOffset(ui.Offset offset) {
    return _layoutService.getPositionForOffset(offset);
  }

  @override
  ui.GlyphInfo? getClosestGlyphInfoForOffset(ui.Offset offset) =>
      _layoutService.getClosestGlyphInfo(offset);

  @override
  ui.GlyphInfo? getGlyphInfoAt(int codeUnitOffset) {
    final int? lineNumber = _findLine(codeUnitOffset, 0, numberOfLines);
    if (lineNumber == null) {
      return null;
    }
    final ParagraphLine line = lines[lineNumber];
    final ui.TextRange? range = line.getCharacterRangeAt(codeUnitOffset);
    if (range == null) {
      return null;
    }
    assert(line.overlapsWith(range.start, range.end));
    for (final LayoutFragment fragment in line.fragments) {
      if (fragment.overlapsWith(range.start, range.end)) {
        // If the grapheme cluster is split into multiple fragments (which really
        // shouldn't happen but currently if they are in different TextSpans they
        // don't combine), use the layout box of the first base character as its
        // layout box has a better chance to be not that far-off.
        final ui.TextBox textBox = fragment.toTextBox(start: range.start, end: range.end);
        return ui.GlyphInfo(textBox.toRect(), range, textBox.direction);
      }
    }
    assert(false, 'This should not be reachable.');
    return null;
  }

  @override
  ui.TextRange getWordBoundary(ui.TextPosition position) {
    final int characterPosition;
    switch (position.affinity) {
      case ui.TextAffinity.upstream:
        characterPosition = position.offset - 1;
      case ui.TextAffinity.downstream:
        characterPosition = position.offset;
    }
    final int start = WordBreaker.prevBreakIndex(plainText, characterPosition + 1);
    final int end = WordBreaker.nextBreakIndex(plainText, characterPosition);
    return ui.TextRange(start: start, end: end);
  }

  @override
  ui.TextRange getLineBoundary(ui.TextPosition position) {
    if (lines.isEmpty) {
      return ui.TextRange.empty;
    }
    final int? lineNumber = getLineNumberAt(position.offset);
    // Fallback to the last line for backward compatibility.
    final ParagraphLine line = lineNumber != null ? lines[lineNumber] : lines.last;
    return ui.TextRange(start: line.startIndex, end: line.endIndex - line.trailingNewlines);
  }

  @override
  List<EngineLineMetrics> computeLineMetrics() {
    return lines.map((ParagraphLine line) => line.lineMetrics).toList();
  }

  @override
  EngineLineMetrics? getLineMetricsAt(int lineNumber) {
    return 0 <= lineNumber && lineNumber < lines.length ? lines[lineNumber].lineMetrics : null;
  }

  @override
  int get numberOfLines => lines.length;

  @override
  int? getLineNumberAt(int codeUnitOffset) => _findLine(codeUnitOffset, 0, lines.length);

  int? _findLine(int codeUnitOffset, int startLine, int endLine) {
    assert(endLine <= lines.length);
    final bool isOutOfBounds =
        endLine <= startLine ||
        codeUnitOffset < lines[startLine].startIndex ||
        (endLine < numberOfLines && lines[endLine].startIndex <= codeUnitOffset);
    if (isOutOfBounds) {
      return null;
    }

    if (endLine == startLine + 1) {
      assert(lines[startLine].startIndex <= codeUnitOffset);
      assert(endLine == numberOfLines || codeUnitOffset < lines[endLine].startIndex);
      return codeUnitOffset >= lines[startLine].visibleEndIndex ? null : startLine;
    }
    // endLine >= startLine + 2 thus we have
    // startLine + 1 <= midIndex <= endLine - 1
    final int midIndex = (startLine + endLine) ~/ 2;
    return _findLine(codeUnitOffset, midIndex, endLine) ??
        _findLine(codeUnitOffset, startLine, midIndex);
  }

  bool _disposed = false;

  @override
  void dispose() {
    // TODO(dnfield): It should be possible to clear resources here, but would
    // need refcounting done on any surfaces/pictures holding references to this
    // object.
    _disposed = true;
  }

  @override
  bool get debugDisposed {
    bool? result;
    assert(() {
      result = _disposed;
      return true;
    }());

    if (result != null) {
      return result!;
    }

    throw StateError('Paragraph.debugDisposed is only avialalbe when asserts are enabled.');
  }
}

void _positionSpanElement(DomElement element, ParagraphLine line, LayoutFragment fragment) {
  final ui.Rect boxRect = fragment.toPaintingTextBox().toRect();
  element.style
    ..position = 'absolute'
    ..top = '${boxRect.top}px'
    ..left = '${boxRect.left}px'
    // This is needed for space-only spans that are used to justify the paragraph.
    ..width = '${boxRect.width}px'
    // Makes sure the baseline of each span is positioned as expected.
    ..lineHeight = '${boxRect.height}px';
}

/// Represents a span in the paragraph.
///
/// Instead of keeping spans and styles in a tree hierarchy like the framework
/// does, we flatten the structure and resolve/merge all the styles from parent
/// nodes.
///
/// These spans are stored as a flat list in the paragraph object.
class ParagraphSpan {
  /// Creates a [ParagraphSpan] with the given [style], representing the span of
  /// text in the range between [start] and [end].
  ParagraphSpan({required this.style, required this.start, required this.end});

  /// The resolved style of the span.
  final EngineTextStyle style;

  /// The index of the beginning of the range of text represented by this span.
  final int start;

  /// The index of the end of the range of text represented by this span.
  final int end;
}

class PlaceholderSpan extends ParagraphPlaceholder implements ParagraphSpan {
  PlaceholderSpan(
    this.style,
    this.start,
    this.end,
    double width,
    double height,
    ui.PlaceholderAlignment alignment, {
    required double baselineOffset,
    required ui.TextBaseline baseline,
  }) : super(width, height, alignment, baselineOffset: baselineOffset, baseline: baseline);

  @override
  final EngineTextStyle style;

  @override
  final int start;

  @override
  final int end;
}

/// Represents a node in the tree of text styles pushed to [ui.ParagraphBuilder].
///
/// The [ui.ParagraphBuilder.pushText] and [ui.ParagraphBuilder.pop] operations
/// represent the entire tree of styles in the paragraph. In our implementation,
/// we don't need to keep the entire tree structure in memory. At any point in
/// time, we only need a stack of nodes that represent the current branch in the
/// tree. The items in the stack are [StyleNode] objects.
abstract class StyleNode {
  /// Create a child for this style node.
  ///
  /// We are not creating a tree structure, hence there's no need to keep track
  /// of the children.
  ChildStyleNode createChild(EngineTextStyle style) {
    return ChildStyleNode(parent: this, style: style);
  }

  EngineTextStyle? _cachedStyle;

  /// Generates the final text style to be applied to the text span.
  ///
  /// The resolved text style is equivalent to the entire ascendent chain of
  /// parent style nodes.
  EngineTextStyle resolveStyle() {
    return _cachedStyle ??= EngineTextStyle(
      color: _color,
      decoration: _decoration,
      decorationColor: _decorationColor,
      decorationStyle: _decorationStyle,
      decorationThickness: _decorationThickness,
      fontWeight: _fontWeight,
      fontStyle: _fontStyle,
      textBaseline: _textBaseline,
      fontFamily: _fontFamily,
      fontFamilyFallback: _fontFamilyFallback,
      fontFeatures: _fontFeatures,
      fontVariations: _fontVariations,
      fontSize: _fontSize,
      letterSpacing: _letterSpacing,
      wordSpacing: _wordSpacing,
      height: _height,
      leadingDistribution: _leadingDistribution,
      locale: _locale,
      background: _background,
      foreground: _foreground,
      shadows: _shadows,
    );
  }

  ui.Color? get _color;
  ui.TextDecoration? get _decoration;
  ui.Color? get _decorationColor;
  ui.TextDecorationStyle? get _decorationStyle;
  double? get _decorationThickness;
  ui.FontWeight? get _fontWeight;
  ui.FontStyle? get _fontStyle;
  ui.TextBaseline? get _textBaseline;
  String get _fontFamily;
  List<String>? get _fontFamilyFallback;
  List<ui.FontFeature>? get _fontFeatures;
  List<ui.FontVariation>? get _fontVariations;
  double get _fontSize;
  double? get _letterSpacing;
  double? get _wordSpacing;
  double? get _height;
  ui.TextLeadingDistribution? get _leadingDistribution;
  ui.Locale? get _locale;
  ui.Paint? get _background;
  ui.Paint? get _foreground;
  List<ui.Shadow>? get _shadows;
}

/// Represents a non-root [StyleNode].
class ChildStyleNode extends StyleNode {
  /// Creates a [ChildStyleNode] with the given [parent] and [style].
  ChildStyleNode({required this.parent, required this.style});

  /// The parent node to be used when resolving text styles.
  final StyleNode parent;

  /// The text style associated with the current node.
  final EngineTextStyle style;

  // Read these properties from the TextStyle associated with this node. If the
  // property isn't defined, go to the parent node.

  @override
  ui.Color? get _color => style.color ?? (_foreground == null ? parent._color : null);

  @override
  ui.TextDecoration? get _decoration => style.decoration ?? parent._decoration;

  @override
  ui.Color? get _decorationColor => style.decorationColor ?? parent._decorationColor;

  @override
  ui.TextDecorationStyle? get _decorationStyle => style.decorationStyle ?? parent._decorationStyle;

  @override
  double? get _decorationThickness => style.decorationThickness ?? parent._decorationThickness;

  @override
  ui.FontWeight? get _fontWeight => style.fontWeight ?? parent._fontWeight;

  @override
  ui.FontStyle? get _fontStyle => style.fontStyle ?? parent._fontStyle;

  @override
  ui.TextBaseline? get _textBaseline => style.textBaseline ?? parent._textBaseline;

  @override
  List<String>? get _fontFamilyFallback => style.fontFamilyFallback ?? parent._fontFamilyFallback;

  @override
  List<ui.FontFeature>? get _fontFeatures => style.fontFeatures ?? parent._fontFeatures;

  @override
  List<ui.FontVariation>? get _fontVariations => style.fontVariations ?? parent._fontVariations;

  @override
  double get _fontSize => style.fontSize ?? parent._fontSize;

  @override
  double? get _letterSpacing => style.letterSpacing ?? parent._letterSpacing;

  @override
  double? get _wordSpacing => style.wordSpacing ?? parent._wordSpacing;

  @override
  double? get _height {
    return style.height == ui.kTextHeightNone ? null : (style.height ?? parent._height);
  }

  @override
  ui.TextLeadingDistribution? get _leadingDistribution =>
      style.leadingDistribution ?? parent._leadingDistribution;

  @override
  ui.Locale? get _locale => style.locale ?? parent._locale;

  @override
  ui.Paint? get _background => style.background ?? parent._background;

  @override
  ui.Paint? get _foreground => style.foreground ?? parent._foreground;

  @override
  List<ui.Shadow>? get _shadows => style.shadows ?? parent._shadows;

  // Font family is slightly different from the other properties above. It's
  // never null on the TextStyle object, so we use `isFontFamilyProvided` to
  // check if font family is defined or not.
  @override
  String get _fontFamily => style.isFontFamilyProvided ? style.fontFamily : parent._fontFamily;
}

/// The root style node for the paragraph.
///
/// The style of the root is derived from a [ui.ParagraphStyle] and is the root
/// style for all spans in the paragraph.
class RootStyleNode extends StyleNode {
  /// Creates a [RootStyleNode] from [paragraphStyle].
  RootStyleNode(this.paragraphStyle);

  /// The style of the paragraph being built.
  final EngineParagraphStyle paragraphStyle;

  @override
  ui.Color? get _color => null;

  @override
  ui.TextDecoration? get _decoration => null;

  @override
  ui.Color? get _decorationColor => null;

  @override
  ui.TextDecorationStyle? get _decorationStyle => null;

  @override
  double? get _decorationThickness => null;

  @override
  ui.FontWeight? get _fontWeight => paragraphStyle.fontWeight;
  @override
  ui.FontStyle? get _fontStyle => paragraphStyle.fontStyle;

  @override
  ui.TextBaseline? get _textBaseline => null;

  @override
  String get _fontFamily => paragraphStyle.fontFamily ?? StyleManager.defaultFontFamily;

  @override
  List<String>? get _fontFamilyFallback => null;

  @override
  List<ui.FontFeature>? get _fontFeatures => null;

  @override
  List<ui.FontVariation>? get _fontVariations => null;

  @override
  double get _fontSize => paragraphStyle.fontSize ?? StyleManager.defaultFontSize;

  @override
  double? get _letterSpacing => null;

  @override
  double? get _wordSpacing => null;

  @override
  double? get _height => paragraphStyle.height;

  @override
  ui.TextLeadingDistribution? get _leadingDistribution => null;

  @override
  ui.Locale? get _locale => paragraphStyle.locale;

  @override
  ui.Paint? get _background => null;

  @override
  ui.Paint? get _foreground => null;

  @override
  List<ui.Shadow>? get _shadows => null;
}

/// Builds a [CanvasParagraph] containing text with the given styling
/// information.
class CanvasParagraphBuilder implements ui.ParagraphBuilder {
  /// Creates a [CanvasParagraphBuilder] object, which is used to create a
  /// [CanvasParagraph].
  CanvasParagraphBuilder(EngineParagraphStyle style)
    : _paragraphStyle = style,
      _rootStyleNode = RootStyleNode(style);

  final StringBuffer _plainTextBuffer = StringBuffer();
  final EngineParagraphStyle _paragraphStyle;

  final List<ParagraphSpan> _spans = <ParagraphSpan>[];
  final List<StyleNode> _styleStack = <StyleNode>[];

  final RootStyleNode _rootStyleNode;
  StyleNode get _currentStyleNode =>
      _styleStack.isEmpty ? _rootStyleNode : _styleStack[_styleStack.length - 1];

  @override
  int get placeholderCount => _placeholderCount;
  int _placeholderCount = 0;

  @override
  List<double> get placeholderScales => _placeholderScales;
  final List<double> _placeholderScales = <double>[];

  @override
  void addPlaceholder(
    double width,
    double height,
    ui.PlaceholderAlignment alignment, {
    double scale = 1.0,
    double? baselineOffset,
    ui.TextBaseline? baseline,
  }) {
    // Require a baseline to be specified if using a baseline-based alignment.
    assert(
      !(alignment == ui.PlaceholderAlignment.aboveBaseline ||
              alignment == ui.PlaceholderAlignment.belowBaseline ||
              alignment == ui.PlaceholderAlignment.baseline) ||
          baseline != null,
    );

    final int start = _plainTextBuffer.length;
    _plainTextBuffer.write(placeholderChar);
    final int end = _plainTextBuffer.length;

    final EngineTextStyle style = _currentStyleNode.resolveStyle();
    _updateCanDrawOnCanvas(style);

    _placeholderCount++;
    _placeholderScales.add(scale);
    _spans.add(
      PlaceholderSpan(
        style,
        start,
        end,
        width * scale,
        height * scale,
        alignment,
        baselineOffset: (baselineOffset ?? height) * scale,
        baseline: baseline ?? ui.TextBaseline.alphabetic,
      ),
    );
  }

  @override
  void pushStyle(ui.TextStyle style) {
    _styleStack.add(_currentStyleNode.createChild(style as EngineTextStyle));
  }

  @override
  void pop() {
    if (_styleStack.isNotEmpty) {
      _styleStack.removeLast();
    }
  }

  bool _canDrawOnCanvas = true;

  @override
  void addText(String text) {
    final int start = _plainTextBuffer.length;
    _plainTextBuffer.write(text);
    final int end = _plainTextBuffer.length;

    final EngineTextStyle style = _currentStyleNode.resolveStyle();
    _updateCanDrawOnCanvas(style);

    _spans.add(ParagraphSpan(style: style, start: start, end: end));
  }

  void _updateCanDrawOnCanvas(EngineTextStyle style) {
    if (!_canDrawOnCanvas) {
      return;
    }

    final double? letterSpacing = style.letterSpacing;
    if (letterSpacing != null && letterSpacing != 0.0) {
      _canDrawOnCanvas = false;
      return;
    }

    final ui.TextDecoration? decoration = style.decoration;
    if (decoration != null && decoration != ui.TextDecoration.none) {
      _canDrawOnCanvas = false;
      return;
    }

    final List<ui.FontFeature>? fontFeatures = style.fontFeatures;
    if (fontFeatures != null && fontFeatures.isNotEmpty) {
      _canDrawOnCanvas = false;
      return;
    }

    final List<ui.FontVariation>? fontVariations = style.fontVariations;
    if (fontVariations != null && fontVariations.isNotEmpty) {
      _canDrawOnCanvas = false;
      return;
    }
  }

  @override
  CanvasParagraph build() {
    if (_spans.isEmpty) {
      // In case `addText` and `addPlaceholder` were never called.
      //
      // We want the paragraph to always have a non-empty list of spans to match
      // the expectations of the [LayoutFragmenter].
      _spans.add(ParagraphSpan(style: _rootStyleNode.resolveStyle(), start: 0, end: 0));
    }

    return CanvasParagraph(
      _spans,
      paragraphStyle: _paragraphStyle,
      plainText: _plainTextBuffer.toString(),
      canDrawOnCanvas: _canDrawOnCanvas,
    );
  }
}
