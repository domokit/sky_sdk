// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/cupertino.dart';
library;

import 'dart:collection';
import 'dart:math' as math;
import 'dart:ui' as ui show BoxHeightStyle, BoxWidthStyle, LineMetrics, TextBox;

import 'package:characters/characters.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/semantics.dart';
import 'package:flutter/services.dart';

import 'box.dart';
import 'custom_paint.dart';
import 'layer.dart';
import 'layout_helper.dart';
import 'object.dart';
import 'paragraph.dart';
import 'viewport_offset.dart';

const double _kCaretGap = 1.0; // pixels
const double _kCaretHeightOffset = 2.0; // pixels

// The additional size on the x and y axis with which to expand the prototype
// cursor to render the floating cursor in pixels.
const EdgeInsets _kFloatingCursorSizeIncrease = EdgeInsets.symmetric(
  horizontal: 0.5,
  vertical: 1.0,
);

// The corner radius of the floating cursor in pixels.
const Radius _kFloatingCursorRadius = Radius.circular(1.0);

// This constant represents the shortest squared distance required between the floating cursor
// and the regular cursor when both are present in the text field.
// If the squared distance between the two cursors is less than this value,
// it's not necessary to display both cursors at the same time.
// This behavior is consistent with the one observed in iOS UITextField.
const double _kShortestDistanceSquaredWithFloatingAndRegularCursors = 15.0 * 15.0;

/// Represents the coordinates of the point in a selection, and the text
/// direction at that point, relative to top left of the [RenderEditable] that
/// holds the selection.
@immutable
class TextSelectionPoint {
  /// Creates a description of a point in a text selection.
  const TextSelectionPoint(this.point, this.direction);

  /// Coordinates of the lower left or lower right corner of the selection,
  /// relative to the top left of the [RenderEditable] object.
  final Offset point;

  /// Direction of the text at this edge of the selection.
  final TextDirection? direction;

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is TextSelectionPoint && other.point == point && other.direction == direction;
  }

  @override
  String toString() {
    return switch (direction) {
      TextDirection.ltr => '$point-ltr',
      TextDirection.rtl => '$point-rtl',
      null => '$point',
    };
  }

  @override
  int get hashCode => Object.hash(point, direction);
}

/// The consecutive sequence of [TextPosition]s that the caret should move to
/// when the user navigates the paragraph using the upward arrow key or the
/// downward arrow key.
///
/// {@template flutter.rendering.RenderEditable.verticalArrowKeyMovement}
/// When the user presses the upward arrow key or the downward arrow key, on
/// many platforms (macOS for instance), the caret will move to the previous
/// line or the next line, while maintaining its original horizontal location.
/// When it encounters a shorter line, the caret moves to the closest horizontal
/// location within that line, and restores the original horizontal location
/// when a long enough line is encountered.
///
/// Additionally, the caret will move to the beginning of the document if the
/// upward arrow key is pressed and the caret is already on the first line. If
/// the downward arrow key is pressed next, the caret will restore its original
/// horizontal location and move to the second line. Similarly the caret moves
/// to the end of the document if the downward arrow key is pressed when it's
/// already on the last line.
///
/// Consider a left-aligned paragraph:
///   aa|
///   a
///   aaa
/// where the caret was initially placed at the end of the first line. Pressing
/// the downward arrow key once will move the caret to the end of the second
/// line, and twice the arrow key moves to the third line after the second "a"
/// on that line. Pressing the downward arrow key again, the caret will move to
/// the end of the third line (the end of the document). Pressing the upward
/// arrow key in this state will result in the caret moving to the end of the
/// second line.
///
/// Vertical caret runs are typically interrupted when the layout of the text
/// changes (including when the text itself changes), or when the selection is
/// changed by other input events or programmatically (for example, when the
/// user pressed the left arrow key).
/// {@endtemplate}
///
/// The [movePrevious] method moves the caret location (which is
/// [VerticalCaretMovementRun.current]) to the previous line, and in case
/// the caret is already on the first line, the method does nothing and returns
/// false. Similarly the [moveNext] method moves the caret to the next line, and
/// returns false if the caret is already on the last line.
///
/// The [moveByOffset] method takes a pixel offset from the current position to move
/// the caret up or down.
///
/// If the underlying paragraph's layout changes, [isValid] becomes false and
/// the [VerticalCaretMovementRun] must not be used. The [isValid] property must
/// be checked before calling [movePrevious], [moveNext] and [moveByOffset],
/// or accessing [current].
class VerticalCaretMovementRun implements Iterator<TextPosition> {
  VerticalCaretMovementRun._(
    this._editable,
    this._lineMetrics,
    this._currentTextPosition,
    this._currentLine,
    this._currentOffset,
  );

  Offset _currentOffset;
  int _currentLine;
  TextPosition _currentTextPosition;

  final List<ui.LineMetrics> _lineMetrics;
  final RenderEditable _editable;

  bool _isValid = true;

  /// Whether this [VerticalCaretMovementRun] can still continue.
  ///
  /// A [VerticalCaretMovementRun] run is valid if the underlying text layout
  /// hasn't changed.
  ///
  /// The [current] value and the [movePrevious], [moveNext] and [moveByOffset]
  /// methods must not be accessed when [isValid] is false.
  bool get isValid {
    if (!_isValid) {
      return false;
    }
    final List<ui.LineMetrics> newLineMetrics = _editable._textPainter.computeLineMetrics();
    // Use the implementation detail of the computeLineMetrics method to figure
    // out if the current text layout has been invalidated.
    if (!listEquals(newLineMetrics, _lineMetrics)) {
      _isValid = false;
    }
    return _isValid;
  }

  final Map<int, MapEntry<Offset, TextPosition>> _positionCache =
      <int, MapEntry<Offset, TextPosition>>{};

  MapEntry<Offset, TextPosition> _getTextPositionForLine(int lineNumber) {
    assert(isValid);
    assert(lineNumber >= 0);
    final MapEntry<Offset, TextPosition>? cachedPosition = _positionCache[lineNumber];
    if (cachedPosition != null) {
      return cachedPosition;
    }
    assert(lineNumber != _currentLine);

    final Offset newOffset = Offset(_currentOffset.dx, _lineMetrics[lineNumber].baseline);
    final TextPosition closestPosition = _editable._textPainter.getPositionForOffset(newOffset);
    final MapEntry<Offset, TextPosition> position = MapEntry<Offset, TextPosition>(
      newOffset,
      closestPosition,
    );
    _positionCache[lineNumber] = position;
    return position;
  }

  @override
  TextPosition get current {
    assert(isValid);
    return _currentTextPosition;
  }

  @override
  bool moveNext() {
    assert(isValid);
    if (_currentLine + 1 >= _lineMetrics.length) {
      return false;
    }
    final MapEntry<Offset, TextPosition> position = _getTextPositionForLine(_currentLine + 1);
    _currentLine += 1;
    _currentOffset = position.key;
    _currentTextPosition = position.value;
    return true;
  }

  /// Move back to the previous element.
  ///
  /// Returns true and updates [current] if successful.
  bool movePrevious() {
    assert(isValid);
    if (_currentLine <= 0) {
      return false;
    }
    final MapEntry<Offset, TextPosition> position = _getTextPositionForLine(_currentLine - 1);
    _currentLine -= 1;
    _currentOffset = position.key;
    _currentTextPosition = position.value;
    return true;
  }

  /// Move forward or backward by a number of elements determined
  /// by pixel [offset].
  ///
  /// If [offset] is negative, move backward; otherwise move forward.
  ///
  /// Returns true and updates [current] if successful.
  bool moveByOffset(double offset) {
    final Offset initialOffset = _currentOffset;
    if (offset >= 0.0) {
      while (_currentOffset.dy < initialOffset.dy + offset) {
        if (!moveNext()) {
          break;
        }
      }
    } else {
      while (_currentOffset.dy > initialOffset.dy + offset) {
        if (!movePrevious()) {
          break;
        }
      }
    }
    return initialOffset != _currentOffset;
  }
}

/// Displays some text in a scrollable container with a potentially blinking
/// cursor and with gesture recognizers.
///
/// This is the renderer for an editable text field. It does not directly
/// provide affordances for editing the text, but it does handle text selection
/// and manipulation of the text cursor.
///
/// The [text] is displayed, scrolled by the given [offset], aligned according
/// to [textAlign]. The [maxLines] property controls whether the text displays
/// on one line or many. The [selection], if it is not collapsed, is painted in
/// the [selectionColor]. If it _is_ collapsed, then it represents the cursor
/// position. The cursor is shown while [showCursor] is true. It is painted in
/// the [cursorColor].
///
/// Keyboard handling, IME handling, scrolling, toggling the [showCursor] value
/// to actually blink the cursor, and other features not mentioned above are the
/// responsibility of higher layers and not handled by this object.
class RenderEditable extends RenderBox
    with
        RelayoutWhenSystemFontsChangeMixin,
        ContainerRenderObjectMixin<RenderBox, TextParentData>,
        RenderInlineChildrenContainerDefaults
    implements TextLayoutMetrics {
  /// Creates a render object that implements the visual aspects of a text field.
  ///
  /// The [textAlign] argument defaults to [TextAlign.start].
  ///
  /// If [showCursor] is not specified, then it defaults to hiding the cursor.
  ///
  /// The [maxLines] property can be set to null to remove the restriction on
  /// the number of lines. By default, it is 1, meaning this is a single-line
  /// text field. If it is not null, it must be greater than zero.
  ///
  /// Use [ViewportOffset.zero] for the [offset] if there is no need for
  /// scrolling.
  RenderEditable({
    InlineSpan? text,
    required TextDirection textDirection,
    TextAlign textAlign = TextAlign.start,
    Color? cursorColor,
    Color? backgroundCursorColor,
    ValueNotifier<bool>? showCursor,
    bool? hasFocus,
    required LayerLink startHandleLayerLink,
    required LayerLink endHandleLayerLink,
    int? maxLines = 1,
    int? minLines,
    bool expands = false,
    StrutStyle? strutStyle,
    Color? selectionColor,
    @Deprecated(
      'Use textScaler instead. '
      'Use of textScaleFactor was deprecated in preparation for the upcoming nonlinear text scaling support. '
      'This feature was deprecated after v3.12.0-2.0.pre.',
    )
    double textScaleFactor = 1.0,
    TextScaler textScaler = TextScaler.noScaling,
    TextSelection? selection,
    required ViewportOffset offset,
    this.ignorePointer = false,
    bool readOnly = false,
    bool forceLine = true,
    TextHeightBehavior? textHeightBehavior,
    TextWidthBasis textWidthBasis = TextWidthBasis.parent,
    String obscuringCharacter = '•',
    bool obscureText = false,
    Locale? locale,
    double cursorWidth = 1.0,
    double? cursorHeight,
    Radius? cursorRadius,
    bool paintCursorAboveText = false,
    Offset cursorOffset = Offset.zero,
    double devicePixelRatio = 1.0,
    ui.BoxHeightStyle selectionHeightStyle = ui.BoxHeightStyle.tight,
    ui.BoxWidthStyle selectionWidthStyle = ui.BoxWidthStyle.tight,
    bool? enableInteractiveSelection,
    this.floatingCursorAddedMargin = const EdgeInsets.fromLTRB(4, 4, 4, 5),
    TextRange? promptRectRange,
    Color? promptRectColor,
    Clip clipBehavior = Clip.hardEdge,
    required this.textSelectionDelegate,
    RenderEditablePainter? painter,
    RenderEditablePainter? foregroundPainter,
    List<RenderBox>? children,
  }) : assert(maxLines == null || maxLines > 0),
       assert(minLines == null || minLines > 0),
       assert(
         (maxLines == null) || (minLines == null) || (maxLines >= minLines),
         "minLines can't be greater than maxLines",
       ),
       assert(
         !expands || (maxLines == null && minLines == null),
         'minLines and maxLines must be null when expands is true.',
       ),
       assert(
         identical(textScaler, TextScaler.noScaling) || textScaleFactor == 1.0,
         'textScaleFactor is deprecated and cannot be specified when textScaler is specified.',
       ),
       assert(obscuringCharacter.characters.length == 1),
       assert(cursorWidth >= 0.0),
       assert(cursorHeight == null || cursorHeight >= 0.0),
       _textPainter = TextPainter(
         text: text,
         textAlign: textAlign,
         textDirection: textDirection,
         textScaler:
             textScaler == TextScaler.noScaling ? TextScaler.linear(textScaleFactor) : textScaler,
         locale: locale,
         maxLines: maxLines == 1 ? 1 : null,
         strutStyle: strutStyle,
         textHeightBehavior: textHeightBehavior,
         textWidthBasis: textWidthBasis,
       ),
       _showCursor = showCursor ?? ValueNotifier<bool>(false),
       _maxLines = maxLines,
       _minLines = minLines,
       _expands = expands,
       _selection = selection,
       _offset = offset,
       _cursorWidth = cursorWidth,
       _cursorHeight = cursorHeight,
       _paintCursorOnTop = paintCursorAboveText,
       _enableInteractiveSelection = enableInteractiveSelection,
       _devicePixelRatio = devicePixelRatio,
       _startHandleLayerLink = startHandleLayerLink,
       _endHandleLayerLink = endHandleLayerLink,
       _obscuringCharacter = obscuringCharacter,
       _obscureText = obscureText,
       _readOnly = readOnly,
       _forceLine = forceLine,
       _clipBehavior = clipBehavior,
       _hasFocus = hasFocus ?? false,
       _disposeShowCursor = showCursor == null {
    assert(!_showCursor.value || cursorColor != null);

    _selectionPainter.highlightColor = selectionColor;
    _selectionPainter.highlightedRange = selection;
    _selectionPainter.selectionHeightStyle = selectionHeightStyle;
    _selectionPainter.selectionWidthStyle = selectionWidthStyle;

    _autocorrectHighlightPainter.highlightColor = promptRectColor;
    _autocorrectHighlightPainter.highlightedRange = promptRectRange;

    _caretPainter.caretColor = cursorColor;
    _caretPainter.cursorRadius = cursorRadius;
    _caretPainter.cursorOffset = cursorOffset;
    _caretPainter.backgroundCursorColor = backgroundCursorColor;

    _updateForegroundPainter(foregroundPainter);
    _updatePainter(painter);
    addAll(children);
  }

  /// Child render objects
  _RenderEditableCustomPaint? _foregroundRenderObject;
  _RenderEditableCustomPaint? _backgroundRenderObject;

  @override
  void dispose() {
    _leaderLayerHandler.layer = null;
    _foregroundRenderObject?.dispose();
    _foregroundRenderObject = null;
    _backgroundRenderObject?.dispose();
    _backgroundRenderObject = null;
    _clipRectLayer.layer = null;
    _cachedBuiltInForegroundPainters?.dispose();
    _cachedBuiltInPainters?.dispose();
    _selectionStartInViewport.dispose();
    _selectionEndInViewport.dispose();
    _autocorrectHighlightPainter.dispose();
    _selectionPainter.dispose();
    _caretPainter.dispose();
    _textPainter.dispose();
    _textIntrinsicsCache?.dispose();
    if (_disposeShowCursor) {
      _showCursor.dispose();
      _disposeShowCursor = false;
    }
    super.dispose();
  }

  void _updateForegroundPainter(RenderEditablePainter? newPainter) {
    final _CompositeRenderEditablePainter effectivePainter =
        newPainter == null
            ? _builtInForegroundPainters
            : _CompositeRenderEditablePainter(
              painters: <RenderEditablePainter>[_builtInForegroundPainters, newPainter],
            );

    if (_foregroundRenderObject == null) {
      final _RenderEditableCustomPaint foregroundRenderObject = _RenderEditableCustomPaint(
        painter: effectivePainter,
      );
      adoptChild(foregroundRenderObject);
      _foregroundRenderObject = foregroundRenderObject;
    } else {
      _foregroundRenderObject?.painter = effectivePainter;
    }
    _foregroundPainter = newPainter;
  }

  /// The [RenderEditablePainter] to use for painting above this
  /// [RenderEditable]'s text content.
  ///
  /// The new [RenderEditablePainter] will replace the previously specified
  /// foreground painter, and schedule a repaint if the new painter's
  /// `shouldRepaint` method returns true.
  RenderEditablePainter? get foregroundPainter => _foregroundPainter;
  RenderEditablePainter? _foregroundPainter;
  set foregroundPainter(RenderEditablePainter? newPainter) {
    if (newPainter == _foregroundPainter) {
      return;
    }
    _updateForegroundPainter(newPainter);
  }

  void _updatePainter(RenderEditablePainter? newPainter) {
    final _CompositeRenderEditablePainter effectivePainter =
        newPainter == null
            ? _builtInPainters
            : _CompositeRenderEditablePainter(
              painters: <RenderEditablePainter>[_builtInPainters, newPainter],
            );

    if (_backgroundRenderObject == null) {
      final _RenderEditableCustomPaint backgroundRenderObject = _RenderEditableCustomPaint(
        painter: effectivePainter,
      );
      adoptChild(backgroundRenderObject);
      _backgroundRenderObject = backgroundRenderObject;
    } else {
      _backgroundRenderObject?.painter = effectivePainter;
    }
    _painter = newPainter;
  }

  /// Sets the [RenderEditablePainter] to use for painting beneath this
  /// [RenderEditable]'s text content.
  ///
  /// The new [RenderEditablePainter] will replace the previously specified
  /// painter, and schedule a repaint if the new painter's `shouldRepaint`
  /// method returns true.
  RenderEditablePainter? get painter => _painter;
  RenderEditablePainter? _painter;
  set painter(RenderEditablePainter? newPainter) {
    if (newPainter == _painter) {
      return;
    }
    _updatePainter(newPainter);
  }

  // Caret Painters:
  // A single painter for both the regular caret and the floating cursor.
  late final _CaretPainter _caretPainter = _CaretPainter();

  // Text Highlight painters:
  final _TextHighlightPainter _selectionPainter = _TextHighlightPainter();
  final _TextHighlightPainter _autocorrectHighlightPainter = _TextHighlightPainter();

  _CompositeRenderEditablePainter get _builtInForegroundPainters =>
      _cachedBuiltInForegroundPainters ??= _createBuiltInForegroundPainters();
  _CompositeRenderEditablePainter? _cachedBuiltInForegroundPainters;
  _CompositeRenderEditablePainter _createBuiltInForegroundPainters() {
    return _CompositeRenderEditablePainter(
      painters: <RenderEditablePainter>[if (paintCursorAboveText) _caretPainter],
    );
  }

  _CompositeRenderEditablePainter get _builtInPainters =>
      _cachedBuiltInPainters ??= _createBuiltInPainters();
  _CompositeRenderEditablePainter? _cachedBuiltInPainters;
  _CompositeRenderEditablePainter _createBuiltInPainters() {
    return _CompositeRenderEditablePainter(
      painters: <RenderEditablePainter>[
        _autocorrectHighlightPainter,
        _selectionPainter,
        if (!paintCursorAboveText) _caretPainter,
      ],
    );
  }

  /// Whether the [handleEvent] will propagate pointer events to selection
  /// handlers.
  ///
  /// If this property is true, the [handleEvent] assumes that this renderer
  /// will be notified of input gestures via [handleTapDown], [handleTap],
  /// [handleDoubleTap], and [handleLongPress].
  ///
  /// If there are any gesture recognizers in the text span, the [handleEvent]
  /// will still propagate pointer events to those recognizers.
  ///
  /// The default value of this property is false.
  bool ignorePointer;

  /// {@macro dart.ui.textHeightBehavior}
  TextHeightBehavior? get textHeightBehavior => _textPainter.textHeightBehavior;
  set textHeightBehavior(TextHeightBehavior? value) {
    if (_textPainter.textHeightBehavior == value) {
      return;
    }
    _textPainter.textHeightBehavior = value;
    markNeedsLayout();
  }

  /// {@macro flutter.painting.textPainter.textWidthBasis}
  TextWidthBasis get textWidthBasis => _textPainter.textWidthBasis;
  set textWidthBasis(TextWidthBasis value) {
    if (_textPainter.textWidthBasis == value) {
      return;
    }
    _textPainter.textWidthBasis = value;
    markNeedsLayout();
  }

  /// The pixel ratio of the current device.
  ///
  /// Should be obtained by querying MediaQuery for the devicePixelRatio.
  double get devicePixelRatio => _devicePixelRatio;
  double _devicePixelRatio;
  set devicePixelRatio(double value) {
    if (devicePixelRatio == value) {
      return;
    }
    _devicePixelRatio = value;
    markNeedsLayout();
  }

  /// Character used for obscuring text if [obscureText] is true.
  ///
  /// Must have a length of exactly one.
  String get obscuringCharacter => _obscuringCharacter;
  String _obscuringCharacter;
  set obscuringCharacter(String value) {
    if (_obscuringCharacter == value) {
      return;
    }
    assert(value.characters.length == 1);
    _obscuringCharacter = value;
    markNeedsLayout();
  }

  /// Whether to hide the text being edited (e.g., for passwords).
  bool get obscureText => _obscureText;
  bool _obscureText;
  set obscureText(bool value) {
    if (_obscureText == value) {
      return;
    }
    _obscureText = value;
    _cachedAttributedValue = null;
    markNeedsSemanticsUpdate();
  }

  /// Controls how tall the selection highlight boxes are computed to be.
  ///
  /// See [ui.BoxHeightStyle] for details on available styles.
  ui.BoxHeightStyle get selectionHeightStyle => _selectionPainter.selectionHeightStyle;
  set selectionHeightStyle(ui.BoxHeightStyle value) {
    _selectionPainter.selectionHeightStyle = value;
  }

  /// Controls how wide the selection highlight boxes are computed to be.
  ///
  /// See [ui.BoxWidthStyle] for details on available styles.
  ui.BoxWidthStyle get selectionWidthStyle => _selectionPainter.selectionWidthStyle;
  set selectionWidthStyle(ui.BoxWidthStyle value) {
    _selectionPainter.selectionWidthStyle = value;
  }

  /// The object that controls the text selection, used by this render object
  /// for implementing cut, copy, and paste keyboard shortcuts.
  ///
  /// It will make cut, copy and paste functionality work with the most recently
  /// set [TextSelectionDelegate].
  TextSelectionDelegate textSelectionDelegate;

  /// Track whether position of the start of the selected text is within the viewport.
  ///
  /// For example, if the text contains "Hello World", and the user selects
  /// "Hello", then scrolls so only "World" is visible, this will become false.
  /// If the user scrolls back so that the "H" is visible again, this will
  /// become true.
  ///
  /// This bool indicates whether the text is scrolled so that the handle is
  /// inside the text field viewport, as opposed to whether it is actually
  /// visible on the screen.
  ValueListenable<bool> get selectionStartInViewport => _selectionStartInViewport;
  final ValueNotifier<bool> _selectionStartInViewport = ValueNotifier<bool>(true);

  /// Track whether position of the end of the selected text is within the viewport.
  ///
  /// For example, if the text contains "Hello World", and the user selects
  /// "World", then scrolls so only "Hello" is visible, this will become
  /// 'false'. If the user scrolls back so that the "d" is visible again, this
  /// will become 'true'.
  ///
  /// This bool indicates whether the text is scrolled so that the handle is
  /// inside the text field viewport, as opposed to whether it is actually
  /// visible on the screen.
  ValueListenable<bool> get selectionEndInViewport => _selectionEndInViewport;
  final ValueNotifier<bool> _selectionEndInViewport = ValueNotifier<bool>(true);

  /// Returns the TextPosition above or below the given offset.
  TextPosition _getTextPositionVertical(TextPosition position, double verticalOffset) {
    final Offset caretOffset = _textPainter.getOffsetForCaret(position, _caretPrototype);
    final Offset caretOffsetTranslated = caretOffset.translate(0.0, verticalOffset);
    return _textPainter.getPositionForOffset(caretOffsetTranslated);
  }

  // Start TextLayoutMetrics.

  /// {@macro flutter.services.TextLayoutMetrics.getLineAtOffset}
  @override
  TextSelection getLineAtOffset(TextPosition position) {
    final TextRange line = _textPainter.getLineBoundary(position);
    // If text is obscured, the entire string should be treated as one line.
    if (obscureText) {
      return TextSelection(baseOffset: 0, extentOffset: plainText.length);
    }
    return TextSelection(baseOffset: line.start, extentOffset: line.end);
  }

  /// {@macro flutter.painting.TextPainter.getWordBoundary}
  @override
  TextRange getWordBoundary(TextPosition position) {
    return _textPainter.getWordBoundary(position);
  }

  /// {@macro flutter.services.TextLayoutMetrics.getTextPositionAbove}
  @override
  TextPosition getTextPositionAbove(TextPosition position) {
    // The caret offset gives a location in the upper left hand corner of
    // the caret so the middle of the line above is a half line above that
    // point and the line below is 1.5 lines below that point.
    final double preferredLineHeight = _textPainter.preferredLineHeight;
    final double verticalOffset = -0.5 * preferredLineHeight;
    return _getTextPositionVertical(position, verticalOffset);
  }

  /// {@macro flutter.services.TextLayoutMetrics.getTextPositionBelow}
  @override
  TextPosition getTextPositionBelow(TextPosition position) {
    // The caret offset gives a location in the upper left hand corner of
    // the caret so the middle of the line above is a half line above that
    // point and the line below is 1.5 lines below that point.
    final double preferredLineHeight = _textPainter.preferredLineHeight;
    final double verticalOffset = 1.5 * preferredLineHeight;
    return _getTextPositionVertical(position, verticalOffset);
  }

  // End TextLayoutMetrics.

  void _updateSelectionExtentsVisibility(Offset effectiveOffset) {
    assert(selection != null);
    if (!selection!.isValid) {
      _selectionStartInViewport.value = false;
      _selectionEndInViewport.value = false;
      return;
    }
    final Rect visibleRegion = Offset.zero & size;

    final Offset startOffset = _textPainter.getOffsetForCaret(
      TextPosition(offset: selection!.start, affinity: selection!.affinity),
      _caretPrototype,
    );
    // Check if the selection is visible with an approximation because a
    // difference between rounded and unrounded values causes the caret to be
    // reported as having a slightly (< 0.5) negative y offset. This rounding
    // happens in paragraph.cc's layout and TextPainter's
    // _applyFloatingPointHack. Ideally, the rounding mismatch will be fixed and
    // this can be changed to be a strict check instead of an approximation.
    const double visibleRegionSlop = 0.5;
    _selectionStartInViewport.value = visibleRegion
        .inflate(visibleRegionSlop)
        .contains(startOffset + effectiveOffset);

    final Offset endOffset = _textPainter.getOffsetForCaret(
      TextPosition(offset: selection!.end, affinity: selection!.affinity),
      _caretPrototype,
    );
    _selectionEndInViewport.value = visibleRegion
        .inflate(visibleRegionSlop)
        .contains(endOffset + effectiveOffset);
  }

  void _setTextEditingValue(TextEditingValue newValue, SelectionChangedCause cause) {
    textSelectionDelegate.userUpdateTextEditingValue(newValue, cause);
  }

  void _setSelection(TextSelection nextSelection, SelectionChangedCause cause) {
    if (nextSelection.isValid) {
      // The nextSelection is calculated based on plainText, which can be out
      // of sync with the textSelectionDelegate.textEditingValue by one frame.
      // This is due to the render editable and editable text handle pointer
      // event separately. If the editable text changes the text during the
      // event handler, the render editable will use the outdated text stored in
      // the plainText when handling the pointer event.
      //
      // If this happens, we need to make sure the new selection is still valid.
      final int textLength = textSelectionDelegate.textEditingValue.text.length;
      nextSelection = nextSelection.copyWith(
        baseOffset: math.min(nextSelection.baseOffset, textLength),
        extentOffset: math.min(nextSelection.extentOffset, textLength),
      );
    }
    _setTextEditingValue(
      textSelectionDelegate.textEditingValue.copyWith(selection: nextSelection),
      cause,
    );
  }

  @override
  void markNeedsPaint() {
    super.markNeedsPaint();
    // Tell the painters to repaint since text layout may have changed.
    _foregroundRenderObject?.markNeedsPaint();
    _backgroundRenderObject?.markNeedsPaint();
  }

  @override
  void systemFontsDidChange() {
    super.systemFontsDidChange();
    _textPainter.markNeedsLayout();
  }

  /// Returns a plain text version of the text in [TextPainter].
  ///
  /// If [obscureText] is true, returns the obscured text. See
  /// [obscureText] and [obscuringCharacter].
  /// In order to get the styled text as an [InlineSpan] tree, use [text].
  String get plainText => _textPainter.plainText;

  /// The text to paint in the form of a tree of [InlineSpan]s.
  ///
  /// In order to get the plain text representation, use [plainText].
  InlineSpan? get text => _textPainter.text;
  final TextPainter _textPainter;
  AttributedString? _cachedAttributedValue;
  List<InlineSpanSemanticsInformation>? _cachedCombinedSemanticsInfos;
  set text(InlineSpan? value) {
    if (_textPainter.text == value) {
      return;
    }
    _cachedLineBreakCount = null;
    _textPainter.text = value;
    _cachedAttributedValue = null;
    _cachedCombinedSemanticsInfos = null;
    markNeedsLayout();
    markNeedsSemanticsUpdate();
  }

  TextPainter? _textIntrinsicsCache;
  TextPainter get _textIntrinsics {
    return (_textIntrinsicsCache ??= TextPainter())
      ..text = _textPainter.text
      ..textAlign = _textPainter.textAlign
      ..textDirection = _textPainter.textDirection
      ..textScaler = _textPainter.textScaler
      ..maxLines = _textPainter.maxLines
      ..ellipsis = _textPainter.ellipsis
      ..locale = _textPainter.locale
      ..strutStyle = _textPainter.strutStyle
      ..textWidthBasis = _textPainter.textWidthBasis
      ..textHeightBehavior = _textPainter.textHeightBehavior;
  }

  /// How the text should be aligned horizontally.
  TextAlign get textAlign => _textPainter.textAlign;
  set textAlign(TextAlign value) {
    if (_textPainter.textAlign == value) {
      return;
    }
    _textPainter.textAlign = value;
    markNeedsLayout();
  }

  /// The directionality of the text.
  ///
  /// This decides how the [TextAlign.start], [TextAlign.end], and
  /// [TextAlign.justify] values of [textAlign] are interpreted.
  ///
  /// This is also used to disambiguate how to render bidirectional text. For
  /// example, if the [text] is an English phrase followed by a Hebrew phrase,
  /// in a [TextDirection.ltr] context the English phrase will be on the left
  /// and the Hebrew phrase to its right, while in a [TextDirection.rtl]
  /// context, the English phrase will be on the right and the Hebrew phrase on
  /// its left.
  // TextPainter.textDirection is nullable, but it is set to a
  // non-null value in the RenderEditable constructor and we refuse to
  // set it to null here, so _textPainter.textDirection cannot be null.
  TextDirection get textDirection => _textPainter.textDirection!;
  set textDirection(TextDirection value) {
    if (_textPainter.textDirection == value) {
      return;
    }
    _textPainter.textDirection = value;
    markNeedsLayout();
    markNeedsSemanticsUpdate();
  }

  /// Used by this renderer's internal [TextPainter] to select a locale-specific
  /// font.
  ///
  /// In some cases the same Unicode character may be rendered differently depending
  /// on the locale. For example the '骨' character is rendered differently in
  /// the Chinese and Japanese locales. In these cases the [locale] may be used
  /// to select a locale-specific font.
  ///
  /// If this value is null, a system-dependent algorithm is used to select
  /// the font.
  Locale? get locale => _textPainter.locale;
  set locale(Locale? value) {
    if (_textPainter.locale == value) {
      return;
    }
    _textPainter.locale = value;
    markNeedsLayout();
  }

  /// The [StrutStyle] used by the renderer's internal [TextPainter] to
  /// determine the strut to use.
  StrutStyle? get strutStyle => _textPainter.strutStyle;
  set strutStyle(StrutStyle? value) {
    if (_textPainter.strutStyle == value) {
      return;
    }
    _textPainter.strutStyle = value;
    markNeedsLayout();
  }

  /// The color to use when painting the cursor.
  Color? get cursorColor => _caretPainter.caretColor;
  set cursorColor(Color? value) {
    _caretPainter.caretColor = value;
  }

  /// The color to use when painting the cursor aligned to the text while
  /// rendering the floating cursor.
  ///
  /// Typically this would be set to [CupertinoColors.inactiveGray].
  ///
  /// If this is null, the background cursor is not painted.
  ///
  /// See also:
  ///
  ///  * [FloatingCursorDragState], which explains the floating cursor feature
  ///    in detail.
  Color? get backgroundCursorColor => _caretPainter.backgroundCursorColor;
  set backgroundCursorColor(Color? value) {
    _caretPainter.backgroundCursorColor = value;
  }

  bool _disposeShowCursor;

  /// Whether to paint the cursor.
  ValueNotifier<bool> get showCursor => _showCursor;
  ValueNotifier<bool> _showCursor;
  set showCursor(ValueNotifier<bool> value) {
    if (_showCursor == value) {
      return;
    }
    if (attached) {
      _showCursor.removeListener(_showHideCursor);
    }
    if (_disposeShowCursor) {
      _showCursor.dispose();
      _disposeShowCursor = false;
    }
    _showCursor = value;
    if (attached) {
      _showHideCursor();
      _showCursor.addListener(_showHideCursor);
    }
  }

  void _showHideCursor() {
    _caretPainter.shouldPaint = showCursor.value;
  }

  /// Whether the editable is currently focused.
  bool get hasFocus => _hasFocus;
  bool _hasFocus = false;
  set hasFocus(bool value) {
    if (_hasFocus == value) {
      return;
    }
    _hasFocus = value;
    markNeedsSemanticsUpdate();
  }

  /// Whether this rendering object will take a full line regardless the text width.
  bool get forceLine => _forceLine;
  bool _forceLine = false;
  set forceLine(bool value) {
    if (_forceLine == value) {
      return;
    }
    _forceLine = value;
    markNeedsLayout();
  }

  /// Whether this rendering object is read only.
  bool get readOnly => _readOnly;
  bool _readOnly = false;
  set readOnly(bool value) {
    if (_readOnly == value) {
      return;
    }
    _readOnly = value;
    markNeedsSemanticsUpdate();
  }

  /// The maximum number of lines for the text to span, wrapping if necessary.
  ///
  /// If this is 1 (the default), the text will not wrap, but will extend
  /// indefinitely instead.
  ///
  /// If this is null, there is no limit to the number of lines.
  ///
  /// When this is not null, the intrinsic height of the render object is the
  /// height of one line of text multiplied by this value. In other words, this
  /// also controls the height of the actual editing widget.
  int? get maxLines => _maxLines;
  int? _maxLines;

  /// The value may be null. If it is not null, then it must be greater than zero.
  set maxLines(int? value) {
    assert(value == null || value > 0);
    if (maxLines == value) {
      return;
    }
    _maxLines = value;

    // Special case maxLines == 1 to keep only the first line so we can get the
    // height of the first line in case there are hard line breaks in the text.
    // See the `_preferredHeight` method.
    _textPainter.maxLines = value == 1 ? 1 : null;
    markNeedsLayout();
  }

  /// {@macro flutter.widgets.editableText.minLines}
  int? get minLines => _minLines;
  int? _minLines;

  /// The value may be null. If it is not null, then it must be greater than zero.
  set minLines(int? value) {
    assert(value == null || value > 0);
    if (minLines == value) {
      return;
    }
    _minLines = value;
    markNeedsLayout();
  }

  /// {@macro flutter.widgets.editableText.expands}
  bool get expands => _expands;
  bool _expands;
  set expands(bool value) {
    if (expands == value) {
      return;
    }
    _expands = value;
    markNeedsLayout();
  }

  /// The color to use when painting the selection.
  Color? get selectionColor => _selectionPainter.highlightColor;
  set selectionColor(Color? value) {
    _selectionPainter.highlightColor = value;
  }

  /// Deprecated. Will be removed in a future version of Flutter. Use
  /// [textScaler] instead.
  ///
  /// The number of font pixels for each logical pixel.
  ///
  /// For example, if the text scale factor is 1.5, text will be 50% larger than
  /// the specified font size.
  @Deprecated(
    'Use textScaler instead. '
    'Use of textScaleFactor was deprecated in preparation for the upcoming nonlinear text scaling support. '
    'This feature was deprecated after v3.12.0-2.0.pre.',
  )
  double get textScaleFactor => _textPainter.textScaleFactor;
  @Deprecated(
    'Use textScaler instead. '
    'Use of textScaleFactor was deprecated in preparation for the upcoming nonlinear text scaling support. '
    'This feature was deprecated after v3.12.0-2.0.pre.',
  )
  set textScaleFactor(double value) {
    textScaler = TextScaler.linear(value);
  }

  /// {@macro flutter.painting.textPainter.textScaler}
  TextScaler get textScaler => _textPainter.textScaler;
  set textScaler(TextScaler value) {
    if (_textPainter.textScaler == value) {
      return;
    }
    _textPainter.textScaler = value;
    markNeedsLayout();
  }

  /// The region of text that is selected, if any.
  ///
  /// The caret position is represented by a collapsed selection.
  ///
  /// If [selection] is null, there is no selection and attempts to
  /// manipulate the selection will throw.
  TextSelection? get selection => _selection;
  TextSelection? _selection;
  set selection(TextSelection? value) {
    if (_selection == value) {
      return;
    }
    _selection = value;
    _selectionPainter.highlightedRange = value;
    markNeedsPaint();
    markNeedsSemanticsUpdate();
  }

  /// The offset at which the text should be painted.
  ///
  /// If the text content is larger than the editable line itself, the editable
  /// line clips the text. This property controls which part of the text is
  /// visible by shifting the text by the given offset before clipping.
  ViewportOffset get offset => _offset;
  ViewportOffset _offset;
  set offset(ViewportOffset value) {
    if (_offset == value) {
      return;
    }
    if (attached) {
      _offset.removeListener(markNeedsPaint);
    }
    _offset = value;
    if (attached) {
      _offset.addListener(markNeedsPaint);
    }
    markNeedsLayout();
  }

  /// How thick the cursor will be.
  double get cursorWidth => _cursorWidth;
  double _cursorWidth = 1.0;
  set cursorWidth(double value) {
    if (_cursorWidth == value) {
      return;
    }
    _cursorWidth = value;
    markNeedsLayout();
  }

  /// How tall the cursor will be.
  ///
  /// This can be null, in which case the getter will actually return [preferredLineHeight].
  ///
  /// Setting this to itself fixes the value to the current [preferredLineHeight]. Setting
  /// this to null returns the behavior of deferring to [preferredLineHeight].
  // TODO(ianh): This is a confusing API. We should have a separate getter for the effective cursor height.
  double get cursorHeight => _cursorHeight ?? preferredLineHeight;
  double? _cursorHeight;
  set cursorHeight(double? value) {
    if (_cursorHeight == value) {
      return;
    }
    _cursorHeight = value;
    markNeedsLayout();
  }

  /// {@template flutter.rendering.RenderEditable.paintCursorAboveText}
  /// If the cursor should be painted on top of the text or underneath it.
  ///
  /// By default, the cursor should be painted on top for iOS platforms and
  /// underneath for Android platforms.
  /// {@endtemplate}
  bool get paintCursorAboveText => _paintCursorOnTop;
  bool _paintCursorOnTop;
  set paintCursorAboveText(bool value) {
    if (_paintCursorOnTop == value) {
      return;
    }
    _paintCursorOnTop = value;
    // Clear cached built-in painters and reconfigure painters.
    _cachedBuiltInForegroundPainters = null;
    _cachedBuiltInPainters = null;
    // Call update methods to rebuild and set the effective painters.
    _updateForegroundPainter(_foregroundPainter);
    _updatePainter(_painter);
  }

  /// {@template flutter.rendering.RenderEditable.cursorOffset}
  /// The offset that is used, in pixels, when painting the cursor on screen.
  ///
  /// By default, the cursor position should be set to an offset of
  /// (-[cursorWidth] * 0.5, 0.0) on iOS platforms and (0, 0) on Android
  /// platforms. The origin from where the offset is applied to is the arbitrary
  /// location where the cursor ends up being rendered from by default.
  /// {@endtemplate}
  Offset get cursorOffset => _caretPainter.cursorOffset;
  set cursorOffset(Offset value) {
    _caretPainter.cursorOffset = value;
  }

  /// How rounded the corners of the cursor should be.
  ///
  /// A null value is the same as [Radius.zero].
  Radius? get cursorRadius => _caretPainter.cursorRadius;
  set cursorRadius(Radius? value) {
    _caretPainter.cursorRadius = value;
  }

  /// The [LayerLink] of start selection handle.
  ///
  /// [RenderEditable] is responsible for calculating the [Offset] of this
  /// [LayerLink], which will be used as [CompositedTransformTarget] of start handle.
  LayerLink get startHandleLayerLink => _startHandleLayerLink;
  LayerLink _startHandleLayerLink;
  set startHandleLayerLink(LayerLink value) {
    if (_startHandleLayerLink == value) {
      return;
    }
    _startHandleLayerLink = value;
    markNeedsPaint();
  }

  /// The [LayerLink] of end selection handle.
  ///
  /// [RenderEditable] is responsible for calculating the [Offset] of this
  /// [LayerLink], which will be used as [CompositedTransformTarget] of end handle.
  LayerLink get endHandleLayerLink => _endHandleLayerLink;
  LayerLink _endHandleLayerLink;
  set endHandleLayerLink(LayerLink value) {
    if (_endHandleLayerLink == value) {
      return;
    }
    _endHandleLayerLink = value;
    markNeedsPaint();
  }

  /// The padding applied to text field. Used to determine the bounds when
  /// moving the floating cursor.
  ///
  /// Defaults to a padding with left, top and right set to 4, bottom to 5.
  ///
  /// See also:
  ///
  ///  * [FloatingCursorDragState], which explains the floating cursor feature
  ///    in detail.
  EdgeInsets floatingCursorAddedMargin;

  /// Returns true if the floating cursor is visible, false otherwise.
  bool get floatingCursorOn => _floatingCursorOn;
  bool _floatingCursorOn = false;
  late TextPosition _floatingCursorTextPosition;

  /// Whether to allow the user to change the selection.
  ///
  /// Since [RenderEditable] does not handle selection manipulation
  /// itself, this actually only affects whether the accessibility
  /// hints provided to the system (via
  /// [describeSemanticsConfiguration]) will enable selection
  /// manipulation. It's the responsibility of this object's owner
  /// to provide selection manipulation affordances.
  ///
  /// This field is used by [selectionEnabled] (which then controls
  /// the accessibility hints mentioned above). When null,
  /// [obscureText] is used to determine the value of
  /// [selectionEnabled] instead.
  bool? get enableInteractiveSelection => _enableInteractiveSelection;
  bool? _enableInteractiveSelection;
  set enableInteractiveSelection(bool? value) {
    if (_enableInteractiveSelection == value) {
      return;
    }
    _enableInteractiveSelection = value;
    markNeedsLayout();
    markNeedsSemanticsUpdate();
  }

  /// Whether interactive selection are enabled based on the values of
  /// [enableInteractiveSelection] and [obscureText].
  ///
  /// Since [RenderEditable] does not handle selection manipulation
  /// itself, this actually only affects whether the accessibility
  /// hints provided to the system (via
  /// [describeSemanticsConfiguration]) will enable selection
  /// manipulation. It's the responsibility of this object's owner
  /// to provide selection manipulation affordances.
  ///
  /// By default, [enableInteractiveSelection] is null, [obscureText] is false,
  /// and this getter returns true.
  ///
  /// If [enableInteractiveSelection] is null and [obscureText] is true, then this
  /// getter returns false. This is the common case for password fields.
  ///
  /// If [enableInteractiveSelection] is non-null then its value is
  /// returned. An application might [enableInteractiveSelection] to
  /// true to enable interactive selection for a password field, or to
  /// false to unconditionally disable interactive selection.
  bool get selectionEnabled {
    return enableInteractiveSelection ?? !obscureText;
  }

  /// The color used to paint the prompt rectangle.
  ///
  /// The prompt rectangle will only be requested on non-web iOS applications.
  // TODO(ianh): We should change the getter to return null when _promptRectRange is null
  // (otherwise, if you set it to null and then get it, you get back non-null).
  // Alternatively, we could stop supporting setting this to null.
  Color? get promptRectColor => _autocorrectHighlightPainter.highlightColor;
  set promptRectColor(Color? newValue) {
    _autocorrectHighlightPainter.highlightColor = newValue;
  }

  /// Dismisses the currently displayed prompt rectangle and displays a new prompt rectangle
  /// over [newRange] in the given color [promptRectColor].
  ///
  /// The prompt rectangle will only be requested on non-web iOS applications.
  ///
  /// When set to null, the currently displayed prompt rectangle (if any) will be dismissed.
  // ignore: use_setters_to_change_properties, (API predates enforcing the lint)
  void setPromptRectRange(TextRange? newRange) {
    _autocorrectHighlightPainter.highlightedRange = newRange;
  }

  /// The maximum amount the text is allowed to scroll.
  ///
  /// This value is only valid after layout and can change as additional
  /// text is entered or removed in order to accommodate expanding when
  /// [expands] is set to true.
  double get maxScrollExtent => _maxScrollExtent;
  double _maxScrollExtent = 0;

  double get _caretMargin => _kCaretGap + cursorWidth;

  /// {@macro flutter.material.Material.clipBehavior}
  ///
  /// Defaults to [Clip.hardEdge].
  Clip get clipBehavior => _clipBehavior;
  Clip _clipBehavior = Clip.hardEdge;
  set clipBehavior(Clip value) {
    if (value != _clipBehavior) {
      _clipBehavior = value;
      markNeedsPaint();
      markNeedsSemanticsUpdate();
    }
  }

  /// Collected during [describeSemanticsConfiguration], used by
  /// [assembleSemanticsNode].
  List<InlineSpanSemanticsInformation>? _semanticsInfo;

  // Caches [SemanticsNode]s created during [assembleSemanticsNode] so they
  // can be re-used when [assembleSemanticsNode] is called again. This ensures
  // stable ids for the [SemanticsNode]s of [TextSpan]s across
  // [assembleSemanticsNode] invocations.
  LinkedHashMap<Key, SemanticsNode>? _cachedChildNodes;

  /// Returns a list of rects that bound the given selection, and the text
  /// direction. The text direction is used by the engine to calculate
  /// the closest position to a given point.
  ///
  /// See [TextPainter.getBoxesForSelection] for more details.
  List<TextBox> getBoxesForSelection(TextSelection selection) {
    _computeTextMetricsIfNeeded();
    return _textPainter
        .getBoxesForSelection(selection)
        .map(
          (TextBox textBox) => TextBox.fromLTRBD(
            textBox.left + _paintOffset.dx,
            textBox.top + _paintOffset.dy,
            textBox.right + _paintOffset.dx,
            textBox.bottom + _paintOffset.dy,
            textBox.direction,
          ),
        )
        .toList();
  }

  @override
  void describeSemanticsConfiguration(SemanticsConfiguration config) {
    super.describeSemanticsConfiguration(config);
    _semanticsInfo = _textPainter.text!.getSemanticsInformation();
    // TODO(chunhtai): the macOS does not provide a public API to support text
    // selections across multiple semantics nodes. Remove this platform check
    // once we can support it.
    // https://github.com/flutter/flutter/issues/77957
    if (_semanticsInfo!.any((InlineSpanSemanticsInformation info) => info.recognizer != null) &&
        defaultTargetPlatform != TargetPlatform.macOS) {
      assert(readOnly && !obscureText);
      // For Selectable rich text with recognizer, we need to create a semantics
      // node for each text fragment.
      config
        ..isSemanticBoundary = true
        ..explicitChildNodes = true;
      return;
    }
    if (_cachedAttributedValue == null) {
      if (obscureText) {
        _cachedAttributedValue = AttributedString(obscuringCharacter * plainText.length);
      } else {
        final StringBuffer buffer = StringBuffer();
        int offset = 0;
        final List<StringAttribute> attributes = <StringAttribute>[];
        for (final InlineSpanSemanticsInformation info in _semanticsInfo!) {
          final String label = info.semanticsLabel ?? info.text;
          for (final StringAttribute infoAttribute in info.stringAttributes) {
            final TextRange originalRange = infoAttribute.range;
            attributes.add(
              infoAttribute.copy(
                range: TextRange(
                  start: offset + originalRange.start,
                  end: offset + originalRange.end,
                ),
              ),
            );
          }
          buffer.write(label);
          offset += label.length;
        }
        _cachedAttributedValue = AttributedString(buffer.toString(), attributes: attributes);
      }
    }
    config
      ..attributedValue = _cachedAttributedValue!
      ..isObscured = obscureText
      ..isMultiline = _isMultiline
      ..textDirection = textDirection
      ..isFocused = hasFocus
      ..isTextField = true
      ..isReadOnly = readOnly;

    if (hasFocus && selectionEnabled) {
      config.onSetSelection = _handleSetSelection;
    }

    if (hasFocus && !readOnly) {
      config.onSetText = _handleSetText;
    }

    if (selectionEnabled && (selection?.isValid ?? false)) {
      config.textSelection = selection;
      if (_textPainter.getOffsetBefore(selection!.extentOffset) != null) {
        config
          ..onMoveCursorBackwardByWord = _handleMoveCursorBackwardByWord
          ..onMoveCursorBackwardByCharacter = _handleMoveCursorBackwardByCharacter;
      }
      if (_textPainter.getOffsetAfter(selection!.extentOffset) != null) {
        config
          ..onMoveCursorForwardByWord = _handleMoveCursorForwardByWord
          ..onMoveCursorForwardByCharacter = _handleMoveCursorForwardByCharacter;
      }
    }
  }

  void _handleSetText(String text) {
    textSelectionDelegate.userUpdateTextEditingValue(
      TextEditingValue(text: text, selection: TextSelection.collapsed(offset: text.length)),
      SelectionChangedCause.keyboard,
    );
  }

  @override
  void assembleSemanticsNode(
    SemanticsNode node,
    SemanticsConfiguration config,
    Iterable<SemanticsNode> children,
  ) {
    assert(_semanticsInfo != null && _semanticsInfo!.isNotEmpty);
    final List<SemanticsNode> newChildren = <SemanticsNode>[];
    TextDirection currentDirection = textDirection;
    Rect currentRect;
    double ordinal = 0.0;
    int start = 0;
    int placeholderIndex = 0;
    int childIndex = 0;
    RenderBox? child = firstChild;
    final LinkedHashMap<Key, SemanticsNode> newChildCache = LinkedHashMap<Key, SemanticsNode>();
    _cachedCombinedSemanticsInfos ??= combineSemanticsInfo(_semanticsInfo!);
    for (final InlineSpanSemanticsInformation info in _cachedCombinedSemanticsInfos!) {
      final TextSelection selection = TextSelection(
        baseOffset: start,
        extentOffset: start + info.text.length,
      );
      start += info.text.length;

      if (info.isPlaceholder) {
        // A placeholder span may have 0 to multiple semantics nodes, we need
        // to annotate all of the semantics nodes belong to this span.
        while (children.length > childIndex &&
            children
                .elementAt(childIndex)
                .isTagged(PlaceholderSpanIndexSemanticsTag(placeholderIndex))) {
          final SemanticsNode childNode = children.elementAt(childIndex);
          final TextParentData parentData = child!.parentData! as TextParentData;
          assert(parentData.offset != null);
          newChildren.add(childNode);
          childIndex += 1;
        }
        child = childAfter(child!);
        placeholderIndex += 1;
      } else {
        final TextDirection initialDirection = currentDirection;
        final List<ui.TextBox> rects = _textPainter.getBoxesForSelection(selection);
        if (rects.isEmpty) {
          continue;
        }
        Rect rect = rects.first.toRect();
        currentDirection = rects.first.direction;
        for (final ui.TextBox textBox in rects.skip(1)) {
          rect = rect.expandToInclude(textBox.toRect());
          currentDirection = textBox.direction;
        }
        // Any of the text boxes may have had infinite dimensions.
        // We shouldn't pass infinite dimensions up to the bridges.
        rect = Rect.fromLTWH(
          math.max(0.0, rect.left),
          math.max(0.0, rect.top),
          math.min(rect.width, constraints.maxWidth),
          math.min(rect.height, constraints.maxHeight),
        );
        // Round the current rectangle to make this API testable and add some
        // padding so that the accessibility rects do not overlap with the text.
        currentRect = Rect.fromLTRB(
          rect.left.floorToDouble() - 4.0,
          rect.top.floorToDouble() - 4.0,
          rect.right.ceilToDouble() + 4.0,
          rect.bottom.ceilToDouble() + 4.0,
        );
        final SemanticsConfiguration configuration =
            SemanticsConfiguration()
              ..sortKey = OrdinalSortKey(ordinal++)
              ..textDirection = initialDirection
              ..attributedLabel = AttributedString(
                info.semanticsLabel ?? info.text,
                attributes: info.stringAttributes,
              );
        switch (info.recognizer) {
          case TapGestureRecognizer(onTap: final VoidCallback? handler):
          case DoubleTapGestureRecognizer(onDoubleTap: final VoidCallback? handler):
            if (handler != null) {
              configuration.onTap = handler;
              configuration.isLink = true;
            }
          case LongPressGestureRecognizer(onLongPress: final GestureLongPressCallback? onLongPress):
            if (onLongPress != null) {
              configuration.onLongPress = onLongPress;
            }
          case null:
            break;
          default:
            assert(false, '${info.recognizer.runtimeType} is not supported.');
        }
        if (node.parentPaintClipRect != null) {
          final Rect paintRect = node.parentPaintClipRect!.intersect(currentRect);
          configuration.isHidden = paintRect.isEmpty && !currentRect.isEmpty;
        }
        late final SemanticsNode newChild;
        if (_cachedChildNodes?.isNotEmpty ?? false) {
          newChild = _cachedChildNodes!.remove(_cachedChildNodes!.keys.first)!;
        } else {
          final UniqueKey key = UniqueKey();
          newChild = SemanticsNode(key: key, showOnScreen: _createShowOnScreenFor(key));
        }
        newChild
          ..updateWith(config: configuration)
          ..rect = currentRect;
        newChildCache[newChild.key!] = newChild;
        newChildren.add(newChild);
      }
    }
    _cachedChildNodes = newChildCache;
    node.updateWith(config: config, childrenInInversePaintOrder: newChildren);
  }

  VoidCallback? _createShowOnScreenFor(Key key) {
    return () {
      final SemanticsNode node = _cachedChildNodes![key]!;
      showOnScreen(descendant: this, rect: node.rect);
    };
  }

  // TODO(ianh): in theory, [selection] could become null between when
  // we last called describeSemanticsConfiguration and when the
  // callbacks are invoked, in which case the callbacks will crash...

  void _handleSetSelection(TextSelection selection) {
    _setSelection(selection, SelectionChangedCause.keyboard);
  }

  void _handleMoveCursorForwardByCharacter(bool extendSelection) {
    assert(selection != null);
    final int? extentOffset = _textPainter.getOffsetAfter(selection!.extentOffset);
    if (extentOffset == null) {
      return;
    }
    final int baseOffset = !extendSelection ? extentOffset : selection!.baseOffset;
    _setSelection(
      TextSelection(baseOffset: baseOffset, extentOffset: extentOffset),
      SelectionChangedCause.keyboard,
    );
  }

  void _handleMoveCursorBackwardByCharacter(bool extendSelection) {
    assert(selection != null);
    final int? extentOffset = _textPainter.getOffsetBefore(selection!.extentOffset);
    if (extentOffset == null) {
      return;
    }
    final int baseOffset = !extendSelection ? extentOffset : selection!.baseOffset;
    _setSelection(
      TextSelection(baseOffset: baseOffset, extentOffset: extentOffset),
      SelectionChangedCause.keyboard,
    );
  }

  void _handleMoveCursorForwardByWord(bool extendSelection) {
    assert(selection != null);
    final TextRange currentWord = _textPainter.getWordBoundary(selection!.extent);
    final TextRange? nextWord = _getNextWord(currentWord.end);
    if (nextWord == null) {
      return;
    }
    final int baseOffset = extendSelection ? selection!.baseOffset : nextWord.start;
    _setSelection(
      TextSelection(baseOffset: baseOffset, extentOffset: nextWord.start),
      SelectionChangedCause.keyboard,
    );
  }

  void _handleMoveCursorBackwardByWord(bool extendSelection) {
    assert(selection != null);
    final TextRange currentWord = _textPainter.getWordBoundary(selection!.extent);
    final TextRange? previousWord = _getPreviousWord(currentWord.start - 1);
    if (previousWord == null) {
      return;
    }
    final int baseOffset = extendSelection ? selection!.baseOffset : previousWord.start;
    _setSelection(
      TextSelection(baseOffset: baseOffset, extentOffset: previousWord.start),
      SelectionChangedCause.keyboard,
    );
  }

  TextRange? _getNextWord(int offset) {
    while (true) {
      final TextRange range = _textPainter.getWordBoundary(TextPosition(offset: offset));
      if (!range.isValid || range.isCollapsed) {
        return null;
      }
      if (!_onlyWhitespace(range)) {
        return range;
      }
      offset = range.end;
    }
  }

  TextRange? _getPreviousWord(int offset) {
    while (offset >= 0) {
      final TextRange range = _textPainter.getWordBoundary(TextPosition(offset: offset));
      if (!range.isValid || range.isCollapsed) {
        return null;
      }
      if (!_onlyWhitespace(range)) {
        return range;
      }
      offset = range.start - 1;
    }
    return null;
  }

  // Check if the given text range only contains white space or separator
  // characters.
  //
  // Includes newline characters from ASCII and separators from the
  // [unicode separator category](https://www.compart.com/en/unicode/category/Zs)
  // TODO(zanderso): replace when we expose this ICU information.
  bool _onlyWhitespace(TextRange range) {
    for (int i = range.start; i < range.end; i++) {
      final int codeUnit = text!.codeUnitAt(i)!;
      if (!TextLayoutMetrics.isWhitespace(codeUnit)) {
        return false;
      }
    }
    return true;
  }

  @override
  void attach(PipelineOwner owner) {
    super.attach(owner);
    _foregroundRenderObject?.attach(owner);
    _backgroundRenderObject?.attach(owner);

    _tap =
        TapGestureRecognizer(debugOwner: this)
          ..onTapDown = _handleTapDown
          ..onTap = _handleTap;
    _longPress = LongPressGestureRecognizer(debugOwner: this)..onLongPress = _handleLongPress;
    _offset.addListener(markNeedsPaint);
    _showHideCursor();
    _showCursor.addListener(_showHideCursor);
  }

  @override
  void detach() {
    _tap.dispose();
    _longPress.dispose();
    _offset.removeListener(markNeedsPaint);
    _showCursor.removeListener(_showHideCursor);
    super.detach();
    _foregroundRenderObject?.detach();
    _backgroundRenderObject?.detach();
  }

  @override
  void redepthChildren() {
    final RenderObject? foregroundChild = _foregroundRenderObject;
    final RenderObject? backgroundChild = _backgroundRenderObject;
    if (foregroundChild != null) {
      redepthChild(foregroundChild);
    }
    if (backgroundChild != null) {
      redepthChild(backgroundChild);
    }
    super.redepthChildren();
  }

  @override
  void visitChildren(RenderObjectVisitor visitor) {
    final RenderObject? foregroundChild = _foregroundRenderObject;
    final RenderObject? backgroundChild = _backgroundRenderObject;
    if (foregroundChild != null) {
      visitor(foregroundChild);
    }
    if (backgroundChild != null) {
      visitor(backgroundChild);
    }
    super.visitChildren(visitor);
  }

  bool get _isMultiline => maxLines != 1;

  Axis get _viewportAxis => _isMultiline ? Axis.vertical : Axis.horizontal;

  Offset get _paintOffset => switch (_viewportAxis) {
    Axis.horizontal => Offset(-offset.pixels, 0.0),
    Axis.vertical => Offset(0.0, -offset.pixels),
  };

  double get _viewportExtent {
    assert(hasSize);
    return switch (_viewportAxis) {
      Axis.horizontal => size.width,
      Axis.vertical => size.height,
    };
  }

  double _getMaxScrollExtent(Size contentSize) {
    assert(hasSize);
    return switch (_viewportAxis) {
      Axis.horizontal => math.max(0.0, contentSize.width - size.width),
      Axis.vertical => math.max(0.0, contentSize.height - size.height),
    };
  }

  // We need to check the paint offset here because during animation, the start of
  // the text may position outside the visible region even when the text fits.
  bool get _hasVisualOverflow => _maxScrollExtent > 0 || _paintOffset != Offset.zero;

  /// Returns the local coordinates of the endpoints of the given selection.
  ///
  /// If the selection is collapsed (and therefore occupies a single point), the
  /// returned list is of length one. Otherwise, the selection is not collapsed
  /// and the returned list is of length two. In this case, however, the two
  /// points might actually be co-located (e.g., because of a bidirectional
  /// selection that contains some text but whose ends meet in the middle).
  ///
  /// See also:
  ///
  ///  * [getLocalRectForCaret], which is the equivalent but for
  ///    a [TextPosition] rather than a [TextSelection].
  List<TextSelectionPoint> getEndpointsForSelection(TextSelection selection) {
    _computeTextMetricsIfNeeded();

    final Offset paintOffset = _paintOffset;

    final List<ui.TextBox> boxes =
        selection.isCollapsed
            ? <ui.TextBox>[]
            : _textPainter.getBoxesForSelection(
              selection,
              boxHeightStyle: selectionHeightStyle,
              boxWidthStyle: selectionWidthStyle,
            );
    if (boxes.isEmpty) {
      // TODO(mpcomplete): This doesn't work well at an RTL/LTR boundary.
      final Offset caretOffset = _textPainter.getOffsetForCaret(selection.extent, _caretPrototype);
      final Offset start = Offset(0.0, preferredLineHeight) + caretOffset + paintOffset;
      return <TextSelectionPoint>[TextSelectionPoint(start, null)];
    } else {
      final Offset start =
          Offset(clampDouble(boxes.first.start, 0, _textPainter.size.width), boxes.first.bottom) +
          paintOffset;
      final Offset end =
          Offset(clampDouble(boxes.last.end, 0, _textPainter.size.width), boxes.last.bottom) +
          paintOffset;
      return <TextSelectionPoint>[
        TextSelectionPoint(start, boxes.first.direction),
        TextSelectionPoint(end, boxes.last.direction),
      ];
    }
  }

  /// Returns the smallest [Rect], in the local coordinate system, that covers
  /// the text within the [TextRange] specified.
  ///
  /// This method is used to calculate the approximate position of the IME bar
  /// on iOS.
  ///
  /// Returns null if [TextRange.isValid] is false for the given `range`, or the
  /// given `range` is collapsed.
  Rect? getRectForComposingRange(TextRange range) {
    if (!range.isValid || range.isCollapsed) {
      return null;
    }
    _computeTextMetricsIfNeeded();

    final List<ui.TextBox> boxes = _textPainter.getBoxesForSelection(
      TextSelection(baseOffset: range.start, extentOffset: range.end),
      boxHeightStyle: selectionHeightStyle,
      boxWidthStyle: selectionWidthStyle,
    );

    return boxes
        .fold(
          null,
          (Rect? accum, TextBox incoming) =>
              accum?.expandToInclude(incoming.toRect()) ?? incoming.toRect(),
        )
        ?.shift(_paintOffset);
  }

  /// Returns the position in the text for the given global coordinate.
  ///
  /// See also:
  ///
  ///  * [getLocalRectForCaret], which is the reverse operation, taking
  ///    a [TextPosition] and returning a [Rect].
  ///  * [TextPainter.getPositionForOffset], which is the equivalent method
  ///    for a [TextPainter] object.
  TextPosition getPositionForPoint(Offset globalPosition) {
    _computeTextMetricsIfNeeded();
    return _textPainter.getPositionForOffset(globalToLocal(globalPosition) - _paintOffset);
  }

  /// Returns the [Rect] in local coordinates for the caret at the given text
  /// position.
  ///
  /// See also:
  ///
  ///  * [getPositionForPoint], which is the reverse operation, taking
  ///    an [Offset] in global coordinates and returning a [TextPosition].
  ///  * [getEndpointsForSelection], which is the equivalent but for
  ///    a selection rather than a particular text position.
  ///  * [TextPainter.getOffsetForCaret], the equivalent method for a
  ///    [TextPainter] object.
  Rect getLocalRectForCaret(TextPosition caretPosition) {
    _computeTextMetricsIfNeeded();
    final Rect caretPrototype = _caretPrototype;
    final Offset caretOffset = _textPainter.getOffsetForCaret(caretPosition, caretPrototype);
    Rect caretRect = caretPrototype.shift(caretOffset + cursorOffset);
    final double scrollableWidth = math.max(_textPainter.width + _caretMargin, size.width);

    final double caretX = clampDouble(
      caretRect.left,
      0,
      math.max(scrollableWidth - _caretMargin, 0),
    );
    caretRect = Offset(caretX, caretRect.top) & caretRect.size;

    final double fullHeight = _textPainter.getFullHeightForCaret(caretPosition, caretPrototype);
    switch (defaultTargetPlatform) {
      case TargetPlatform.iOS:
      case TargetPlatform.macOS:
        // Center the caret vertically along the text.
        final double heightDiff = fullHeight - caretRect.height;
        caretRect = Rect.fromLTWH(
          caretRect.left,
          caretRect.top + heightDiff / 2,
          caretRect.width,
          caretRect.height,
        );
      case TargetPlatform.android:
      case TargetPlatform.fuchsia:
      case TargetPlatform.linux:
      case TargetPlatform.windows:
        // Override the height to take the full height of the glyph at the TextPosition
        // when not on iOS. iOS has special handling that creates a taller caret.
        // TODO(garyq): see https://github.com/flutter/flutter/issues/120836.
        final double caretHeight = cursorHeight;
        // Center the caret vertically along the text.
        final double heightDiff = fullHeight - caretHeight;
        caretRect = Rect.fromLTWH(
          caretRect.left,
          caretRect.top - _kCaretHeightOffset + heightDiff / 2,
          caretRect.width,
          caretHeight,
        );
    }

    caretRect = caretRect.shift(_paintOffset);
    return caretRect.shift(_snapToPhysicalPixel(caretRect.topLeft));
  }

  @override
  double computeMinIntrinsicWidth(double height) {
    final List<PlaceholderDimensions> placeholderDimensions = layoutInlineChildren(
      double.infinity,
      (RenderBox child, BoxConstraints constraints) =>
          Size(child.getMinIntrinsicWidth(double.infinity), 0.0),
      ChildLayoutHelper.getDryBaseline,
    );
    final (double minWidth, double maxWidth) = _adjustConstraints();
    return (_textIntrinsics
          ..setPlaceholderDimensions(placeholderDimensions)
          ..layout(minWidth: minWidth, maxWidth: maxWidth))
        .minIntrinsicWidth;
  }

  @override
  double computeMaxIntrinsicWidth(double height) {
    final List<PlaceholderDimensions> placeholderDimensions = layoutInlineChildren(
      double.infinity,
      // Height and baseline is irrelevant as all text will be laid
      // out in a single line. Therefore, using 0.0 as a dummy for the height.
      (RenderBox child, BoxConstraints constraints) =>
          Size(child.getMaxIntrinsicWidth(double.infinity), 0.0),
      ChildLayoutHelper.getDryBaseline,
    );
    final (double minWidth, double maxWidth) = _adjustConstraints();
    return (_textIntrinsics
              ..setPlaceholderDimensions(placeholderDimensions)
              ..layout(minWidth: minWidth, maxWidth: maxWidth))
            .maxIntrinsicWidth +
        _caretMargin;
  }

  /// An estimate of the height of a line in the text. See [TextPainter.preferredLineHeight].
  /// This does not require the layout to be updated.
  double get preferredLineHeight => _textPainter.preferredLineHeight;

  int? _cachedLineBreakCount;
  int _countHardLineBreaks(String text) {
    final int? cachedValue = _cachedLineBreakCount;
    if (cachedValue != null) {
      return cachedValue;
    }
    int count = 0;
    for (int index = 0; index < text.length; index += 1) {
      switch (text.codeUnitAt(index)) {
        case 0x000A: // LF
        case 0x0085: // NEL
        case 0x000B: // VT
        case 0x000C: // FF, treating it as a regular line separator
        case 0x2028: // LS
        case 0x2029: // PS
          count += 1;
      }
    }
    return _cachedLineBreakCount = count;
  }

  double _preferredHeight(double width) {
    final int? maxLines = this.maxLines;
    final int? minLines = this.minLines ?? maxLines;
    final double minHeight = preferredLineHeight * (minLines ?? 0);
    assert(maxLines != 1 || _textIntrinsics.maxLines == 1);

    if (maxLines == null) {
      final double estimatedHeight;
      if (width == double.infinity) {
        estimatedHeight = preferredLineHeight * (_countHardLineBreaks(plainText) + 1);
      } else {
        final (double minWidth, double maxWidth) = _adjustConstraints(maxWidth: width);
        estimatedHeight = (_textIntrinsics..layout(minWidth: minWidth, maxWidth: maxWidth)).height;
      }
      return math.max(estimatedHeight, minHeight);
    }

    // Special case maxLines == 1 since it forces the scrollable direction
    // to be horizontal. Report the real height to prevent the text from being
    // clipped.
    if (maxLines == 1) {
      // The _layoutText call lays out the paragraph using infinite width when
      // maxLines == 1. Also _textPainter.maxLines will be set to 1 so should
      // there be any line breaks only the first line is shown.
      final (double minWidth, double maxWidth) = _adjustConstraints(maxWidth: width);
      return (_textIntrinsics..layout(minWidth: minWidth, maxWidth: maxWidth)).height;
    }
    if (minLines == maxLines) {
      return minHeight;
    }
    final double maxHeight = preferredLineHeight * maxLines;
    final (double minWidth, double maxWidth) = _adjustConstraints(maxWidth: width);
    return clampDouble(
      (_textIntrinsics..layout(minWidth: minWidth, maxWidth: maxWidth)).height,
      minHeight,
      maxHeight,
    );
  }

  @override
  double computeMinIntrinsicHeight(double width) => getMaxIntrinsicHeight(width);

  @override
  double computeMaxIntrinsicHeight(double width) {
    _textIntrinsics.setPlaceholderDimensions(
      layoutInlineChildren(
        width,
        ChildLayoutHelper.dryLayoutChild,
        ChildLayoutHelper.getDryBaseline,
      ),
    );
    return _preferredHeight(width);
  }

  @override
  double computeDistanceToActualBaseline(TextBaseline baseline) {
    _computeTextMetricsIfNeeded();
    return _textPainter.computeDistanceToActualBaseline(baseline);
  }

  @override
  bool hitTestSelf(Offset position) => true;

  @override
  @protected
  bool hitTestChildren(BoxHitTestResult result, {required Offset position}) {
    final Offset effectivePosition = position - _paintOffset;
    final GlyphInfo? glyph = _textPainter.getClosestGlyphForOffset(effectivePosition);
    // The hit-test can't fall through the horizontal gaps between visually
    // adjacent characters on the same line, even with a large letter-spacing or
    // text justification, as graphemeClusterLayoutBounds.width is the advance
    // width to the next character, so there's no gap between their
    // graphemeClusterLayoutBounds rects.
    final InlineSpan? spanHit =
        glyph != null && glyph.graphemeClusterLayoutBounds.contains(effectivePosition)
            ? _textPainter.text!.getSpanForPosition(
              TextPosition(offset: glyph.graphemeClusterCodeUnitRange.start),
            )
            : null;
    switch (spanHit) {
      case final HitTestTarget span:
        result.add(HitTestEntry(span));
        return true;
      case _:
        return hitTestInlineChildren(result, effectivePosition);
    }
  }

  late TapGestureRecognizer _tap;
  late LongPressGestureRecognizer _longPress;

  @override
  void handleEvent(PointerEvent event, BoxHitTestEntry entry) {
    assert(debugHandleEvent(event, entry));
    if (event is PointerDownEvent) {
      assert(!debugNeedsLayout);

      if (!ignorePointer) {
        // Propagates the pointer event to selection handlers.
        _tap.addPointer(event);
        _longPress.addPointer(event);
      }
    }
  }

  Offset? _lastTapDownPosition;
  Offset? _lastSecondaryTapDownPosition;

  /// {@template flutter.rendering.RenderEditable.lastSecondaryTapDownPosition}
  /// The position of the most recent secondary tap down event on this text
  /// input.
  /// {@endtemplate}
  Offset? get lastSecondaryTapDownPosition => _lastSecondaryTapDownPosition;

  /// Tracks the position of a secondary tap event.
  ///
  /// Should be called before attempting to change the selection based on the
  /// position of a secondary tap.
  void handleSecondaryTapDown(TapDownDetails details) {
    _lastTapDownPosition = details.globalPosition;
    _lastSecondaryTapDownPosition = details.globalPosition;
  }

  /// If [ignorePointer] is false (the default) then this method is called by
  /// the internal gesture recognizer's [TapGestureRecognizer.onTapDown]
  /// callback.
  ///
  /// When [ignorePointer] is true, an ancestor widget must respond to tap
  /// down events by calling this method.
  void handleTapDown(TapDownDetails details) {
    _lastTapDownPosition = details.globalPosition;
  }

  void _handleTapDown(TapDownDetails details) {
    assert(!ignorePointer);
    handleTapDown(details);
  }

  /// If [ignorePointer] is false (the default) then this method is called by
  /// the internal gesture recognizer's [TapGestureRecognizer.onTap]
  /// callback.
  ///
  /// When [ignorePointer] is true, an ancestor widget must respond to tap
  /// events by calling this method.
  void handleTap() {
    selectPosition(cause: SelectionChangedCause.tap);
  }

  void _handleTap() {
    assert(!ignorePointer);
    handleTap();
  }

  /// If [ignorePointer] is false (the default) then this method is called by
  /// the internal gesture recognizer's [DoubleTapGestureRecognizer.onDoubleTap]
  /// callback.
  ///
  /// When [ignorePointer] is true, an ancestor widget must respond to double
  /// tap events by calling this method.
  void handleDoubleTap() {
    selectWord(cause: SelectionChangedCause.doubleTap);
  }

  /// If [ignorePointer] is false (the default) then this method is called by
  /// the internal gesture recognizer's [LongPressGestureRecognizer.onLongPress]
  /// callback.
  ///
  /// When [ignorePointer] is true, an ancestor widget must respond to long
  /// press events by calling this method.
  void handleLongPress() {
    selectWord(cause: SelectionChangedCause.longPress);
  }

  void _handleLongPress() {
    assert(!ignorePointer);
    handleLongPress();
  }

  /// Move selection to the location of the last tap down.
  ///
  /// {@template flutter.rendering.RenderEditable.selectPosition}
  /// This method is mainly used to translate user inputs in global positions
  /// into a [TextSelection]. When used in conjunction with a [EditableText],
  /// the selection change is fed back into [TextEditingController.selection].
  ///
  /// If you have a [TextEditingController], it's generally easier to
  /// programmatically manipulate its `value` or `selection` directly.
  /// {@endtemplate}
  void selectPosition({required SelectionChangedCause cause}) {
    selectPositionAt(from: _lastTapDownPosition!, cause: cause);
  }

  /// Select text between the global positions [from] and [to].
  ///
  /// [from] corresponds to the [TextSelection.baseOffset], and [to] corresponds
  /// to the [TextSelection.extentOffset].
  void selectPositionAt({required Offset from, Offset? to, required SelectionChangedCause cause}) {
    _computeTextMetricsIfNeeded();
    final TextPosition fromPosition = _textPainter.getPositionForOffset(
      globalToLocal(from) - _paintOffset,
    );
    final TextPosition? toPosition =
        to == null ? null : _textPainter.getPositionForOffset(globalToLocal(to) - _paintOffset);

    final int baseOffset = fromPosition.offset;
    final int extentOffset = toPosition?.offset ?? fromPosition.offset;

    final TextSelection newSelection = TextSelection(
      baseOffset: baseOffset,
      extentOffset: extentOffset,
      affinity: fromPosition.affinity,
    );

    _setSelection(newSelection, cause);
  }

  /// {@macro flutter.painting.TextPainter.wordBoundaries}
  WordBoundary get wordBoundaries => _textPainter.wordBoundaries;

  /// Select a word around the location of the last tap down.
  ///
  /// {@macro flutter.rendering.RenderEditable.selectPosition}
  void selectWord({required SelectionChangedCause cause}) {
    selectWordsInRange(from: _lastTapDownPosition!, cause: cause);
  }

  /// Selects the set words of a paragraph that intersect a given range of global positions.
  ///
  /// The set of words selected are not strictly bounded by the range of global positions.
  ///
  /// The first and last endpoints of the selection will always be at the
  /// beginning and end of a word respectively.
  ///
  /// {@macro flutter.rendering.RenderEditable.selectPosition}
  void selectWordsInRange({
    required Offset from,
    Offset? to,
    required SelectionChangedCause cause,
  }) {
    _computeTextMetricsIfNeeded();
    final TextPosition fromPosition = _textPainter.getPositionForOffset(
      globalToLocal(from) - _paintOffset,
    );
    final TextSelection fromWord = getWordAtOffset(fromPosition);
    final TextPosition toPosition =
        to == null
            ? fromPosition
            : _textPainter.getPositionForOffset(globalToLocal(to) - _paintOffset);
    final TextSelection toWord =
        toPosition == fromPosition ? fromWord : getWordAtOffset(toPosition);
    final bool isFromWordBeforeToWord = fromWord.start < toWord.end;

    _setSelection(
      TextSelection(
        baseOffset: isFromWordBeforeToWord ? fromWord.base.offset : fromWord.extent.offset,
        extentOffset: isFromWordBeforeToWord ? toWord.extent.offset : toWord.base.offset,
        affinity: fromWord.affinity,
      ),
      cause,
    );
  }

  /// Move the selection to the beginning or end of a word.
  ///
  /// {@macro flutter.rendering.RenderEditable.selectPosition}
  void selectWordEdge({required SelectionChangedCause cause}) {
    _computeTextMetricsIfNeeded();
    assert(_lastTapDownPosition != null);
    final TextPosition position = _textPainter.getPositionForOffset(
      globalToLocal(_lastTapDownPosition!) - _paintOffset,
    );
    final TextRange word = _textPainter.getWordBoundary(position);
    late TextSelection newSelection;
    if (position.offset <= word.start) {
      newSelection = TextSelection.collapsed(offset: word.start);
    } else {
      newSelection = TextSelection.collapsed(offset: word.end, affinity: TextAffinity.upstream);
    }
    _setSelection(newSelection, cause);
  }

  /// Returns a [TextSelection] that encompasses the word at the given
  /// [TextPosition].
  @visibleForTesting
  TextSelection getWordAtOffset(TextPosition position) {
    // When long-pressing past the end of the text, we want a collapsed cursor.
    if (position.offset >= plainText.length) {
      return TextSelection.fromPosition(
        TextPosition(offset: plainText.length, affinity: TextAffinity.upstream),
      );
    }
    // If text is obscured, the entire sentence should be treated as one word.
    if (obscureText) {
      return TextSelection(baseOffset: 0, extentOffset: plainText.length);
    }
    final TextRange word = _textPainter.getWordBoundary(position);
    final int effectiveOffset;
    switch (position.affinity) {
      case TextAffinity.upstream:
        // upstream affinity is effectively -1 in text position.
        effectiveOffset = position.offset - 1;
      case TextAffinity.downstream:
        effectiveOffset = position.offset;
    }
    assert(effectiveOffset >= 0);

    // On iOS, select the previous word if there is a previous word, or select
    // to the end of the next word if there is a next word. Select nothing if
    // there is neither a previous word nor a next word.
    //
    // If the platform is Android and the text is read only, try to select the
    // previous word if there is one; otherwise, select the single whitespace at
    // the position.
    if (effectiveOffset > 0 &&
        TextLayoutMetrics.isWhitespace(plainText.codeUnitAt(effectiveOffset))) {
      final TextRange? previousWord = _getPreviousWord(word.start);
      switch (defaultTargetPlatform) {
        case TargetPlatform.iOS:
          if (previousWord == null) {
            final TextRange? nextWord = _getNextWord(word.start);
            if (nextWord == null) {
              return TextSelection.collapsed(offset: position.offset);
            }
            return TextSelection(baseOffset: position.offset, extentOffset: nextWord.end);
          }
          return TextSelection(baseOffset: previousWord.start, extentOffset: position.offset);
        case TargetPlatform.android:
          if (readOnly) {
            if (previousWord == null) {
              return TextSelection(baseOffset: position.offset, extentOffset: position.offset + 1);
            }
            return TextSelection(baseOffset: previousWord.start, extentOffset: position.offset);
          }
        case TargetPlatform.fuchsia:
        case TargetPlatform.macOS:
        case TargetPlatform.linux:
        case TargetPlatform.windows:
          break;
      }
    }

    return TextSelection(baseOffset: word.start, extentOffset: word.end);
  }

  // Placeholder dimensions representing the sizes of child inline widgets.
  //
  // These need to be cached because the text painter's placeholder dimensions
  // will be overwritten during intrinsic width/height calculations and must be
  // restored to the original values before final layout and painting.
  List<PlaceholderDimensions>? _placeholderDimensions;

  (double minWidth, double maxWidth) _adjustConstraints({
    double minWidth = 0.0,
    double maxWidth = double.infinity,
  }) {
    final double availableMaxWidth = math.max(0.0, maxWidth - _caretMargin);
    final double availableMinWidth = math.min(minWidth, availableMaxWidth);
    return (
      forceLine ? availableMaxWidth : availableMinWidth,
      _isMultiline ? availableMaxWidth : double.infinity,
    );
  }

  // Computes the text metrics if `_textPainter`'s layout information was marked
  // as dirty.
  //
  // This method must be called in `RenderEditable`'s public methods that expose
  // `_textPainter`'s metrics. For instance, `systemFontsDidChange` sets
  // _textPainter._paragraph to null, so accessing _textPainter's metrics
  // immediately after `systemFontsDidChange` without first calling this method
  // may crash.
  //
  // This method is also called in various paint methods (`RenderEditable.paint`
  // as well as its foreground/background painters' `paint`). It's needed
  // because invisible render objects kept in the tree by `KeepAlive` may not
  // get a chance to do layout but can still paint.
  // See https://github.com/flutter/flutter/issues/84896.
  //
  // This method only re-computes layout if the underlying `_textPainter`'s
  // layout cache is invalidated (by calling `TextPainter.markNeedsLayout`), or
  // the constraints used to layout the `_textPainter` is different. See
  // `TextPainter.layout`.
  void _computeTextMetricsIfNeeded() {
    final (double minWidth, double maxWidth) = _adjustConstraints(
      minWidth: constraints.minWidth,
      maxWidth: constraints.maxWidth,
    );
    _textPainter.layout(minWidth: minWidth, maxWidth: maxWidth);
  }

  late Rect _caretPrototype;

  // TODO(LongCatIsLooong): https://github.com/flutter/flutter/issues/120836
  //
  /// On iOS, the cursor is taller than the cursor on Android. The height
  /// of the cursor for iOS is approximate and obtained through an eyeball
  /// comparison.
  void _computeCaretPrototype() {
    switch (defaultTargetPlatform) {
      case TargetPlatform.iOS:
      case TargetPlatform.macOS:
        _caretPrototype = Rect.fromLTWH(0.0, 0.0, cursorWidth, cursorHeight + 2);
      case TargetPlatform.android:
      case TargetPlatform.fuchsia:
      case TargetPlatform.linux:
      case TargetPlatform.windows:
        _caretPrototype = Rect.fromLTWH(
          0.0,
          _kCaretHeightOffset,
          cursorWidth,
          cursorHeight - 2.0 * _kCaretHeightOffset,
        );
    }
  }

  // Computes the offset to apply to the given [sourceOffset] so it perfectly
  // snaps to physical pixels.
  Offset _snapToPhysicalPixel(Offset sourceOffset) {
    final Offset globalOffset = localToGlobal(sourceOffset);
    final double pixelMultiple = 1.0 / _devicePixelRatio;
    return Offset(
      globalOffset.dx.isFinite
          ? (globalOffset.dx / pixelMultiple).round() * pixelMultiple - globalOffset.dx
          : 0,
      globalOffset.dy.isFinite
          ? (globalOffset.dy / pixelMultiple).round() * pixelMultiple - globalOffset.dy
          : 0,
    );
  }

  @override
  @protected
  Size computeDryLayout(covariant BoxConstraints constraints) {
    final (double minWidth, double maxWidth) = _adjustConstraints(
      minWidth: constraints.minWidth,
      maxWidth: constraints.maxWidth,
    );
    _textIntrinsics
      ..setPlaceholderDimensions(
        layoutInlineChildren(
          constraints.maxWidth,
          ChildLayoutHelper.dryLayoutChild,
          ChildLayoutHelper.getDryBaseline,
        ),
      )
      ..layout(minWidth: minWidth, maxWidth: maxWidth);
    final double width =
        forceLine
            ? constraints.maxWidth
            : constraints.constrainWidth(_textIntrinsics.size.width + _caretMargin);
    return Size(width, constraints.constrainHeight(_preferredHeight(constraints.maxWidth)));
  }

  @override
  double computeDryBaseline(covariant BoxConstraints constraints, TextBaseline baseline) {
    final (double minWidth, double maxWidth) = _adjustConstraints(
      minWidth: constraints.minWidth,
      maxWidth: constraints.maxWidth,
    );
    _textIntrinsics
      ..setPlaceholderDimensions(
        layoutInlineChildren(
          constraints.maxWidth,
          ChildLayoutHelper.dryLayoutChild,
          ChildLayoutHelper.getDryBaseline,
        ),
      )
      ..layout(minWidth: minWidth, maxWidth: maxWidth);
    return _textIntrinsics.computeDistanceToActualBaseline(baseline);
  }

  @override
  void performLayout() {
    final BoxConstraints constraints = this.constraints;
    _placeholderDimensions = layoutInlineChildren(
      constraints.maxWidth,
      ChildLayoutHelper.layoutChild,
      ChildLayoutHelper.getBaseline,
    );
    final (double minWidth, double maxWidth) = _adjustConstraints(
      minWidth: constraints.minWidth,
      maxWidth: constraints.maxWidth,
    );
    _textPainter
      ..setPlaceholderDimensions(_placeholderDimensions)
      ..layout(minWidth: minWidth, maxWidth: maxWidth);
    positionInlineChildren(_textPainter.inlinePlaceholderBoxes!);
    _computeCaretPrototype();

    final double width =
        forceLine
            ? constraints.maxWidth
            : constraints.constrainWidth(_textPainter.width + _caretMargin);
    assert(maxLines != 1 || _textPainter.maxLines == 1);
    final double preferredHeight = switch (maxLines) {
      null => math.max(_textPainter.height, preferredLineHeight * (minLines ?? 0)),
      1 => _textPainter.height,
      final int maxLines => clampDouble(
        _textPainter.height,
        preferredLineHeight * (minLines ?? maxLines),
        preferredLineHeight * maxLines,
      ),
    };

    size = Size(width, constraints.constrainHeight(preferredHeight));
    final Size contentSize = Size(_textPainter.width + _caretMargin, _textPainter.height);

    final BoxConstraints painterConstraints = BoxConstraints.tight(contentSize);

    _foregroundRenderObject?.layout(painterConstraints);
    _backgroundRenderObject?.layout(painterConstraints);

    _maxScrollExtent = _getMaxScrollExtent(contentSize);
    offset.applyViewportDimension(_viewportExtent);
    offset.applyContentDimensions(0.0, _maxScrollExtent);
  }

  // The relative origin in relation to the distance the user has theoretically
  // dragged the floating cursor offscreen. This value is used to account for the
  // difference in the rendering position and the raw offset value.
  Offset _relativeOrigin = Offset.zero;
  Offset? _previousOffset;
  bool _shouldResetOrigin = true;
  bool _resetOriginOnLeft = false;
  bool _resetOriginOnRight = false;
  bool _resetOriginOnTop = false;
  bool _resetOriginOnBottom = false;
  double? _resetFloatingCursorAnimationValue;

  static Offset _calculateAdjustedCursorOffset(Offset offset, Rect boundingRects) {
    final double adjustedX = clampDouble(offset.dx, boundingRects.left, boundingRects.right);
    final double adjustedY = clampDouble(offset.dy, boundingRects.top, boundingRects.bottom);
    return Offset(adjustedX, adjustedY);
  }

  /// Returns the position within the text field closest to the raw cursor offset.
  ///
  /// See also:
  ///
  ///  * [FloatingCursorDragState], which explains the floating cursor feature
  ///    in detail.
  Offset calculateBoundedFloatingCursorOffset(Offset rawCursorOffset, {bool? shouldResetOrigin}) {
    Offset deltaPosition = Offset.zero;
    final double topBound = -floatingCursorAddedMargin.top;
    final double bottomBound =
        math.min(size.height, _textPainter.height) -
        preferredLineHeight +
        floatingCursorAddedMargin.bottom;
    final double leftBound = -floatingCursorAddedMargin.left;
    final double rightBound =
        math.min(size.width, _textPainter.width) + floatingCursorAddedMargin.right;
    final Rect boundingRects = Rect.fromLTRB(leftBound, topBound, rightBound, bottomBound);

    if (shouldResetOrigin != null) {
      _shouldResetOrigin = shouldResetOrigin;
    }

    if (!_shouldResetOrigin) {
      return _calculateAdjustedCursorOffset(rawCursorOffset, boundingRects);
    }

    if (_previousOffset != null) {
      deltaPosition = rawCursorOffset - _previousOffset!;
    }

    // If the raw cursor offset has gone off an edge, we want to reset the relative
    // origin of the dragging when the user drags back into the field.
    if (_resetOriginOnLeft && deltaPosition.dx > 0) {
      _relativeOrigin = Offset(rawCursorOffset.dx - boundingRects.left, _relativeOrigin.dy);
      _resetOriginOnLeft = false;
    } else if (_resetOriginOnRight && deltaPosition.dx < 0) {
      _relativeOrigin = Offset(rawCursorOffset.dx - boundingRects.right, _relativeOrigin.dy);
      _resetOriginOnRight = false;
    }
    if (_resetOriginOnTop && deltaPosition.dy > 0) {
      _relativeOrigin = Offset(_relativeOrigin.dx, rawCursorOffset.dy - boundingRects.top);
      _resetOriginOnTop = false;
    } else if (_resetOriginOnBottom && deltaPosition.dy < 0) {
      _relativeOrigin = Offset(_relativeOrigin.dx, rawCursorOffset.dy - boundingRects.bottom);
      _resetOriginOnBottom = false;
    }

    final double currentX = rawCursorOffset.dx - _relativeOrigin.dx;
    final double currentY = rawCursorOffset.dy - _relativeOrigin.dy;
    final Offset adjustedOffset = _calculateAdjustedCursorOffset(
      Offset(currentX, currentY),
      boundingRects,
    );

    if (currentX < boundingRects.left && deltaPosition.dx < 0) {
      _resetOriginOnLeft = true;
    } else if (currentX > boundingRects.right && deltaPosition.dx > 0) {
      _resetOriginOnRight = true;
    }
    if (currentY < boundingRects.top && deltaPosition.dy < 0) {
      _resetOriginOnTop = true;
    } else if (currentY > boundingRects.bottom && deltaPosition.dy > 0) {
      _resetOriginOnBottom = true;
    }

    _previousOffset = rawCursorOffset;

    return adjustedOffset;
  }

  /// Sets the screen position of the floating cursor and the text position
  /// closest to the cursor.
  ///
  /// See also:
  ///
  ///  * [FloatingCursorDragState], which explains the floating cursor feature
  ///    in detail.
  void setFloatingCursor(
    FloatingCursorDragState state,
    Offset boundedOffset,
    TextPosition lastTextPosition, {
    double? resetLerpValue,
  }) {
    if (state == FloatingCursorDragState.End) {
      _relativeOrigin = Offset.zero;
      _previousOffset = null;
      _shouldResetOrigin = true;
      _resetOriginOnBottom = false;
      _resetOriginOnTop = false;
      _resetOriginOnRight = false;
      _resetOriginOnBottom = false;
    }
    _floatingCursorOn = state != FloatingCursorDragState.End;
    _resetFloatingCursorAnimationValue = resetLerpValue;
    if (_floatingCursorOn) {
      _floatingCursorTextPosition = lastTextPosition;
      final double? animationValue = _resetFloatingCursorAnimationValue;
      final EdgeInsets sizeAdjustment =
          animationValue != null
              ? EdgeInsets.lerp(_kFloatingCursorSizeIncrease, EdgeInsets.zero, animationValue)!
              : _kFloatingCursorSizeIncrease;
      _caretPainter.floatingCursorRect = sizeAdjustment
          .inflateRect(_caretPrototype)
          .shift(boundedOffset);
    } else {
      _caretPainter.floatingCursorRect = null;
    }
    _caretPainter.showRegularCaret = _resetFloatingCursorAnimationValue == null;
  }

  MapEntry<int, Offset> _lineNumberFor(TextPosition startPosition, List<ui.LineMetrics> metrics) {
    // TODO(LongCatIsLooong): include line boundaries information in
    // ui.LineMetrics, then we can get rid of this.
    final Offset offset = _textPainter.getOffsetForCaret(startPosition, Rect.zero);
    for (final ui.LineMetrics lineMetrics in metrics) {
      if (lineMetrics.baseline > offset.dy) {
        return MapEntry<int, Offset>(
          lineMetrics.lineNumber,
          Offset(offset.dx, lineMetrics.baseline),
        );
      }
    }
    assert(startPosition.offset == 0, 'unable to find the line for $startPosition');
    return MapEntry<int, Offset>(
      math.max(0, metrics.length - 1),
      Offset(offset.dx, metrics.isNotEmpty ? metrics.last.baseline + metrics.last.descent : 0.0),
    );
  }

  /// Starts a [VerticalCaretMovementRun] at the given location in the text, for
  /// handling consecutive vertical caret movements.
  ///
  /// This can be used to handle consecutive upward/downward arrow key movements
  /// in an input field.
  ///
  /// {@macro flutter.rendering.RenderEditable.verticalArrowKeyMovement}
  ///
  /// The [VerticalCaretMovementRun.isValid] property indicates whether the text
  /// layout has changed and the vertical caret run is invalidated.
  ///
  /// The caller should typically discard a [VerticalCaretMovementRun] when
  /// its [VerticalCaretMovementRun.isValid] becomes false, or on other
  /// occasions where the vertical caret run should be interrupted.
  VerticalCaretMovementRun startVerticalCaretMovement(TextPosition startPosition) {
    final List<ui.LineMetrics> metrics = _textPainter.computeLineMetrics();
    final MapEntry<int, Offset> currentLine = _lineNumberFor(startPosition, metrics);
    return VerticalCaretMovementRun._(
      this,
      metrics,
      startPosition,
      currentLine.key,
      currentLine.value,
    );
  }

  void _paintContents(PaintingContext context, Offset offset) {
    final Offset effectiveOffset = offset + _paintOffset;

    if (selection != null && !_floatingCursorOn) {
      _updateSelectionExtentsVisibility(effectiveOffset);
    }

    final RenderBox? foregroundChild = _foregroundRenderObject;
    final RenderBox? backgroundChild = _backgroundRenderObject;

    // The painters paint in the viewport's coordinate space, since the
    // textPainter's coordinate space is not known to high level widgets.
    if (backgroundChild != null) {
      context.paintChild(backgroundChild, offset);
    }

    _textPainter.paint(context.canvas, effectiveOffset);
    paintInlineChildren(context, effectiveOffset);

    if (foregroundChild != null) {
      context.paintChild(foregroundChild, offset);
    }
  }

  final LayerHandle<LeaderLayer> _leaderLayerHandler = LayerHandle<LeaderLayer>();

  void _paintHandleLayers(
    PaintingContext context,
    List<TextSelectionPoint> endpoints,
    Offset offset,
  ) {
    Offset startPoint = endpoints[0].point;
    startPoint = Offset(
      clampDouble(startPoint.dx, 0.0, size.width),
      clampDouble(startPoint.dy, 0.0, size.height),
    );
    _leaderLayerHandler.layer = LeaderLayer(
      link: startHandleLayerLink,
      offset: startPoint + offset,
    );
    context.pushLayer(_leaderLayerHandler.layer!, super.paint, Offset.zero);
    if (endpoints.length == 2) {
      Offset endPoint = endpoints[1].point;
      endPoint = Offset(
        clampDouble(endPoint.dx, 0.0, size.width),
        clampDouble(endPoint.dy, 0.0, size.height),
      );
      context.pushLayer(
        LeaderLayer(link: endHandleLayerLink, offset: endPoint + offset),
        super.paint,
        Offset.zero,
      );
    } else if (selection!.isCollapsed) {
      context.pushLayer(
        LeaderLayer(link: endHandleLayerLink, offset: startPoint + offset),
        super.paint,
        Offset.zero,
      );
    }
  }

  @override
  void applyPaintTransform(RenderBox child, Matrix4 transform) {
    if (child == _foregroundRenderObject || child == _backgroundRenderObject) {
      return;
    }
    defaultApplyPaintTransform(child, transform);
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    _computeTextMetricsIfNeeded();
    if (_hasVisualOverflow && clipBehavior != Clip.none) {
      _clipRectLayer.layer = context.pushClipRect(
        needsCompositing,
        offset,
        Offset.zero & size,
        _paintContents,
        clipBehavior: clipBehavior,
        oldLayer: _clipRectLayer.layer,
      );
    } else {
      _clipRectLayer.layer = null;
      _paintContents(context, offset);
    }
    final TextSelection? selection = this.selection;
    if (selection != null && selection.isValid) {
      _paintHandleLayers(context, getEndpointsForSelection(selection), offset);
    }
  }

  final LayerHandle<ClipRectLayer> _clipRectLayer = LayerHandle<ClipRectLayer>();

  @override
  Rect? describeApproximatePaintClip(RenderObject child) {
    switch (clipBehavior) {
      case Clip.none:
        return null;
      case Clip.hardEdge:
      case Clip.antiAlias:
      case Clip.antiAliasWithSaveLayer:
        return _hasVisualOverflow ? Offset.zero & size : null;
    }
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(ColorProperty('cursorColor', cursorColor));
    properties.add(DiagnosticsProperty<ValueNotifier<bool>>('showCursor', showCursor));
    properties.add(IntProperty('maxLines', maxLines));
    properties.add(IntProperty('minLines', minLines));
    properties.add(DiagnosticsProperty<bool>('expands', expands, defaultValue: false));
    properties.add(ColorProperty('selectionColor', selectionColor));
    properties.add(
      DiagnosticsProperty<TextScaler>('textScaler', textScaler, defaultValue: TextScaler.noScaling),
    );
    properties.add(DiagnosticsProperty<Locale>('locale', locale, defaultValue: null));
    properties.add(DiagnosticsProperty<TextSelection>('selection', selection));
    properties.add(DiagnosticsProperty<ViewportOffset>('offset', offset));
  }

  @override
  List<DiagnosticsNode> debugDescribeChildren() {
    return <DiagnosticsNode>[
      if (text != null)
        text!.toDiagnosticsNode(name: 'text', style: DiagnosticsTreeStyle.transition),
    ];
  }
}

class _RenderEditableCustomPaint extends RenderBox {
  _RenderEditableCustomPaint({RenderEditablePainter? painter}) : _painter = painter, super();

  @override
  RenderEditable? get parent => super.parent as RenderEditable?;

  @override
  bool get isRepaintBoundary => true;

  @override
  bool get sizedByParent => true;

  RenderEditablePainter? get painter => _painter;
  RenderEditablePainter? _painter;
  set painter(RenderEditablePainter? newValue) {
    if (newValue == painter) {
      return;
    }

    final RenderEditablePainter? oldPainter = painter;
    _painter = newValue;

    if (newValue?.shouldRepaint(oldPainter) ?? true) {
      markNeedsPaint();
    }

    if (attached) {
      oldPainter?.removeListener(markNeedsPaint);
      newValue?.addListener(markNeedsPaint);
    }
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    final RenderEditable? parent = this.parent;
    assert(parent != null);
    final RenderEditablePainter? painter = this.painter;
    if (painter != null && parent != null) {
      parent._computeTextMetricsIfNeeded();
      painter.paint(context.canvas, size, parent);
    }
  }

  @override
  void attach(PipelineOwner owner) {
    super.attach(owner);
    _painter?.addListener(markNeedsPaint);
  }

  @override
  void detach() {
    _painter?.removeListener(markNeedsPaint);
    super.detach();
  }

  @override
  @protected
  Size computeDryLayout(covariant BoxConstraints constraints) => constraints.biggest;
}

/// An interface that paints within a [RenderEditable]'s bounds, above or
/// beneath its text content.
///
/// This painter is typically used for painting auxiliary content that depends
/// on text layout metrics (for instance, for painting carets and text highlight
/// blocks). It can paint independently from its [RenderEditable], allowing it
/// to repaint without triggering a repaint on the entire [RenderEditable] stack
/// when only auxiliary content changes (e.g. a blinking cursor) are present. It
/// will be scheduled to repaint when:
///
///  * It's assigned to a new [RenderEditable] (replacing a prior
///    [RenderEditablePainter]) and the [shouldRepaint] method returns true.
///  * Any of the [RenderEditable]s it is attached to repaints.
///  * The [notifyListeners] method is called, which typically happens when the
///    painter's attributes change.
///
/// See also:
///
///  * [RenderEditable.foregroundPainter], which takes a [RenderEditablePainter]
///    and sets it as the foreground painter of the [RenderEditable].
///  * [RenderEditable.painter], which takes a [RenderEditablePainter]
///    and sets it as the background painter of the [RenderEditable].
///  * [CustomPainter], a similar class which paints within a [RenderCustomPaint].
abstract class RenderEditablePainter extends ChangeNotifier {
  /// Determines whether repaint is needed when a new [RenderEditablePainter]
  /// is provided to a [RenderEditable].
  ///
  /// If the new instance represents different information than the old
  /// instance, then the method should return true, otherwise it should return
  /// false. When [oldDelegate] is null, this method should always return true
  /// unless the new painter initially does not paint anything.
  ///
  /// If the method returns false, then the [paint] call might be optimized
  /// away. However, the [paint] method will get called whenever the
  /// [RenderEditable]s it attaches to repaint, even if [shouldRepaint] returns
  /// false.
  bool shouldRepaint(RenderEditablePainter? oldDelegate);

  /// Paints within the bounds of a [RenderEditable].
  ///
  /// The given [Canvas] has the same coordinate space as the [RenderEditable],
  /// which may be different from the coordinate space the [RenderEditable]'s
  /// [TextPainter] uses, when the text moves inside the [RenderEditable].
  ///
  /// Paint operations performed outside of the region defined by the [canvas]'s
  /// origin and the [size] parameter may get clipped, when [RenderEditable]'s
  /// [RenderEditable.clipBehavior] is not [Clip.none].
  void paint(Canvas canvas, Size size, RenderEditable renderEditable);
}

class _TextHighlightPainter extends RenderEditablePainter {
  _TextHighlightPainter({TextRange? highlightedRange, Color? highlightColor})
    : _highlightedRange = highlightedRange,
      _highlightColor = highlightColor;

  final Paint highlightPaint = Paint();

  Color? get highlightColor => _highlightColor;
  Color? _highlightColor;
  set highlightColor(Color? newValue) {
    if (newValue == _highlightColor) {
      return;
    }
    _highlightColor = newValue;
    notifyListeners();
  }

  TextRange? get highlightedRange => _highlightedRange;
  TextRange? _highlightedRange;
  set highlightedRange(TextRange? newValue) {
    if (newValue == _highlightedRange) {
      return;
    }
    _highlightedRange = newValue;
    notifyListeners();
  }

  /// Controls how tall the selection highlight boxes are computed to be.
  ///
  /// See [ui.BoxHeightStyle] for details on available styles.
  ui.BoxHeightStyle get selectionHeightStyle => _selectionHeightStyle;
  ui.BoxHeightStyle _selectionHeightStyle = ui.BoxHeightStyle.tight;
  set selectionHeightStyle(ui.BoxHeightStyle value) {
    if (_selectionHeightStyle == value) {
      return;
    }
    _selectionHeightStyle = value;
    notifyListeners();
  }

  /// Controls how wide the selection highlight boxes are computed to be.
  ///
  /// See [ui.BoxWidthStyle] for details on available styles.
  ui.BoxWidthStyle get selectionWidthStyle => _selectionWidthStyle;
  ui.BoxWidthStyle _selectionWidthStyle = ui.BoxWidthStyle.tight;
  set selectionWidthStyle(ui.BoxWidthStyle value) {
    if (_selectionWidthStyle == value) {
      return;
    }
    _selectionWidthStyle = value;
    notifyListeners();
  }

  @override
  void paint(Canvas canvas, Size size, RenderEditable renderEditable) {
    final TextRange? range = highlightedRange;
    final Color? color = highlightColor;
    if (range == null || color == null || range.isCollapsed) {
      return;
    }

    highlightPaint.color = color;
    final TextPainter textPainter = renderEditable._textPainter;
    final List<TextBox> boxes = textPainter.getBoxesForSelection(
      TextSelection(baseOffset: range.start, extentOffset: range.end),
      boxHeightStyle: selectionHeightStyle,
      boxWidthStyle: selectionWidthStyle,
    );

    for (final TextBox box in boxes) {
      canvas.drawRect(
        box
            .toRect()
            .shift(renderEditable._paintOffset)
            .intersect(Rect.fromLTWH(0, 0, textPainter.width, textPainter.height)),
        highlightPaint,
      );
    }
  }

  @override
  bool shouldRepaint(RenderEditablePainter? oldDelegate) {
    if (identical(oldDelegate, this)) {
      return false;
    }
    if (oldDelegate == null) {
      return highlightColor != null && highlightedRange != null;
    }
    return oldDelegate is! _TextHighlightPainter ||
        oldDelegate.highlightColor != highlightColor ||
        oldDelegate.highlightedRange != highlightedRange ||
        oldDelegate.selectionHeightStyle != selectionHeightStyle ||
        oldDelegate.selectionWidthStyle != selectionWidthStyle;
  }
}

class _CaretPainter extends RenderEditablePainter {
  _CaretPainter();

  bool get shouldPaint => _shouldPaint;
  bool _shouldPaint = true;
  set shouldPaint(bool value) {
    if (shouldPaint == value) {
      return;
    }
    _shouldPaint = value;
    notifyListeners();
  }

  // This is directly manipulated by the RenderEditable during
  // setFloatingCursor.
  //
  // When changing this value, the caller is responsible for ensuring that
  // listeners are notified.
  bool showRegularCaret = false;

  final Paint caretPaint = Paint();
  late final Paint floatingCursorPaint = Paint();

  Color? get caretColor => _caretColor;
  Color? _caretColor;
  set caretColor(Color? value) {
    if (caretColor?.value == value?.value) {
      return;
    }

    _caretColor = value;
    notifyListeners();
  }

  Radius? get cursorRadius => _cursorRadius;
  Radius? _cursorRadius;
  set cursorRadius(Radius? value) {
    if (_cursorRadius == value) {
      return;
    }
    _cursorRadius = value;
    notifyListeners();
  }

  Offset get cursorOffset => _cursorOffset;
  Offset _cursorOffset = Offset.zero;
  set cursorOffset(Offset value) {
    if (_cursorOffset == value) {
      return;
    }
    _cursorOffset = value;
    notifyListeners();
  }

  Color? get backgroundCursorColor => _backgroundCursorColor;
  Color? _backgroundCursorColor;
  set backgroundCursorColor(Color? value) {
    if (backgroundCursorColor?.value == value?.value) {
      return;
    }

    _backgroundCursorColor = value;
    if (showRegularCaret) {
      notifyListeners();
    }
  }

  Rect? get floatingCursorRect => _floatingCursorRect;
  Rect? _floatingCursorRect;
  set floatingCursorRect(Rect? value) {
    if (_floatingCursorRect == value) {
      return;
    }
    _floatingCursorRect = value;
    notifyListeners();
  }

  void paintRegularCursor(
    Canvas canvas,
    RenderEditable renderEditable,
    Color caretColor,
    TextPosition textPosition,
  ) {
    final Rect integralRect = renderEditable.getLocalRectForCaret(textPosition);
    if (shouldPaint) {
      if (floatingCursorRect != null) {
        final double distanceSquared =
            (floatingCursorRect!.center - integralRect.center).distanceSquared;
        if (distanceSquared < _kShortestDistanceSquaredWithFloatingAndRegularCursors) {
          return;
        }
      }
      final Radius? radius = cursorRadius;
      caretPaint.color = caretColor;
      if (radius == null) {
        canvas.drawRect(integralRect, caretPaint);
      } else {
        final RRect caretRRect = RRect.fromRectAndRadius(integralRect, radius);
        canvas.drawRRect(caretRRect, caretPaint);
      }
    }
  }

  @override
  void paint(Canvas canvas, Size size, RenderEditable renderEditable) {
    // Compute the caret location even when `shouldPaint` is false.

    final TextSelection? selection = renderEditable.selection;

    if (selection == null || !selection.isCollapsed || !selection.isValid) {
      return;
    }

    final Rect? floatingCursorRect = this.floatingCursorRect;

    final Color? caretColor =
        floatingCursorRect == null
            ? this.caretColor
            : showRegularCaret
            ? backgroundCursorColor
            : null;
    final TextPosition caretTextPosition =
        floatingCursorRect == null ? selection.extent : renderEditable._floatingCursorTextPosition;

    if (caretColor != null) {
      paintRegularCursor(canvas, renderEditable, caretColor, caretTextPosition);
    }

    final Color? floatingCursorColor = this.caretColor?.withOpacity(0.75);
    // Floating Cursor.
    if (floatingCursorRect == null || floatingCursorColor == null || !shouldPaint) {
      return;
    }

    canvas.drawRRect(
      RRect.fromRectAndRadius(floatingCursorRect, _kFloatingCursorRadius),
      floatingCursorPaint..color = floatingCursorColor,
    );
  }

  @override
  bool shouldRepaint(RenderEditablePainter? oldDelegate) {
    if (identical(this, oldDelegate)) {
      return false;
    }

    if (oldDelegate == null) {
      return shouldPaint;
    }
    return oldDelegate is! _CaretPainter ||
        oldDelegate.shouldPaint != shouldPaint ||
        oldDelegate.showRegularCaret != showRegularCaret ||
        oldDelegate.caretColor != caretColor ||
        oldDelegate.cursorRadius != cursorRadius ||
        oldDelegate.cursorOffset != cursorOffset ||
        oldDelegate.backgroundCursorColor != backgroundCursorColor ||
        oldDelegate.floatingCursorRect != floatingCursorRect;
  }
}

class _CompositeRenderEditablePainter extends RenderEditablePainter {
  _CompositeRenderEditablePainter({required this.painters});

  final List<RenderEditablePainter> painters;

  @override
  void addListener(VoidCallback listener) {
    for (final RenderEditablePainter painter in painters) {
      painter.addListener(listener);
    }
  }

  @override
  void removeListener(VoidCallback listener) {
    for (final RenderEditablePainter painter in painters) {
      painter.removeListener(listener);
    }
  }

  @override
  void paint(Canvas canvas, Size size, RenderEditable renderEditable) {
    for (final RenderEditablePainter painter in painters) {
      painter.paint(canvas, size, renderEditable);
    }
  }

  @override
  bool shouldRepaint(RenderEditablePainter? oldDelegate) {
    if (identical(oldDelegate, this)) {
      return false;
    }
    if (oldDelegate is! _CompositeRenderEditablePainter ||
        oldDelegate.painters.length != painters.length) {
      return true;
    }

    final Iterator<RenderEditablePainter> oldPainters = oldDelegate.painters.iterator;
    final Iterator<RenderEditablePainter> newPainters = painters.iterator;
    while (oldPainters.moveNext() && newPainters.moveNext()) {
      if (newPainters.current.shouldRepaint(oldPainters.current)) {
        return true;
      }
    }

    return false;
  }
}
