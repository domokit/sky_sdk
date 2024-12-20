// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'color_scheme.dart';
/// @docImport 'scaffold.dart';
/// @docImport 'selection_area.dart';
/// @docImport 'text_field.dart';
library;

import 'dart:ui' as ui show BoxHeightStyle, BoxWidthStyle;

import 'package:flutter/cupertino.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/scheduler.dart';

import 'adaptive_text_selection_toolbar.dart';
import 'desktop_text_selection.dart';
import 'magnifier.dart';
import 'text_selection.dart';
import 'theme.dart';

// Examples can assume:
// late BuildContext context;
// late FocusNode myFocusNode;

/// An eyeballed value that moves the cursor slightly left of where it is
/// rendered for text on Android so its positioning more accurately matches the
/// native iOS text cursor positioning.
///
/// This value is in device pixels, not logical pixels as is typically used
/// throughout the codebase.
const int iOSHorizontalOffset = -2;

class _TextSpanEditingController extends TextEditingController {
  _TextSpanEditingController({required TextSpan textSpan})
    : _textSpan = textSpan,
      super(text: textSpan.toPlainText(includeSemanticsLabels: false));

  final TextSpan _textSpan;

  @override
  TextSpan buildTextSpan({
    required BuildContext context,
    TextStyle? style,
    required bool withComposing,
  }) {
    // This does not care about composing.
    return TextSpan(style: style, children: <TextSpan>[_textSpan]);
  }

  @override
  set text(String? newText) {
    // This should never be reached.
    throw UnimplementedError();
  }
}

class _SelectableTextSelectionGestureDetectorBuilder extends TextSelectionGestureDetectorBuilder {
  _SelectableTextSelectionGestureDetectorBuilder({required _SelectableTextState state})
    : _state = state,
      super(delegate: state);

  final _SelectableTextState _state;

  @override
  void onSingleTapUp(TapDragUpDetails details) {
    if (!delegate.selectionEnabled) {
      return;
    }
    super.onSingleTapUp(details);
    _state.widget.onTap?.call();
  }
}

/// A run of selectable text with a single style.
///
/// Consider using [SelectionArea] or [SelectableRegion] instead, which enable
/// selection on a widget subtree, including but not limited to [Text] widgets.
///
/// The [SelectableText] widget displays a string of text with a single style.
/// The string might break across multiple lines or might all be displayed on
/// the same line depending on the layout constraints.
///
/// {@youtube 560 315 https://www.youtube.com/watch?v=ZSU3ZXOs6hc}
///
/// The [style] argument is optional. When omitted, the text will use the style
/// from the closest enclosing [DefaultTextStyle]. If the given style's
/// [TextStyle.inherit] property is true (the default), the given style will
/// be merged with the closest enclosing [DefaultTextStyle]. This merging
/// behavior is useful, for example, to make the text bold while using the
/// default font family and size.
///
/// {@macro flutter.material.textfield.wantKeepAlive}
///
/// {@tool snippet}
///
/// ```dart
/// const SelectableText(
///   'Hello! How are you?',
///   textAlign: TextAlign.center,
///   style: TextStyle(fontWeight: FontWeight.bold),
/// )
/// ```
/// {@end-tool}
///
/// Using the [SelectableText.rich] constructor, the [SelectableText] widget can
/// display a paragraph with differently styled [TextSpan]s. The sample
/// that follows displays "Hello beautiful world" with different styles
/// for each word.
///
/// {@tool snippet}
///
/// ```dart
/// const SelectableText.rich(
///   TextSpan(
///     text: 'Hello', // default text style
///     children: <TextSpan>[
///       TextSpan(text: ' beautiful ', style: TextStyle(fontStyle: FontStyle.italic)),
///       TextSpan(text: 'world', style: TextStyle(fontWeight: FontWeight.bold)),
///     ],
///   ),
/// )
/// ```
/// {@end-tool}
///
/// ## Interactivity
///
/// To make [SelectableText] react to touch events, use callback [onTap] to achieve
/// the desired behavior.
///
/// ## Scrolling Considerations
///
/// If this [SelectableText] is not a descendant of [Scaffold] and is being used
/// within a [Scrollable] or nested [Scrollable]s, consider placing a
/// [ScrollNotificationObserver] above the root [Scrollable] that contains this
/// [SelectableText] to ensure proper scroll coordination for [SelectableText]
/// and its components like [TextSelectionOverlay].
///
/// See also:
///
///  * [Text], which is the non selectable version of this widget.
///  * [TextField], which is the editable version of this widget.
///  * [SelectionArea], which enables the selection of multiple [Text] widgets
///    and of other widgets.
class SelectableText extends StatefulWidget {
  /// Creates a selectable text widget.
  ///
  /// If the [style] argument is null, the text will use the style from the
  /// closest enclosing [DefaultTextStyle].
  ///

  /// If the [showCursor], [autofocus], [dragStartBehavior],
  /// [selectionHeightStyle], [selectionWidthStyle] and [data] arguments are
  /// specified, the [maxLines] argument must be greater than zero.
  const SelectableText(
    String this.data, {
    super.key,
    this.focusNode,
    this.style,
    this.strutStyle,
    this.textAlign,
    this.textDirection,
    @Deprecated(
      'Use textScaler instead. '
      'Use of textScaleFactor was deprecated in preparation for the upcoming nonlinear text scaling support. '
      'This feature was deprecated after v3.12.0-2.0.pre.',
    )
    this.textScaleFactor,
    this.textScaler,
    this.showCursor = false,
    this.autofocus = false,
    @Deprecated(
      'Use `contextMenuBuilder` instead. '
      'This feature was deprecated after v3.3.0-0.5.pre.',
    )
    this.toolbarOptions,
    this.minLines,
    this.maxLines,
    this.cursorWidth = 2.0,
    this.cursorHeight,
    this.cursorRadius,
    this.cursorColor,
    this.selectionHeightStyle = ui.BoxHeightStyle.tight,
    this.selectionWidthStyle = ui.BoxWidthStyle.tight,
    this.dragStartBehavior = DragStartBehavior.start,
    this.enableInteractiveSelection = true,
    this.selectionControls,
    this.onTap,
    this.scrollPhysics,
    this.scrollBehavior,
    this.semanticsLabel,
    this.textHeightBehavior,
    this.textWidthBasis,
    this.onSelectionChanged,
    this.contextMenuBuilder = _defaultContextMenuBuilder,
    this.magnifierConfiguration,
  }) : assert(maxLines == null || maxLines > 0),
       assert(minLines == null || minLines > 0),
       assert(
         (maxLines == null) || (minLines == null) || (maxLines >= minLines),
         "minLines can't be greater than maxLines",
       ),
       assert(
         textScaler == null || textScaleFactor == null,
         'textScaleFactor is deprecated and cannot be specified when textScaler is specified.',
       ),
       textSpan = null;

  /// Creates a selectable text widget with a [TextSpan].
  ///
  /// The [TextSpan.children] attribute of the [textSpan] parameter must only
  /// contain [TextSpan]s. Other types of [InlineSpan] are not allowed.
  const SelectableText.rich(
    TextSpan this.textSpan, {
    super.key,
    this.focusNode,
    this.style,
    this.strutStyle,
    this.textAlign,
    this.textDirection,
    @Deprecated(
      'Use textScaler instead. '
      'Use of textScaleFactor was deprecated in preparation for the upcoming nonlinear text scaling support. '
      'This feature was deprecated after v3.12.0-2.0.pre.',
    )
    this.textScaleFactor,
    this.textScaler,
    this.showCursor = false,
    this.autofocus = false,
    @Deprecated(
      'Use `contextMenuBuilder` instead. '
      'This feature was deprecated after v3.3.0-0.5.pre.',
    )
    this.toolbarOptions,
    this.minLines,
    this.maxLines,
    this.cursorWidth = 2.0,
    this.cursorHeight,
    this.cursorRadius,
    this.cursorColor,
    this.selectionHeightStyle = ui.BoxHeightStyle.tight,
    this.selectionWidthStyle = ui.BoxWidthStyle.tight,
    this.dragStartBehavior = DragStartBehavior.start,
    this.enableInteractiveSelection = true,
    this.selectionControls,
    this.onTap,
    this.scrollPhysics,
    this.scrollBehavior,
    this.semanticsLabel,
    this.textHeightBehavior,
    this.textWidthBasis,
    this.onSelectionChanged,
    this.contextMenuBuilder = _defaultContextMenuBuilder,
    this.magnifierConfiguration,
  }) : assert(maxLines == null || maxLines > 0),
       assert(minLines == null || minLines > 0),
       assert(
         (maxLines == null) || (minLines == null) || (maxLines >= minLines),
         "minLines can't be greater than maxLines",
       ),
       assert(
         textScaler == null || textScaleFactor == null,
         'textScaleFactor is deprecated and cannot be specified when textScaler is specified.',
       ),
       data = null;

  /// The text to display.
  ///
  /// This will be null if a [textSpan] is provided instead.
  final String? data;

  /// The text to display as a [TextSpan].
  ///
  /// This will be null if [data] is provided instead.
  final TextSpan? textSpan;

  /// Defines the focus for this widget.
  ///
  /// Text is only selectable when widget is focused.
  ///
  /// The [focusNode] is a long-lived object that's typically managed by a
  /// [StatefulWidget] parent. See [FocusNode] for more information.
  ///
  /// To give the focus to this widget, provide a [focusNode] and then
  /// use the current [FocusScope] to request the focus:
  ///
  /// ```dart
  /// FocusScope.of(context).requestFocus(myFocusNode);
  /// ```
  ///
  /// This happens automatically when the widget is tapped.
  ///
  /// To be notified when the widget gains or loses the focus, add a listener
  /// to the [focusNode]:
  ///
  /// ```dart
  /// myFocusNode.addListener(() { print(myFocusNode.hasFocus); });
  /// ```
  ///
  /// If null, this widget will create its own [FocusNode] with
  /// [FocusNode.skipTraversal] parameter set to `true`, which causes the widget
  /// to be skipped over during focus traversal.
  final FocusNode? focusNode;

  /// The style to use for the text.
  ///
  /// If null, defaults [DefaultTextStyle] of context.
  final TextStyle? style;

  /// {@macro flutter.widgets.editableText.strutStyle}
  final StrutStyle? strutStyle;

  /// {@macro flutter.widgets.editableText.textAlign}
  final TextAlign? textAlign;

  /// {@macro flutter.widgets.editableText.textDirection}
  final TextDirection? textDirection;

  /// {@macro flutter.widgets.editableText.textScaleFactor}
  @Deprecated(
    'Use textScaler instead. '
    'Use of textScaleFactor was deprecated in preparation for the upcoming nonlinear text scaling support. '
    'This feature was deprecated after v3.12.0-2.0.pre.',
  )
  final double? textScaleFactor;

  /// {@macro flutter.painting.textPainter.textScaler}
  final TextScaler? textScaler;

  /// {@macro flutter.widgets.editableText.autofocus}
  final bool autofocus;

  /// {@macro flutter.widgets.editableText.minLines}
  final int? minLines;

  /// {@macro flutter.widgets.editableText.maxLines}
  final int? maxLines;

  /// {@macro flutter.widgets.editableText.showCursor}
  final bool showCursor;

  /// {@macro flutter.widgets.editableText.cursorWidth}
  final double cursorWidth;

  /// {@macro flutter.widgets.editableText.cursorHeight}
  final double? cursorHeight;

  /// {@macro flutter.widgets.editableText.cursorRadius}
  final Radius? cursorRadius;

  /// The color of the cursor.
  ///
  /// The cursor indicates the current text insertion point.
  ///
  /// If null then [DefaultSelectionStyle.cursorColor] is used. If that is also
  /// null and [ThemeData.platform] is [TargetPlatform.iOS] or
  /// [TargetPlatform.macOS], then [CupertinoThemeData.primaryColor] is used.
  /// Otherwise [ColorScheme.primary] of [ThemeData.colorScheme] is used.
  final Color? cursorColor;

  /// Controls how tall the selection highlight boxes are computed to be.
  ///
  /// See [ui.BoxHeightStyle] for details on available styles.
  final ui.BoxHeightStyle selectionHeightStyle;

  /// Controls how wide the selection highlight boxes are computed to be.
  ///
  /// See [ui.BoxWidthStyle] for details on available styles.
  final ui.BoxWidthStyle selectionWidthStyle;

  /// {@macro flutter.widgets.editableText.enableInteractiveSelection}
  final bool enableInteractiveSelection;

  /// {@macro flutter.widgets.editableText.selectionControls}
  final TextSelectionControls? selectionControls;

  /// {@macro flutter.widgets.scrollable.dragStartBehavior}
  final DragStartBehavior dragStartBehavior;

  /// Configuration of toolbar options.
  ///
  /// Paste and cut will be disabled regardless.
  ///
  /// If not set, select all and copy will be enabled by default.
  @Deprecated(
    'Use `contextMenuBuilder` instead. '
    'This feature was deprecated after v3.3.0-0.5.pre.',
  )
  final ToolbarOptions? toolbarOptions;

  /// {@macro flutter.widgets.editableText.selectionEnabled}
  bool get selectionEnabled => enableInteractiveSelection;

  /// Called when the user taps on this selectable text.
  ///
  /// The selectable text builds a [GestureDetector] to handle input events like tap,
  /// to trigger focus requests, to move the caret, adjust the selection, etc.
  /// Handling some of those events by wrapping the selectable text with a competing
  /// GestureDetector is problematic.
  ///
  /// To unconditionally handle taps, without interfering with the selectable text's
  /// internal gesture detector, provide this callback.
  ///
  /// To be notified when the text field gains or loses the focus, provide a
  /// [focusNode] and add a listener to that.
  ///
  /// To listen to arbitrary pointer events without competing with the
  /// selectable text's internal gesture detector, use a [Listener].
  final GestureTapCallback? onTap;

  /// {@macro flutter.widgets.editableText.scrollPhysics}
  final ScrollPhysics? scrollPhysics;

  /// {@macro flutter.widgets.editableText.scrollBehavior}
  final ScrollBehavior? scrollBehavior;

  /// {@macro flutter.widgets.Text.semanticsLabel}
  final String? semanticsLabel;

  /// {@macro dart.ui.textHeightBehavior}
  final TextHeightBehavior? textHeightBehavior;

  /// {@macro flutter.painting.textPainter.textWidthBasis}
  final TextWidthBasis? textWidthBasis;

  /// {@macro flutter.widgets.editableText.onSelectionChanged}
  final SelectionChangedCallback? onSelectionChanged;

  /// {@macro flutter.widgets.EditableText.contextMenuBuilder}
  final EditableTextContextMenuBuilder? contextMenuBuilder;

  static Widget _defaultContextMenuBuilder(
    BuildContext context,
    EditableTextState editableTextState,
  ) {
    return AdaptiveTextSelectionToolbar.editableText(editableTextState: editableTextState);
  }

  /// The configuration for the magnifier used when the text is selected.
  ///
  /// By default, builds a [CupertinoTextMagnifier] on iOS and [TextMagnifier]
  /// on Android, and builds nothing on all other platforms. To suppress the
  /// magnifier, consider passing [TextMagnifierConfiguration.disabled].
  ///
  /// {@macro flutter.widgets.magnifier.intro}
  final TextMagnifierConfiguration? magnifierConfiguration;

  @override
  State<SelectableText> createState() => _SelectableTextState();

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<String>('data', data, defaultValue: null));
    properties.add(
      DiagnosticsProperty<String>('semanticsLabel', semanticsLabel, defaultValue: null),
    );
    properties.add(DiagnosticsProperty<FocusNode>('focusNode', focusNode, defaultValue: null));
    properties.add(DiagnosticsProperty<TextStyle>('style', style, defaultValue: null));
    properties.add(DiagnosticsProperty<bool>('autofocus', autofocus, defaultValue: false));
    properties.add(DiagnosticsProperty<bool>('showCursor', showCursor, defaultValue: false));
    properties.add(IntProperty('minLines', minLines, defaultValue: null));
    properties.add(IntProperty('maxLines', maxLines, defaultValue: null));
    properties.add(EnumProperty<TextAlign>('textAlign', textAlign, defaultValue: null));
    properties.add(EnumProperty<TextDirection>('textDirection', textDirection, defaultValue: null));
    properties.add(DoubleProperty('textScaleFactor', textScaleFactor, defaultValue: null));
    properties.add(DiagnosticsProperty<TextScaler>('textScaler', textScaler, defaultValue: null));
    properties.add(DoubleProperty('cursorWidth', cursorWidth, defaultValue: 2.0));
    properties.add(DoubleProperty('cursorHeight', cursorHeight, defaultValue: null));
    properties.add(DiagnosticsProperty<Radius>('cursorRadius', cursorRadius, defaultValue: null));
    properties.add(DiagnosticsProperty<Color>('cursorColor', cursorColor, defaultValue: null));
    properties.add(
      FlagProperty(
        'selectionEnabled',
        value: selectionEnabled,
        defaultValue: true,
        ifFalse: 'selection disabled',
      ),
    );
    properties.add(
      DiagnosticsProperty<TextSelectionControls>(
        'selectionControls',
        selectionControls,
        defaultValue: null,
      ),
    );
    properties.add(
      DiagnosticsProperty<ScrollPhysics>('scrollPhysics', scrollPhysics, defaultValue: null),
    );
    properties.add(
      DiagnosticsProperty<ScrollBehavior>('scrollBehavior', scrollBehavior, defaultValue: null),
    );
    properties.add(
      DiagnosticsProperty<TextHeightBehavior>(
        'textHeightBehavior',
        textHeightBehavior,
        defaultValue: null,
      ),
    );
  }
}

class _SelectableTextState extends State<SelectableText>
    implements TextSelectionGestureDetectorBuilderDelegate {
  EditableTextState? get _editableText => editableTextKey.currentState;

  late _TextSpanEditingController _controller;

  FocusNode? _focusNode;
  FocusNode get _effectiveFocusNode =>
      widget.focusNode ?? (_focusNode ??= FocusNode(skipTraversal: true));

  bool _showSelectionHandles = false;

  late _SelectableTextSelectionGestureDetectorBuilder _selectionGestureDetectorBuilder;

  // API for TextSelectionGestureDetectorBuilderDelegate.
  @override
  late bool forcePressEnabled;

  @override
  final GlobalKey<EditableTextState> editableTextKey = GlobalKey<EditableTextState>();

  @override
  bool get selectionEnabled => widget.selectionEnabled;
  // End of API for TextSelectionGestureDetectorBuilderDelegate.

  @override
  void initState() {
    super.initState();
    _selectionGestureDetectorBuilder = _SelectableTextSelectionGestureDetectorBuilder(state: this);
    _controller = _TextSpanEditingController(
      textSpan: widget.textSpan ?? TextSpan(text: widget.data),
    );
    _controller.addListener(_onControllerChanged);
    _effectiveFocusNode.addListener(_handleFocusChanged);
  }

  @override
  void didUpdateWidget(SelectableText oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (widget.data != oldWidget.data || widget.textSpan != oldWidget.textSpan) {
      _controller.removeListener(_onControllerChanged);
      _controller.dispose();
      _controller = _TextSpanEditingController(
        textSpan: widget.textSpan ?? TextSpan(text: widget.data),
      );
      _controller.addListener(_onControllerChanged);
    }
    if (widget.focusNode != oldWidget.focusNode) {
      (oldWidget.focusNode ?? _focusNode)?.removeListener(_handleFocusChanged);
      (widget.focusNode ?? _focusNode)?.addListener(_handleFocusChanged);
    }
    if (_effectiveFocusNode.hasFocus && _controller.selection.isCollapsed) {
      _showSelectionHandles = false;
    } else {
      _showSelectionHandles = true;
    }
  }

  @override
  void dispose() {
    _effectiveFocusNode.removeListener(_handleFocusChanged);
    _focusNode?.dispose();
    _controller.dispose();
    super.dispose();
  }

  void _onControllerChanged() {
    final bool showSelectionHandles =
        !_effectiveFocusNode.hasFocus || !_controller.selection.isCollapsed;
    if (showSelectionHandles == _showSelectionHandles) {
      return;
    }
    setState(() {
      _showSelectionHandles = showSelectionHandles;
    });
  }

  void _handleFocusChanged() {
    if (!_effectiveFocusNode.hasFocus &&
        SchedulerBinding.instance.lifecycleState == AppLifecycleState.resumed) {
      // We should only clear the selection when this SelectableText loses
      // focus while the application is currently running. It is possible
      // that the application is not currently running, for example on desktop
      // platforms, clicking on a different window switches the focus to
      // the new window causing the Flutter application to go inactive. In this
      // case we want to retain the selection so it remains when we return to
      // the Flutter application.
      _controller.value = TextEditingValue(text: _controller.value.text);
    }
  }

  void _handleSelectionChanged(TextSelection selection, SelectionChangedCause? cause) {
    final bool willShowSelectionHandles = _shouldShowSelectionHandles(cause);
    if (willShowSelectionHandles != _showSelectionHandles) {
      setState(() {
        _showSelectionHandles = willShowSelectionHandles;
      });
    }

    widget.onSelectionChanged?.call(selection, cause);

    switch (Theme.of(context).platform) {
      case TargetPlatform.iOS:
      case TargetPlatform.macOS:
        if (cause == SelectionChangedCause.longPress) {
          _editableText?.bringIntoView(selection.base);
        }
        return;
      case TargetPlatform.android:
      case TargetPlatform.fuchsia:
      case TargetPlatform.linux:
      case TargetPlatform.windows:
      // Do nothing.
    }
  }

  /// Toggle the toolbar when a selection handle is tapped.
  void _handleSelectionHandleTapped() {
    if (_controller.selection.isCollapsed) {
      _editableText!.toggleToolbar();
    }
  }

  bool _shouldShowSelectionHandles(SelectionChangedCause? cause) {
    // When the text field is activated by something that doesn't trigger the
    // selection overlay, we shouldn't show the handles either.
    if (!_selectionGestureDetectorBuilder.shouldShowSelectionToolbar) {
      return false;
    }

    if (_controller.selection.isCollapsed) {
      return false;
    }

    if (cause == SelectionChangedCause.keyboard) {
      return false;
    }

    if (cause == SelectionChangedCause.longPress) {
      return true;
    }

    if (_controller.text.isNotEmpty) {
      return true;
    }

    return false;
  }

  @override
  Widget build(BuildContext context) {
    // TODO(garyq): Assert to block WidgetSpans from being used here are removed,
    // but we still do not yet have nice handling of things like carets, clipboard,
    // and other features. We should add proper support. Currently, caret handling
    // is blocked on SkParagraph switch and https://github.com/flutter/engine/pull/27010
    // should be landed in SkParagraph after the switch is complete.
    assert(debugCheckHasMediaQuery(context));
    assert(debugCheckHasDirectionality(context));
    assert(
      !(widget.style != null &&
          !widget.style!.inherit &&
          (widget.style!.fontSize == null || widget.style!.textBaseline == null)),
      'inherit false style must supply fontSize and textBaseline',
    );

    final ThemeData theme = Theme.of(context);
    final DefaultSelectionStyle selectionStyle = DefaultSelectionStyle.of(context);
    final FocusNode focusNode = _effectiveFocusNode;

    TextSelectionControls? textSelectionControls = widget.selectionControls;
    final bool paintCursorAboveText;
    final bool cursorOpacityAnimates;
    Offset? cursorOffset;
    final Color cursorColor;
    final Color selectionColor;
    Radius? cursorRadius = widget.cursorRadius;

    switch (theme.platform) {
      case TargetPlatform.iOS:
        final CupertinoThemeData cupertinoTheme = CupertinoTheme.of(context);
        forcePressEnabled = true;
        textSelectionControls ??= cupertinoTextSelectionHandleControls;
        paintCursorAboveText = true;
        cursorOpacityAnimates = true;
        cursorColor =
            widget.cursorColor ?? selectionStyle.cursorColor ?? cupertinoTheme.primaryColor;
        selectionColor =
            selectionStyle.selectionColor ?? cupertinoTheme.primaryColor.withOpacity(0.40);
        cursorRadius ??= const Radius.circular(2.0);
        cursorOffset = Offset(iOSHorizontalOffset / MediaQuery.devicePixelRatioOf(context), 0);

      case TargetPlatform.macOS:
        final CupertinoThemeData cupertinoTheme = CupertinoTheme.of(context);
        forcePressEnabled = false;
        textSelectionControls ??= cupertinoDesktopTextSelectionHandleControls;
        paintCursorAboveText = true;
        cursorOpacityAnimates = true;
        cursorColor =
            widget.cursorColor ?? selectionStyle.cursorColor ?? cupertinoTheme.primaryColor;
        selectionColor =
            selectionStyle.selectionColor ?? cupertinoTheme.primaryColor.withOpacity(0.40);
        cursorRadius ??= const Radius.circular(2.0);
        cursorOffset = Offset(iOSHorizontalOffset / MediaQuery.devicePixelRatioOf(context), 0);

      case TargetPlatform.android:
      case TargetPlatform.fuchsia:
        forcePressEnabled = false;
        textSelectionControls ??= materialTextSelectionHandleControls;
        paintCursorAboveText = false;
        cursorOpacityAnimates = false;
        cursorColor = widget.cursorColor ?? selectionStyle.cursorColor ?? theme.colorScheme.primary;
        selectionColor =
            selectionStyle.selectionColor ?? theme.colorScheme.primary.withOpacity(0.40);

      case TargetPlatform.linux:
      case TargetPlatform.windows:
        forcePressEnabled = false;
        textSelectionControls ??= desktopTextSelectionHandleControls;
        paintCursorAboveText = false;
        cursorOpacityAnimates = false;
        cursorColor = widget.cursorColor ?? selectionStyle.cursorColor ?? theme.colorScheme.primary;
        selectionColor =
            selectionStyle.selectionColor ?? theme.colorScheme.primary.withOpacity(0.40);
    }

    final DefaultTextStyle defaultTextStyle = DefaultTextStyle.of(context);
    TextStyle? effectiveTextStyle = widget.style;
    if (effectiveTextStyle == null || effectiveTextStyle.inherit) {
      effectiveTextStyle = defaultTextStyle.style.merge(
        widget.style ?? _controller._textSpan.style,
      );
    }
    final TextScaler? effectiveScaler =
        widget.textScaler ??
        switch (widget.textScaleFactor) {
          null => null,
          final double textScaleFactor => TextScaler.linear(textScaleFactor),
        };
    final Widget child = RepaintBoundary(
      child: EditableText(
        key: editableTextKey,
        style: effectiveTextStyle,
        readOnly: true,
        toolbarOptions: widget.toolbarOptions,
        textWidthBasis: widget.textWidthBasis ?? defaultTextStyle.textWidthBasis,
        textHeightBehavior: widget.textHeightBehavior ?? defaultTextStyle.textHeightBehavior,
        showSelectionHandles: _showSelectionHandles,
        showCursor: widget.showCursor,
        controller: _controller,
        focusNode: focusNode,
        strutStyle: widget.strutStyle ?? const StrutStyle(),
        textAlign: widget.textAlign ?? defaultTextStyle.textAlign ?? TextAlign.start,
        textDirection: widget.textDirection,
        textScaler: effectiveScaler,
        autofocus: widget.autofocus,
        forceLine: false,
        minLines: widget.minLines,
        maxLines: widget.maxLines ?? defaultTextStyle.maxLines,
        selectionColor: selectionColor,
        selectionControls: widget.selectionEnabled ? textSelectionControls : null,
        onSelectionChanged: _handleSelectionChanged,
        onSelectionHandleTapped: _handleSelectionHandleTapped,
        rendererIgnoresPointer: true,
        cursorWidth: widget.cursorWidth,
        cursorHeight: widget.cursorHeight,
        cursorRadius: cursorRadius,
        cursorColor: cursorColor,
        selectionHeightStyle: widget.selectionHeightStyle,
        selectionWidthStyle: widget.selectionWidthStyle,
        cursorOpacityAnimates: cursorOpacityAnimates,
        cursorOffset: cursorOffset,
        paintCursorAboveText: paintCursorAboveText,
        backgroundCursorColor: CupertinoColors.inactiveGray,
        enableInteractiveSelection: widget.enableInteractiveSelection,
        magnifierConfiguration:
            widget.magnifierConfiguration ?? TextMagnifier.adaptiveMagnifierConfiguration,
        dragStartBehavior: widget.dragStartBehavior,
        scrollPhysics: widget.scrollPhysics,
        scrollBehavior: widget.scrollBehavior,
        autofillHints: null,
        contextMenuBuilder: widget.contextMenuBuilder,
      ),
    );

    return Semantics(
      label: widget.semanticsLabel,
      excludeSemantics: widget.semanticsLabel != null,
      onLongPress: () {
        _effectiveFocusNode.requestFocus();
      },
      child: _selectionGestureDetectorBuilder.buildGestureDetector(
        behavior: HitTestBehavior.translucent,
        child: child,
      ),
    );
  }
}
