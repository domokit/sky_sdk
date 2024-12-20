// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
library;

import 'dart:ui' as ui show BoxHeightStyle, BoxWidthStyle;

import 'package:flutter/foundation.dart' show defaultTargetPlatform;
import 'package:flutter/gestures.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter/widgets.dart';

import 'adaptive_text_selection_toolbar.dart';
import 'colors.dart';
import 'desktop_text_selection.dart';
import 'icons.dart';
import 'localizations.dart';
import 'magnifier.dart';
import 'spell_check_suggestions_toolbar.dart';
import 'text_selection.dart';
import 'theme.dart';

export 'package:flutter/services.dart'
    show SmartDashesType, SmartQuotesType, TextCapitalization, TextInputAction, TextInputType;

const TextStyle _kDefaultPlaceholderStyle = TextStyle(
  fontWeight: FontWeight.w400,
  color: CupertinoColors.placeholderText,
);

// Value inspected from Xcode 11 & iOS 13.0 Simulator.
const BorderSide _kDefaultRoundedBorderSide = BorderSide(
  color: CupertinoDynamicColor.withBrightness(
    color: Color(0x33000000),
    darkColor: Color(0x33FFFFFF),
  ),
  width: 0.0,
);
const Border _kDefaultRoundedBorder = Border(
  top: _kDefaultRoundedBorderSide,
  bottom: _kDefaultRoundedBorderSide,
  left: _kDefaultRoundedBorderSide,
  right: _kDefaultRoundedBorderSide,
);

const BoxDecoration _kDefaultRoundedBorderDecoration = BoxDecoration(
  color: CupertinoDynamicColor.withBrightness(
    color: CupertinoColors.white,
    darkColor: CupertinoColors.black,
  ),
  border: _kDefaultRoundedBorder,
  borderRadius: BorderRadius.all(Radius.circular(5.0)),
);

const Color _kDisabledBackground = CupertinoDynamicColor.withBrightness(
  color: Color(0xFFFAFAFA),
  darkColor: Color(0xFF050505),
);

// Value inspected from Xcode 12 & iOS 14.0 Simulator.
// Note it may not be consistent with https://developer.apple.com/design/resources/.
const CupertinoDynamicColor _kClearButtonColor = CupertinoDynamicColor.withBrightness(
  color: Color(0x33000000),
  darkColor: Color(0x33FFFFFF),
);

// An eyeballed value that moves the cursor slightly left of where it is
// rendered for text on Android so it's positioning more accurately matches the
// native iOS text cursor positioning.
//
// This value is in device pixels, not logical pixels as is typically used
// throughout the codebase.
const int _iOSHorizontalCursorOffsetPixels = -2;

/// Visibility of text field overlays based on the state of the current text entry.
///
/// Used to toggle the visibility behavior of the optional decorating widgets
/// surrounding the [EditableText] such as the clear text button.
enum OverlayVisibilityMode {
  /// Overlay will never appear regardless of the text entry state.
  never,

  /// Overlay will only appear when the current text entry is not empty.
  ///
  /// This includes prefilled text that the user did not type in manually. But
  /// does not include text in placeholders.
  editing,

  /// Overlay will only appear when the current text entry is empty.
  ///
  /// This also includes not having prefilled text that the user did not type
  /// in manually. Texts in placeholders are ignored.
  notEditing,

  /// Always show the overlay regardless of the text entry state.
  always,
}

class _CupertinoTextFieldSelectionGestureDetectorBuilder
    extends TextSelectionGestureDetectorBuilder {
  _CupertinoTextFieldSelectionGestureDetectorBuilder({required _CupertinoTextFieldState state})
    : _state = state,
      super(delegate: state);

  final _CupertinoTextFieldState _state;

  @override
  void onSingleTapUp(TapDragUpDetails details) {
    // Because TextSelectionGestureDetector listens to taps that happen on
    // widgets in front of it, tapping the clear button will also trigger
    // this handler. If the clear button widget recognizes the up event,
    // then do not handle it.
    if (_state._clearGlobalKey.currentContext != null) {
      final RenderBox renderBox =
          _state._clearGlobalKey.currentContext!.findRenderObject()! as RenderBox;
      final Offset localOffset = renderBox.globalToLocal(details.globalPosition);
      if (renderBox.hitTest(BoxHitTestResult(), position: localOffset)) {
        return;
      }
    }
    super.onSingleTapUp(details);
    _state.widget.onTap?.call();
  }

  @override
  void onDragSelectionEnd(TapDragEndDetails details) {
    _state._requestKeyboard();
    super.onDragSelectionEnd(details);
  }
}

/// An iOS-style text field.
///
/// A text field lets the user enter text, either with a hardware keyboard or with
/// an onscreen keyboard.
///
/// This widget corresponds to both a `UITextField` and an editable `UITextView`
/// on iOS.
///
/// The text field calls the [onChanged] callback whenever the user changes the
/// text in the field. If the user indicates that they are done typing in the
/// field (e.g., by pressing a button on the soft keyboard), the text field
/// calls the [onSubmitted] callback.
///
/// {@macro flutter.widgets.EditableText.onChanged}
///
/// {@tool dartpad}
/// This example shows how to set the initial value of the [CupertinoTextField] using
/// a [controller] that already contains some text.
///
/// ** See code in examples/api/lib/cupertino/text_field/cupertino_text_field.0.dart **
/// {@end-tool}
///
/// The [controller] can also control the selection and composing region (and to
/// observe changes to the text, selection, and composing region).
///
/// The text field has an overridable [decoration] that, by default, draws a
/// rounded rectangle border around the text field. If you set the [decoration]
/// property to null, the decoration will be removed entirely.
///
/// {@macro flutter.material.textfield.wantKeepAlive}
///
/// Remember to call [TextEditingController.dispose] when it is no longer
/// needed. This will ensure we discard any resources used by the object.
///
/// {@macro flutter.widgets.editableText.showCaretOnScreen}
///
/// ## Scrolling Considerations
///
/// If this [CupertinoTextField] is not a descendant of [Scaffold] and is being
/// used within a [Scrollable] or nested [Scrollable]s, consider placing a
/// [ScrollNotificationObserver] above the root [Scrollable] that contains this
/// [CupertinoTextField] to ensure proper scroll coordination for
/// [CupertinoTextField] and its components like [TextSelectionOverlay].
///
/// See also:
///
///  * <https://developer.apple.com/documentation/uikit/uitextfield>
///  * [TextField], an alternative text field widget that follows the Material
///    Design UI conventions.
///  * [EditableText], which is the raw text editing control at the heart of a
///    [TextField].
///  * Learn how to use a [TextEditingController] in one of our [cookbook recipes](https://docs.flutter.dev/cookbook/forms/text-field-changes#2-use-a-texteditingcontroller).
///  * <https://developer.apple.com/design/human-interface-guidelines/ios/controls/text-fields/>
class CupertinoTextField extends StatefulWidget {
  /// Creates an iOS-style text field.
  ///
  /// To provide a prefilled text entry, pass in a [TextEditingController] with
  /// an initial value to the [controller] parameter.
  ///
  /// To provide a hint placeholder text that appears when the text entry is
  /// empty, pass a [String] to the [placeholder] parameter.
  ///
  /// The [maxLines] property can be set to null to remove the restriction on
  /// the number of lines. In this mode, the intrinsic height of the widget will
  /// grow as the number of lines of text grows. By default, it is `1`, meaning
  /// this is a single-line text field and will scroll horizontally when
  /// it overflows. [maxLines] must not be zero.
  ///
  /// The text cursor is not shown if [showCursor] is false or if [showCursor]
  /// is null (the default) and [readOnly] is true.
  ///
  /// If specified, the [maxLength] property must be greater than zero.
  ///
  /// The [selectionHeightStyle] and [selectionWidthStyle] properties allow
  /// changing the shape of the selection highlighting. These properties default
  /// to [ui.BoxHeightStyle.tight] and [ui.BoxWidthStyle.tight], respectively.
  ///
  /// The [autocorrect], [autofocus], [clearButtonMode], [dragStartBehavior],
  /// [expands], [obscureText], [prefixMode], [readOnly], [scrollPadding],
  /// [suffixMode], [textAlign], [selectionHeightStyle], [selectionWidthStyle],
  /// [enableSuggestions], and [enableIMEPersonalizedLearning] properties must
  /// not be null.
  ///
  /// {@macro flutter.widgets.editableText.accessibility}
  ///
  /// See also:
  ///
  ///  * [minLines], which is the minimum number of lines to occupy when the
  ///    content spans fewer lines.
  ///  * [expands], to allow the widget to size itself to its parent's height.
  ///  * [maxLength], which discusses the precise meaning of "number of
  ///    characters" and how it may differ from the intuitive meaning.
  const CupertinoTextField({
    super.key,
    this.groupId = EditableText,
    this.controller,
    this.focusNode,
    this.undoController,
    this.decoration = _kDefaultRoundedBorderDecoration,
    this.padding = const EdgeInsets.all(7.0),
    this.placeholder,
    this.placeholderStyle = const TextStyle(
      fontWeight: FontWeight.w400,
      color: CupertinoColors.placeholderText,
    ),
    this.prefix,
    this.prefixMode = OverlayVisibilityMode.always,
    this.suffix,
    this.suffixMode = OverlayVisibilityMode.always,
    this.crossAxisAlignment = CrossAxisAlignment.center,
    this.clearButtonMode = OverlayVisibilityMode.never,
    this.clearButtonSemanticLabel,
    TextInputType? keyboardType,
    this.textInputAction,
    this.textCapitalization = TextCapitalization.none,
    this.style,
    this.strutStyle,
    this.textAlign = TextAlign.start,
    this.textAlignVertical,
    this.textDirection,
    this.readOnly = false,
    @Deprecated(
      'Use `contextMenuBuilder` instead. '
      'This feature was deprecated after v3.3.0-0.5.pre.',
    )
    this.toolbarOptions,
    this.showCursor,
    this.autofocus = false,
    this.obscuringCharacter = '•',
    this.obscureText = false,
    this.autocorrect = true,
    SmartDashesType? smartDashesType,
    SmartQuotesType? smartQuotesType,
    this.enableSuggestions = true,
    this.maxLines = 1,
    this.minLines,
    this.expands = false,
    this.maxLength,
    this.maxLengthEnforcement,
    this.onChanged,
    this.onEditingComplete,
    this.onSubmitted,
    this.onTapOutside,
    this.onTapUpOutside,
    this.inputFormatters,
    this.enabled = true,
    this.cursorWidth = 2.0,
    this.cursorHeight,
    this.cursorRadius = const Radius.circular(2.0),
    this.cursorOpacityAnimates = true,
    this.cursorColor,
    this.selectionHeightStyle = ui.BoxHeightStyle.tight,
    this.selectionWidthStyle = ui.BoxWidthStyle.tight,
    this.keyboardAppearance,
    this.scrollPadding = const EdgeInsets.all(20.0),
    this.dragStartBehavior = DragStartBehavior.start,
    bool? enableInteractiveSelection,
    this.selectionControls,
    this.onTap,
    this.scrollController,
    this.scrollPhysics,
    this.autofillHints = const <String>[],
    this.contentInsertionConfiguration,
    this.clipBehavior = Clip.hardEdge,
    this.restorationId,
    @Deprecated(
      'Use `stylusHandwritingEnabled` instead. '
      'This feature was deprecated after v3.27.0-0.2.pre.',
    )
    this.scribbleEnabled = true,
    this.stylusHandwritingEnabled = EditableText.defaultStylusHandwritingEnabled,
    this.enableIMEPersonalizedLearning = true,
    this.contextMenuBuilder = _defaultContextMenuBuilder,
    this.spellCheckConfiguration,
    this.magnifierConfiguration,
  }) : assert(obscuringCharacter.length == 1),
       smartDashesType =
           smartDashesType ?? (obscureText ? SmartDashesType.disabled : SmartDashesType.enabled),
       smartQuotesType =
           smartQuotesType ?? (obscureText ? SmartQuotesType.disabled : SmartQuotesType.enabled),
       assert(maxLines == null || maxLines > 0),
       assert(minLines == null || minLines > 0),
       assert(
         (maxLines == null) || (minLines == null) || (maxLines >= minLines),
         "minLines can't be greater than maxLines",
       ),
       assert(
         !expands || (maxLines == null && minLines == null),
         'minLines and maxLines must be null when expands is true.',
       ),
       assert(!obscureText || maxLines == 1, 'Obscured fields cannot be multiline.'),
       assert(maxLength == null || maxLength > 0),
       // Assert the following instead of setting it directly to avoid surprising the user by silently changing the value they set.
       assert(
         !identical(textInputAction, TextInputAction.newline) ||
             maxLines == 1 ||
             !identical(keyboardType, TextInputType.text),
         'Use keyboardType TextInputType.multiline when using TextInputAction.newline on a multiline TextField.',
       ),
       keyboardType =
           keyboardType ?? (maxLines == 1 ? TextInputType.text : TextInputType.multiline),
       enableInteractiveSelection = enableInteractiveSelection ?? (!readOnly || !obscureText);

  /// Creates a borderless iOS-style text field.
  ///
  /// To provide a prefilled text entry, pass in a [TextEditingController] with
  /// an initial value to the [controller] parameter.
  ///
  /// To provide a hint placeholder text that appears when the text entry is
  /// empty, pass a [String] to the [placeholder] parameter.
  ///
  /// The [maxLines] property can be set to null to remove the restriction on
  /// the number of lines. In this mode, the intrinsic height of the widget will
  /// grow as the number of lines of text grows. By default, it is `1`, meaning
  /// this is a single-line text field and will scroll horizontally when
  /// it overflows. [maxLines] must not be zero.
  ///
  /// The text cursor is not shown if [showCursor] is false or if [showCursor]
  /// is null (the default) and [readOnly] is true.
  ///
  /// If specified, the [maxLength] property must be greater than zero.
  ///
  /// The [selectionHeightStyle] and [selectionWidthStyle] properties allow
  /// changing the shape of the selection highlighting. These properties default
  /// to [ui.BoxHeightStyle.tight] and [ui.BoxWidthStyle.tight] respectively.
  ///
  /// See also:
  ///
  ///  * [minLines], which is the minimum number of lines to occupy when the
  ///    content spans fewer lines.
  ///  * [expands], to allow the widget to size itself to its parent's height.
  ///  * [maxLength], which discusses the precise meaning of "number of
  ///    characters" and how it may differ from the intuitive meaning.
  const CupertinoTextField.borderless({
    super.key,
    this.groupId = EditableText,
    this.controller,
    this.focusNode,
    this.undoController,
    this.decoration,
    this.padding = const EdgeInsets.all(7.0),
    this.placeholder,
    this.placeholderStyle = _kDefaultPlaceholderStyle,
    this.prefix,
    this.prefixMode = OverlayVisibilityMode.always,
    this.suffix,
    this.suffixMode = OverlayVisibilityMode.always,
    this.crossAxisAlignment = CrossAxisAlignment.center,
    this.clearButtonMode = OverlayVisibilityMode.never,
    this.clearButtonSemanticLabel,
    TextInputType? keyboardType,
    this.textInputAction,
    this.textCapitalization = TextCapitalization.none,
    this.style,
    this.strutStyle,
    this.textAlign = TextAlign.start,
    this.textAlignVertical,
    this.textDirection,
    this.readOnly = false,
    @Deprecated(
      'Use `contextMenuBuilder` instead. '
      'This feature was deprecated after v3.3.0-0.5.pre.',
    )
    this.toolbarOptions,
    this.showCursor,
    this.autofocus = false,
    this.obscuringCharacter = '•',
    this.obscureText = false,
    this.autocorrect = true,
    SmartDashesType? smartDashesType,
    SmartQuotesType? smartQuotesType,
    this.enableSuggestions = true,
    this.maxLines = 1,
    this.minLines,
    this.expands = false,
    this.maxLength,
    this.maxLengthEnforcement,
    this.onChanged,
    this.onEditingComplete,
    this.onSubmitted,
    this.onTapOutside,
    this.onTapUpOutside,
    this.inputFormatters,
    this.enabled = true,
    this.cursorWidth = 2.0,
    this.cursorHeight,
    this.cursorRadius = const Radius.circular(2.0),
    this.cursorOpacityAnimates = true,
    this.cursorColor,
    this.selectionHeightStyle = ui.BoxHeightStyle.tight,
    this.selectionWidthStyle = ui.BoxWidthStyle.tight,
    this.keyboardAppearance,
    this.scrollPadding = const EdgeInsets.all(20.0),
    this.dragStartBehavior = DragStartBehavior.start,
    bool? enableInteractiveSelection,
    this.selectionControls,
    this.onTap,
    this.scrollController,
    this.scrollPhysics,
    this.autofillHints = const <String>[],
    this.contentInsertionConfiguration,
    this.clipBehavior = Clip.hardEdge,
    this.restorationId,
    @Deprecated(
      'Use `stylusHandwritingEnabled` instead. '
      'This feature was deprecated after v3.27.0-0.2.pre.',
    )
    this.scribbleEnabled = true,
    this.stylusHandwritingEnabled = true,
    this.enableIMEPersonalizedLearning = true,
    this.contextMenuBuilder = _defaultContextMenuBuilder,
    this.spellCheckConfiguration,
    this.magnifierConfiguration,
  }) : assert(obscuringCharacter.length == 1),
       smartDashesType =
           smartDashesType ?? (obscureText ? SmartDashesType.disabled : SmartDashesType.enabled),
       smartQuotesType =
           smartQuotesType ?? (obscureText ? SmartQuotesType.disabled : SmartQuotesType.enabled),
       assert(maxLines == null || maxLines > 0),
       assert(minLines == null || minLines > 0),
       assert(
         (maxLines == null) || (minLines == null) || (maxLines >= minLines),
         "minLines can't be greater than maxLines",
       ),
       assert(
         !expands || (maxLines == null && minLines == null),
         'minLines and maxLines must be null when expands is true.',
       ),
       assert(!obscureText || maxLines == 1, 'Obscured fields cannot be multiline.'),
       assert(maxLength == null || maxLength > 0),
       // Assert the following instead of setting it directly to avoid surprising the user by silently changing the value they set.
       assert(
         !identical(textInputAction, TextInputAction.newline) ||
             maxLines == 1 ||
             !identical(keyboardType, TextInputType.text),
         'Use keyboardType TextInputType.multiline when using TextInputAction.newline on a multiline TextField.',
       ),
       keyboardType =
           keyboardType ?? (maxLines == 1 ? TextInputType.text : TextInputType.multiline),
       enableInteractiveSelection = enableInteractiveSelection ?? (!readOnly || !obscureText);

  /// {@macro flutter.widgets.editableText.groupId}
  final Object groupId;

  /// Controls the text being edited.
  ///
  /// If null, this widget will create its own [TextEditingController].
  final TextEditingController? controller;

  /// {@macro flutter.widgets.Focus.focusNode}
  final FocusNode? focusNode;

  /// Controls the [BoxDecoration] of the box behind the text input.
  ///
  /// Defaults to having a rounded rectangle grey border and can be null to have
  /// no box decoration.
  final BoxDecoration? decoration;

  /// Padding around the text entry area between the [prefix] and [suffix]
  /// or the clear button when [clearButtonMode] is not never.
  ///
  /// Defaults to a padding of 6 pixels on all sides and can be null.
  final EdgeInsetsGeometry padding;

  /// A lighter colored placeholder hint that appears on the first line of the
  /// text field when the text entry is empty.
  ///
  /// Defaults to having no placeholder text.
  ///
  /// The text style of the placeholder text matches that of the text field's
  /// main text entry except a lighter font weight and a grey font color.
  final String? placeholder;

  /// The style to use for the placeholder text.
  ///
  /// The [placeholderStyle] is merged with the [style] [TextStyle] when applied
  /// to the [placeholder] text. To avoid merging with [style], specify
  /// [TextStyle.inherit] as false.
  ///
  /// Defaults to the [style] property with w300 font weight and grey color.
  ///
  /// If specifically set to null, placeholder's style will be the same as [style].
  final TextStyle? placeholderStyle;

  /// An optional [Widget] to display before the text.
  final Widget? prefix;

  /// Controls the visibility of the [prefix] widget based on the state of
  /// text entry when the [prefix] argument is not null.
  ///
  /// Defaults to [OverlayVisibilityMode.always].
  ///
  /// Has no effect when [prefix] is null.
  final OverlayVisibilityMode prefixMode;

  /// An optional [Widget] to display after the text.
  final Widget? suffix;

  /// Controls the visibility of the [suffix] widget based on the state of
  /// text entry when the [suffix] argument is not null.
  ///
  /// Defaults to [OverlayVisibilityMode.always].
  ///
  /// Has no effect when [suffix] is null.
  final OverlayVisibilityMode suffixMode;

  /// Controls the vertical alignment of the [prefix] and the [suffix] widget in relation to content.
  ///
  /// Defaults to [CrossAxisAlignment.center].
  ///
  /// Has no effect when both the [prefix] and [suffix] are null.
  final CrossAxisAlignment crossAxisAlignment;

  /// Show an iOS-style clear button to clear the current text entry.
  ///
  /// Can be made to appear depending on various text states of the
  /// [TextEditingController].
  ///
  /// Will only appear if no [suffix] widget is appearing.
  ///
  /// Defaults to [OverlayVisibilityMode.never].
  final OverlayVisibilityMode clearButtonMode;

  /// The semantic label for the clear button used by screen readers.
  ///
  /// This will be used by screen reading software to identify the clear button
  /// widget. Defaults to "Clear".
  final String? clearButtonSemanticLabel;

  /// {@macro flutter.widgets.editableText.keyboardType}
  final TextInputType keyboardType;

  /// The type of action button to use for the keyboard.
  ///
  /// Defaults to [TextInputAction.newline] if [keyboardType] is
  /// [TextInputType.multiline] and [TextInputAction.done] otherwise.
  final TextInputAction? textInputAction;

  /// {@macro flutter.widgets.editableText.textCapitalization}
  final TextCapitalization textCapitalization;

  /// The style to use for the text being edited.
  ///
  /// Also serves as a base for the [placeholder] text's style.
  ///
  /// Defaults to the standard iOS font style from [CupertinoTheme] if null.
  final TextStyle? style;

  /// {@macro flutter.widgets.editableText.strutStyle}
  final StrutStyle? strutStyle;

  /// {@macro flutter.widgets.editableText.textAlign}
  final TextAlign textAlign;

  /// Configuration of toolbar options.
  ///
  /// If not set, select all and paste will default to be enabled. Copy and cut
  /// will be disabled if [obscureText] is true. If [readOnly] is true,
  /// paste and cut will be disabled regardless.
  @Deprecated(
    'Use `contextMenuBuilder` instead. '
    'This feature was deprecated after v3.3.0-0.5.pre.',
  )
  final ToolbarOptions? toolbarOptions;

  /// {@macro flutter.material.InputDecorator.textAlignVertical}
  final TextAlignVertical? textAlignVertical;

  /// {@macro flutter.widgets.editableText.textDirection}
  final TextDirection? textDirection;

  /// {@macro flutter.widgets.editableText.readOnly}
  final bool readOnly;

  /// {@macro flutter.widgets.editableText.showCursor}
  final bool? showCursor;

  /// {@macro flutter.widgets.editableText.autofocus}
  final bool autofocus;

  /// {@macro flutter.widgets.editableText.obscuringCharacter}
  final String obscuringCharacter;

  /// {@macro flutter.widgets.editableText.obscureText}
  final bool obscureText;

  /// {@macro flutter.widgets.editableText.autocorrect}
  final bool autocorrect;

  /// {@macro flutter.services.TextInputConfiguration.smartDashesType}
  final SmartDashesType smartDashesType;

  /// {@macro flutter.services.TextInputConfiguration.smartQuotesType}
  final SmartQuotesType smartQuotesType;

  /// {@macro flutter.services.TextInputConfiguration.enableSuggestions}
  final bool enableSuggestions;

  /// {@macro flutter.widgets.editableText.maxLines}
  ///  * [expands], which determines whether the field should fill the height of
  ///    its parent.
  final int? maxLines;

  /// {@macro flutter.widgets.editableText.minLines}
  ///  * [expands], which determines whether the field should fill the height of
  ///    its parent.
  final int? minLines;

  /// {@macro flutter.widgets.editableText.expands}
  final bool expands;

  /// The maximum number of characters (Unicode grapheme clusters) to allow in
  /// the text field.
  ///
  /// After [maxLength] characters have been input, additional input
  /// is ignored, unless [maxLengthEnforcement] is set to
  /// [MaxLengthEnforcement.none].
  ///
  /// The TextField enforces the length with a
  /// [LengthLimitingTextInputFormatter], which is evaluated after the supplied
  /// [inputFormatters], if any.
  ///
  /// This value must be either null or greater than zero. If set to null
  /// (the default), there is no limit to the number of characters allowed.
  ///
  /// Whitespace characters (e.g. newline, space, tab) are included in the
  /// character count.
  ///
  /// {@macro flutter.services.lengthLimitingTextInputFormatter.maxLength}
  final int? maxLength;

  /// Determines how the [maxLength] limit should be enforced.
  ///
  /// If [MaxLengthEnforcement.none] is set, additional input beyond [maxLength]
  /// will not be enforced by the limit.
  ///
  /// {@macro flutter.services.textFormatter.effectiveMaxLengthEnforcement}
  ///
  /// {@macro flutter.services.textFormatter.maxLengthEnforcement}
  final MaxLengthEnforcement? maxLengthEnforcement;

  /// {@macro flutter.widgets.editableText.onChanged}
  final ValueChanged<String>? onChanged;

  /// {@macro flutter.widgets.editableText.onEditingComplete}
  final VoidCallback? onEditingComplete;

  /// {@macro flutter.widgets.editableText.onSubmitted}
  ///
  /// See also:
  ///
  ///  * [TextInputAction.next] and [TextInputAction.previous], which
  ///    automatically shift the focus to the next/previous focusable item when
  ///    the user is done editing.
  final ValueChanged<String>? onSubmitted;

  /// {@macro flutter.widgets.editableText.onTapOutside}
  final TapRegionCallback? onTapOutside;

  /// {@macro flutter.widgets.editableText.onTapUpOutside}
  final TapRegionCallback? onTapUpOutside;

  /// {@macro flutter.widgets.editableText.inputFormatters}
  final List<TextInputFormatter>? inputFormatters;

  /// Disables the text field when false.
  ///
  /// Text fields in disabled states have a light grey background and don't
  /// respond to touch events including the [prefix], [suffix] and the clear
  /// button.
  ///
  /// Defaults to true.
  final bool enabled;

  /// {@macro flutter.widgets.editableText.cursorWidth}
  final double cursorWidth;

  /// {@macro flutter.widgets.editableText.cursorHeight}
  final double? cursorHeight;

  /// {@macro flutter.widgets.editableText.cursorRadius}
  final Radius cursorRadius;

  /// {@macro flutter.widgets.editableText.cursorOpacityAnimates}
  final bool cursorOpacityAnimates;

  /// The color to use when painting the cursor.
  ///
  /// Defaults to the [DefaultSelectionStyle.cursorColor]. If that color is
  /// null, it uses the [CupertinoThemeData.primaryColor] of the ambient theme,
  /// which itself defaults to [CupertinoColors.activeBlue] in the light theme
  /// and [CupertinoColors.activeOrange] in the dark theme.
  final Color? cursorColor;

  /// Controls how tall the selection highlight boxes are computed to be.
  ///
  /// See [ui.BoxHeightStyle] for details on available styles.
  final ui.BoxHeightStyle selectionHeightStyle;

  /// Controls how wide the selection highlight boxes are computed to be.
  ///
  /// See [ui.BoxWidthStyle] for details on available styles.
  final ui.BoxWidthStyle selectionWidthStyle;

  /// The appearance of the keyboard.
  ///
  /// This setting is only honored on iOS devices.
  ///
  /// If null, defaults to [Brightness.light].
  final Brightness? keyboardAppearance;

  /// {@macro flutter.widgets.editableText.scrollPadding}
  final EdgeInsets scrollPadding;

  /// {@macro flutter.widgets.editableText.enableInteractiveSelection}
  final bool enableInteractiveSelection;

  /// {@macro flutter.widgets.editableText.selectionControls}
  final TextSelectionControls? selectionControls;

  /// {@macro flutter.widgets.scrollable.dragStartBehavior}
  final DragStartBehavior dragStartBehavior;

  /// {@macro flutter.widgets.editableText.scrollController}
  final ScrollController? scrollController;

  /// {@macro flutter.widgets.editableText.scrollPhysics}
  final ScrollPhysics? scrollPhysics;

  /// {@macro flutter.widgets.editableText.selectionEnabled}
  bool get selectionEnabled => enableInteractiveSelection;

  /// {@macro flutter.material.textfield.onTap}
  final GestureTapCallback? onTap;

  /// {@macro flutter.widgets.editableText.autofillHints}
  /// {@macro flutter.services.AutofillConfiguration.autofillHints}
  final Iterable<String>? autofillHints;

  /// {@macro flutter.material.Material.clipBehavior}
  ///
  /// Defaults to [Clip.hardEdge].
  final Clip clipBehavior;

  /// {@macro flutter.material.textfield.restorationId}
  final String? restorationId;

  /// {@macro flutter.widgets.editableText.scribbleEnabled}
  @Deprecated(
    'Use `stylusHandwritingEnabled` instead. '
    'This feature was deprecated after v3.27.0-0.2.pre.',
  )
  final bool scribbleEnabled;

  /// {@macro flutter.widgets.editableText.stylusHandwritingEnabled}
  final bool stylusHandwritingEnabled;

  /// {@macro flutter.services.TextInputConfiguration.enableIMEPersonalizedLearning}
  final bool enableIMEPersonalizedLearning;

  /// {@macro flutter.widgets.editableText.contentInsertionConfiguration}
  final ContentInsertionConfiguration? contentInsertionConfiguration;

  /// {@macro flutter.widgets.EditableText.contextMenuBuilder}
  ///
  /// If not provided, will build a default menu based on the platform.
  ///
  /// See also:
  ///
  ///  * [CupertinoAdaptiveTextSelectionToolbar], which is built by default.
  final EditableTextContextMenuBuilder? contextMenuBuilder;

  static Widget _defaultContextMenuBuilder(
    BuildContext context,
    EditableTextState editableTextState,
  ) {
    return CupertinoAdaptiveTextSelectionToolbar.editableText(editableTextState: editableTextState);
  }

  /// Configuration for the text field magnifier.
  ///
  /// By default (when this property is set to null), a [CupertinoTextMagnifier]
  /// is used on mobile platforms, and nothing on desktop platforms. To suppress
  /// the magnifier on all platforms, consider passing
  /// [TextMagnifierConfiguration.disabled] explicitly.
  ///
  /// {@macro flutter.widgets.magnifier.intro}
  ///
  /// {@tool dartpad}
  /// This sample demonstrates how to customize the magnifier that this text field uses.
  ///
  /// ** See code in examples/api/lib/widgets/text_magnifier/text_magnifier.0.dart **
  /// {@end-tool}
  final TextMagnifierConfiguration? magnifierConfiguration;

  /// {@macro flutter.widgets.EditableText.spellCheckConfiguration}
  ///
  /// If [SpellCheckConfiguration.misspelledTextStyle] is not specified in this
  /// configuration, then [cupertinoMisspelledTextStyle] is used by default.
  final SpellCheckConfiguration? spellCheckConfiguration;

  /// The [TextStyle] used to indicate misspelled words in the Cupertino style.
  ///
  /// See also:
  ///  * [SpellCheckConfiguration.misspelledTextStyle], the style configured to
  ///    mark misspelled words with.
  ///  * [TextField.materialMisspelledTextStyle], the style configured
  ///    to mark misspelled words with in the Material style.
  static const TextStyle cupertinoMisspelledTextStyle = TextStyle(
    decoration: TextDecoration.underline,
    decorationColor: CupertinoColors.systemRed,
    decorationStyle: TextDecorationStyle.dotted,
  );

  /// The color of the selection highlight when the spell check menu is visible.
  ///
  /// Eyeballed from a screenshot taken on an iPhone 11 running iOS 16.2.
  @visibleForTesting
  static const Color kMisspelledSelectionColor = Color(0x62ff9699);

  /// Default builder for the spell check suggestions toolbar in the Cupertino
  /// style.
  ///
  /// See also:
  ///  * [spellCheckConfiguration], where this is typically specified for
  ///    [CupertinoTextField].
  ///  * [SpellCheckConfiguration.spellCheckSuggestionsToolbarBuilder], the
  ///    parameter for which this is the default value for [CupertinoTextField].
  ///  * [TextField.defaultSpellCheckSuggestionsToolbarBuilder], which is like
  ///    this but specifies the default for [CupertinoTextField].
  @visibleForTesting
  static Widget defaultSpellCheckSuggestionsToolbarBuilder(
    BuildContext context,
    EditableTextState editableTextState,
  ) {
    return CupertinoSpellCheckSuggestionsToolbar.editableText(editableTextState: editableTextState);
  }

  /// {@macro flutter.widgets.undoHistory.controller}
  final UndoHistoryController? undoController;

  @override
  State<CupertinoTextField> createState() => _CupertinoTextFieldState();

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(
      DiagnosticsProperty<TextEditingController>('controller', controller, defaultValue: null),
    );
    properties.add(DiagnosticsProperty<FocusNode>('focusNode', focusNode, defaultValue: null));
    properties.add(
      DiagnosticsProperty<UndoHistoryController>(
        'undoController',
        undoController,
        defaultValue: null,
      ),
    );
    properties.add(DiagnosticsProperty<BoxDecoration>('decoration', decoration));
    properties.add(DiagnosticsProperty<EdgeInsetsGeometry>('padding', padding));
    properties.add(StringProperty('placeholder', placeholder));
    properties.add(DiagnosticsProperty<TextStyle>('placeholderStyle', placeholderStyle));
    properties.add(
      DiagnosticsProperty<OverlayVisibilityMode>('prefix', prefix == null ? null : prefixMode),
    );
    properties.add(
      DiagnosticsProperty<OverlayVisibilityMode>('suffix', suffix == null ? null : suffixMode),
    );
    properties.add(DiagnosticsProperty<OverlayVisibilityMode>('clearButtonMode', clearButtonMode));
    properties.add(
      DiagnosticsProperty<String>('clearButtonSemanticLabel', clearButtonSemanticLabel),
    );
    properties.add(
      DiagnosticsProperty<TextInputType>(
        'keyboardType',
        keyboardType,
        defaultValue: TextInputType.text,
      ),
    );
    properties.add(DiagnosticsProperty<TextStyle>('style', style, defaultValue: null));
    properties.add(DiagnosticsProperty<bool>('autofocus', autofocus, defaultValue: false));
    properties.add(
      DiagnosticsProperty<String>('obscuringCharacter', obscuringCharacter, defaultValue: '•'),
    );
    properties.add(DiagnosticsProperty<bool>('obscureText', obscureText, defaultValue: false));
    properties.add(DiagnosticsProperty<bool>('autocorrect', autocorrect, defaultValue: true));
    properties.add(
      EnumProperty<SmartDashesType>(
        'smartDashesType',
        smartDashesType,
        defaultValue: obscureText ? SmartDashesType.disabled : SmartDashesType.enabled,
      ),
    );
    properties.add(
      EnumProperty<SmartQuotesType>(
        'smartQuotesType',
        smartQuotesType,
        defaultValue: obscureText ? SmartQuotesType.disabled : SmartQuotesType.enabled,
      ),
    );
    properties.add(
      DiagnosticsProperty<bool>('enableSuggestions', enableSuggestions, defaultValue: true),
    );
    properties.add(IntProperty('maxLines', maxLines, defaultValue: 1));
    properties.add(IntProperty('minLines', minLines, defaultValue: null));
    properties.add(DiagnosticsProperty<bool>('expands', expands, defaultValue: false));
    properties.add(IntProperty('maxLength', maxLength, defaultValue: null));
    properties.add(
      EnumProperty<MaxLengthEnforcement>(
        'maxLengthEnforcement',
        maxLengthEnforcement,
        defaultValue: null,
      ),
    );
    properties.add(DoubleProperty('cursorWidth', cursorWidth, defaultValue: 2.0));
    properties.add(DoubleProperty('cursorHeight', cursorHeight, defaultValue: null));
    properties.add(DiagnosticsProperty<Radius>('cursorRadius', cursorRadius, defaultValue: null));
    properties.add(
      DiagnosticsProperty<bool>('cursorOpacityAnimates', cursorOpacityAnimates, defaultValue: true),
    );
    properties.add(createCupertinoColorProperty('cursorColor', cursorColor, defaultValue: null));
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
      DiagnosticsProperty<ScrollController>(
        'scrollController',
        scrollController,
        defaultValue: null,
      ),
    );
    properties.add(
      DiagnosticsProperty<ScrollPhysics>('scrollPhysics', scrollPhysics, defaultValue: null),
    );
    properties.add(EnumProperty<TextAlign>('textAlign', textAlign, defaultValue: TextAlign.start));
    properties.add(
      DiagnosticsProperty<TextAlignVertical>(
        'textAlignVertical',
        textAlignVertical,
        defaultValue: null,
      ),
    );
    properties.add(EnumProperty<TextDirection>('textDirection', textDirection, defaultValue: null));
    properties.add(
      DiagnosticsProperty<Clip>('clipBehavior', clipBehavior, defaultValue: Clip.hardEdge),
    );
    properties.add(
      DiagnosticsProperty<bool>('scribbleEnabled', scribbleEnabled, defaultValue: true),
    );
    properties.add(
      DiagnosticsProperty<bool>(
        'stylusHandwritingEnabled',
        stylusHandwritingEnabled,
        defaultValue: EditableText.defaultStylusHandwritingEnabled,
      ),
    );
    properties.add(
      DiagnosticsProperty<bool>(
        'enableIMEPersonalizedLearning',
        enableIMEPersonalizedLearning,
        defaultValue: true,
      ),
    );
    properties.add(
      DiagnosticsProperty<SpellCheckConfiguration>(
        'spellCheckConfiguration',
        spellCheckConfiguration,
        defaultValue: null,
      ),
    );
    properties.add(
      DiagnosticsProperty<List<String>>(
        'contentCommitMimeTypes',
        contentInsertionConfiguration?.allowedMimeTypes ?? const <String>[],
        defaultValue:
            contentInsertionConfiguration == null
                ? const <String>[]
                : kDefaultContentInsertionMimeTypes,
      ),
    );
  }

  static final TextMagnifierConfiguration _iosMagnifierConfiguration = TextMagnifierConfiguration(
    magnifierBuilder: (
      BuildContext context,
      MagnifierController controller,
      ValueNotifier<MagnifierInfo> magnifierInfo,
    ) {
      switch (defaultTargetPlatform) {
        case TargetPlatform.android:
        case TargetPlatform.iOS:
          return CupertinoTextMagnifier(controller: controller, magnifierInfo: magnifierInfo);
        case TargetPlatform.fuchsia:
        case TargetPlatform.linux:
        case TargetPlatform.macOS:
        case TargetPlatform.windows:
          return null;
      }
    },
  );

  /// Returns a new [SpellCheckConfiguration] where the given configuration has
  /// had any missing values replaced with their defaults for the iOS platform.
  static SpellCheckConfiguration inferIOSSpellCheckConfiguration(
    SpellCheckConfiguration? configuration,
  ) {
    if (configuration == null || configuration == const SpellCheckConfiguration.disabled()) {
      return const SpellCheckConfiguration.disabled();
    }

    return configuration.copyWith(
      misspelledTextStyle:
          configuration.misspelledTextStyle ?? CupertinoTextField.cupertinoMisspelledTextStyle,
      misspelledSelectionColor:
          configuration.misspelledSelectionColor ?? CupertinoTextField.kMisspelledSelectionColor,
      spellCheckSuggestionsToolbarBuilder:
          configuration.spellCheckSuggestionsToolbarBuilder ??
          CupertinoTextField.defaultSpellCheckSuggestionsToolbarBuilder,
    );
  }
}

class _CupertinoTextFieldState extends State<CupertinoTextField>
    with RestorationMixin, AutomaticKeepAliveClientMixin<CupertinoTextField>
    implements TextSelectionGestureDetectorBuilderDelegate, AutofillClient {
  final GlobalKey _clearGlobalKey = GlobalKey();

  RestorableTextEditingController? _controller;
  TextEditingController get _effectiveController => widget.controller ?? _controller!.value;

  FocusNode? _focusNode;
  FocusNode get _effectiveFocusNode => widget.focusNode ?? (_focusNode ??= FocusNode());

  MaxLengthEnforcement get _effectiveMaxLengthEnforcement =>
      widget.maxLengthEnforcement ??
      LengthLimitingTextInputFormatter.getDefaultMaxLengthEnforcement();

  bool _showSelectionHandles = false;

  late _CupertinoTextFieldSelectionGestureDetectorBuilder _selectionGestureDetectorBuilder;

  // API for TextSelectionGestureDetectorBuilderDelegate.
  @override
  bool get forcePressEnabled => true;

  @override
  final GlobalKey<EditableTextState> editableTextKey = GlobalKey<EditableTextState>();

  @override
  bool get selectionEnabled => widget.selectionEnabled;
  // End of API for TextSelectionGestureDetectorBuilderDelegate.

  @override
  void initState() {
    super.initState();
    _selectionGestureDetectorBuilder = _CupertinoTextFieldSelectionGestureDetectorBuilder(
      state: this,
    );
    if (widget.controller == null) {
      _createLocalController();
    }
    _effectiveFocusNode.canRequestFocus = widget.enabled;
    _effectiveFocusNode.addListener(_handleFocusChanged);
  }

  @override
  void didUpdateWidget(CupertinoTextField oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (widget.controller == null && oldWidget.controller != null) {
      _createLocalController(oldWidget.controller!.value);
    } else if (widget.controller != null && oldWidget.controller == null) {
      unregisterFromRestoration(_controller!);
      _controller!.dispose();
      _controller = null;
    }

    if (widget.focusNode != oldWidget.focusNode) {
      (oldWidget.focusNode ?? _focusNode)?.removeListener(_handleFocusChanged);
      (widget.focusNode ?? _focusNode)?.addListener(_handleFocusChanged);
    }
    _effectiveFocusNode.canRequestFocus = widget.enabled;
  }

  @override
  void restoreState(RestorationBucket? oldBucket, bool initialRestore) {
    if (_controller != null) {
      _registerController();
    }
  }

  void _registerController() {
    assert(_controller != null);
    registerForRestoration(_controller!, 'controller');
    _controller!.value.addListener(updateKeepAlive);
  }

  void _createLocalController([TextEditingValue? value]) {
    assert(_controller == null);
    _controller =
        value == null
            ? RestorableTextEditingController()
            : RestorableTextEditingController.fromValue(value);
    if (!restorePending) {
      _registerController();
    }
  }

  @override
  String? get restorationId => widget.restorationId;

  @override
  void dispose() {
    _effectiveFocusNode.removeListener(_handleFocusChanged);
    _focusNode?.dispose();
    _controller?.dispose();
    super.dispose();
  }

  EditableTextState get _editableText => editableTextKey.currentState!;

  void _requestKeyboard() {
    _editableText.requestKeyboard();
  }

  void _handleFocusChanged() {
    setState(() {
      // Rebuild the widget on focus change to show/hide the text selection
      // highlight.
    });
  }

  bool _shouldShowSelectionHandles(SelectionChangedCause? cause) {
    // When the text field is activated by something that doesn't trigger the
    // selection overlay, we shouldn't show the handles either.
    if (!_selectionGestureDetectorBuilder.shouldShowSelectionToolbar) {
      return false;
    }

    // On iOS, we don't show handles when the selection is collapsed.
    if (_effectiveController.selection.isCollapsed) {
      return false;
    }

    if (cause == SelectionChangedCause.keyboard) {
      return false;
    }

    if (cause == SelectionChangedCause.scribble) {
      return true;
    }

    if (_effectiveController.text.isNotEmpty) {
      return true;
    }

    return false;
  }

  void _handleSelectionChanged(TextSelection selection, SelectionChangedCause? cause) {
    final bool willShowSelectionHandles = _shouldShowSelectionHandles(cause);
    if (willShowSelectionHandles != _showSelectionHandles) {
      setState(() {
        _showSelectionHandles = willShowSelectionHandles;
      });
    }

    switch (defaultTargetPlatform) {
      case TargetPlatform.iOS:
      case TargetPlatform.macOS:
      case TargetPlatform.linux:
      case TargetPlatform.windows:
      case TargetPlatform.fuchsia:
      case TargetPlatform.android:
        if (cause == SelectionChangedCause.longPress) {
          _editableText.bringIntoView(selection.extent);
        }
    }

    switch (defaultTargetPlatform) {
      case TargetPlatform.iOS:
      case TargetPlatform.fuchsia:
      case TargetPlatform.android:
        break;
      case TargetPlatform.macOS:
      case TargetPlatform.linux:
      case TargetPlatform.windows:
        if (cause == SelectionChangedCause.drag) {
          _editableText.hideToolbar();
        }
    }
  }

  @override
  bool get wantKeepAlive => _controller?.value.text.isNotEmpty ?? false;

  static bool _shouldShowAttachment({
    required OverlayVisibilityMode attachment,
    required bool hasText,
  }) {
    return switch (attachment) {
      OverlayVisibilityMode.never => false,
      OverlayVisibilityMode.always => true,
      OverlayVisibilityMode.editing => hasText,
      OverlayVisibilityMode.notEditing => !hasText,
    };
  }

  // True if any surrounding decoration widgets will be shown.
  bool get _hasDecoration {
    return widget.placeholder != null ||
        widget.clearButtonMode != OverlayVisibilityMode.never ||
        widget.prefix != null ||
        widget.suffix != null;
  }

  // Provide default behavior if widget.textAlignVertical is not set.
  // CupertinoTextField has top alignment by default, unless it has decoration
  // like a prefix or suffix, in which case it's aligned to the center.
  TextAlignVertical get _textAlignVertical {
    if (widget.textAlignVertical != null) {
      return widget.textAlignVertical!;
    }
    return _hasDecoration ? TextAlignVertical.center : TextAlignVertical.top;
  }

  void _onClearButtonTapped() {
    final bool hadText = _effectiveController.text.isNotEmpty;
    _effectiveController.clear();
    if (hadText) {
      // Tapping the clear button is also considered a "user initiated" change
      // (instead of a programmatical one), so call `onChanged` if the text
      // changed as a result.
      widget.onChanged?.call(_effectiveController.text);
    }
  }

  Widget _buildClearButton() {
    final String clearLabel =
        widget.clearButtonSemanticLabel ?? CupertinoLocalizations.of(context).clearButtonLabel;

    return Semantics(
      button: true,
      label: clearLabel,
      child: GestureDetector(
        key: _clearGlobalKey,
        onTap: widget.enabled ? _onClearButtonTapped : null,
        child: Padding(
          padding: const EdgeInsets.symmetric(horizontal: 6.0),
          child: Icon(
            CupertinoIcons.clear_thick_circled,
            size: 18.0,
            color: CupertinoDynamicColor.resolve(_kClearButtonColor, context),
          ),
        ),
      ),
    );
  }

  Widget _addTextDependentAttachments(
    Widget editableText,
    TextStyle textStyle,
    TextStyle placeholderStyle,
  ) {
    // If there are no surrounding widgets, just return the core editable text
    // part.
    if (!_hasDecoration) {
      return editableText;
    }

    // Otherwise, listen to the current state of the text entry.
    return ValueListenableBuilder<TextEditingValue>(
      valueListenable: _effectiveController,
      child: editableText,
      builder: (BuildContext context, TextEditingValue text, Widget? child) {
        final bool hasText = text.text.isNotEmpty;
        final String? placeholderText = widget.placeholder;
        final Widget? placeholder =
            placeholderText == null
                ? null
                // Make the placeholder invisible when hasText is true.
                : Visibility(
                  maintainAnimation: true,
                  maintainSize: true,
                  maintainState: true,
                  visible: !hasText,
                  child: SizedBox(
                    width: double.infinity,
                    child: Padding(
                      padding: widget.padding,
                      child: Text(
                        placeholderText,
                        // This is to make sure the text field is always tall enough
                        // to accommodate the first line of the placeholder, so the
                        // text does not shrink vertically as you type (however in
                        // rare circumstances, the height may still change when
                        // there's no placeholder text).
                        maxLines: hasText ? 1 : widget.maxLines,
                        overflow: placeholderStyle.overflow,
                        style: placeholderStyle,
                        textAlign: widget.textAlign,
                      ),
                    ),
                  ),
                );

        final Widget? prefixWidget =
            _shouldShowAttachment(attachment: widget.prefixMode, hasText: hasText)
                ? widget.prefix
                : null;

        // Show user specified suffix if applicable and fall back to clear button.
        final bool showUserSuffix = _shouldShowAttachment(
          attachment: widget.suffixMode,
          hasText: hasText,
        );
        final bool showClearButton = _shouldShowAttachment(
          attachment: widget.clearButtonMode,
          hasText: hasText,
        );
        final Widget? suffixWidget = switch ((showUserSuffix, showClearButton)) {
          (false, false) => null,
          (true, false) => widget.suffix,
          (true, true) => widget.suffix ?? _buildClearButton(),
          (false, true) => _buildClearButton(),
        };
        return Row(
          crossAxisAlignment: widget.crossAxisAlignment,
          children: <Widget>[
            // Insert a prefix at the front if the prefix visibility mode matches
            // the current text state.
            if (prefixWidget != null) prefixWidget,
            // In the middle part, stack the placeholder on top of the main EditableText
            // if needed.
            Expanded(
              child: Stack(
                // Ideally this should be baseline aligned. However that comes at
                // the cost of the ability to compute the intrinsic dimensions of
                // this widget.
                // See also https://github.com/flutter/flutter/issues/13715.
                alignment: AlignmentDirectional.center,
                textDirection: widget.textDirection,
                children: <Widget>[if (placeholder != null) placeholder, editableText],
              ),
            ),
            if (suffixWidget != null) suffixWidget,
          ],
        );
      },
    );
  }

  // AutofillClient implementation start.
  @override
  String get autofillId => _editableText.autofillId;

  @override
  void autofill(TextEditingValue newEditingValue) => _editableText.autofill(newEditingValue);

  @override
  TextInputConfiguration get textInputConfiguration {
    final List<String>? autofillHints = widget.autofillHints?.toList(growable: false);
    final AutofillConfiguration autofillConfiguration =
        autofillHints != null
            ? AutofillConfiguration(
              uniqueIdentifier: autofillId,
              autofillHints: autofillHints,
              currentEditingValue: _effectiveController.value,
              hintText: widget.placeholder,
            )
            : AutofillConfiguration.disabled;

    return _editableText.textInputConfiguration.copyWith(
      autofillConfiguration: autofillConfiguration,
    );
  }
  // AutofillClient implementation end.

  @override
  Widget build(BuildContext context) {
    super.build(context); // See AutomaticKeepAliveClientMixin.
    assert(debugCheckHasDirectionality(context));
    final TextEditingController controller = _effectiveController;

    TextSelectionControls? textSelectionControls = widget.selectionControls;
    VoidCallback? handleDidGainAccessibilityFocus;
    VoidCallback? handleDidLoseAccessibilityFocus;
    switch (defaultTargetPlatform) {
      case TargetPlatform.iOS:
      case TargetPlatform.android:
      case TargetPlatform.fuchsia:
        textSelectionControls ??= cupertinoTextSelectionHandleControls;
      case TargetPlatform.linux:
      case TargetPlatform.macOS:
      case TargetPlatform.windows:
        textSelectionControls ??= cupertinoDesktopTextSelectionHandleControls;
        handleDidGainAccessibilityFocus = () {
          // Automatically activate the TextField when it receives accessibility focus.
          if (!_effectiveFocusNode.hasFocus && _effectiveFocusNode.canRequestFocus) {
            _effectiveFocusNode.requestFocus();
          }
        };
        handleDidLoseAccessibilityFocus = () {
          _effectiveFocusNode.unfocus();
        };
    }

    final bool enabled = widget.enabled;
    final Offset cursorOffset = Offset(
      _iOSHorizontalCursorOffsetPixels / MediaQuery.devicePixelRatioOf(context),
      0,
    );
    final List<TextInputFormatter> formatters = <TextInputFormatter>[
      ...?widget.inputFormatters,
      if (widget.maxLength != null)
        LengthLimitingTextInputFormatter(
          widget.maxLength,
          maxLengthEnforcement: _effectiveMaxLengthEnforcement,
        ),
    ];
    final CupertinoThemeData themeData = CupertinoTheme.of(context);

    final TextStyle? resolvedStyle = widget.style?.copyWith(
      color: CupertinoDynamicColor.maybeResolve(widget.style?.color, context),
      backgroundColor: CupertinoDynamicColor.maybeResolve(widget.style?.backgroundColor, context),
    );

    final TextStyle textStyle = themeData.textTheme.textStyle.merge(resolvedStyle);

    final TextStyle? resolvedPlaceholderStyle = widget.placeholderStyle?.copyWith(
      color: CupertinoDynamicColor.maybeResolve(widget.placeholderStyle?.color, context),
      backgroundColor: CupertinoDynamicColor.maybeResolve(
        widget.placeholderStyle?.backgroundColor,
        context,
      ),
    );

    final TextStyle placeholderStyle = textStyle.merge(resolvedPlaceholderStyle);

    final Brightness keyboardAppearance =
        widget.keyboardAppearance ?? CupertinoTheme.brightnessOf(context);
    final Color cursorColor =
        CupertinoDynamicColor.maybeResolve(
          widget.cursorColor ?? DefaultSelectionStyle.of(context).cursorColor,
          context,
        ) ??
        themeData.primaryColor;

    final Color disabledColor = CupertinoDynamicColor.resolve(_kDisabledBackground, context);

    final Color? decorationColor = CupertinoDynamicColor.maybeResolve(
      widget.decoration?.color,
      context,
    );

    final BoxBorder? border = widget.decoration?.border;
    Border? resolvedBorder = border as Border?;
    if (border is Border) {
      BorderSide resolveBorderSide(BorderSide side) {
        return side == BorderSide.none
            ? side
            : side.copyWith(color: CupertinoDynamicColor.resolve(side.color, context));
      }

      resolvedBorder =
          border.runtimeType != Border
              ? border
              : Border(
                top: resolveBorderSide(border.top),
                left: resolveBorderSide(border.left),
                bottom: resolveBorderSide(border.bottom),
                right: resolveBorderSide(border.right),
              );
    }

    // Use the default disabled color only if the box decoration was not set.
    final BoxDecoration? effectiveDecoration = widget.decoration?.copyWith(
      border: resolvedBorder,
      color:
          enabled
              ? decorationColor
              : (widget.decoration == _kDefaultRoundedBorderDecoration
                  ? disabledColor
                  : widget.decoration?.color),
    );

    final Color selectionColor =
        CupertinoDynamicColor.maybeResolve(
          DefaultSelectionStyle.of(context).selectionColor,
          context,
        ) ??
        CupertinoTheme.of(context).primaryColor.withOpacity(0.2);

    // Set configuration as disabled if not otherwise specified. If specified,
    // ensure that configuration uses Cupertino text style for misspelled words
    // unless a custom style is specified.
    final SpellCheckConfiguration spellCheckConfiguration =
        CupertinoTextField.inferIOSSpellCheckConfiguration(widget.spellCheckConfiguration);

    final Widget paddedEditable = Padding(
      padding: widget.padding,
      child: RepaintBoundary(
        child: UnmanagedRestorationScope(
          bucket: bucket,
          child: EditableText(
            key: editableTextKey,
            controller: controller,
            undoController: widget.undoController,
            readOnly: widget.readOnly || !enabled,
            toolbarOptions: widget.toolbarOptions,
            showCursor: widget.showCursor,
            showSelectionHandles: _showSelectionHandles,
            focusNode: _effectiveFocusNode,
            keyboardType: widget.keyboardType,
            textInputAction: widget.textInputAction,
            textCapitalization: widget.textCapitalization,
            style: textStyle,
            strutStyle: widget.strutStyle,
            textAlign: widget.textAlign,
            textDirection: widget.textDirection,
            autofocus: widget.autofocus,
            obscuringCharacter: widget.obscuringCharacter,
            obscureText: widget.obscureText,
            autocorrect: widget.autocorrect,
            smartDashesType: widget.smartDashesType,
            smartQuotesType: widget.smartQuotesType,
            enableSuggestions: widget.enableSuggestions,
            maxLines: widget.maxLines,
            minLines: widget.minLines,
            expands: widget.expands,
            magnifierConfiguration:
                widget.magnifierConfiguration ?? CupertinoTextField._iosMagnifierConfiguration,
            // Only show the selection highlight when the text field is focused.
            selectionColor: _effectiveFocusNode.hasFocus ? selectionColor : null,
            selectionControls: widget.selectionEnabled ? textSelectionControls : null,
            groupId: widget.groupId,
            onChanged: widget.onChanged,
            onSelectionChanged: _handleSelectionChanged,
            onEditingComplete: widget.onEditingComplete,
            onSubmitted: widget.onSubmitted,
            onTapOutside: widget.onTapOutside,
            inputFormatters: formatters,
            rendererIgnoresPointer: true,
            cursorWidth: widget.cursorWidth,
            cursorHeight: widget.cursorHeight,
            cursorRadius: widget.cursorRadius,
            cursorColor: cursorColor,
            cursorOpacityAnimates: widget.cursorOpacityAnimates,
            cursorOffset: cursorOffset,
            paintCursorAboveText: true,
            autocorrectionTextRectColor: selectionColor,
            backgroundCursorColor: CupertinoDynamicColor.resolve(
              CupertinoColors.inactiveGray,
              context,
            ),
            selectionHeightStyle: widget.selectionHeightStyle,
            selectionWidthStyle: widget.selectionWidthStyle,
            scrollPadding: widget.scrollPadding,
            keyboardAppearance: keyboardAppearance,
            dragStartBehavior: widget.dragStartBehavior,
            scrollController: widget.scrollController,
            scrollPhysics: widget.scrollPhysics,
            enableInteractiveSelection: widget.enableInteractiveSelection,
            autofillClient: this,
            clipBehavior: widget.clipBehavior,
            restorationId: 'editable',
            scribbleEnabled: widget.scribbleEnabled,
            stylusHandwritingEnabled: widget.stylusHandwritingEnabled,
            enableIMEPersonalizedLearning: widget.enableIMEPersonalizedLearning,
            contentInsertionConfiguration: widget.contentInsertionConfiguration,
            contextMenuBuilder: widget.contextMenuBuilder,
            spellCheckConfiguration: spellCheckConfiguration,
          ),
        ),
      ),
    );

    return Semantics(
      enabled: enabled,
      onTap:
          !enabled || widget.readOnly
              ? null
              : () {
                if (!controller.selection.isValid) {
                  controller.selection = TextSelection.collapsed(offset: controller.text.length);
                }
                _requestKeyboard();
              },
      onDidGainAccessibilityFocus: handleDidGainAccessibilityFocus,
      onDidLoseAccessibilityFocus: handleDidLoseAccessibilityFocus,
      onFocus:
          enabled
              ? () {
                assert(
                  _effectiveFocusNode.canRequestFocus,
                  'Received SemanticsAction.focus from the engine. However, the FocusNode '
                  'of this text field cannot gain focus. This likely indicates a bug. '
                  'If this text field cannot be focused (e.g. because it is not '
                  'enabled), then its corresponding semantics node must be configured '
                  'such that the assistive technology cannot request focus on it.',
                );

                if (_effectiveFocusNode.canRequestFocus && !_effectiveFocusNode.hasFocus) {
                  _effectiveFocusNode.requestFocus();
                } else if (!widget.readOnly) {
                  // If the platform requested focus, that means that previously the
                  // platform believed that the text field did not have focus (even
                  // though Flutter's widget system believed otherwise). This likely
                  // means that the on-screen keyboard is hidden, or more generally,
                  // there is no current editing session in this field. To correct
                  // that, keyboard must be requested.
                  //
                  // A concrete scenario where this can happen is when the user
                  // dismisses the keyboard on the web. The editing session is
                  // closed by the engine, but the text field widget stays focused
                  // in the framework.
                  _requestKeyboard();
                }
              }
              : null,
      child: TextFieldTapRegion(
        child: IgnorePointer(
          ignoring: !enabled,
          child: Container(
            decoration: effectiveDecoration,
            color: !enabled && effectiveDecoration == null ? disabledColor : null,
            child: _selectionGestureDetectorBuilder.buildGestureDetector(
              behavior: HitTestBehavior.translucent,
              child: Align(
                alignment: Alignment(-1.0, _textAlignVertical.y),
                widthFactor: 1.0,
                heightFactor: 1.0,
                child: _addTextDependentAttachments(paddedEditable, textStyle, placeholderStyle),
              ),
            ),
          ),
        ),
      ),
    );
  }
}
