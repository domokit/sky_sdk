// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' as ui show BoxHeightStyle, BoxWidthStyle;

import 'package:flutter/gestures.dart';
import 'package:flutter/services.dart';
import 'package:flutter/widgets.dart';

import 'adaptive_text_selection_toolbar.dart';
import 'input_decorator.dart';
import 'material_state.dart';
import 'text_field.dart';
import 'theme.dart';

export 'package:flutter/services.dart' show SmartDashesType, SmartQuotesType;

/// A [FormField] that contains a [TextField].
///
/// This is a convenience widget that wraps a [TextField] widget in a
/// [FormField].
///
/// A [Form] ancestor is not required. The [Form] allows one to
/// save, reset, or validate multiple fields at once. To use without a [Form],
/// pass a `GlobalKey<FormFieldState>` (see [GlobalKey]) to the constructor and use
/// [GlobalKey.currentState] to save or reset the form field.
///
/// When a [controller] is specified, its [TextEditingController.text]
/// defines the [initialValue]. If this [FormField] is part of a scrolling
/// container that lazily constructs its children, like a [ListView] or a
/// [CustomScrollView], then a [controller] should be specified.
/// The controller's lifetime should be managed by a stateful widget ancestor
/// of the scrolling container.
///
/// If a [controller] is not specified, [initialValue] can be used to give
/// the automatically generated controller an initial value.
///
/// {@macro flutter.material.textfield.wantKeepAlive}
///
/// Remember to call [TextEditingController.dispose] of the [TextEditingController]
/// when it is no longer needed. This will ensure any resources used by the object
/// are discarded.
///
/// By default, `decoration` will apply the [ThemeData.inputDecorationTheme] for
/// the current context to the [InputDecoration], see
/// [InputDecoration.applyDefaults].
///
/// For a documentation about the various parameters, see [TextField].
///
/// {@tool snippet}
///
/// Creates a [TextFormField] with an [InputDecoration] and validator function.
///
/// ![If the user enters valid text, the TextField appears normally without any warnings to the user](https://flutter.github.io/assets-for-api-docs/assets/material/text_form_field.png)
///
/// ![If the user enters invalid text, the error message returned from the validator function is displayed in dark red underneath the input](https://flutter.github.io/assets-for-api-docs/assets/material/text_form_field_error.png)
///
/// ```dart
/// TextFormField(
///   decoration: const InputDecoration(
///     icon: Icon(Icons.person),
///     hintText: 'What do people call you?',
///     labelText: 'Name *',
///   ),
///   onSaved: (String? value) {
///     // This optional block of code can be used to run
///     // code when the user saves the form.
///   },
///   validator: (String? value) {
///     return (value != null && value.contains('@')) ? 'Do not use the @ char.' : null;
///   },
/// )
/// ```
/// {@end-tool}
///
/// {@tool dartpad}
/// This example shows how to move the focus to the next field when the user
/// presses the SPACE key.
///
/// ** See code in examples/api/lib/material/text_form_field/text_form_field.1.dart **
/// {@end-tool}
///
/// {@tool dartpad}
/// This example shows how to force an error text to the field after making
/// an asynchronous call.
///
/// ** See code in examples/api/lib/material/text_form_field/text_form_field.2.dart **
/// {@end-tool}
///
/// See also:
///
///  * <https://material.io/design/components/text-fields.html>
///  * [TextField], which is the underlying text field without the [Form]
///    integration.
///  * [InputDecorator], which shows the labels and other visual elements that
///    surround the actual text editing widget.
///  * Learn how to use a [TextEditingController] in one of our [cookbook recipes](https://docs.flutter.dev/cookbook/forms/text-field-changes#2-use-a-texteditingcontroller).
class TextFormField extends FormField<String> {
  /// Creates a [FormField] that contains a [TextField].
  ///
  /// When a [controller] is specified, [initialValue] must be null (the
  /// default). If [controller] is null, then a [TextEditingController]
  /// will be constructed automatically and its `text` will be initialized
  /// to [initialValue] or the empty string.
  ///
  /// For documentation about the various parameters, see the [TextField] class
  /// and [TextField.new], the constructor.
  TextFormField({
    super.key,
    this.groupId = EditableText,
    this.controller,
    String? initialValue,
    FocusNode? focusNode,
    super.forceErrorText,
    InputDecoration? decoration = const InputDecoration(),
    TextInputType? keyboardType,
    TextCapitalization textCapitalization = TextCapitalization.none,
    TextInputAction? textInputAction,
    TextStyle? style,
    StrutStyle? strutStyle,
    TextDirection? textDirection,
    TextAlign textAlign = TextAlign.start,
    TextAlignVertical? textAlignVertical,
    bool autofocus = false,
    bool readOnly = false,
    @Deprecated(
      'Use `contextMenuBuilder` instead. '
      'This feature was deprecated after v3.3.0-0.5.pre.',
    )
    ToolbarOptions? toolbarOptions,
    bool? showCursor,
    String obscuringCharacter = '•',
    bool obscureText = false,
    bool autocorrect = true,
    SmartDashesType? smartDashesType,
    SmartQuotesType? smartQuotesType,
    bool enableSuggestions = true,
    MaxLengthEnforcement? maxLengthEnforcement,
    int? maxLines = 1,
    int? minLines,
    bool expands = false,
    int? maxLength,
    this.onChanged,
    GestureTapCallback? onTap,
    bool onTapAlwaysCalled = false,
    TapRegionCallback? onTapOutside,
    TapRegionUpCallback? onTapUpOutside,
    VoidCallback? onEditingComplete,
    ValueChanged<String>? onFieldSubmitted,
    super.onSaved,
    super.validator,
    List<TextInputFormatter>? inputFormatters,
    bool? enabled,
    bool? ignorePointers,
    double cursorWidth = 2.0,
    double? cursorHeight,
    Radius? cursorRadius,
    Color? cursorColor,
    Color? cursorErrorColor,
    Brightness? keyboardAppearance,
    EdgeInsets scrollPadding = const EdgeInsets.all(20.0),
    bool? enableInteractiveSelection,
    TextSelectionControls? selectionControls,
    InputCounterWidgetBuilder? buildCounter,
    ScrollPhysics? scrollPhysics,
    Iterable<String>? autofillHints,
    AutovalidateMode? autovalidateMode,
    ScrollController? scrollController,
    super.restorationId,
    bool enableIMEPersonalizedLearning = true,
    MouseCursor? mouseCursor,
    EditableTextContextMenuBuilder? contextMenuBuilder = _defaultContextMenuBuilder,
    SpellCheckConfiguration? spellCheckConfiguration,
    TextMagnifierConfiguration? magnifierConfiguration,
    UndoHistoryController? undoController,
    AppPrivateCommandCallback? onAppPrivateCommand,
    bool? cursorOpacityAnimates,
    ui.BoxHeightStyle selectionHeightStyle = ui.BoxHeightStyle.tight,
    ui.BoxWidthStyle selectionWidthStyle = ui.BoxWidthStyle.tight,
    DragStartBehavior dragStartBehavior = DragStartBehavior.start,
    ContentInsertionConfiguration? contentInsertionConfiguration,
    MaterialStatesController? statesController,
    Clip clipBehavior = Clip.hardEdge,
    @Deprecated(
      'Use `stylusHandwritingEnabled` instead. '
      'This feature was deprecated after v3.27.0-0.2.pre.',
    )
    bool scribbleEnabled = true,
    bool stylusHandwritingEnabled = EditableText.defaultStylusHandwritingEnabled,
    bool canRequestFocus = true,
  }) : assert(initialValue == null || controller == null),
       assert(obscuringCharacter.length == 1),
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
       assert(maxLength == null || maxLength == TextField.noMaxLength || maxLength > 0),
       super(
         initialValue: controller != null ? controller.text : (initialValue ?? ''),
         enabled: enabled ?? decoration?.enabled ?? true,
         autovalidateMode: autovalidateMode ?? AutovalidateMode.disabled,
         builder: (FormFieldState<String> field) {
           final _TextFormFieldState state = field as _TextFormFieldState;
           final InputDecoration effectiveDecoration = (decoration ?? const InputDecoration())
               .applyDefaults(Theme.of(field.context).inputDecorationTheme);
           void onChangedHandler(String value) {
             field.didChange(value);
             onChanged?.call(value);
           }

           return UnmanagedRestorationScope(
             bucket: field.bucket,
             child: TextField(
               groupId: groupId,
               restorationId: restorationId,
               controller: state._effectiveController,
               focusNode: focusNode,
               decoration: effectiveDecoration.copyWith(errorText: field.errorText),
               keyboardType: keyboardType,
               textInputAction: textInputAction,
               style: style,
               strutStyle: strutStyle,
               textAlign: textAlign,
               textAlignVertical: textAlignVertical,
               textDirection: textDirection,
               textCapitalization: textCapitalization,
               autofocus: autofocus,
               statesController: statesController,
               toolbarOptions: toolbarOptions,
               readOnly: readOnly,
               showCursor: showCursor,
               obscuringCharacter: obscuringCharacter,
               obscureText: obscureText,
               autocorrect: autocorrect,
               smartDashesType:
                   smartDashesType ??
                   (obscureText ? SmartDashesType.disabled : SmartDashesType.enabled),
               smartQuotesType:
                   smartQuotesType ??
                   (obscureText ? SmartQuotesType.disabled : SmartQuotesType.enabled),
               enableSuggestions: enableSuggestions,
               maxLengthEnforcement: maxLengthEnforcement,
               maxLines: maxLines,
               minLines: minLines,
               expands: expands,
               maxLength: maxLength,
               onChanged: onChangedHandler,
               onTap: onTap,
               onTapAlwaysCalled: onTapAlwaysCalled,
               onTapOutside: onTapOutside,
               onTapUpOutside: onTapUpOutside,
               onEditingComplete: onEditingComplete,
               onSubmitted: onFieldSubmitted,
               inputFormatters: inputFormatters,
               enabled: enabled ?? decoration?.enabled ?? true,
               ignorePointers: ignorePointers,
               cursorWidth: cursorWidth,
               cursorHeight: cursorHeight,
               cursorRadius: cursorRadius,
               cursorColor: cursorColor,
               cursorErrorColor: cursorErrorColor,
               scrollPadding: scrollPadding,
               scrollPhysics: scrollPhysics,
               keyboardAppearance: keyboardAppearance,
               enableInteractiveSelection:
                   enableInteractiveSelection ?? (!obscureText || !readOnly),
               selectionControls: selectionControls,
               buildCounter: buildCounter,
               autofillHints: autofillHints,
               scrollController: scrollController,
               enableIMEPersonalizedLearning: enableIMEPersonalizedLearning,
               mouseCursor: mouseCursor,
               contextMenuBuilder: contextMenuBuilder,
               spellCheckConfiguration: spellCheckConfiguration,
               magnifierConfiguration: magnifierConfiguration,
               undoController: undoController,
               onAppPrivateCommand: onAppPrivateCommand,
               cursorOpacityAnimates: cursorOpacityAnimates,
               selectionHeightStyle: selectionHeightStyle,
               selectionWidthStyle: selectionWidthStyle,
               dragStartBehavior: dragStartBehavior,
               contentInsertionConfiguration: contentInsertionConfiguration,
               clipBehavior: clipBehavior,
               scribbleEnabled: scribbleEnabled,
               stylusHandwritingEnabled: stylusHandwritingEnabled,
               canRequestFocus: canRequestFocus,
             ),
           );
         },
       );

  /// Controls the text being edited.
  ///
  /// If null, this widget will create its own [TextEditingController] and
  /// initialize its [TextEditingController.text] with [initialValue].
  final TextEditingController? controller;

  /// {@macro flutter.widgets.editableText.groupId}
  final Object groupId;

  /// {@template flutter.material.TextFormField.onChanged}
  /// Called when the user initiates a change to the TextField's
  /// value: when they have inserted or deleted text or reset the form.
  /// {@endtemplate}
  final ValueChanged<String>? onChanged;

  static Widget _defaultContextMenuBuilder(
    BuildContext context,
    EditableTextState editableTextState,
  ) {
    return AdaptiveTextSelectionToolbar.editableText(editableTextState: editableTextState);
  }

  @override
  FormFieldState<String> createState() => _TextFormFieldState();
}

class _TextFormFieldState extends FormFieldState<String> {
  RestorableTextEditingController? _controller;

  TextEditingController get _effectiveController => _textFormField.controller ?? _controller!.value;

  TextFormField get _textFormField => super.widget as TextFormField;

  @override
  void restoreState(RestorationBucket? oldBucket, bool initialRestore) {
    super.restoreState(oldBucket, initialRestore);
    if (_controller != null) {
      _registerController();
    }
    // Make sure to update the internal [FormFieldState] value to sync up with
    // text editing controller value.
    setValue(_effectiveController.text);
  }

  void _registerController() {
    assert(_controller != null);
    registerForRestoration(_controller!, 'controller');
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
  void initState() {
    super.initState();
    if (_textFormField.controller == null) {
      _createLocalController(
        widget.initialValue != null ? TextEditingValue(text: widget.initialValue!) : null,
      );
    } else {
      _textFormField.controller!.addListener(_handleControllerChanged);
    }
  }

  @override
  void didUpdateWidget(TextFormField oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (_textFormField.controller != oldWidget.controller) {
      oldWidget.controller?.removeListener(_handleControllerChanged);
      _textFormField.controller?.addListener(_handleControllerChanged);

      if (oldWidget.controller != null && _textFormField.controller == null) {
        _createLocalController(oldWidget.controller!.value);
      }

      if (_textFormField.controller != null) {
        setValue(_textFormField.controller!.text);
        if (oldWidget.controller == null) {
          unregisterFromRestoration(_controller!);
          _controller!.dispose();
          _controller = null;
        }
      }
    }
  }

  @override
  void dispose() {
    _textFormField.controller?.removeListener(_handleControllerChanged);
    _controller?.dispose();
    super.dispose();
  }

  @override
  void didChange(String? value) {
    super.didChange(value);

    if (_effectiveController.text != value) {
      _effectiveController.text = value ?? '';
    }
  }

  @override
  void reset() {
    // Set the controller value before calling super.reset() to let
    // _handleControllerChanged suppress the change.
    _effectiveController.text = widget.initialValue ?? '';
    super.reset();
    _textFormField.onChanged?.call(_effectiveController.text);
  }

  void _handleControllerChanged() {
    // Suppress changes that originated from within this class.
    //
    // In the case where a controller has been passed in to this widget, we
    // register this change listener. In these cases, we'll also receive change
    // notifications for changes originating from within this class -- for
    // example, the reset() method. In such cases, the FormField value will
    // already have been set.
    if (_effectiveController.text != value) {
      didChange(_effectiveController.text);
    }
  }
}
