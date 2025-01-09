// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/services.dart';
import 'package:flutter/widgets.dart';

import 'adaptive_text_selection_toolbar.dart';
import 'colors.dart';
import 'form_row.dart';
import 'text_field.dart';

/// Creates a [CupertinoFormRow] containing a [FormField] that wraps
/// a [CupertinoTextField].
///
/// A [Form] ancestor is not required. The [Form] allows one to
/// save, reset, or validate multiple fields at once. To use without a [Form],
/// pass a [GlobalKey] to the constructor and use [GlobalKey.currentState] to
/// save or reset the form field.
///
/// When a [controller] is specified, its [TextEditingController.text]
/// defines the [initialValue]. If this [FormField] is part of a scrolling
/// container that lazily constructs its children, like a [ListView] or a
/// [CustomScrollView], then a [controller] should be specified.
/// The controller's lifetime should be managed by a stateful widget ancestor
/// of the scrolling container.
///
/// The [prefix] parameter is displayed at the start of the row. Standard iOS
/// guidelines encourage passing a [Text] widget to [prefix] to detail the
/// nature of the input.
///
/// The [padding] parameter is used to pad the contents of the row. It is
/// directly passed to [CupertinoFormRow]. If the [padding]
/// parameter is null, [CupertinoFormRow] constructs its own default
/// padding (which is the standard form row padding in iOS.) If no edge
/// insets are intended, explicitly pass [EdgeInsets.zero] to [padding].
///
/// If a [controller] is not specified, [initialValue] can be used to give
/// the automatically generated controller an initial value.
///
/// Consider calling [TextEditingController.dispose] of the [controller], if one
/// is specified, when it is no longer needed. This will ensure we discard any
/// resources used by the object.
///
/// For documentation about the various parameters, see the
/// [CupertinoTextField] class and [CupertinoTextField.borderless],
/// the constructor.
///
/// {@tool snippet}
///
/// Creates a [CupertinoTextFormFieldRow] with a leading text and validator
/// function.
///
/// If the user enters valid text, the CupertinoTextField appears normally
/// without any warnings to the user.
///
/// If the user enters invalid text, the error message returned from the
/// validator function is displayed in dark red underneath the input.
///
/// ```dart
/// CupertinoTextFormFieldRow(
///   prefix: const Text('Username'),
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
/// ** See code in examples/api/lib/cupertino/text_form_field_row/cupertino_text_form_field_row.1.dart **
/// {@end-tool}
class CupertinoTextFormFieldRow extends FormField<String> {
  /// Creates a [CupertinoFormRow] containing a [FormField] that wraps
  /// a [CupertinoTextField].
  ///
  /// When a [controller] is specified, [initialValue] must be null (the
  /// default). If [controller] is null, then a [TextEditingController]
  /// will be constructed automatically and its `text` will be initialized
  /// to [initialValue] or the empty string.
  ///
  /// The [prefix] parameter is displayed at the start of the row. Standard iOS
  /// guidelines encourage passing a [Text] widget to [prefix] to detail the
  /// nature of the input.
  ///
  /// The [padding] parameter is used to pad the contents of the row. It is
  /// directly passed to [CupertinoFormRow]. If the [padding]
  /// parameter is null, [CupertinoFormRow] constructs its own default
  /// padding (which is the standard form row padding in iOS.) If no edge
  /// insets are intended, explicitly pass [EdgeInsets.zero] to [padding].
  ///
  /// For documentation about the various parameters, see the
  /// [CupertinoTextField] class and [CupertinoTextField.borderless],
  /// the constructor.
  CupertinoTextFormFieldRow({
    super.key,
    this.prefix,
    this.padding,
    this.controller,
    String? initialValue,
    FocusNode? focusNode,
    BoxDecoration? decoration,
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
    int? maxLines = 1,
    int? minLines,
    bool expands = false,
    int? maxLength,
    this.onChanged,
    GestureTapCallback? onTap,
    VoidCallback? onEditingComplete,
    ValueChanged<String>? onFieldSubmitted,
    super.onSaved,
    super.validator,
    List<TextInputFormatter>? inputFormatters,
    bool? enabled,
    double cursorWidth = 2.0,
    double? cursorHeight,
    Color? cursorColor,
    Brightness? keyboardAppearance,
    EdgeInsets scrollPadding = const EdgeInsets.all(20.0),
    bool enableInteractiveSelection = true,
    TextSelectionControls? selectionControls,
    ScrollPhysics? scrollPhysics,
    Iterable<String>? autofillHints,
    AutovalidateMode super.autovalidateMode = AutovalidateMode.disabled,
    String? placeholder,
    TextStyle? placeholderStyle = const TextStyle(
      fontWeight: FontWeight.w400,
      color: CupertinoColors.placeholderText,
    ),
    EditableTextContextMenuBuilder? contextMenuBuilder = _defaultContextMenuBuilder,
    SpellCheckConfiguration? spellCheckConfiguration,
    super.restorationId,
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
       assert(maxLength == null || maxLength > 0),
       super(
         initialValue: controller?.text ?? initialValue ?? '',
         builder: (FormFieldState<String> field) {
           final _CupertinoTextFormFieldRowState state = field as _CupertinoTextFormFieldRowState;

           void onChangedHandler(String value) {
             field.didChange(value);
             onChanged?.call(value);
           }

           return CupertinoFormRow(
             prefix: prefix,
             padding: padding,
             error: (field.errorText == null) ? null : Text(field.errorText!),
             child: UnmanagedRestorationScope(
               bucket: field.bucket,
               child: CupertinoTextField.borderless(
                 restorationId: restorationId,
                 controller: state._effectiveController,
                 focusNode: focusNode,
                 keyboardType: keyboardType,
                 decoration: decoration,
                 textInputAction: textInputAction,
                 style: style,
                 strutStyle: strutStyle,
                 textAlign: textAlign,
                 textAlignVertical: textAlignVertical,
                 textCapitalization: textCapitalization,
                 textDirection: textDirection,
                 autofocus: autofocus,
                 toolbarOptions: toolbarOptions,
                 readOnly: readOnly,
                 showCursor: showCursor,
                 obscuringCharacter: obscuringCharacter,
                 obscureText: obscureText,
                 autocorrect: autocorrect,
                 smartDashesType: smartDashesType,
                 smartQuotesType: smartQuotesType,
                 enableSuggestions: enableSuggestions,
                 maxLines: maxLines,
                 minLines: minLines,
                 expands: expands,
                 maxLength: maxLength,
                 onChanged: onChangedHandler,
                 onTap: onTap,
                 onEditingComplete: onEditingComplete,
                 onSubmitted: onFieldSubmitted,
                 inputFormatters: inputFormatters,
                 enabled: enabled ?? true,
                 cursorWidth: cursorWidth,
                 cursorHeight: cursorHeight,
                 cursorColor: cursorColor,
                 scrollPadding: scrollPadding,
                 scrollPhysics: scrollPhysics,
                 keyboardAppearance: keyboardAppearance,
                 enableInteractiveSelection: enableInteractiveSelection,
                 selectionControls: selectionControls,
                 autofillHints: autofillHints,
                 placeholder: placeholder,
                 placeholderStyle: placeholderStyle,
                 contextMenuBuilder: contextMenuBuilder,
                 spellCheckConfiguration: spellCheckConfiguration,
               ),
             ),
           );
         },
       );

  /// A widget that is displayed at the start of the row.
  ///
  /// The [prefix] widget is displayed at the start of the row. Standard iOS
  /// guidelines encourage passing a [Text] widget to [prefix] to detail the
  /// nature of the input.
  final Widget? prefix;

  /// Content padding for the row.
  ///
  /// The [padding] widget is passed to [CupertinoFormRow]. If the [padding]
  /// parameter is null, [CupertinoFormRow] constructs its own default
  /// padding, which is the standard form row padding in iOS.
  ///
  /// If no edge insets are intended, explicitly pass [EdgeInsets.zero] to
  /// [padding].
  final EdgeInsetsGeometry? padding;

  /// Controls the text being edited.
  ///
  /// If null, this widget will create its own [TextEditingController] and
  /// initialize its [TextEditingController.text] with [initialValue].
  final TextEditingController? controller;

  /// {@macro flutter.material.TextFormField.onChanged}
  final ValueChanged<String>? onChanged;

  static Widget _defaultContextMenuBuilder(
    BuildContext context,
    EditableTextState editableTextState,
  ) {
    return CupertinoAdaptiveTextSelectionToolbar.editableText(editableTextState: editableTextState);
  }

  @override
  FormFieldState<String> createState() => _CupertinoTextFormFieldRowState();
}

class _CupertinoTextFormFieldRowState extends FormFieldState<String> {
  RestorableTextEditingController? _controller;

  TextEditingController get _effectiveController =>
      _cupertinoTextFormFieldRow.controller ?? _controller!.value;

  CupertinoTextFormFieldRow get _cupertinoTextFormFieldRow =>
      super.widget as CupertinoTextFormFieldRow;

  @override
  void restoreState(RestorationBucket? oldBucket, bool initialRestore) {
    super.restoreState(oldBucket, initialRestore);
    if (_controller != null) {
      _registerController();
    }
    // This makes sure to update the internal [FormFieldState] value to sync up with
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
    if (_cupertinoTextFormFieldRow.controller == null) {
      _createLocalController(
        widget.initialValue != null ? TextEditingValue(text: widget.initialValue!) : null,
      );
    } else {
      _cupertinoTextFormFieldRow.controller!.addListener(_handleControllerChanged);
    }
  }

  @override
  void didUpdateWidget(CupertinoTextFormFieldRow oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (_cupertinoTextFormFieldRow.controller != oldWidget.controller) {
      oldWidget.controller?.removeListener(_handleControllerChanged);
      _cupertinoTextFormFieldRow.controller?.addListener(_handleControllerChanged);

      if (oldWidget.controller != null && _cupertinoTextFormFieldRow.controller == null) {
        _createLocalController(oldWidget.controller!.value);
      }

      if (_cupertinoTextFormFieldRow.controller != null) {
        setValue(_cupertinoTextFormFieldRow.controller!.text);
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
    _cupertinoTextFormFieldRow.controller?.removeListener(_handleControllerChanged);
    _controller?.dispose();
    super.dispose();
  }

  @override
  void didChange(String? value) {
    super.didChange(value);

    if (value != null && _effectiveController.text != value) {
      _effectiveController.text = value;
    }
  }

  @override
  void reset() {
    // Set the controller value before calling super.reset() to let
    // _handleControllerChanged suppress the change.
    _effectiveController.text = widget.initialValue!;
    super.reset();
    _cupertinoTextFormFieldRow.onChanged?.call(_effectiveController.text);
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
