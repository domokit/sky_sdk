// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'text_theme.dart';
library;

import 'package:flutter/services.dart';
import 'package:flutter/widgets.dart';

import 'dropdown_menu.dart';
import 'input_decorator.dart';
import 'menu_style.dart';

/// A [FormField] that contains a [DropdownMenu].
///
/// This is a convenience widget that wraps a [DropdownMenu] widget in a
/// [FormField].
///
/// A [Form] ancestor is not required. The [Form] allows one to
/// save, reset, or validate multiple fields at once. To use without a [Form],
/// pass a [GlobalKey] to the constructor and use [GlobalKey.currentState] to
/// save or reset the form field.
///
/// The `value` parameter maps to [FormField.initialValue].
///
/// See also:
///
///  * [DropdownMenu], which is the underlying text field without the [Form]
///    integration.
class DropdownMenuFormField<T> extends FormField<T> {
  /// Creates a [DropdownMenu] widget that is a [FormField].
  ///
  /// For a description of the `onSaved`, `validator`, or `autovalidateMode`
  /// parameters, see [FormField]. For the rest, see [DropdownMenu].
  DropdownMenuFormField({
    super.key,
    bool enabled = true,
    double? width,
    double? menuHeight,
    Widget? leadingIcon,
    Widget? trailingIcon,
    Widget? label,
    String? hintText,
    String? helperText,
    Widget? selectedTrailingIcon,
    bool enableFilter = false,
    bool enableSearch = true,
    TextStyle? textStyle,
    TextAlign textAlign = TextAlign.start,
    InputDecorationTheme? inputDecorationTheme,
    MenuStyle? menuStyle,
    this.controller,
    T? initialSelection,
    this.onSelected,
    bool? requestFocusOnTap,
    EdgeInsetsGeometry? expandedInsets,
    Offset? alignmentOffset,
    FilterCallback<T>? filterCallback,
    SearchCallback<T>? searchCallback,
    required List<DropdownMenuEntry<T>> dropdownMenuEntries,
    List<TextInputFormatter>? inputFormatters,
    DropdownMenuCloseBehavior closeBehavior = DropdownMenuCloseBehavior.all,
    super.onSaved,
    AutovalidateMode? autovalidateMode,
    super.validator,
    super.forceErrorText,
  }) : super(
         initialValue: initialSelection,
         autovalidateMode: autovalidateMode ?? AutovalidateMode.disabled,
         builder: (FormFieldState<T> field) {
           final _DropdownMenuFormFieldState<T> state = field as _DropdownMenuFormFieldState<T>;
           void onSelectedHandler(T? value) {
             field.didChange(value);
             onSelected?.call(value);
           }

           return DropdownMenu<T>(
             enabled: enabled,
             width: width,
             menuHeight: menuHeight,
             leadingIcon: leadingIcon,
             trailingIcon: trailingIcon,
             label: label,
             hintText: hintText,
             helperText: helperText,
             errorText: state.errorText,
             selectedTrailingIcon: selectedTrailingIcon,
             enableFilter: enableFilter,
             enableSearch: enableSearch,
             textStyle: textStyle,
             textAlign: textAlign,
             inputDecorationTheme: inputDecorationTheme,
             menuStyle: menuStyle,
             controller: controller,
             initialSelection: state.value,
             onSelected: onSelectedHandler,
             requestFocusOnTap: requestFocusOnTap,
             expandedInsets: expandedInsets,
             alignmentOffset: alignmentOffset,
             filterCallback: filterCallback,
             searchCallback: searchCallback,
             inputFormatters: inputFormatters,
             closeBehavior: closeBehavior,
             dropdownMenuEntries: dropdownMenuEntries,
           );
         },
       );

  /// The callback is called when a selection is made.
  ///
  /// Defaults to null. If null, only the text field is updated.
  final ValueChanged<T?>? onSelected;

  /// Controls the text being edited.
  ///
  /// If null, this widget will create its own [TextEditingController].
  final TextEditingController? controller;

  @override
  FormFieldState<T> createState() => _DropdownMenuFormFieldState<T>();
}

class _DropdownMenuFormFieldState<T> extends FormFieldState<T> {
  DropdownMenuFormField<T> get _dropdownMenuFormField => widget as DropdownMenuFormField<T>;

  @override
  void didChange(T? value) {
    super.didChange(value);
    _dropdownMenuFormField.onSelected?.call(value);
  }

  @override
  void didUpdateWidget(DropdownMenuFormField<T> oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (oldWidget.initialValue != widget.initialValue) {
      setValue(widget.initialValue);
    }
  }

  @override
  void reset() {
    super.reset();
    _dropdownMenuFormField.onSelected?.call(value);
  }
}
