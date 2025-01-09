// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'date_picker.dart';
/// @docImport 'text_field.dart';
library;

import 'package:flutter/widgets.dart';

import 'date.dart';
import 'date_picker_theme.dart';
import 'input_border.dart';
import 'input_decorator.dart';
import 'material_localizations.dart';
import 'text_form_field.dart';
import 'theme.dart';

/// A [TextFormField] configured to accept and validate a date entered by a user.
///
/// When the field is saved or submitted, the text will be parsed into a
/// [DateTime] according to the ambient locale's compact date format. If the
/// input text doesn't parse into a date, the [errorFormatText] message will
/// be displayed under the field.
///
/// [firstDate], [lastDate], and [selectableDayPredicate] provide constraints on
/// what days are valid. If the input date isn't in the date range or doesn't pass
/// the given predicate, then the [errorInvalidText] message will be displayed
/// under the field.
///
/// See also:
///
///  * [showDatePicker], which shows a dialog that contains a Material Design
///    date picker which includes support for text entry of dates.
///  * [MaterialLocalizations.parseCompactDate], which is used to parse the text
///    input into a [DateTime].
///
class InputDatePickerFormField extends StatefulWidget {
  /// Creates a [TextFormField] configured to accept and validate a date.
  ///
  /// If the optional [initialDate] is provided, then it will be used to populate
  /// the text field. If the [fieldHintText] is provided, it will be shown.
  ///
  /// If [initialDate] is provided, it must not be before [firstDate] or after
  /// [lastDate]. If [selectableDayPredicate] is provided, it must return `true`
  /// for [initialDate].
  ///
  /// [firstDate] must be on or before [lastDate].
  InputDatePickerFormField({
    super.key,
    DateTime? initialDate,
    required DateTime firstDate,
    required DateTime lastDate,
    this.onDateSubmitted,
    this.onDateSaved,
    this.selectableDayPredicate,
    this.errorFormatText,
    this.errorInvalidText,
    this.fieldHintText,
    this.fieldLabelText,
    this.keyboardType,
    this.autofocus = false,
    this.acceptEmptyDate = false,
    this.focusNode,
  }) : initialDate = initialDate != null ? DateUtils.dateOnly(initialDate) : null,
       firstDate = DateUtils.dateOnly(firstDate),
       lastDate = DateUtils.dateOnly(lastDate) {
    assert(
      !this.lastDate.isBefore(this.firstDate),
      'lastDate ${this.lastDate} must be on or after firstDate ${this.firstDate}.',
    );
    assert(
      initialDate == null || !this.initialDate!.isBefore(this.firstDate),
      'initialDate ${this.initialDate} must be on or after firstDate ${this.firstDate}.',
    );
    assert(
      initialDate == null || !this.initialDate!.isAfter(this.lastDate),
      'initialDate ${this.initialDate} must be on or before lastDate ${this.lastDate}.',
    );
    assert(
      selectableDayPredicate == null ||
          initialDate == null ||
          selectableDayPredicate!(this.initialDate!),
      'Provided initialDate ${this.initialDate} must satisfy provided selectableDayPredicate.',
    );
  }

  /// If provided, it will be used as the default value of the field.
  final DateTime? initialDate;

  /// The earliest allowable [DateTime] that the user can input.
  final DateTime firstDate;

  /// The latest allowable [DateTime] that the user can input.
  final DateTime lastDate;

  /// An optional method to call when the user indicates they are done editing
  /// the text in the field. Will only be called if the input represents a valid
  /// [DateTime].
  final ValueChanged<DateTime>? onDateSubmitted;

  /// An optional method to call with the final date when the form is
  /// saved via [FormState.save]. Will only be called if the input represents
  /// a valid [DateTime].
  final ValueChanged<DateTime>? onDateSaved;

  /// Function to provide full control over which [DateTime] can be selected.
  final SelectableDayPredicate? selectableDayPredicate;

  /// The error text displayed if the entered date is not in the correct format.
  final String? errorFormatText;

  /// The error text displayed if the date is not valid.
  ///
  /// A date is not valid if it is earlier than [firstDate], later than
  /// [lastDate], or doesn't pass the [selectableDayPredicate].
  final String? errorInvalidText;

  /// The hint text displayed in the [TextField].
  ///
  /// If this is null, it will default to the date format string. For example,
  /// 'mm/dd/yyyy' for en_US.
  final String? fieldHintText;

  /// The label text displayed in the [TextField].
  ///
  /// If this is null, it will default to the words representing the date format
  /// string. For example, 'Month, Day, Year' for en_US.
  final String? fieldLabelText;

  /// The keyboard type of the [TextField].
  ///
  /// If this is null, it will default to [TextInputType.datetime]
  final TextInputType? keyboardType;

  /// {@macro flutter.widgets.editableText.autofocus}
  final bool autofocus;

  /// Determines if an empty date would show [errorFormatText] or not.
  ///
  /// Defaults to false.
  ///
  /// If true, [errorFormatText] is not shown when the date input field is empty.
  final bool acceptEmptyDate;

  /// {@macro flutter.widgets.Focus.focusNode}
  final FocusNode? focusNode;

  @override
  State<InputDatePickerFormField> createState() => _InputDatePickerFormFieldState();
}

class _InputDatePickerFormFieldState extends State<InputDatePickerFormField> {
  final TextEditingController _controller = TextEditingController();
  DateTime? _selectedDate;
  String? _inputText;
  bool _autoSelected = false;

  @override
  void initState() {
    super.initState();
    _selectedDate = widget.initialDate;
  }

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    _updateValueForSelectedDate();
  }

  @override
  void didUpdateWidget(InputDatePickerFormField oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (widget.initialDate != oldWidget.initialDate) {
      // Can't update the form field in the middle of a build, so do it next frame
      WidgetsBinding.instance.addPostFrameCallback((Duration timeStamp) {
        setState(() {
          _selectedDate = widget.initialDate;
          _updateValueForSelectedDate();
        });
      }, debugLabel: 'InputDatePickerFormField.update');
    }
  }

  void _updateValueForSelectedDate() {
    if (_selectedDate != null) {
      final MaterialLocalizations localizations = MaterialLocalizations.of(context);
      _inputText = localizations.formatCompactDate(_selectedDate!);
      TextEditingValue textEditingValue = TextEditingValue(text: _inputText!);
      // Select the new text if we are auto focused and haven't selected the text before.
      if (widget.autofocus && !_autoSelected) {
        textEditingValue = textEditingValue.copyWith(
          selection: TextSelection(baseOffset: 0, extentOffset: _inputText!.length),
        );
        _autoSelected = true;
      }
      _controller.value = textEditingValue;
    } else {
      _inputText = '';
      _controller.value = TextEditingValue(text: _inputText!);
    }
  }

  DateTime? _parseDate(String? text) {
    final MaterialLocalizations localizations = MaterialLocalizations.of(context);
    return localizations.parseCompactDate(text);
  }

  bool _isValidAcceptableDate(DateTime? date) {
    return date != null &&
        !date.isBefore(widget.firstDate) &&
        !date.isAfter(widget.lastDate) &&
        (widget.selectableDayPredicate == null || widget.selectableDayPredicate!(date));
  }

  String? _validateDate(String? text) {
    if ((text == null || text.isEmpty) && widget.acceptEmptyDate) {
      return null;
    }
    final DateTime? date = _parseDate(text);
    if (date == null) {
      return widget.errorFormatText ?? MaterialLocalizations.of(context).invalidDateFormatLabel;
    } else if (!_isValidAcceptableDate(date)) {
      return widget.errorInvalidText ?? MaterialLocalizations.of(context).dateOutOfRangeLabel;
    }
    return null;
  }

  void _updateDate(String? text, ValueChanged<DateTime>? callback) {
    final DateTime? date = _parseDate(text);
    if (_isValidAcceptableDate(date)) {
      _selectedDate = date;
      _inputText = text;
      callback?.call(_selectedDate!);
    }
  }

  void _handleSaved(String? text) {
    _updateDate(text, widget.onDateSaved);
  }

  void _handleSubmitted(String text) {
    _updateDate(text, widget.onDateSubmitted);
  }

  @override
  Widget build(BuildContext context) {
    final ThemeData theme = Theme.of(context);
    final bool useMaterial3 = theme.useMaterial3;
    final MaterialLocalizations localizations = MaterialLocalizations.of(context);
    final DatePickerThemeData datePickerTheme = theme.datePickerTheme;
    final InputDecorationTheme inputTheme = theme.inputDecorationTheme;
    final InputBorder effectiveInputBorder =
        datePickerTheme.inputDecorationTheme?.border ??
        theme.inputDecorationTheme.border ??
        (useMaterial3 ? const OutlineInputBorder() : const UnderlineInputBorder());

    return Semantics(
      container: true,
      child: TextFormField(
        decoration: InputDecoration(
          hintText: widget.fieldHintText ?? localizations.dateHelpText,
          labelText: widget.fieldLabelText ?? localizations.dateInputLabel,
        ).applyDefaults(
          inputTheme
              .merge(datePickerTheme.inputDecorationTheme)
              .copyWith(border: effectiveInputBorder),
        ),
        validator: _validateDate,
        keyboardType: widget.keyboardType ?? TextInputType.datetime,
        onSaved: _handleSaved,
        onFieldSubmitted: _handleSubmitted,
        autofocus: widget.autofocus,
        controller: _controller,
        focusNode: widget.focusNode,
      ),
    );
  }
}
