// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'dart:ui';
///
/// @docImport 'app.dart';
/// @docImport 'time_picker.dart';
library;

import 'dart:math' as math;

import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter/widgets.dart';

import 'app_bar.dart';
import 'back_button.dart';
import 'button_style.dart';
import 'calendar_date_picker.dart';
import 'color_scheme.dart';
import 'date.dart';
import 'date_picker_theme.dart';
import 'debug.dart';
import 'dialog.dart';
import 'dialog_theme.dart';
import 'divider.dart';
import 'icon_button.dart';
import 'icons.dart';
import 'ink_well.dart';
import 'input_border.dart';
import 'input_date_picker_form_field.dart';
import 'input_decorator.dart';
import 'material.dart';
import 'material_localizations.dart';
import 'material_state.dart';
import 'scaffold.dart';
import 'text_button.dart';
import 'text_field.dart';
import 'text_theme.dart';
import 'theme.dart';

// The M3 sizes are coming from the tokens, but are hand coded,
// as the current token DB does not contain landscape versions.
const Size _calendarPortraitDialogSizeM2 = Size(330.0, 518.0);
const Size _calendarPortraitDialogSizeM3 = Size(328.0, 512.0);
const Size _calendarLandscapeDialogSize = Size(496.0, 346.0);
const Size _inputPortraitDialogSizeM2 = Size(330.0, 270.0);
const Size _inputPortraitDialogSizeM3 = Size(328.0, 270.0);
const Size _inputLandscapeDialogSize = Size(496, 160.0);
const Size _inputRangeLandscapeDialogSize = Size(496, 164.0);
const Duration _dialogSizeAnimationDuration = Duration(milliseconds: 200);
const double _inputFormPortraitHeight = 98.0;
const double _inputFormLandscapeHeight = 108.0;

// 3.0 is the maximum scale factor on mobile phones. As of 07/30/24, iOS goes up
// to a max of 3.0 text scale factor, and Android goes up to 2.0. This is the
// default used for non-range date pickers. This default is changed to a lower
// value at different parts of the date pickers depending on content, and device
// orientation.
const double _kMaxTextScaleFactor = 3.0;

// The max scale factor for the date range pickers.
const double _kMaxRangeTextScaleFactor = 1.3;

// The max text scale factor for the header. This is lower than the default as
// the title text already starts at a large size.
const double _kMaxHeaderTextScaleFactor = 1.6;

// The entry button shares a line with the header text, so there is less room to
// scale up.
const double _kMaxHeaderWithEntryTextScaleFactor = 1.4;

const double _kMaxHelpPortraitTextScaleFactor = 1.6;
const double _kMaxHelpLandscapeTextScaleFactor = 1.4;

// 14 is a common font size used to compute the effective text scale.
const double _fontSizeToScale = 14.0;

/// Shows a dialog containing a Material Design date picker.
///
/// The returned [Future] resolves to the date selected by the user when the
/// user confirms the dialog. If the user cancels the dialog, null is returned.
///
/// When the date picker is first displayed, if [initialDate] is not null, it
/// will show the month of [initialDate], with [initialDate] selected. Otherwise
/// it will show the [currentDate]'s month.
///
/// The [firstDate] is the earliest allowable date. The [lastDate] is the latest
/// allowable date. If [initialDate] is not null, it must either fall between
/// these dates, or be equal to one of them. For each of these [DateTime]
/// parameters, only their dates are considered. Their time fields are ignored.
/// They must all be non-null.
///
/// The [currentDate] represents the current day (i.e. today). This
/// date will be highlighted in the day grid. If null, the date of
/// [DateTime.now] will be used.
///
/// An optional [initialEntryMode] argument can be used to display the date
/// picker in the [DatePickerEntryMode.calendar] (a calendar month grid)
/// or [DatePickerEntryMode.input] (a text input field) mode.
/// It defaults to [DatePickerEntryMode.calendar].
///
/// {@template flutter.material.date_picker.switchToInputEntryModeIcon}
/// An optional [switchToInputEntryModeIcon] argument can be used to
/// display a custom Icon in the corner of the dialog
/// when [DatePickerEntryMode] is [DatePickerEntryMode.calendar]. Clicking on
/// icon changes the [DatePickerEntryMode] to [DatePickerEntryMode.input].
/// If null, `Icon(useMaterial3 ? Icons.edit_outlined : Icons.edit)` is used.
/// {@endtemplate}
///
/// {@template flutter.material.date_picker.switchToCalendarEntryModeIcon}
/// An optional [switchToCalendarEntryModeIcon] argument can be used to
/// display a custom Icon in the corner of the dialog
/// when [DatePickerEntryMode] is [DatePickerEntryMode.input]. Clicking on
/// icon changes the [DatePickerEntryMode] to [DatePickerEntryMode.calendar].
/// If null, `Icon(Icons.calendar_today)` is used.
/// {@endtemplate}
///
/// An optional [selectableDayPredicate] function can be passed in to only allow
/// certain days for selection. If provided, only the days that
/// [selectableDayPredicate] returns true for will be selectable. For example,
/// this can be used to only allow weekdays for selection. If provided, it must
/// return true for [initialDate].
///
/// The following optional string parameters allow you to override the default
/// text used for various parts of the dialog:
///
///   * [helpText], label displayed at the top of the dialog.
///   * [cancelText], label on the cancel button.
///   * [confirmText], label on the ok button.
///   * [errorFormatText], message used when the input text isn't in a proper date format.
///   * [errorInvalidText], message used when the input text isn't a selectable date.
///   * [fieldHintText], text used to prompt the user when no text has been entered in the field.
///   * [fieldLabelText], label for the date text input field.
///
/// An optional [locale] argument can be used to set the locale for the date
/// picker. It defaults to the ambient locale provided by [Localizations].
///
/// An optional [textDirection] argument can be used to set the text direction
/// ([TextDirection.ltr] or [TextDirection.rtl]) for the date picker. It
/// defaults to the ambient text direction provided by [Directionality]. If both
/// [locale] and [textDirection] are non-null, [textDirection] overrides the
/// direction chosen for the [locale].
///
/// The [context], [barrierDismissible], [barrierColor], [barrierLabel],
/// [useRootNavigator] and [routeSettings] arguments are passed to [showDialog],
/// the documentation for which discusses how it is used.
///
/// The [builder] parameter can be used to wrap the dialog widget
/// to add inherited widgets like [Theme].
///
/// An optional [initialDatePickerMode] argument can be used to have the
/// calendar date picker initially appear in the [DatePickerMode.year] or
/// [DatePickerMode.day] mode. It defaults to [DatePickerMode.day].
///
/// {@macro flutter.widgets.RawDialogRoute}
///
/// {@tool dartpad}
/// This sample demonstrates how to create a basic date picker.
/// Tapping the button displays a date picker which returns the selected date.
///
/// ** See code in examples/api/lib/material/date_picker/show_date_picker.1.dart **
/// {@end-tool}
///
/// ### State Restoration
///
/// Using this method will not enable state restoration for the date picker.
/// In order to enable state restoration for a date picker, use
/// [Navigator.restorablePush] or [Navigator.restorablePushNamed] with
/// [DatePickerDialog].
///
/// For more information about state restoration, see [RestorationManager].
///
/// {@macro flutter.widgets.RestorationManager}
///
/// {@tool dartpad}
/// This sample demonstrates how to create a restorable Material date picker.
/// This is accomplished by enabling state restoration by specifying
/// [MaterialApp.restorationScopeId] and using [Navigator.restorablePush] to
/// push [DatePickerDialog] when the button is tapped.
///
/// ** See code in examples/api/lib/material/date_picker/show_date_picker.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [showDateRangePicker], which shows a Material Design date range picker
///    used to select a range of dates.
///  * [CalendarDatePicker], which provides the calendar grid used by the date picker dialog.
///  * [InputDatePickerFormField], which provides a text input field for entering dates.
///  * [DisplayFeatureSubScreen], which documents the specifics of how
///    [DisplayFeature]s can split the screen into sub-screens.
///  * [showTimePicker], which shows a dialog that contains a Material Design time picker.
Future<DateTime?> showDatePicker({
  required BuildContext context,
  DateTime? initialDate,
  required DateTime firstDate,
  required DateTime lastDate,
  DateTime? currentDate,
  DatePickerEntryMode initialEntryMode = DatePickerEntryMode.calendar,
  SelectableDayPredicate? selectableDayPredicate,
  String? helpText,
  String? cancelText,
  String? confirmText,
  Locale? locale,
  bool barrierDismissible = true,
  Color? barrierColor,
  String? barrierLabel,
  bool useRootNavigator = true,
  RouteSettings? routeSettings,
  TextDirection? textDirection,
  TransitionBuilder? builder,
  DatePickerMode initialDatePickerMode = DatePickerMode.day,
  String? errorFormatText,
  String? errorInvalidText,
  String? fieldHintText,
  String? fieldLabelText,
  TextInputType? keyboardType,
  Offset? anchorPoint,
  final ValueChanged<DatePickerEntryMode>? onDatePickerModeChange,
  final Icon? switchToInputEntryModeIcon,
  final Icon? switchToCalendarEntryModeIcon,
}) async {
  initialDate = initialDate == null ? null : DateUtils.dateOnly(initialDate);
  firstDate = DateUtils.dateOnly(firstDate);
  lastDate = DateUtils.dateOnly(lastDate);
  assert(
    !lastDate.isBefore(firstDate),
    'lastDate $lastDate must be on or after firstDate $firstDate.',
  );
  assert(
    initialDate == null || !initialDate.isBefore(firstDate),
    'initialDate $initialDate must be on or after firstDate $firstDate.',
  );
  assert(
    initialDate == null || !initialDate.isAfter(lastDate),
    'initialDate $initialDate must be on or before lastDate $lastDate.',
  );
  assert(
    selectableDayPredicate == null || initialDate == null || selectableDayPredicate(initialDate),
    'Provided initialDate $initialDate must satisfy provided selectableDayPredicate.',
  );
  assert(debugCheckHasMaterialLocalizations(context));

  Widget dialog = DatePickerDialog(
    initialDate: initialDate,
    firstDate: firstDate,
    lastDate: lastDate,
    currentDate: currentDate,
    initialEntryMode: initialEntryMode,
    selectableDayPredicate: selectableDayPredicate,
    helpText: helpText,
    cancelText: cancelText,
    confirmText: confirmText,
    initialCalendarMode: initialDatePickerMode,
    errorFormatText: errorFormatText,
    errorInvalidText: errorInvalidText,
    fieldHintText: fieldHintText,
    fieldLabelText: fieldLabelText,
    keyboardType: keyboardType,
    onDatePickerModeChange: onDatePickerModeChange,
    switchToInputEntryModeIcon: switchToInputEntryModeIcon,
    switchToCalendarEntryModeIcon: switchToCalendarEntryModeIcon,
  );

  if (textDirection != null) {
    dialog = Directionality(textDirection: textDirection, child: dialog);
  }

  if (locale != null) {
    dialog = Localizations.override(context: context, locale: locale, child: dialog);
  } else {
    final DatePickerThemeData datePickerTheme = DatePickerTheme.of(context);
    if (datePickerTheme.locale != null) {
      dialog = Localizations.override(
        context: context,
        locale: datePickerTheme.locale,
        child: dialog,
      );
    }
  }

  return showDialog<DateTime>(
    context: context,
    barrierDismissible: barrierDismissible,
    barrierColor: barrierColor,
    barrierLabel: barrierLabel,
    useRootNavigator: useRootNavigator,
    routeSettings: routeSettings,
    builder: (BuildContext context) {
      return builder == null ? dialog : builder(context, dialog);
    },
    anchorPoint: anchorPoint,
  );
}

/// A Material-style date picker dialog.
///
/// It is used internally by [showDatePicker] or can be directly pushed
/// onto the [Navigator] stack to enable state restoration. See
/// [showDatePicker] for a state restoration app example.
///
/// See also:
///
///  * [showDatePicker], which is a way to display the date picker.
class DatePickerDialog extends StatefulWidget {
  /// A Material-style date picker dialog.
  DatePickerDialog({
    super.key,
    DateTime? initialDate,
    required DateTime firstDate,
    required DateTime lastDate,
    DateTime? currentDate,
    this.initialEntryMode = DatePickerEntryMode.calendar,
    this.selectableDayPredicate,
    this.cancelText,
    this.confirmText,
    this.helpText,
    this.initialCalendarMode = DatePickerMode.day,
    this.errorFormatText,
    this.errorInvalidText,
    this.fieldHintText,
    this.fieldLabelText,
    this.keyboardType,
    this.restorationId,
    this.onDatePickerModeChange,
    this.switchToInputEntryModeIcon,
    this.switchToCalendarEntryModeIcon,
    this.insetPadding = const EdgeInsets.symmetric(horizontal: 16.0, vertical: 24.0),
  }) : initialDate = initialDate == null ? null : DateUtils.dateOnly(initialDate),
       firstDate = DateUtils.dateOnly(firstDate),
       lastDate = DateUtils.dateOnly(lastDate),
       currentDate = DateUtils.dateOnly(currentDate ?? DateTime.now()) {
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
      'Provided initialDate ${this.initialDate} must satisfy provided selectableDayPredicate',
    );
  }

  /// The initially selected [DateTime] that the picker should display.
  ///
  /// If this is null, there is no selected date. A date must be selected to
  /// submit the dialog.
  final DateTime? initialDate;

  /// The earliest allowable [DateTime] that the user can select.
  final DateTime firstDate;

  /// The latest allowable [DateTime] that the user can select.
  final DateTime lastDate;

  /// The [DateTime] representing today. It will be highlighted in the day grid.
  final DateTime currentDate;

  /// The initial mode of date entry method for the date picker dialog.
  ///
  /// See [DatePickerEntryMode] for more details on the different data entry
  /// modes available.
  final DatePickerEntryMode initialEntryMode;

  /// Function to provide full control over which [DateTime] can be selected.
  final SelectableDayPredicate? selectableDayPredicate;

  /// The text that is displayed on the cancel button.
  final String? cancelText;

  /// The text that is displayed on the confirm button.
  final String? confirmText;

  /// The text that is displayed at the top of the header.
  ///
  /// This is used to indicate to the user what they are selecting a date for.
  final String? helpText;

  /// The initial display of the calendar picker.
  final DatePickerMode initialCalendarMode;

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

  /// {@template flutter.material.datePickerDialog}
  /// The keyboard type of the [TextField].
  ///
  /// If this is null, it will default to [TextInputType.datetime]
  /// {@endtemplate}
  final TextInputType? keyboardType;

  /// Restoration ID to save and restore the state of the [DatePickerDialog].
  ///
  /// If it is non-null, the date picker will persist and restore the
  /// date selected on the dialog.
  ///
  /// The state of this widget is persisted in a [RestorationBucket] claimed
  /// from the surrounding [RestorationScope] using the provided restoration ID.
  ///
  /// See also:
  ///
  ///  * [RestorationManager], which explains how state restoration works in
  ///    Flutter.
  final String? restorationId;

  /// Called when the [DatePickerDialog] is toggled between
  /// [DatePickerEntryMode.calendar],[DatePickerEntryMode.input].
  ///
  /// An example of how this callback might be used is an app that saves the
  /// user's preferred entry mode and uses it to initialize the
  /// `initialEntryMode` parameter the next time the date picker is shown.
  final ValueChanged<DatePickerEntryMode>? onDatePickerModeChange;

  /// {@macro flutter.material.date_picker.switchToInputEntryModeIcon}
  final Icon? switchToInputEntryModeIcon;

  /// {@macro flutter.material.date_picker.switchToCalendarEntryModeIcon}
  final Icon? switchToCalendarEntryModeIcon;

  /// The amount of padding added to [MediaQueryData.viewInsets] on the outside
  /// of the dialog. This defines the minimum space between the screen's edges
  /// and the dialog.
  ///
  /// Defaults to `EdgeInsets.symmetric(horizontal: 16.0, vertical: 24.0)`.
  final EdgeInsets insetPadding;

  @override
  State<DatePickerDialog> createState() => _DatePickerDialogState();
}

class _DatePickerDialogState extends State<DatePickerDialog> with RestorationMixin {
  late final RestorableDateTimeN _selectedDate = RestorableDateTimeN(widget.initialDate);
  late final _RestorableDatePickerEntryMode _entryMode = _RestorableDatePickerEntryMode(
    widget.initialEntryMode,
  );
  final _RestorableAutovalidateMode _autovalidateMode = _RestorableAutovalidateMode(
    AutovalidateMode.disabled,
  );

  @override
  void dispose() {
    _selectedDate.dispose();
    _entryMode.dispose();
    _autovalidateMode.dispose();
    super.dispose();
  }

  @override
  String? get restorationId => widget.restorationId;

  @override
  void restoreState(RestorationBucket? oldBucket, bool initialRestore) {
    registerForRestoration(_selectedDate, 'selected_date');
    registerForRestoration(_autovalidateMode, 'autovalidateMode');
    registerForRestoration(_entryMode, 'calendar_entry_mode');
  }

  final GlobalKey _calendarPickerKey = GlobalKey();
  final GlobalKey<FormState> _formKey = GlobalKey<FormState>();

  void _handleOk() {
    if (_entryMode.value == DatePickerEntryMode.input ||
        _entryMode.value == DatePickerEntryMode.inputOnly) {
      final FormState form = _formKey.currentState!;
      if (!form.validate()) {
        setState(() => _autovalidateMode.value = AutovalidateMode.always);
        return;
      }
      form.save();
    }
    Navigator.pop(context, _selectedDate.value);
  }

  void _handleCancel() {
    Navigator.pop(context);
  }

  void _handleOnDatePickerModeChange() {
    widget.onDatePickerModeChange?.call(_entryMode.value);
  }

  void _handleEntryModeToggle() {
    setState(() {
      switch (_entryMode.value) {
        case DatePickerEntryMode.calendar:
          _autovalidateMode.value = AutovalidateMode.disabled;
          _entryMode.value = DatePickerEntryMode.input;
          _handleOnDatePickerModeChange();
        case DatePickerEntryMode.input:
          _formKey.currentState!.save();
          _entryMode.value = DatePickerEntryMode.calendar;
          _handleOnDatePickerModeChange();
        case DatePickerEntryMode.calendarOnly:
        case DatePickerEntryMode.inputOnly:
          assert(false, 'Can not change entry mode from ${_entryMode.value}');
      }
    });
  }

  void _handleDateChanged(DateTime date) {
    setState(() => _selectedDate.value = date);
  }

  Size _dialogSize(BuildContext context) {
    final bool useMaterial3 = Theme.of(context).useMaterial3;
    final bool isCalendar = switch (_entryMode.value) {
      DatePickerEntryMode.calendar || DatePickerEntryMode.calendarOnly => true,
      DatePickerEntryMode.input || DatePickerEntryMode.inputOnly => false,
    };
    final Orientation orientation = MediaQuery.orientationOf(context);

    return switch ((isCalendar, orientation)) {
      (true, Orientation.portrait) when useMaterial3 => _calendarPortraitDialogSizeM3,
      (false, Orientation.portrait) when useMaterial3 => _inputPortraitDialogSizeM3,
      (true, Orientation.portrait) => _calendarPortraitDialogSizeM2,
      (false, Orientation.portrait) => _inputPortraitDialogSizeM2,
      (true, Orientation.landscape) => _calendarLandscapeDialogSize,
      (false, Orientation.landscape) => _inputLandscapeDialogSize,
    };
  }

  static const Map<ShortcutActivator, Intent> _formShortcutMap = <ShortcutActivator, Intent>{
    // Pressing enter on the field will move focus to the next field or control.
    SingleActivator(LogicalKeyboardKey.enter): NextFocusIntent(),
  };

  @override
  Widget build(BuildContext context) {
    final ThemeData theme = Theme.of(context);
    final bool useMaterial3 = theme.useMaterial3;
    final MaterialLocalizations localizations = MaterialLocalizations.of(context);
    final Orientation orientation = MediaQuery.orientationOf(context);
    final bool isLandscapeOrientation = orientation == Orientation.landscape;
    final DatePickerThemeData datePickerTheme = DatePickerTheme.of(context);
    final DatePickerThemeData defaults = DatePickerTheme.defaults(context);
    final TextTheme textTheme = theme.textTheme;

    // There's no M3 spec for a landscape layout input (not calendar)
    // date picker. To ensure that the date displayed in the input
    // date picker's header fits in landscape mode, we override the M3
    // default here.
    TextStyle? headlineStyle;
    if (useMaterial3) {
      headlineStyle = datePickerTheme.headerHeadlineStyle ?? defaults.headerHeadlineStyle;
      switch (_entryMode.value) {
        case DatePickerEntryMode.input:
        case DatePickerEntryMode.inputOnly:
          if (orientation == Orientation.landscape) {
            headlineStyle = textTheme.headlineSmall;
          }
        case DatePickerEntryMode.calendar:
        case DatePickerEntryMode.calendarOnly:
        // M3 default is OK.
      }
    } else {
      headlineStyle = isLandscapeOrientation ? textTheme.headlineSmall : textTheme.headlineMedium;
    }
    final Color? headerForegroundColor =
        datePickerTheme.headerForegroundColor ?? defaults.headerForegroundColor;
    headlineStyle = headlineStyle?.copyWith(color: headerForegroundColor);

    final Widget actions = ConstrainedBox(
      constraints: const BoxConstraints(minHeight: 52.0),
      child: MediaQuery.withClampedTextScaling(
        maxScaleFactor: isLandscapeOrientation ? 1.6 : _kMaxTextScaleFactor,
        child: Padding(
          padding: const EdgeInsets.symmetric(horizontal: 8),
          child: Align(
            alignment: AlignmentDirectional.centerEnd,
            child: OverflowBar(
              spacing: 8,
              children: <Widget>[
                TextButton(
                  style: datePickerTheme.cancelButtonStyle ?? defaults.cancelButtonStyle,
                  onPressed: _handleCancel,
                  child: Text(
                    widget.cancelText ??
                        (useMaterial3
                            ? localizations.cancelButtonLabel
                            : localizations.cancelButtonLabel.toUpperCase()),
                  ),
                ),
                TextButton(
                  style: datePickerTheme.confirmButtonStyle ?? defaults.confirmButtonStyle,
                  onPressed: _handleOk,
                  child: Text(widget.confirmText ?? localizations.okButtonLabel),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    CalendarDatePicker calendarDatePicker() {
      return CalendarDatePicker(
        key: _calendarPickerKey,
        initialDate: _selectedDate.value,
        firstDate: widget.firstDate,
        lastDate: widget.lastDate,
        currentDate: widget.currentDate,
        onDateChanged: _handleDateChanged,
        selectableDayPredicate: widget.selectableDayPredicate,
        initialCalendarMode: widget.initialCalendarMode,
      );
    }

    Form inputDatePicker() {
      return Form(
        key: _formKey,
        autovalidateMode: _autovalidateMode.value,
        child: SizedBox(
          height:
              orientation == Orientation.portrait
                  ? _inputFormPortraitHeight
                  : _inputFormLandscapeHeight,
          child: Padding(
            padding: const EdgeInsets.symmetric(horizontal: 24),
            child: Shortcuts(
              shortcuts: _formShortcutMap,
              child: Column(
                mainAxisAlignment: MainAxisAlignment.center,
                children: <Widget>[
                  Flexible(
                    child: MediaQuery.withClampedTextScaling(
                      maxScaleFactor: 2.0,
                      child: InputDatePickerFormField(
                        initialDate: _selectedDate.value,
                        firstDate: widget.firstDate,
                        lastDate: widget.lastDate,
                        onDateSubmitted: _handleDateChanged,
                        onDateSaved: _handleDateChanged,
                        selectableDayPredicate: widget.selectableDayPredicate,
                        errorFormatText: widget.errorFormatText,
                        errorInvalidText: widget.errorInvalidText,
                        fieldHintText: widget.fieldHintText,
                        fieldLabelText: widget.fieldLabelText,
                        keyboardType: widget.keyboardType,
                        autofocus: true,
                      ),
                    ),
                  ),
                ],
              ),
            ),
          ),
        ),
      );
    }

    final Widget picker;
    final Widget? entryModeButton;
    switch (_entryMode.value) {
      case DatePickerEntryMode.calendar:
        picker = calendarDatePicker();
        entryModeButton = IconButton(
          icon:
              widget.switchToInputEntryModeIcon ??
              Icon(useMaterial3 ? Icons.edit_outlined : Icons.edit),
          color: headerForegroundColor,
          tooltip: localizations.inputDateModeButtonLabel,
          onPressed: _handleEntryModeToggle,
        );

      case DatePickerEntryMode.calendarOnly:
        picker = calendarDatePicker();
        entryModeButton = null;

      case DatePickerEntryMode.input:
        picker = inputDatePicker();
        entryModeButton = IconButton(
          icon: widget.switchToCalendarEntryModeIcon ?? const Icon(Icons.calendar_today),
          color: headerForegroundColor,
          tooltip: localizations.calendarModeButtonLabel,
          onPressed: _handleEntryModeToggle,
        );

      case DatePickerEntryMode.inputOnly:
        picker = inputDatePicker();
        entryModeButton = null;
    }

    final Widget header = _DatePickerHeader(
      helpText:
          widget.helpText ??
          (useMaterial3
              ? localizations.datePickerHelpText
              : localizations.datePickerHelpText.toUpperCase()),
      titleText:
          _selectedDate.value == null ? '' : localizations.formatMediumDate(_selectedDate.value!),
      titleStyle: headlineStyle,
      orientation: orientation,
      isShort: orientation == Orientation.landscape,
      entryModeButton: entryModeButton,
    );

    // Constrain the textScaleFactor to the largest supported value to prevent
    // layout issues.
    final double textScaleFactor =
        MediaQuery.textScalerOf(
          context,
        ).clamp(maxScaleFactor: _kMaxTextScaleFactor).scale(_fontSizeToScale) /
        _fontSizeToScale;
    final Size dialogSize = _dialogSize(context) * textScaleFactor;
    final DialogThemeData dialogTheme = theme.dialogTheme;
    return Dialog(
      backgroundColor: datePickerTheme.backgroundColor ?? defaults.backgroundColor,
      elevation:
          useMaterial3
              ? datePickerTheme.elevation ?? defaults.elevation!
              : datePickerTheme.elevation ?? dialogTheme.elevation ?? 24,
      shadowColor: datePickerTheme.shadowColor ?? defaults.shadowColor,
      surfaceTintColor: datePickerTheme.surfaceTintColor ?? defaults.surfaceTintColor,
      shape:
          useMaterial3
              ? datePickerTheme.shape ?? defaults.shape
              : datePickerTheme.shape ?? dialogTheme.shape ?? defaults.shape,
      insetPadding: widget.insetPadding,
      clipBehavior: Clip.antiAlias,
      child: AnimatedContainer(
        width: dialogSize.width,
        height: dialogSize.height,
        duration: _dialogSizeAnimationDuration,
        curve: Curves.easeIn,
        child: MediaQuery.withClampedTextScaling(
          // Constrain the textScaleFactor to the largest supported value to prevent
          // layout issues.
          maxScaleFactor: _kMaxTextScaleFactor,
          child: LayoutBuilder(
            builder: (BuildContext context, BoxConstraints constraints) {
              final Size portraitDialogSize =
                  useMaterial3 ? _inputPortraitDialogSizeM3 : _inputPortraitDialogSizeM2;
              // Make sure the portrait dialog can fit the contents comfortably when
              // resized from the landscape dialog.
              final bool isFullyPortrait =
                  constraints.maxHeight >= math.min(dialogSize.height, portraitDialogSize.height);

              switch (orientation) {
                case Orientation.portrait:
                  return Column(
                    mainAxisSize: MainAxisSize.min,
                    crossAxisAlignment: CrossAxisAlignment.stretch,
                    children: <Widget>[
                      header,
                      if (useMaterial3) Divider(height: 0, color: datePickerTheme.dividerColor),
                      if (isFullyPortrait) ...<Widget>[Expanded(child: picker), actions],
                    ],
                  );
                case Orientation.landscape:
                  return Row(
                    mainAxisSize: MainAxisSize.min,
                    crossAxisAlignment: CrossAxisAlignment.stretch,
                    children: <Widget>[
                      header,
                      if (useMaterial3)
                        VerticalDivider(width: 0, color: datePickerTheme.dividerColor),
                      Flexible(
                        child: Column(
                          mainAxisSize: MainAxisSize.min,
                          crossAxisAlignment: CrossAxisAlignment.stretch,
                          children: <Widget>[Expanded(child: picker), actions],
                        ),
                      ),
                    ],
                  );
              }
            },
          ),
        ),
      ),
    );
  }
}

// A restorable [DatePickerEntryMode] value.
//
// This serializes each entry as a unique `int` value.
class _RestorableDatePickerEntryMode extends RestorableValue<DatePickerEntryMode> {
  _RestorableDatePickerEntryMode(DatePickerEntryMode defaultValue) : _defaultValue = defaultValue;

  final DatePickerEntryMode _defaultValue;

  @override
  DatePickerEntryMode createDefaultValue() => _defaultValue;

  @override
  void didUpdateValue(DatePickerEntryMode? oldValue) {
    assert(debugIsSerializableForRestoration(value.index));
    notifyListeners();
  }

  @override
  DatePickerEntryMode fromPrimitives(Object? data) => DatePickerEntryMode.values[data! as int];

  @override
  Object? toPrimitives() => value.index;
}

// A restorable [AutovalidateMode] value.
//
// This serializes each entry as a unique `int` value.
class _RestorableAutovalidateMode extends RestorableValue<AutovalidateMode> {
  _RestorableAutovalidateMode(AutovalidateMode defaultValue) : _defaultValue = defaultValue;

  final AutovalidateMode _defaultValue;

  @override
  AutovalidateMode createDefaultValue() => _defaultValue;

  @override
  void didUpdateValue(AutovalidateMode? oldValue) {
    assert(debugIsSerializableForRestoration(value.index));
    notifyListeners();
  }

  @override
  AutovalidateMode fromPrimitives(Object? data) => AutovalidateMode.values[data! as int];

  @override
  Object? toPrimitives() => value.index;
}

/// Re-usable widget that displays the selected date (in large font) and the
/// help text above it.
///
/// These types include:
///
/// * Single Date picker with calendar mode.
/// * Single Date picker with text input mode.
/// * Date Range picker with text input mode.
class _DatePickerHeader extends StatelessWidget {
  /// Creates a header for use in a date picker dialog.
  const _DatePickerHeader({
    required this.helpText,
    required this.titleText,
    this.titleSemanticsLabel,
    required this.titleStyle,
    required this.orientation,
    this.isShort = false,
    this.entryModeButton,
  });

  static const double _datePickerHeaderLandscapeWidth = 152.0;
  static const double _datePickerHeaderPortraitHeight = 120.0;
  static const double _headerPaddingLandscape = 16.0;

  /// The text that is displayed at the top of the header.
  ///
  /// This is used to indicate to the user what they are selecting a date for.
  final String helpText;

  /// The text that is displayed at the center of the header.
  final String titleText;

  /// The semantic label associated with the [titleText].
  final String? titleSemanticsLabel;

  /// The [TextStyle] that the title text is displayed with.
  final TextStyle? titleStyle;

  /// The orientation is used to decide how to layout its children.
  final Orientation orientation;

  /// Indicates the header is being displayed in a shorter/narrower context.
  ///
  /// This will be used to tighten up the space between the help text and date
  /// text if `true`. Additionally, it will use a smaller typography style if
  /// `true`.
  ///
  /// This is necessary for displaying the manual input mode in
  /// landscape orientation, in order to account for the keyboard height.
  final bool isShort;

  final Widget? entryModeButton;

  @override
  Widget build(BuildContext context) {
    final ThemeData theme = Theme.of(context);
    final DatePickerThemeData datePickerTheme = DatePickerTheme.of(context);
    final DatePickerThemeData defaults = DatePickerTheme.defaults(context);
    final Color? backgroundColor =
        datePickerTheme.headerBackgroundColor ?? defaults.headerBackgroundColor;
    final Color? foregroundColor =
        datePickerTheme.headerForegroundColor ?? defaults.headerForegroundColor;
    final TextStyle? helpStyle = (datePickerTheme.headerHelpStyle ?? defaults.headerHelpStyle)
        ?.copyWith(color: foregroundColor);
    final double currentScale =
        MediaQuery.textScalerOf(context).scale(_fontSizeToScale) / _fontSizeToScale;
    final double maxHeaderTextScaleFactor = math.min(
      currentScale,
      entryModeButton != null ? _kMaxHeaderWithEntryTextScaleFactor : _kMaxHeaderTextScaleFactor,
    );
    final double textScaleFactor =
        MediaQuery.textScalerOf(
          context,
        ).clamp(maxScaleFactor: maxHeaderTextScaleFactor).scale(_fontSizeToScale) /
        _fontSizeToScale;
    final double scaledFontSize = MediaQuery.textScalerOf(
      context,
    ).scale(titleStyle?.fontSize ?? 32);
    final double headerScaleFactor = textScaleFactor > 1 ? textScaleFactor : 1.0;

    final Text help = Text(
      helpText,
      style: helpStyle,
      maxLines: 1,
      overflow: TextOverflow.ellipsis,
      textScaler: MediaQuery.textScalerOf(context).clamp(
        maxScaleFactor: math.min(
          textScaleFactor,
          orientation == Orientation.portrait
              ? _kMaxHelpPortraitTextScaleFactor
              : _kMaxHelpLandscapeTextScaleFactor,
        ),
      ),
    );
    final Text title = Text(
      titleText,
      semanticsLabel: titleSemanticsLabel ?? titleText,
      style: titleStyle,
      maxLines:
          orientation == Orientation.portrait
              ? (scaledFontSize > 70 ? 2 : 1)
              : scaledFontSize > 40
              ? 3
              : 2,
      overflow: TextOverflow.ellipsis,
      textScaler: MediaQuery.textScalerOf(context).clamp(maxScaleFactor: textScaleFactor),
    );

    final double fontScaleAdjustedHeaderHeight =
        headerScaleFactor > 1.3 ? headerScaleFactor - 0.2 : 1.0;

    switch (orientation) {
      case Orientation.portrait:
        return Semantics(
          container: true,
          child: SizedBox(
            height: _datePickerHeaderPortraitHeight * fontScaleAdjustedHeaderHeight,
            child: Material(
              color: backgroundColor,
              child: Padding(
                padding: const EdgeInsetsDirectional.only(start: 24, end: 12, bottom: 12),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: <Widget>[
                    const SizedBox(height: 16),
                    help,
                    const Flexible(child: SizedBox(height: 38)),
                    Row(
                      children: <Widget>[
                        Expanded(child: title),
                        if (entryModeButton != null)
                          Semantics(container: true, child: entryModeButton),
                      ],
                    ),
                  ],
                ),
              ),
            ),
          ),
        );
      case Orientation.landscape:
        return Semantics(
          container: true,
          child: SizedBox(
            width: _datePickerHeaderLandscapeWidth,
            child: Material(
              color: backgroundColor,
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: <Widget>[
                  const SizedBox(height: 16),
                  Padding(
                    padding: const EdgeInsets.symmetric(horizontal: _headerPaddingLandscape),
                    child: help,
                  ),
                  SizedBox(height: isShort ? 16 : 56),
                  Expanded(
                    child: Padding(
                      padding: const EdgeInsets.symmetric(horizontal: _headerPaddingLandscape),
                      child: title,
                    ),
                  ),
                  if (entryModeButton != null)
                    Padding(
                      padding:
                          theme.useMaterial3
                              // TODO(TahaTesser): This is an eye-balled M3 entry mode button padding
                              // from https://m3.material.io/components/date-pickers/specs#c16c142b-4706-47f3-9400-3cde654b9aa8.
                              // Update this value to use tokens when available.
                              ? const EdgeInsetsDirectional.only(start: 8.0, end: 4.0, bottom: 6.0)
                              : const EdgeInsets.symmetric(horizontal: 4),
                      child: Semantics(container: true, child: entryModeButton),
                    ),
                ],
              ),
            ),
          ),
        );
    }
  }
}

/// Signature for predicating enabled dates in date range pickers.
///
/// The [selectedStartDay] and [selectedEndDay] are the currently selected start
/// and end dates of a date range, which conditionally enables or disables each
/// date in the picker based on the user selection. (Example: in a hostel's room
/// selection, you are not able to select the end date after the next
/// non-selectable day).
///
/// See [showDateRangePicker], which has a [SelectableDayForRangePredicate]
/// parameter used to specify allowable days in the date range picker.
typedef SelectableDayForRangePredicate =
    bool Function(DateTime day, DateTime? selectedStartDay, DateTime? selectedEndDay);

/// Shows a full screen modal dialog containing a Material Design date range
/// picker.
///
/// The returned [Future] resolves to the [DateTimeRange] selected by the user
/// when the user saves their selection. If the user cancels the dialog, null is
/// returned.
///
/// If [initialDateRange] is non-null, then it will be used as the initially
/// selected date range. If it is provided, `initialDateRange.start` must be
/// before or on `initialDateRange.end`.
///
/// The [firstDate] is the earliest allowable date. The [lastDate] is the latest
/// allowable date.
///
/// If an initial date range is provided, `initialDateRange.start`
/// and `initialDateRange.end` must both fall between or on [firstDate] and
/// [lastDate]. For all of these [DateTime] values, only their dates are
/// considered. Their time fields are ignored.
///
/// The [currentDate] represents the current day (i.e. today). This
/// date will be highlighted in the day grid. If null, the date of
/// `DateTime.now()` will be used.
///
/// An optional [initialEntryMode] argument can be used to display the date
/// picker in the [DatePickerEntryMode.calendar] (a scrollable calendar month
/// grid) or [DatePickerEntryMode.input] (two text input fields) mode.
/// It defaults to [DatePickerEntryMode.calendar].
///
/// {@macro flutter.material.date_picker.switchToInputEntryModeIcon}
///
/// {@macro flutter.material.date_picker.switchToCalendarEntryModeIcon}
///
/// The following optional string parameters allow you to override the default
/// text used for various parts of the dialog:
///
///   * [helpText], the label displayed at the top of the dialog.
///   * [cancelText], the label on the cancel button for the text input mode.
///   * [confirmText],the label on the ok button for the text input mode.
///   * [saveText], the label on the save button for the fullscreen calendar
///     mode.
///   * [errorFormatText], the message used when an input text isn't in a proper
///     date format.
///   * [errorInvalidText], the message used when an input text isn't a
///     selectable date.
///   * [errorInvalidRangeText], the message used when the date range is
///     invalid (e.g. start date is after end date).
///   * [fieldStartHintText], the text used to prompt the user when no text has
///     been entered in the start field.
///   * [fieldEndHintText], the text used to prompt the user when no text has
///     been entered in the end field.
///   * [fieldStartLabelText], the label for the start date text input field.
///   * [fieldEndLabelText], the label for the end date text input field.
///
/// An optional [locale] argument can be used to set the locale for the date
/// picker. It defaults to the ambient locale provided by [Localizations].
///
/// An optional [textDirection] argument can be used to set the text direction
/// ([TextDirection.ltr] or [TextDirection.rtl]) for the date picker. It
/// defaults to the ambient text direction provided by [Directionality]. If both
/// [locale] and [textDirection] are non-null, [textDirection] overrides the
/// direction chosen for the [locale].
///
/// The [context], [barrierDismissible], [barrierColor], [barrierLabel],
/// [useRootNavigator] and [routeSettings] arguments are passed to [showDialog],
/// the documentation for which discusses how it is used.
///
/// The [builder] parameter can be used to wrap the dialog widget
/// to add inherited widgets like [Theme].
///
/// {@macro flutter.widgets.RawDialogRoute}
///
/// ### State Restoration
///
/// Using this method will not enable state restoration for the date range picker.
/// In order to enable state restoration for a date range picker, use
/// [Navigator.restorablePush] or [Navigator.restorablePushNamed] with
/// [DateRangePickerDialog].
///
/// For more information about state restoration, see [RestorationManager].
///
/// {@macro flutter.widgets.RestorationManager}
///
/// {@tool dartpad}
/// This sample demonstrates how to create a restorable Material date range picker.
/// This is accomplished by enabling state restoration by specifying
/// [MaterialApp.restorationScopeId] and using [Navigator.restorablePush] to
/// push [DateRangePickerDialog] when the button is tapped.
///
/// ** See code in examples/api/lib/material/date_picker/show_date_range_picker.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [showDatePicker], which shows a Material Design date picker used to
///    select a single date.
///  * [DateTimeRange], which is used to describe a date range.
///  * [DisplayFeatureSubScreen], which documents the specifics of how
///    [DisplayFeature]s can split the screen into sub-screens.
Future<DateTimeRange?> showDateRangePicker({
  required BuildContext context,
  DateTimeRange? initialDateRange,
  required DateTime firstDate,
  required DateTime lastDate,
  DateTime? currentDate,
  DatePickerEntryMode initialEntryMode = DatePickerEntryMode.calendar,
  String? helpText,
  String? cancelText,
  String? confirmText,
  String? saveText,
  String? errorFormatText,
  String? errorInvalidText,
  String? errorInvalidRangeText,
  String? fieldStartHintText,
  String? fieldEndHintText,
  String? fieldStartLabelText,
  String? fieldEndLabelText,
  Locale? locale,
  bool barrierDismissible = true,
  Color? barrierColor,
  String? barrierLabel,
  bool useRootNavigator = true,
  RouteSettings? routeSettings,
  TextDirection? textDirection,
  TransitionBuilder? builder,
  Offset? anchorPoint,
  TextInputType keyboardType = TextInputType.datetime,
  final Icon? switchToInputEntryModeIcon,
  final Icon? switchToCalendarEntryModeIcon,
  SelectableDayForRangePredicate? selectableDayPredicate,
}) async {
  initialDateRange = initialDateRange == null ? null : DateUtils.datesOnly(initialDateRange);
  firstDate = DateUtils.dateOnly(firstDate);
  lastDate = DateUtils.dateOnly(lastDate);
  assert(
    !lastDate.isBefore(firstDate),
    'lastDate $lastDate must be on or after firstDate $firstDate.',
  );
  assert(
    initialDateRange == null || !initialDateRange.start.isBefore(firstDate),
    "initialDateRange's start date must be on or after firstDate $firstDate.",
  );
  assert(
    initialDateRange == null || !initialDateRange.end.isBefore(firstDate),
    "initialDateRange's end date must be on or after firstDate $firstDate.",
  );
  assert(
    initialDateRange == null || !initialDateRange.start.isAfter(lastDate),
    "initialDateRange's start date must be on or before lastDate $lastDate.",
  );
  assert(
    initialDateRange == null || !initialDateRange.end.isAfter(lastDate),
    "initialDateRange's end date must be on or before lastDate $lastDate.",
  );
  assert(
    initialDateRange == null ||
        selectableDayPredicate == null ||
        selectableDayPredicate(
          initialDateRange.start,
          initialDateRange.start,
          initialDateRange.end,
        ),
    "initialDateRange's start date must be selectable.",
  );
  assert(
    initialDateRange == null ||
        selectableDayPredicate == null ||
        selectableDayPredicate(initialDateRange.end, initialDateRange.start, initialDateRange.end),
    "initialDateRange's end date must be selectable.",
  );
  currentDate = DateUtils.dateOnly(currentDate ?? DateTime.now());
  assert(debugCheckHasMaterialLocalizations(context));

  Widget dialog = DateRangePickerDialog(
    initialDateRange: initialDateRange,
    firstDate: firstDate,
    lastDate: lastDate,
    currentDate: currentDate,
    selectableDayPredicate: selectableDayPredicate,
    initialEntryMode: initialEntryMode,
    helpText: helpText,
    cancelText: cancelText,
    confirmText: confirmText,
    saveText: saveText,
    errorFormatText: errorFormatText,
    errorInvalidText: errorInvalidText,
    errorInvalidRangeText: errorInvalidRangeText,
    fieldStartHintText: fieldStartHintText,
    fieldEndHintText: fieldEndHintText,
    fieldStartLabelText: fieldStartLabelText,
    fieldEndLabelText: fieldEndLabelText,
    keyboardType: keyboardType,
    switchToInputEntryModeIcon: switchToInputEntryModeIcon,
    switchToCalendarEntryModeIcon: switchToCalendarEntryModeIcon,
  );

  if (textDirection != null) {
    dialog = Directionality(textDirection: textDirection, child: dialog);
  }

  if (locale != null) {
    dialog = Localizations.override(context: context, locale: locale, child: dialog);
  }

  return showDialog<DateTimeRange>(
    context: context,
    barrierDismissible: barrierDismissible,
    barrierColor: barrierColor,
    barrierLabel: barrierLabel,
    useRootNavigator: useRootNavigator,
    routeSettings: routeSettings,
    useSafeArea: false,
    builder: (BuildContext context) {
      return builder == null ? dialog : builder(context, dialog);
    },
    anchorPoint: anchorPoint,
  );
}

/// Returns a locale-appropriate string to describe the start of a date range.
///
/// If `startDate` is null, then it defaults to 'Start Date', otherwise if it
/// is in the same year as the `endDate` then it will use the short month
/// day format (i.e. 'Jan 21'). Otherwise it will return the short date format
/// (i.e. 'Jan 21, 2020').
String _formatRangeStartDate(
  MaterialLocalizations localizations,
  DateTime? startDate,
  DateTime? endDate,
) {
  return startDate == null
      ? localizations.dateRangeStartLabel
      : (endDate == null || startDate.year == endDate.year)
      ? localizations.formatShortMonthDay(startDate)
      : localizations.formatShortDate(startDate);
}

/// Returns an locale-appropriate string to describe the end of a date range.
///
/// If `endDate` is null, then it defaults to 'End Date', otherwise if it
/// is in the same year as the `startDate` and the `currentDate` then it will
/// just use the short month day format (i.e. 'Jan 21'), otherwise it will
/// include the year (i.e. 'Jan 21, 2020').
String _formatRangeEndDate(
  MaterialLocalizations localizations,
  DateTime? startDate,
  DateTime? endDate,
  DateTime currentDate,
) {
  return endDate == null
      ? localizations.dateRangeEndLabel
      : (startDate != null && startDate.year == endDate.year && startDate.year == currentDate.year)
      ? localizations.formatShortMonthDay(endDate)
      : localizations.formatShortDate(endDate);
}

/// A Material-style date range picker dialog.
///
/// It is used internally by [showDateRangePicker] or can be directly pushed
/// onto the [Navigator] stack to enable state restoration. See
/// [showDateRangePicker] for a state restoration app example.
///
/// See also:
///
///  * [showDateRangePicker], which is a way to display the date picker.
class DateRangePickerDialog extends StatefulWidget {
  /// A Material-style date range picker dialog.
  const DateRangePickerDialog({
    super.key,
    this.initialDateRange,
    required this.firstDate,
    required this.lastDate,
    this.currentDate,
    this.initialEntryMode = DatePickerEntryMode.calendar,
    this.helpText,
    this.cancelText,
    this.confirmText,
    this.saveText,
    this.errorInvalidRangeText,
    this.errorFormatText,
    this.errorInvalidText,
    this.fieldStartHintText,
    this.fieldEndHintText,
    this.fieldStartLabelText,
    this.fieldEndLabelText,
    this.keyboardType = TextInputType.datetime,
    this.restorationId,
    this.switchToInputEntryModeIcon,
    this.switchToCalendarEntryModeIcon,
    this.selectableDayPredicate,
  });

  /// The date range that the date range picker starts with when it opens.
  ///
  /// If an initial date range is provided, `initialDateRange.start`
  /// and `initialDateRange.end` must both fall between or on [firstDate] and
  /// [lastDate]. For all of these [DateTime] values, only their dates are
  /// considered. Their time fields are ignored.
  ///
  /// If [initialDateRange] is non-null, then it will be used as the initially
  /// selected date range. If it is provided, `initialDateRange.start` must be
  /// before or on `initialDateRange.end`.
  final DateTimeRange? initialDateRange;

  /// The earliest allowable date on the date range.
  final DateTime firstDate;

  /// The latest allowable date on the date range.
  final DateTime lastDate;

  /// The [currentDate] represents the current day (i.e. today).
  ///
  /// This date will be highlighted in the day grid.
  ///
  /// If `null`, the date of `DateTime.now()` will be used.
  final DateTime? currentDate;

  /// The initial date range picker entry mode.
  ///
  /// The date range has two main modes: [DatePickerEntryMode.calendar] (a
  /// scrollable calendar month grid) or [DatePickerEntryMode.input] (two text
  /// input fields) mode.
  ///
  /// It defaults to [DatePickerEntryMode.calendar].
  final DatePickerEntryMode initialEntryMode;

  /// The label on the cancel button for the text input mode.
  ///
  /// If null, the localized value of
  /// [MaterialLocalizations.cancelButtonLabel] is used.
  final String? cancelText;

  /// The label on the "OK" button for the text input mode.
  ///
  /// If null, the localized value of
  /// [MaterialLocalizations.okButtonLabel] is used.
  final String? confirmText;

  /// The label on the save button for the fullscreen calendar mode.
  ///
  /// If null, the localized value of
  /// [MaterialLocalizations.saveButtonLabel] is used.
  final String? saveText;

  /// The label displayed at the top of the dialog.
  ///
  /// If null, the localized value of
  /// [MaterialLocalizations.dateRangePickerHelpText] is used.
  final String? helpText;

  /// The message used when the date range is invalid (e.g. start date is after
  /// end date).
  ///
  /// If null, the localized value of
  /// [MaterialLocalizations.invalidDateRangeLabel] is used.
  final String? errorInvalidRangeText;

  /// The message used when an input text isn't in a proper date format.
  ///
  /// If null, the localized value of
  /// [MaterialLocalizations.invalidDateFormatLabel] is used.
  final String? errorFormatText;

  /// The message used when an input text isn't a selectable date.
  ///
  /// If null, the localized value of
  /// [MaterialLocalizations.dateOutOfRangeLabel] is used.
  final String? errorInvalidText;

  /// The text used to prompt the user when no text has been entered in the
  /// start field.
  ///
  /// If null, the localized value of
  /// [MaterialLocalizations.dateHelpText] is used.
  final String? fieldStartHintText;

  /// The text used to prompt the user when no text has been entered in the
  /// end field.
  ///
  /// If null, the localized value of [MaterialLocalizations.dateHelpText] is
  /// used.
  final String? fieldEndHintText;

  /// The label for the start date text input field.
  ///
  /// If null, the localized value of [MaterialLocalizations.dateRangeStartLabel]
  /// is used.
  final String? fieldStartLabelText;

  /// The label for the end date text input field.
  ///
  /// If null, the localized value of [MaterialLocalizations.dateRangeEndLabel]
  /// is used.
  final String? fieldEndLabelText;

  /// {@macro flutter.material.datePickerDialog}
  final TextInputType keyboardType;

  /// Restoration ID to save and restore the state of the [DateRangePickerDialog].
  ///
  /// If it is non-null, the date range picker will persist and restore the
  /// date range selected on the dialog.
  ///
  /// The state of this widget is persisted in a [RestorationBucket] claimed
  /// from the surrounding [RestorationScope] using the provided restoration ID.
  ///
  /// See also:
  ///
  ///  * [RestorationManager], which explains how state restoration works in
  ///    Flutter.
  final String? restorationId;

  /// {@macro flutter.material.date_picker.switchToInputEntryModeIcon}
  final Icon? switchToInputEntryModeIcon;

  /// {@macro flutter.material.date_picker.switchToCalendarEntryModeIcon}
  final Icon? switchToCalendarEntryModeIcon;

  /// Function to provide full control over which [DateTime] can be selected.
  final SelectableDayForRangePredicate? selectableDayPredicate;

  @override
  State<DateRangePickerDialog> createState() => _DateRangePickerDialogState();
}

class _DateRangePickerDialogState extends State<DateRangePickerDialog> with RestorationMixin {
  late final _RestorableDatePickerEntryMode _entryMode = _RestorableDatePickerEntryMode(
    widget.initialEntryMode,
  );
  late final RestorableDateTimeN _selectedStart = RestorableDateTimeN(
    widget.initialDateRange?.start,
  );
  late final RestorableDateTimeN _selectedEnd = RestorableDateTimeN(widget.initialDateRange?.end);
  final RestorableBool _autoValidate = RestorableBool(false);
  final GlobalKey _calendarPickerKey = GlobalKey();
  final GlobalKey<_InputDateRangePickerState> _inputPickerKey =
      GlobalKey<_InputDateRangePickerState>();

  @override
  String? get restorationId => widget.restorationId;

  @override
  void restoreState(RestorationBucket? oldBucket, bool initialRestore) {
    registerForRestoration(_entryMode, 'entry_mode');
    registerForRestoration(_selectedStart, 'selected_start');
    registerForRestoration(_selectedEnd, 'selected_end');
    registerForRestoration(_autoValidate, 'autovalidate');
  }

  @override
  void dispose() {
    _entryMode.dispose();
    _selectedStart.dispose();
    _selectedEnd.dispose();
    _autoValidate.dispose();
    super.dispose();
  }

  void _handleOk() {
    if (_entryMode.value == DatePickerEntryMode.input ||
        _entryMode.value == DatePickerEntryMode.inputOnly) {
      final _InputDateRangePickerState picker = _inputPickerKey.currentState!;
      if (!picker.validate()) {
        setState(() {
          _autoValidate.value = true;
        });
        return;
      }
    }
    final DateTimeRange? selectedRange =
        _hasSelectedDateRange
            ? DateTimeRange(start: _selectedStart.value!, end: _selectedEnd.value!)
            : null;

    Navigator.pop(context, selectedRange);
  }

  void _handleCancel() {
    Navigator.pop(context);
  }

  void _handleEntryModeToggle() {
    setState(() {
      switch (_entryMode.value) {
        case DatePickerEntryMode.calendar:
          _autoValidate.value = false;
          _entryMode.value = DatePickerEntryMode.input;

        case DatePickerEntryMode.input:
          // Validate the range dates
          if (_selectedStart.value != null &&
              _selectedEnd.value != null &&
              _selectedStart.value!.isAfter(_selectedEnd.value!)) {
            _selectedEnd.value = null;
          }
          if (_selectedStart.value != null && !_isDaySelectable(_selectedStart.value!)) {
            _selectedStart.value = null;
            // With no valid start date, having an end date makes no sense for the UI.
            _selectedEnd.value = null;
          } else if (_selectedEnd.value != null && !_isDaySelectable(_selectedEnd.value!)) {
            _selectedEnd.value = null;
          }
          _entryMode.value = DatePickerEntryMode.calendar;

        case DatePickerEntryMode.calendarOnly:
        case DatePickerEntryMode.inputOnly:
          assert(false, 'Can not change entry mode from $_entryMode');
      }
    });
  }

  bool _isDaySelectable(DateTime day) {
    if (day.isBefore(widget.firstDate) || day.isAfter(widget.lastDate)) {
      return false;
    }
    if (widget.selectableDayPredicate == null) {
      return true;
    }
    return widget.selectableDayPredicate!(day, _selectedStart.value, _selectedEnd.value);
  }

  void _handleStartDateChanged(DateTime? date) {
    setState(() => _selectedStart.value = date);
  }

  void _handleEndDateChanged(DateTime? date) {
    setState(() => _selectedEnd.value = date);
  }

  bool get _hasSelectedDateRange => _selectedStart.value != null && _selectedEnd.value != null;

  @override
  Widget build(BuildContext context) {
    final ThemeData theme = Theme.of(context);
    final bool useMaterial3 = theme.useMaterial3;
    final Orientation orientation = MediaQuery.orientationOf(context);
    final MaterialLocalizations localizations = MaterialLocalizations.of(context);
    final DatePickerThemeData datePickerTheme = DatePickerTheme.of(context);
    final DatePickerThemeData defaults = DatePickerTheme.defaults(context);

    final Widget contents;
    final Size size;
    final double? elevation;
    final Color? shadowColor;
    final Color? surfaceTintColor;
    final ShapeBorder? shape;
    final EdgeInsets insetPadding;
    final bool showEntryModeButton =
        _entryMode.value == DatePickerEntryMode.calendar ||
        _entryMode.value == DatePickerEntryMode.input;
    switch (_entryMode.value) {
      case DatePickerEntryMode.calendar:
      case DatePickerEntryMode.calendarOnly:
        contents = _CalendarRangePickerDialog(
          key: _calendarPickerKey,
          selectedStartDate: _selectedStart.value,
          selectedEndDate: _selectedEnd.value,
          firstDate: widget.firstDate,
          lastDate: widget.lastDate,
          selectableDayPredicate: widget.selectableDayPredicate,
          currentDate: widget.currentDate,
          onStartDateChanged: _handleStartDateChanged,
          onEndDateChanged: _handleEndDateChanged,
          onConfirm: _hasSelectedDateRange ? _handleOk : null,
          onCancel: _handleCancel,
          entryModeButton:
              showEntryModeButton
                  ? IconButton(
                    icon:
                        widget.switchToInputEntryModeIcon ??
                        Icon(useMaterial3 ? Icons.edit_outlined : Icons.edit),
                    padding: EdgeInsets.zero,
                    tooltip: localizations.inputDateModeButtonLabel,
                    onPressed: _handleEntryModeToggle,
                  )
                  : null,
          confirmText:
              widget.saveText ??
              (useMaterial3
                  ? localizations.saveButtonLabel
                  : localizations.saveButtonLabel.toUpperCase()),
          helpText:
              widget.helpText ??
              (useMaterial3
                  ? localizations.dateRangePickerHelpText
                  : localizations.dateRangePickerHelpText.toUpperCase()),
        );
        size = MediaQuery.sizeOf(context);
        insetPadding = EdgeInsets.zero;
        elevation = datePickerTheme.rangePickerElevation ?? defaults.rangePickerElevation!;
        shadowColor = datePickerTheme.rangePickerShadowColor ?? defaults.rangePickerShadowColor!;
        surfaceTintColor =
            datePickerTheme.rangePickerSurfaceTintColor ?? defaults.rangePickerSurfaceTintColor!;
        shape = datePickerTheme.rangePickerShape ?? defaults.rangePickerShape;

      case DatePickerEntryMode.input:
      case DatePickerEntryMode.inputOnly:
        contents = _InputDateRangePickerDialog(
          selectedStartDate: _selectedStart.value,
          selectedEndDate: _selectedEnd.value,
          currentDate: widget.currentDate,
          picker: SizedBox(
            height:
                orientation == Orientation.portrait
                    ? _inputFormPortraitHeight
                    : _inputFormLandscapeHeight,
            child: Padding(
              padding: const EdgeInsets.symmetric(horizontal: 24),
              child: Column(
                children: <Widget>[
                  const Spacer(),
                  _InputDateRangePicker(
                    key: _inputPickerKey,
                    initialStartDate: _selectedStart.value,
                    initialEndDate: _selectedEnd.value,
                    firstDate: widget.firstDate,
                    lastDate: widget.lastDate,
                    selectableDayPredicate: widget.selectableDayPredicate,
                    onStartDateChanged: _handleStartDateChanged,
                    onEndDateChanged: _handleEndDateChanged,
                    autofocus: true,
                    autovalidate: _autoValidate.value,
                    helpText: widget.helpText,
                    errorInvalidRangeText: widget.errorInvalidRangeText,
                    errorFormatText: widget.errorFormatText,
                    errorInvalidText: widget.errorInvalidText,
                    fieldStartHintText: widget.fieldStartHintText,
                    fieldEndHintText: widget.fieldEndHintText,
                    fieldStartLabelText: widget.fieldStartLabelText,
                    fieldEndLabelText: widget.fieldEndLabelText,
                    keyboardType: widget.keyboardType,
                  ),
                  const Spacer(),
                ],
              ),
            ),
          ),
          onConfirm: _handleOk,
          onCancel: _handleCancel,
          entryModeButton:
              showEntryModeButton
                  ? IconButton(
                    icon: widget.switchToCalendarEntryModeIcon ?? const Icon(Icons.calendar_today),
                    padding: EdgeInsets.zero,
                    tooltip: localizations.calendarModeButtonLabel,
                    onPressed: _handleEntryModeToggle,
                  )
                  : null,
          confirmText: widget.confirmText ?? localizations.okButtonLabel,
          cancelText:
              widget.cancelText ??
              (useMaterial3
                  ? localizations.cancelButtonLabel
                  : localizations.cancelButtonLabel.toUpperCase()),
          helpText:
              widget.helpText ??
              (useMaterial3
                  ? localizations.dateRangePickerHelpText
                  : localizations.dateRangePickerHelpText.toUpperCase()),
        );
        final DialogThemeData dialogTheme = theme.dialogTheme;
        size =
            orientation == Orientation.portrait
                ? (useMaterial3 ? _inputPortraitDialogSizeM3 : _inputPortraitDialogSizeM2)
                : _inputRangeLandscapeDialogSize;
        elevation =
            useMaterial3
                ? datePickerTheme.elevation ?? defaults.elevation!
                : datePickerTheme.elevation ?? dialogTheme.elevation ?? 24;
        shadowColor = datePickerTheme.shadowColor ?? defaults.shadowColor;
        surfaceTintColor = datePickerTheme.surfaceTintColor ?? defaults.surfaceTintColor;
        shape =
            useMaterial3
                ? datePickerTheme.shape ?? defaults.shape
                : datePickerTheme.shape ?? dialogTheme.shape ?? defaults.shape;

        insetPadding = const EdgeInsets.symmetric(horizontal: 16.0, vertical: 24.0);
    }

    return Dialog(
      insetPadding: insetPadding,
      backgroundColor: datePickerTheme.backgroundColor ?? defaults.backgroundColor,
      elevation: elevation,
      shadowColor: shadowColor,
      surfaceTintColor: surfaceTintColor,
      shape: shape,
      clipBehavior: Clip.antiAlias,
      child: AnimatedContainer(
        width: size.width,
        height: size.height,
        duration: _dialogSizeAnimationDuration,
        curve: Curves.easeIn,
        child: MediaQuery.withClampedTextScaling(
          maxScaleFactor: _kMaxRangeTextScaleFactor,
          child: Builder(
            builder: (BuildContext context) {
              return contents;
            },
          ),
        ),
      ),
    );
  }
}

class _CalendarRangePickerDialog extends StatelessWidget {
  const _CalendarRangePickerDialog({
    super.key,
    required this.selectedStartDate,
    required this.selectedEndDate,
    required this.firstDate,
    required this.lastDate,
    required this.currentDate,
    required this.onStartDateChanged,
    required this.onEndDateChanged,
    required this.onConfirm,
    required this.onCancel,
    required this.confirmText,
    required this.helpText,
    required this.selectableDayPredicate,
    this.entryModeButton,
  });

  final DateTime? selectedStartDate;
  final DateTime? selectedEndDate;
  final DateTime firstDate;
  final DateTime lastDate;
  final SelectableDayForRangePredicate? selectableDayPredicate;
  final DateTime? currentDate;
  final ValueChanged<DateTime> onStartDateChanged;
  final ValueChanged<DateTime?> onEndDateChanged;
  final VoidCallback? onConfirm;
  final VoidCallback? onCancel;
  final String confirmText;
  final String helpText;
  final Widget? entryModeButton;

  @override
  Widget build(BuildContext context) {
    final ThemeData theme = Theme.of(context);
    final bool useMaterial3 = theme.useMaterial3;
    final MaterialLocalizations localizations = MaterialLocalizations.of(context);
    final Orientation orientation = MediaQuery.orientationOf(context);
    final DatePickerThemeData themeData = DatePickerTheme.of(context);
    final DatePickerThemeData defaults = DatePickerTheme.defaults(context);
    final Color? dialogBackground =
        themeData.rangePickerBackgroundColor ?? defaults.rangePickerBackgroundColor;
    final Color? headerBackground =
        themeData.rangePickerHeaderBackgroundColor ?? defaults.rangePickerHeaderBackgroundColor;
    final Color? headerForeground =
        themeData.rangePickerHeaderForegroundColor ?? defaults.rangePickerHeaderForegroundColor;
    final Color? headerDisabledForeground = headerForeground?.withOpacity(0.38);
    final TextStyle? headlineStyle =
        themeData.rangePickerHeaderHeadlineStyle ?? defaults.rangePickerHeaderHeadlineStyle;
    final TextStyle? headlineHelpStyle = (themeData.rangePickerHeaderHelpStyle ??
            defaults.rangePickerHeaderHelpStyle)
        ?.apply(color: headerForeground);
    final String startDateText = _formatRangeStartDate(
      localizations,
      selectedStartDate,
      selectedEndDate,
    );
    final String endDateText = _formatRangeEndDate(
      localizations,
      selectedStartDate,
      selectedEndDate,
      DateTime.now(),
    );
    final TextStyle? startDateStyle = headlineStyle?.apply(
      color: selectedStartDate != null ? headerForeground : headerDisabledForeground,
    );
    final TextStyle? endDateStyle = headlineStyle?.apply(
      color: selectedEndDate != null ? headerForeground : headerDisabledForeground,
    );
    final ButtonStyle buttonStyle = TextButton.styleFrom(
      foregroundColor: headerForeground,
      disabledForegroundColor: headerDisabledForeground,
    );
    final IconThemeData iconTheme = IconThemeData(color: headerForeground);

    return SafeArea(
      top: false,
      left: false,
      right: false,
      child: Scaffold(
        appBar: AppBar(
          iconTheme: iconTheme,
          actionsIconTheme: iconTheme,
          elevation: useMaterial3 ? 0 : null,
          scrolledUnderElevation: useMaterial3 ? 0 : null,
          backgroundColor: headerBackground,
          leading: CloseButton(onPressed: onCancel),
          actions: <Widget>[
            if (orientation == Orientation.landscape && entryModeButton != null) entryModeButton!,
            TextButton(style: buttonStyle, onPressed: onConfirm, child: Text(confirmText)),
            const SizedBox(width: 8),
          ],
          bottom: PreferredSize(
            preferredSize: const Size(double.infinity, 64),
            child: Row(
              children: <Widget>[
                SizedBox(width: MediaQuery.sizeOf(context).width < 360 ? 42 : 72),
                Expanded(
                  child: Semantics(
                    label: '$helpText $startDateText to $endDateText',
                    excludeSemantics: true,
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      children: <Widget>[
                        Text(
                          helpText,
                          style: headlineHelpStyle,
                          maxLines: 1,
                          overflow: TextOverflow.ellipsis,
                        ),
                        const SizedBox(height: 8),
                        Row(
                          children: <Widget>[
                            Text(
                              startDateText,
                              style: startDateStyle,
                              maxLines: 1,
                              overflow: TextOverflow.ellipsis,
                            ),
                            Text(' – ', style: startDateStyle),
                            Flexible(
                              child: Text(
                                endDateText,
                                style: endDateStyle,
                                maxLines: 1,
                                overflow: TextOverflow.ellipsis,
                              ),
                            ),
                          ],
                        ),
                        const SizedBox(height: 16),
                      ],
                    ),
                  ),
                ),
                if (orientation == Orientation.portrait && entryModeButton != null)
                  Padding(
                    padding: const EdgeInsets.symmetric(horizontal: 8.0),
                    child: IconTheme(data: iconTheme, child: entryModeButton!),
                  ),
              ],
            ),
          ),
        ),
        backgroundColor: dialogBackground,
        body: _CalendarDateRangePicker(
          initialStartDate: selectedStartDate,
          initialEndDate: selectedEndDate,
          firstDate: firstDate,
          lastDate: lastDate,
          currentDate: currentDate,
          onStartDateChanged: onStartDateChanged,
          onEndDateChanged: onEndDateChanged,
          selectableDayPredicate: selectableDayPredicate,
        ),
      ),
    );
  }
}

const Duration _monthScrollDuration = Duration(milliseconds: 200);

const double _monthItemHeaderHeight = 58.0;
const double _monthItemFooterHeight = 12.0;
const double _monthItemRowHeight = 42.0;
const double _monthItemSpaceBetweenRows = 8.0;
const double _horizontalPadding = 8.0;
const double _maxCalendarWidthLandscape = 384.0;
const double _maxCalendarWidthPortrait = 480.0;

/// Displays a scrollable calendar grid that allows a user to select a range
/// of dates.
class _CalendarDateRangePicker extends StatefulWidget {
  /// Creates a scrollable calendar grid for picking date ranges.
  _CalendarDateRangePicker({
    DateTime? initialStartDate,
    DateTime? initialEndDate,
    required DateTime firstDate,
    required DateTime lastDate,
    required this.selectableDayPredicate,
    DateTime? currentDate,
    required this.onStartDateChanged,
    required this.onEndDateChanged,
  }) : initialStartDate = initialStartDate != null ? DateUtils.dateOnly(initialStartDate) : null,
       initialEndDate = initialEndDate != null ? DateUtils.dateOnly(initialEndDate) : null,
       firstDate = DateUtils.dateOnly(firstDate),
       lastDate = DateUtils.dateOnly(lastDate),
       currentDate = DateUtils.dateOnly(currentDate ?? DateTime.now()) {
    assert(
      this.initialStartDate == null ||
          this.initialEndDate == null ||
          !this.initialStartDate!.isAfter(initialEndDate!),
      'initialStartDate must be on or before initialEndDate.',
    );
    assert(!this.lastDate.isBefore(this.firstDate), 'firstDate must be on or before lastDate.');
  }

  /// The [DateTime] that represents the start of the initial date range selection.
  final DateTime? initialStartDate;

  /// The [DateTime] that represents the end of the initial date range selection.
  final DateTime? initialEndDate;

  /// The earliest allowable [DateTime] that the user can select.
  final DateTime firstDate;

  /// The latest allowable [DateTime] that the user can select.
  final DateTime lastDate;

  /// Function to provide full control over which [DateTime] can be selected.
  final SelectableDayForRangePredicate? selectableDayPredicate;

  /// The [DateTime] representing today. It will be highlighted in the day grid.
  final DateTime currentDate;

  /// Called when the user changes the start date of the selected range.
  final ValueChanged<DateTime>? onStartDateChanged;

  /// Called when the user changes the end date of the selected range.
  final ValueChanged<DateTime?>? onEndDateChanged;

  @override
  State<_CalendarDateRangePicker> createState() => _CalendarDateRangePickerState();
}

class _CalendarDateRangePickerState extends State<_CalendarDateRangePicker> {
  final GlobalKey _scrollViewKey = GlobalKey();
  DateTime? _startDate;
  DateTime? _endDate;
  int _initialMonthIndex = 0;
  late ScrollController _controller;
  late bool _showWeekBottomDivider;

  @override
  void initState() {
    super.initState();
    _controller = ScrollController();
    _controller.addListener(_scrollListener);

    _startDate = widget.initialStartDate;
    _endDate = widget.initialEndDate;

    // Calculate the index for the initially displayed month. This is needed to
    // divide the list of months into two `SliverList`s.
    final DateTime initialDate = widget.initialStartDate ?? widget.currentDate;
    if (!initialDate.isBefore(widget.firstDate) && !initialDate.isAfter(widget.lastDate)) {
      _initialMonthIndex = DateUtils.monthDelta(widget.firstDate, initialDate);
    }

    _showWeekBottomDivider = _initialMonthIndex != 0;
  }

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  void _scrollListener() {
    if (_controller.offset <= _controller.position.minScrollExtent) {
      setState(() {
        _showWeekBottomDivider = false;
      });
    } else if (!_showWeekBottomDivider) {
      setState(() {
        _showWeekBottomDivider = true;
      });
    }
  }

  int get _numberOfMonths => DateUtils.monthDelta(widget.firstDate, widget.lastDate) + 1;

  void _vibrate() {
    switch (Theme.of(context).platform) {
      case TargetPlatform.android:
      case TargetPlatform.fuchsia:
        HapticFeedback.vibrate();
      case TargetPlatform.iOS:
      case TargetPlatform.linux:
      case TargetPlatform.macOS:
      case TargetPlatform.windows:
        break;
    }
  }

  // This updates the selected date range using this logic:
  //
  // * From the unselected state, selecting one date creates the start date.
  //   * If the next selection is before the start date, reset date range and
  //     set the start date to that selection.
  //   * If the next selection is on or after the start date, set the end date
  //     to that selection.
  // * After both start and end dates are selected, any subsequent selection
  //   resets the date range and sets start date to that selection.
  void _updateSelection(DateTime date) {
    _vibrate();
    setState(() {
      if (_startDate != null && _endDate == null && !date.isBefore(_startDate!)) {
        _endDate = date;
        widget.onEndDateChanged?.call(_endDate);
      } else {
        _startDate = date;
        widget.onStartDateChanged?.call(_startDate!);
        if (_endDate != null) {
          _endDate = null;
          widget.onEndDateChanged?.call(_endDate);
        }
      }
    });
  }

  Widget _buildMonthItem(BuildContext context, int index, bool beforeInitialMonth) {
    final int monthIndex =
        beforeInitialMonth ? _initialMonthIndex - index - 1 : _initialMonthIndex + index;
    final DateTime month = DateUtils.addMonthsToMonthDate(widget.firstDate, monthIndex);
    return _MonthItem(
      selectedDateStart: _startDate,
      selectedDateEnd: _endDate,
      currentDate: widget.currentDate,
      firstDate: widget.firstDate,
      lastDate: widget.lastDate,
      displayedMonth: month,
      onChanged: _updateSelection,
      selectableDayPredicate: widget.selectableDayPredicate,
    );
  }

  @override
  Widget build(BuildContext context) {
    const Key sliverAfterKey = Key('sliverAfterKey');

    return Column(
      children: <Widget>[
        const _DayHeaders(),
        if (_showWeekBottomDivider) const Divider(height: 0),
        Expanded(
          child: _CalendarKeyboardNavigator(
            firstDate: widget.firstDate,
            lastDate: widget.lastDate,
            initialFocusedDay: _startDate ?? widget.initialStartDate ?? widget.currentDate,
            // In order to prevent performance issues when displaying the
            // correct initial month, 2 `SliverList`s are used to split the
            // months. The first item in the second SliverList is the initial
            // month to be displayed.
            child: CustomScrollView(
              key: _scrollViewKey,
              controller: _controller,
              center: sliverAfterKey,
              slivers: <Widget>[
                SliverList(
                  delegate: SliverChildBuilderDelegate(
                    (BuildContext context, int index) => _buildMonthItem(context, index, true),
                    childCount: _initialMonthIndex,
                  ),
                ),
                SliverList(
                  key: sliverAfterKey,
                  delegate: SliverChildBuilderDelegate(
                    (BuildContext context, int index) => _buildMonthItem(context, index, false),
                    childCount: _numberOfMonths - _initialMonthIndex,
                  ),
                ),
              ],
            ),
          ),
        ),
      ],
    );
  }
}

class _CalendarKeyboardNavigator extends StatefulWidget {
  const _CalendarKeyboardNavigator({
    required this.child,
    required this.firstDate,
    required this.lastDate,
    required this.initialFocusedDay,
  });

  final Widget child;
  final DateTime firstDate;
  final DateTime lastDate;
  final DateTime initialFocusedDay;

  @override
  _CalendarKeyboardNavigatorState createState() => _CalendarKeyboardNavigatorState();
}

class _CalendarKeyboardNavigatorState extends State<_CalendarKeyboardNavigator> {
  final Map<ShortcutActivator, Intent> _shortcutMap = const <ShortcutActivator, Intent>{
    SingleActivator(LogicalKeyboardKey.arrowLeft): DirectionalFocusIntent(TraversalDirection.left),
    SingleActivator(LogicalKeyboardKey.arrowRight): DirectionalFocusIntent(
      TraversalDirection.right,
    ),
    SingleActivator(LogicalKeyboardKey.arrowDown): DirectionalFocusIntent(TraversalDirection.down),
    SingleActivator(LogicalKeyboardKey.arrowUp): DirectionalFocusIntent(TraversalDirection.up),
  };
  late Map<Type, Action<Intent>> _actionMap;
  late FocusNode _dayGridFocus;
  TraversalDirection? _dayTraversalDirection;
  DateTime? _focusedDay;

  @override
  void initState() {
    super.initState();

    _actionMap = <Type, Action<Intent>>{
      NextFocusIntent: CallbackAction<NextFocusIntent>(onInvoke: _handleGridNextFocus),
      PreviousFocusIntent: CallbackAction<PreviousFocusIntent>(onInvoke: _handleGridPreviousFocus),
      DirectionalFocusIntent: CallbackAction<DirectionalFocusIntent>(
        onInvoke: _handleDirectionFocus,
      ),
    };
    _dayGridFocus = FocusNode(debugLabel: 'Day Grid');
  }

  @override
  void dispose() {
    _dayGridFocus.dispose();
    super.dispose();
  }

  void _handleGridFocusChange(bool focused) {
    setState(() {
      if (focused) {
        _focusedDay ??= widget.initialFocusedDay;
      }
    });
  }

  /// Move focus to the next element after the day grid.
  void _handleGridNextFocus(NextFocusIntent intent) {
    _dayGridFocus.requestFocus();
    _dayGridFocus.nextFocus();
  }

  /// Move focus to the previous element before the day grid.
  void _handleGridPreviousFocus(PreviousFocusIntent intent) {
    _dayGridFocus.requestFocus();
    _dayGridFocus.previousFocus();
  }

  /// Move the internal focus date in the direction of the given intent.
  ///
  /// This will attempt to move the focused day to the next selectable day in
  /// the given direction. If the new date is not in the current month, then
  /// the page view will be scrolled to show the new date's month.
  ///
  /// For horizontal directions, it will move forward or backward a day (depending
  /// on the current [TextDirection]). For vertical directions it will move up and
  /// down a week at a time.
  void _handleDirectionFocus(DirectionalFocusIntent intent) {
    assert(_focusedDay != null);
    setState(() {
      final DateTime? nextDate = _nextDateInDirection(_focusedDay!, intent.direction);
      if (nextDate != null) {
        _focusedDay = nextDate;
        _dayTraversalDirection = intent.direction;
      }
    });
  }

  static const Map<TraversalDirection, int> _directionOffset = <TraversalDirection, int>{
    TraversalDirection.up: -DateTime.daysPerWeek,
    TraversalDirection.right: 1,
    TraversalDirection.down: DateTime.daysPerWeek,
    TraversalDirection.left: -1,
  };

  int _dayDirectionOffset(TraversalDirection traversalDirection, TextDirection textDirection) {
    // Swap left and right if the text direction if RTL
    if (textDirection == TextDirection.rtl) {
      if (traversalDirection == TraversalDirection.left) {
        traversalDirection = TraversalDirection.right;
      } else if (traversalDirection == TraversalDirection.right) {
        traversalDirection = TraversalDirection.left;
      }
    }
    return _directionOffset[traversalDirection]!;
  }

  DateTime? _nextDateInDirection(DateTime date, TraversalDirection direction) {
    final TextDirection textDirection = Directionality.of(context);
    final DateTime nextDate = DateUtils.addDaysToDate(
      date,
      _dayDirectionOffset(direction, textDirection),
    );
    if (!nextDate.isBefore(widget.firstDate) && !nextDate.isAfter(widget.lastDate)) {
      return nextDate;
    }
    return null;
  }

  @override
  Widget build(BuildContext context) {
    return FocusableActionDetector(
      shortcuts: _shortcutMap,
      actions: _actionMap,
      focusNode: _dayGridFocus,
      onFocusChange: _handleGridFocusChange,
      child: _FocusedDate(
        date: _dayGridFocus.hasFocus ? _focusedDay : null,
        scrollDirection: _dayGridFocus.hasFocus ? _dayTraversalDirection : null,
        child: widget.child,
      ),
    );
  }
}

/// InheritedWidget indicating what the current focused date is for its children.
// See also: _FocusedDate in calendar_date_picker.dart
class _FocusedDate extends InheritedWidget {
  const _FocusedDate({required super.child, this.date, this.scrollDirection});

  final DateTime? date;
  final TraversalDirection? scrollDirection;

  @override
  bool updateShouldNotify(_FocusedDate oldWidget) {
    return !DateUtils.isSameDay(date, oldWidget.date) ||
        scrollDirection != oldWidget.scrollDirection;
  }

  static _FocusedDate? maybeOf(BuildContext context) {
    return context.dependOnInheritedWidgetOfExactType<_FocusedDate>();
  }
}

class _DayHeaders extends StatelessWidget {
  const _DayHeaders();

  /// Builds widgets showing abbreviated days of week. The first widget in the
  /// returned list corresponds to the first day of week for the current locale.
  ///
  /// Examples:
  ///
  ///     ┌ Sunday is the first day of week in the US (en_US)
  ///     |
  ///     S M T W T F S  ← the returned list contains these widgets
  ///     _ _ _ _ _ 1 2
  ///     3 4 5 6 7 8 9
  ///
  ///     ┌ But it's Monday in the UK (en_GB)
  ///     |
  ///     M T W T F S S  ← the returned list contains these widgets
  ///     _ _ _ _ 1 2 3
  ///     4 5 6 7 8 9 10
  ///
  List<Widget> _getDayHeaders(TextStyle headerStyle, MaterialLocalizations localizations) {
    final List<Widget> result = <Widget>[];
    for (
      int i = localizations.firstDayOfWeekIndex;
      result.length < DateTime.daysPerWeek;
      i = (i + 1) % DateTime.daysPerWeek
    ) {
      final String weekday = localizations.narrowWeekdays[i];
      result.add(ExcludeSemantics(child: Center(child: Text(weekday, style: headerStyle))));
    }
    return result;
  }

  @override
  Widget build(BuildContext context) {
    final ThemeData themeData = Theme.of(context);
    final ColorScheme colorScheme = themeData.colorScheme;
    final TextStyle textStyle = themeData.textTheme.titleSmall!.apply(color: colorScheme.onSurface);
    final MaterialLocalizations localizations = MaterialLocalizations.of(context);
    final List<Widget> labels = _getDayHeaders(textStyle, localizations);

    // Add leading and trailing boxes for edges of the custom grid layout.
    labels.insert(0, const SizedBox.shrink());
    labels.add(const SizedBox.shrink());

    return ConstrainedBox(
      constraints: BoxConstraints(
        maxWidth:
            MediaQuery.orientationOf(context) == Orientation.landscape
                ? _maxCalendarWidthLandscape
                : _maxCalendarWidthPortrait,
        maxHeight: _monthItemRowHeight,
      ),
      child: GridView.custom(
        shrinkWrap: true,
        gridDelegate: _monthItemGridDelegate,
        childrenDelegate: SliverChildListDelegate(labels, addRepaintBoundaries: false),
      ),
    );
  }
}

class _MonthItemGridDelegate extends SliverGridDelegate {
  const _MonthItemGridDelegate();

  @override
  SliverGridLayout getLayout(SliverConstraints constraints) {
    final double tileWidth =
        (constraints.crossAxisExtent - 2 * _horizontalPadding) / DateTime.daysPerWeek;
    return _MonthSliverGridLayout(
      crossAxisCount: DateTime.daysPerWeek + 2,
      dayChildWidth: tileWidth,
      edgeChildWidth: _horizontalPadding,
      reverseCrossAxis: axisDirectionIsReversed(constraints.crossAxisDirection),
    );
  }

  @override
  bool shouldRelayout(_MonthItemGridDelegate oldDelegate) => false;
}

const _MonthItemGridDelegate _monthItemGridDelegate = _MonthItemGridDelegate();

class _MonthSliverGridLayout extends SliverGridLayout {
  /// Creates a layout that uses equally sized and spaced tiles for each day of
  /// the week and an additional edge tile for padding at the start and end of
  /// each row.
  ///
  /// This is necessary to facilitate the painting of the range highlight
  /// correctly.
  const _MonthSliverGridLayout({
    required this.crossAxisCount,
    required this.dayChildWidth,
    required this.edgeChildWidth,
    required this.reverseCrossAxis,
  }) : assert(crossAxisCount > 0),
       assert(dayChildWidth >= 0),
       assert(edgeChildWidth >= 0);

  /// The number of children in the cross axis.
  final int crossAxisCount;

  /// The width in logical pixels of the day child widgets.
  final double dayChildWidth;

  /// The width in logical pixels of the edge child widgets.
  final double edgeChildWidth;

  /// Whether the children should be placed in the opposite order of increasing
  /// coordinates in the cross axis.
  ///
  /// For example, if the cross axis is horizontal, the children are placed from
  /// left to right when [reverseCrossAxis] is false and from right to left when
  /// [reverseCrossAxis] is true.
  ///
  /// Typically set to the return value of [axisDirectionIsReversed] applied to
  /// the [SliverConstraints.crossAxisDirection].
  final bool reverseCrossAxis;

  /// The number of logical pixels from the leading edge of one row to the
  /// leading edge of the next row.
  double get _rowHeight {
    return _monthItemRowHeight + _monthItemSpaceBetweenRows;
  }

  /// The height in logical pixels of the children widgets.
  double get _childHeight {
    return _monthItemRowHeight;
  }

  @override
  int getMinChildIndexForScrollOffset(double scrollOffset) {
    return crossAxisCount * (scrollOffset ~/ _rowHeight);
  }

  @override
  int getMaxChildIndexForScrollOffset(double scrollOffset) {
    final int mainAxisCount = (scrollOffset / _rowHeight).ceil();
    return math.max(0, crossAxisCount * mainAxisCount - 1);
  }

  double _getCrossAxisOffset(double crossAxisStart, bool isPadding) {
    if (reverseCrossAxis) {
      return ((crossAxisCount - 2) * dayChildWidth + 2 * edgeChildWidth) -
          crossAxisStart -
          (isPadding ? edgeChildWidth : dayChildWidth);
    }
    return crossAxisStart;
  }

  @override
  SliverGridGeometry getGeometryForChildIndex(int index) {
    final int adjustedIndex = index % crossAxisCount;
    final bool isEdge = adjustedIndex == 0 || adjustedIndex == crossAxisCount - 1;
    final double crossAxisStart = math.max(0, (adjustedIndex - 1) * dayChildWidth + edgeChildWidth);

    return SliverGridGeometry(
      scrollOffset: (index ~/ crossAxisCount) * _rowHeight,
      crossAxisOffset: _getCrossAxisOffset(crossAxisStart, isEdge),
      mainAxisExtent: _childHeight,
      crossAxisExtent: isEdge ? edgeChildWidth : dayChildWidth,
    );
  }

  @override
  double computeMaxScrollOffset(int childCount) {
    assert(childCount >= 0);
    final int mainAxisCount = ((childCount - 1) ~/ crossAxisCount) + 1;
    final double mainAxisSpacing = _rowHeight - _childHeight;
    return _rowHeight * mainAxisCount - mainAxisSpacing;
  }
}

/// Displays the days of a given month and allows choosing a date range.
///
/// The days are arranged in a rectangular grid with one column for each day of
/// the week.
class _MonthItem extends StatefulWidget {
  /// Creates a month item.
  _MonthItem({
    required this.selectedDateStart,
    required this.selectedDateEnd,
    required this.currentDate,
    required this.onChanged,
    required this.firstDate,
    required this.lastDate,
    required this.displayedMonth,
    required this.selectableDayPredicate,
  }) : assert(!firstDate.isAfter(lastDate)),
       assert(selectedDateStart == null || !selectedDateStart.isBefore(firstDate)),
       assert(selectedDateEnd == null || !selectedDateEnd.isBefore(firstDate)),
       assert(selectedDateStart == null || !selectedDateStart.isAfter(lastDate)),
       assert(selectedDateEnd == null || !selectedDateEnd.isAfter(lastDate)),
       assert(
         selectedDateStart == null ||
             selectedDateEnd == null ||
             !selectedDateStart.isAfter(selectedDateEnd),
       );

  /// The currently selected start date.
  ///
  /// This date is highlighted in the picker.
  final DateTime? selectedDateStart;

  /// The currently selected end date.
  ///
  /// This date is highlighted in the picker.
  final DateTime? selectedDateEnd;

  /// The current date at the time the picker is displayed.
  final DateTime currentDate;

  /// Called when the user picks a day.
  final ValueChanged<DateTime> onChanged;

  /// The earliest date the user is permitted to pick.
  final DateTime firstDate;

  /// The latest date the user is permitted to pick.
  final DateTime lastDate;

  /// The month whose days are displayed by this picker.
  final DateTime displayedMonth;

  final SelectableDayForRangePredicate? selectableDayPredicate;

  @override
  _MonthItemState createState() => _MonthItemState();
}

class _MonthItemState extends State<_MonthItem> {
  /// List of [FocusNode]s, one for each day of the month.
  late List<FocusNode> _dayFocusNodes;

  @override
  void initState() {
    super.initState();
    final int daysInMonth = DateUtils.getDaysInMonth(
      widget.displayedMonth.year,
      widget.displayedMonth.month,
    );
    _dayFocusNodes = List<FocusNode>.generate(
      daysInMonth,
      (int index) => FocusNode(skipTraversal: true, debugLabel: 'Day ${index + 1}'),
    );
  }

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    // Check to see if the focused date is in this month, if so focus it.
    final DateTime? focusedDate = _FocusedDate.maybeOf(context)?.date;
    if (focusedDate != null && DateUtils.isSameMonth(widget.displayedMonth, focusedDate)) {
      _dayFocusNodes[focusedDate.day - 1].requestFocus();
    }
  }

  @override
  void dispose() {
    for (final FocusNode node in _dayFocusNodes) {
      node.dispose();
    }
    super.dispose();
  }

  Color _highlightColor(BuildContext context) {
    return DatePickerTheme.of(context).rangeSelectionBackgroundColor ??
        DatePickerTheme.defaults(context).rangeSelectionBackgroundColor!;
  }

  void _dayFocusChanged(bool focused) {
    if (focused) {
      final TraversalDirection? focusDirection = _FocusedDate.maybeOf(context)?.scrollDirection;
      if (focusDirection != null) {
        ScrollPositionAlignmentPolicy policy = ScrollPositionAlignmentPolicy.explicit;
        switch (focusDirection) {
          case TraversalDirection.up:
          case TraversalDirection.left:
            policy = ScrollPositionAlignmentPolicy.keepVisibleAtStart;
          case TraversalDirection.right:
          case TraversalDirection.down:
            policy = ScrollPositionAlignmentPolicy.keepVisibleAtEnd;
        }
        Scrollable.ensureVisible(
          primaryFocus!.context!,
          duration: _monthScrollDuration,
          alignmentPolicy: policy,
        );
      }
    }
  }

  Widget _buildDayItem(
    BuildContext context,
    DateTime dayToBuild,
    int firstDayOffset,
    int daysInMonth,
  ) {
    final int day = dayToBuild.day;

    final bool isDisabled =
        dayToBuild.isAfter(widget.lastDate) ||
        dayToBuild.isBefore(widget.firstDate) ||
        widget.selectableDayPredicate != null &&
            !widget.selectableDayPredicate!(
              dayToBuild,
              widget.selectedDateStart,
              widget.selectedDateEnd,
            );
    final bool isRangeSelected = widget.selectedDateStart != null && widget.selectedDateEnd != null;
    final bool isSelectedDayStart =
        widget.selectedDateStart != null && dayToBuild.isAtSameMomentAs(widget.selectedDateStart!);
    final bool isSelectedDayEnd =
        widget.selectedDateEnd != null && dayToBuild.isAtSameMomentAs(widget.selectedDateEnd!);
    final bool isInRange =
        isRangeSelected &&
        dayToBuild.isAfter(widget.selectedDateStart!) &&
        dayToBuild.isBefore(widget.selectedDateEnd!);
    final bool isOneDayRange =
        isRangeSelected && widget.selectedDateStart == widget.selectedDateEnd;
    final bool isToday = DateUtils.isSameDay(widget.currentDate, dayToBuild);

    return _DayItem(
      day: dayToBuild,
      focusNode: _dayFocusNodes[day - 1],
      onChanged: widget.onChanged,
      onFocusChange: _dayFocusChanged,
      highlightColor: _highlightColor(context),
      isDisabled: isDisabled,
      isRangeSelected: isRangeSelected,
      isSelectedDayStart: isSelectedDayStart,
      isSelectedDayEnd: isSelectedDayEnd,
      isInRange: isInRange,
      isOneDayRange: isOneDayRange,
      isToday: isToday,
    );
  }

  Widget _buildEdgeBox(BuildContext context, bool isHighlighted) {
    const Widget empty = LimitedBox(maxWidth: 0.0, maxHeight: 0.0, child: SizedBox.expand());
    return isHighlighted ? ColoredBox(color: _highlightColor(context), child: empty) : empty;
  }

  @override
  Widget build(BuildContext context) {
    final ThemeData themeData = Theme.of(context);
    final TextTheme textTheme = themeData.textTheme;
    final MaterialLocalizations localizations = MaterialLocalizations.of(context);
    final int year = widget.displayedMonth.year;
    final int month = widget.displayedMonth.month;
    final int daysInMonth = DateUtils.getDaysInMonth(year, month);
    final int dayOffset = DateUtils.firstDayOffset(year, month, localizations);
    final int weeks = ((daysInMonth + dayOffset) / DateTime.daysPerWeek).ceil();
    final double gridHeight =
        weeks * _monthItemRowHeight + (weeks - 1) * _monthItemSpaceBetweenRows;
    final List<Widget> dayItems = <Widget>[];

    // 1-based day of month, e.g. 1-31 for January, and 1-29 for February on
    // a leap year.
    for (int day = 0 - dayOffset + 1; day <= daysInMonth; day += 1) {
      if (day < 1) {
        dayItems.add(const LimitedBox(maxWidth: 0.0, maxHeight: 0.0, child: SizedBox.expand()));
      } else {
        final DateTime dayToBuild = DateTime(year, month, day);
        final Widget dayItem = _buildDayItem(context, dayToBuild, dayOffset, daysInMonth);
        dayItems.add(dayItem);
      }
    }

    // Add the leading/trailing edge containers to each week in order to
    // correctly extend the range highlight.
    final List<Widget> paddedDayItems = <Widget>[];
    for (int i = 0; i < weeks; i++) {
      final int start = i * DateTime.daysPerWeek;
      final int end = math.min(start + DateTime.daysPerWeek, dayItems.length);
      final List<Widget> weekList = dayItems.sublist(start, end);

      final DateTime dateAfterLeadingPadding = DateTime(year, month, start - dayOffset + 1);
      // Only color the edge container if it is after the start date and
      // on/before the end date.
      final bool isLeadingInRange =
          !(dayOffset > 0 && i == 0) &&
          widget.selectedDateStart != null &&
          widget.selectedDateEnd != null &&
          dateAfterLeadingPadding.isAfter(widget.selectedDateStart!) &&
          !dateAfterLeadingPadding.isAfter(widget.selectedDateEnd!);
      weekList.insert(0, _buildEdgeBox(context, isLeadingInRange));

      // Only add a trailing edge container if it is for a full week and not a
      // partial week.
      if (end < dayItems.length ||
          (end == dayItems.length && dayItems.length % DateTime.daysPerWeek == 0)) {
        final DateTime dateBeforeTrailingPadding = DateTime(year, month, end - dayOffset);
        // Only color the edge container if it is on/after the start date and
        // before the end date.
        final bool isTrailingInRange =
            widget.selectedDateStart != null &&
            widget.selectedDateEnd != null &&
            !dateBeforeTrailingPadding.isBefore(widget.selectedDateStart!) &&
            dateBeforeTrailingPadding.isBefore(widget.selectedDateEnd!);
        weekList.add(_buildEdgeBox(context, isTrailingInRange));
      }

      paddedDayItems.addAll(weekList);
    }

    final double maxWidth =
        MediaQuery.orientationOf(context) == Orientation.landscape
            ? _maxCalendarWidthLandscape
            : _maxCalendarWidthPortrait;
    return Column(
      children: <Widget>[
        ConstrainedBox(
          constraints: BoxConstraints(maxWidth: maxWidth).tighten(height: _monthItemHeaderHeight),
          child: Padding(
            padding: const EdgeInsets.symmetric(horizontal: 16),
            child: Align(
              alignment: AlignmentDirectional.centerStart,
              child: ExcludeSemantics(
                child: Text(
                  localizations.formatMonthYear(widget.displayedMonth),
                  style: textTheme.bodyMedium!.apply(color: themeData.colorScheme.onSurface),
                ),
              ),
            ),
          ),
        ),
        ConstrainedBox(
          constraints: BoxConstraints(maxWidth: maxWidth, maxHeight: gridHeight),
          child: GridView.custom(
            physics: const NeverScrollableScrollPhysics(),
            gridDelegate: _monthItemGridDelegate,
            childrenDelegate: SliverChildListDelegate(paddedDayItems, addRepaintBoundaries: false),
          ),
        ),
        const SizedBox(height: _monthItemFooterHeight),
      ],
    );
  }
}

class _DayItem extends StatefulWidget {
  const _DayItem({
    required this.day,
    required this.focusNode,
    required this.onChanged,
    required this.onFocusChange,
    required this.highlightColor,
    required this.isDisabled,
    required this.isRangeSelected,
    required this.isSelectedDayStart,
    required this.isSelectedDayEnd,
    required this.isInRange,
    required this.isOneDayRange,
    required this.isToday,
  });

  final DateTime day;

  final FocusNode focusNode;

  final ValueChanged<DateTime> onChanged;

  final ValueChanged<bool> onFocusChange;

  final Color highlightColor;

  final bool isDisabled;

  final bool isRangeSelected;

  final bool isSelectedDayStart;

  final bool isSelectedDayEnd;

  final bool isInRange;

  final bool isOneDayRange;

  final bool isToday;

  @override
  State<_DayItem> createState() => _DayItemState();
}

class _DayItemState extends State<_DayItem> {
  final MaterialStatesController _statesController = MaterialStatesController();

  @override
  void dispose() {
    _statesController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final ThemeData theme = Theme.of(context);
    final ColorScheme colorScheme = theme.colorScheme;
    final TextTheme textTheme = theme.textTheme;
    final MaterialLocalizations localizations = MaterialLocalizations.of(context);
    final DatePickerThemeData datePickerTheme = DatePickerTheme.of(context);
    final DatePickerThemeData defaults = DatePickerTheme.defaults(context);
    final TextDirection textDirection = Directionality.of(context);
    final Color highlightColor = widget.highlightColor;

    BoxDecoration? decoration;
    TextStyle? itemStyle = textTheme.bodyMedium;

    T? effectiveValue<T>(T? Function(DatePickerThemeData? theme) getProperty) {
      return getProperty(datePickerTheme) ?? getProperty(defaults);
    }

    T? resolve<T>(
      MaterialStateProperty<T>? Function(DatePickerThemeData? theme) getProperty,
      Set<MaterialState> states,
    ) {
      return effectiveValue((DatePickerThemeData? theme) {
        return getProperty(theme)?.resolve(states);
      });
    }

    final Set<MaterialState> states = <MaterialState>{
      if (widget.isDisabled) MaterialState.disabled,
      if (widget.isSelectedDayStart || widget.isSelectedDayEnd) MaterialState.selected,
    };

    _statesController.value = states;

    final Color? dayForegroundColor = resolve<Color?>(
      (DatePickerThemeData? theme) => theme?.dayForegroundColor,
      states,
    );
    final Color? dayBackgroundColor = resolve<Color?>(
      (DatePickerThemeData? theme) => theme?.dayBackgroundColor,
      states,
    );
    final MaterialStateProperty<Color?> dayOverlayColor = MaterialStateProperty.resolveWith<Color?>(
      (Set<MaterialState> states) => effectiveValue(
        (DatePickerThemeData? theme) =>
            widget.isInRange
                ? theme?.rangeSelectionOverlayColor?.resolve(states)
                : theme?.dayOverlayColor?.resolve(states),
      ),
    );

    _HighlightPainter? highlightPainter;

    if (widget.isSelectedDayStart || widget.isSelectedDayEnd) {
      // The selected start and end dates gets a circle background
      // highlight, and a contrasting text color.
      itemStyle = itemStyle?.apply(color: dayForegroundColor);
      decoration = BoxDecoration(color: dayBackgroundColor, shape: BoxShape.circle);

      if (widget.isRangeSelected && !widget.isOneDayRange) {
        final _HighlightPainterStyle style =
            widget.isSelectedDayStart
                ? _HighlightPainterStyle.highlightTrailing
                : _HighlightPainterStyle.highlightLeading;
        highlightPainter = _HighlightPainter(
          color: highlightColor,
          style: style,
          textDirection: textDirection,
        );
      }
    } else if (widget.isInRange) {
      // The days within the range get a light background highlight.
      highlightPainter = _HighlightPainter(
        color: highlightColor,
        style: _HighlightPainterStyle.highlightAll,
        textDirection: textDirection,
      );
      if (widget.isDisabled) {
        itemStyle = itemStyle?.apply(color: colorScheme.onSurface.withOpacity(0.38));
      }
    } else if (widget.isDisabled) {
      itemStyle = itemStyle?.apply(color: colorScheme.onSurface.withOpacity(0.38));
    } else if (widget.isToday) {
      // The current day gets a different text color and a circle stroke
      // border.
      itemStyle = itemStyle?.apply(color: colorScheme.primary);
      decoration = BoxDecoration(
        border: Border.all(color: colorScheme.primary),
        shape: BoxShape.circle,
      );
    }

    final String dayText = localizations.formatDecimal(widget.day.day);

    // We want the day of month to be spoken first irrespective of the
    // locale-specific preferences or TextDirection. This is because
    // an accessibility user is more likely to be interested in the
    // day of month before the rest of the date, as they are looking
    // for the day of month. To do that we prepend day of month to the
    // formatted full date.
    final String semanticLabelSuffix = widget.isToday ? ', ${localizations.currentDateLabel}' : '';
    String semanticLabel =
        '$dayText, ${localizations.formatFullDate(widget.day)}$semanticLabelSuffix';
    if (widget.isSelectedDayStart) {
      semanticLabel = localizations.dateRangeStartDateSemanticLabel(semanticLabel);
    } else if (widget.isSelectedDayEnd) {
      semanticLabel = localizations.dateRangeEndDateSemanticLabel(semanticLabel);
    }

    Widget dayWidget = Container(
      decoration: decoration,
      alignment: Alignment.center,
      child: Semantics(
        label: semanticLabel,
        selected: widget.isSelectedDayStart || widget.isSelectedDayEnd,
        child: ExcludeSemantics(child: Text(dayText, style: itemStyle)),
      ),
    );

    if (highlightPainter != null) {
      dayWidget = CustomPaint(painter: highlightPainter, child: dayWidget);
    }

    if (!widget.isDisabled) {
      dayWidget = InkResponse(
        focusNode: widget.focusNode,
        onTap: () => widget.onChanged(widget.day),
        radius: _monthItemRowHeight / 2 + 4,
        statesController: _statesController,
        overlayColor: dayOverlayColor,
        onFocusChange: widget.onFocusChange,
        child: dayWidget,
      );
    }

    return dayWidget;
  }
}

/// Determines which style to use to paint the highlight.
enum _HighlightPainterStyle {
  /// Paints nothing.
  none,

  /// Paints a rectangle that occupies the leading half of the space.
  highlightLeading,

  /// Paints a rectangle that occupies the trailing half of the space.
  highlightTrailing,

  /// Paints a rectangle that occupies all available space.
  highlightAll,
}

/// This custom painter will add a background highlight to its child.
///
/// This highlight will be drawn depending on the [style], [color], and
/// [textDirection] supplied. It will either paint a rectangle on the
/// left/right, a full rectangle, or nothing at all. This logic is determined by
/// a combination of the [style] and [textDirection].
class _HighlightPainter extends CustomPainter {
  _HighlightPainter({
    required this.color,
    this.style = _HighlightPainterStyle.none,
    this.textDirection,
  });

  final Color color;
  final _HighlightPainterStyle style;
  final TextDirection? textDirection;

  @override
  void paint(Canvas canvas, Size size) {
    if (style == _HighlightPainterStyle.none) {
      return;
    }

    final Paint paint =
        Paint()
          ..color = color
          ..style = PaintingStyle.fill;

    final bool rtl = switch (textDirection) {
      TextDirection.rtl || null => true,
      TextDirection.ltr => false,
    };

    switch (style) {
      case _HighlightPainterStyle.highlightLeading when rtl:
      case _HighlightPainterStyle.highlightTrailing when !rtl:
        canvas.drawRect(Rect.fromLTWH(size.width / 2, 0, size.width / 2, size.height), paint);
      case _HighlightPainterStyle.highlightLeading:
      case _HighlightPainterStyle.highlightTrailing:
        canvas.drawRect(Rect.fromLTWH(0, 0, size.width / 2, size.height), paint);
      case _HighlightPainterStyle.highlightAll:
        canvas.drawRect(Rect.fromLTWH(0, 0, size.width, size.height), paint);
      case _HighlightPainterStyle.none:
        break;
    }
  }

  @override
  bool shouldRepaint(CustomPainter oldDelegate) => false;
}

class _InputDateRangePickerDialog extends StatelessWidget {
  const _InputDateRangePickerDialog({
    required this.selectedStartDate,
    required this.selectedEndDate,
    required this.currentDate,
    required this.picker,
    required this.onConfirm,
    required this.onCancel,
    required this.confirmText,
    required this.cancelText,
    required this.helpText,
    required this.entryModeButton,
  });

  final DateTime? selectedStartDate;
  final DateTime? selectedEndDate;
  final DateTime? currentDate;
  final Widget picker;
  final VoidCallback onConfirm;
  final VoidCallback onCancel;
  final String? confirmText;
  final String? cancelText;
  final String? helpText;
  final Widget? entryModeButton;

  String _formatDateRange(BuildContext context, DateTime? start, DateTime? end, DateTime now) {
    final MaterialLocalizations localizations = MaterialLocalizations.of(context);
    final String startText = _formatRangeStartDate(localizations, start, end);
    final String endText = _formatRangeEndDate(localizations, start, end, now);
    if (start == null || end == null) {
      return localizations.unspecifiedDateRange;
    }
    return switch (Directionality.of(context)) {
      TextDirection.rtl => '$endText – $startText',
      TextDirection.ltr => '$startText – $endText',
    };
  }

  @override
  Widget build(BuildContext context) {
    final bool useMaterial3 = Theme.of(context).useMaterial3;
    final MaterialLocalizations localizations = MaterialLocalizations.of(context);
    final Orientation orientation = MediaQuery.orientationOf(context);
    final DatePickerThemeData datePickerTheme = DatePickerTheme.of(context);
    final DatePickerThemeData defaults = DatePickerTheme.defaults(context);

    // There's no M3 spec for a landscape layout input (not calendar)
    // date range picker. To ensure that the date range displayed in the
    // input date range picker's header fits in landscape mode, we override
    // the M3 default here.
    TextStyle? headlineStyle =
        (orientation == Orientation.portrait)
            ? datePickerTheme.headerHeadlineStyle ?? defaults.headerHeadlineStyle
            : Theme.of(context).textTheme.headlineSmall;

    final Color? headerForegroundColor =
        datePickerTheme.headerForegroundColor ?? defaults.headerForegroundColor;
    headlineStyle = headlineStyle?.copyWith(color: headerForegroundColor);

    final String dateText = _formatDateRange(
      context,
      selectedStartDate,
      selectedEndDate,
      currentDate!,
    );
    final String semanticDateText =
        selectedStartDate != null && selectedEndDate != null
            ? '${localizations.formatMediumDate(selectedStartDate!)} – ${localizations.formatMediumDate(selectedEndDate!)}'
            : '';

    final Widget header = _DatePickerHeader(
      helpText:
          helpText ??
          (useMaterial3
              ? localizations.dateRangePickerHelpText
              : localizations.dateRangePickerHelpText.toUpperCase()),
      titleText: dateText,
      titleSemanticsLabel: semanticDateText,
      titleStyle: headlineStyle,
      orientation: orientation,
      isShort: orientation == Orientation.landscape,
      entryModeButton: entryModeButton,
    );

    final Widget actions = ConstrainedBox(
      constraints: const BoxConstraints(minHeight: 52.0),
      child: Padding(
        padding: const EdgeInsets.symmetric(horizontal: 8),
        child: Align(
          alignment: AlignmentDirectional.centerEnd,
          child: OverflowBar(
            spacing: 8,
            children: <Widget>[
              TextButton(
                onPressed: onCancel,
                child: Text(
                  cancelText ??
                      (useMaterial3
                          ? localizations.cancelButtonLabel
                          : localizations.cancelButtonLabel.toUpperCase()),
                ),
              ),
              TextButton(
                onPressed: onConfirm,
                child: Text(confirmText ?? localizations.okButtonLabel),
              ),
            ],
          ),
        ),
      ),
    );

    final double textScaleFactor =
        MediaQuery.textScalerOf(
          context,
        ).clamp(maxScaleFactor: _kMaxRangeTextScaleFactor).scale(_fontSizeToScale) /
        _fontSizeToScale;
    final Size dialogSize =
        (useMaterial3 ? _inputPortraitDialogSizeM3 : _inputPortraitDialogSizeM2) * textScaleFactor;
    switch (orientation) {
      case Orientation.portrait:
        return LayoutBuilder(
          builder: (BuildContext context, BoxConstraints constraints) {
            final Size portraitDialogSize =
                useMaterial3 ? _inputPortraitDialogSizeM3 : _inputPortraitDialogSizeM2;
            // Make sure the portrait dialog can fit the contents comfortably when
            // resized from the landscape dialog.
            final bool isFullyPortrait =
                constraints.maxHeight >= math.min(dialogSize.height, portraitDialogSize.height);

            return Column(
              mainAxisSize: MainAxisSize.min,
              crossAxisAlignment: CrossAxisAlignment.stretch,
              children: <Widget>[
                header,
                if (isFullyPortrait) ...<Widget>[Expanded(child: picker), actions],
              ],
            );
          },
        );

      case Orientation.landscape:
        return Row(
          mainAxisSize: MainAxisSize.min,
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: <Widget>[
            header,
            Flexible(
              child: Column(
                mainAxisSize: MainAxisSize.min,
                crossAxisAlignment: CrossAxisAlignment.stretch,
                children: <Widget>[Expanded(child: picker), actions],
              ),
            ),
          ],
        );
    }
  }
}

/// Provides a pair of text fields that allow the user to enter the start and
/// end dates that represent a range of dates.
class _InputDateRangePicker extends StatefulWidget {
  /// Creates a row with two text fields configured to accept the start and end dates
  /// of a date range.
  _InputDateRangePicker({
    super.key,
    DateTime? initialStartDate,
    DateTime? initialEndDate,
    required DateTime firstDate,
    required DateTime lastDate,
    required this.onStartDateChanged,
    required this.onEndDateChanged,
    required this.selectableDayPredicate,
    this.helpText,
    this.errorFormatText,
    this.errorInvalidText,
    this.errorInvalidRangeText,
    this.fieldStartHintText,
    this.fieldEndHintText,
    this.fieldStartLabelText,
    this.fieldEndLabelText,
    this.autofocus = false,
    this.autovalidate = false,
    this.keyboardType = TextInputType.datetime,
  }) : initialStartDate = initialStartDate == null ? null : DateUtils.dateOnly(initialStartDate),
       initialEndDate = initialEndDate == null ? null : DateUtils.dateOnly(initialEndDate),
       firstDate = DateUtils.dateOnly(firstDate),
       lastDate = DateUtils.dateOnly(lastDate);

  /// The [DateTime] that represents the start of the initial date range selection.
  final DateTime? initialStartDate;

  /// The [DateTime] that represents the end of the initial date range selection.
  final DateTime? initialEndDate;

  /// The earliest allowable [DateTime] that the user can select.
  final DateTime firstDate;

  /// The latest allowable [DateTime] that the user can select.
  final DateTime lastDate;

  /// Called when the user changes the start date of the selected range.
  final ValueChanged<DateTime?>? onStartDateChanged;

  /// Called when the user changes the end date of the selected range.
  final ValueChanged<DateTime?>? onEndDateChanged;

  /// The text that is displayed at the top of the header.
  ///
  /// This is used to indicate to the user what they are selecting a date for.
  final String? helpText;

  /// Error text used to indicate the text in a field is not a valid date.
  final String? errorFormatText;

  /// Error text used to indicate the date in a field is not in the valid range
  /// of [firstDate] - [lastDate].
  final String? errorInvalidText;

  /// Error text used to indicate the dates given don't form a valid date
  /// range (i.e. the start date is after the end date).
  final String? errorInvalidRangeText;

  /// Hint text shown when the start date field is empty.
  final String? fieldStartHintText;

  /// Hint text shown when the end date field is empty.
  final String? fieldEndHintText;

  /// Label used for the start date field.
  final String? fieldStartLabelText;

  /// Label used for the end date field.
  final String? fieldEndLabelText;

  /// {@macro flutter.widgets.editableText.autofocus}
  final bool autofocus;

  /// If true, the date fields will validate and update their error text
  /// immediately after every change. Otherwise, you must call
  /// [_InputDateRangePickerState.validate] to validate.
  final bool autovalidate;

  /// {@macro flutter.material.datePickerDialog}
  final TextInputType keyboardType;

  final SelectableDayForRangePredicate? selectableDayPredicate;

  @override
  _InputDateRangePickerState createState() => _InputDateRangePickerState();
}

/// The current state of an [_InputDateRangePicker]. Can be used to
/// [validate] the date field entries.
class _InputDateRangePickerState extends State<_InputDateRangePicker> {
  late String _startInputText;
  late String _endInputText;
  DateTime? _startDate;
  DateTime? _endDate;
  late TextEditingController _startController;
  late TextEditingController _endController;
  String? _startErrorText;
  String? _endErrorText;
  bool _autoSelected = false;

  @override
  void initState() {
    super.initState();
    _startDate = widget.initialStartDate;
    _startController = TextEditingController();
    _endDate = widget.initialEndDate;
    _endController = TextEditingController();
  }

  @override
  void dispose() {
    _startController.dispose();
    _endController.dispose();
    super.dispose();
  }

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    final MaterialLocalizations localizations = MaterialLocalizations.of(context);
    if (_startDate != null) {
      _startInputText = localizations.formatCompactDate(_startDate!);
      final bool selectText = widget.autofocus && !_autoSelected;
      _updateController(_startController, _startInputText, selectText);
      _autoSelected = selectText;
    }

    if (_endDate != null) {
      _endInputText = localizations.formatCompactDate(_endDate!);
      _updateController(_endController, _endInputText, false);
    }
  }

  /// Validates that the text in the start and end fields represent a valid
  /// date range.
  ///
  /// Will return true if the range is valid. If not, it will
  /// return false and display an appropriate error message under one of the
  /// text fields.
  bool validate() {
    String? startError = _validateDate(_startDate);
    final String? endError = _validateDate(_endDate);
    if (startError == null && endError == null) {
      if (_startDate!.isAfter(_endDate!)) {
        startError =
            widget.errorInvalidRangeText ?? MaterialLocalizations.of(context).invalidDateRangeLabel;
      }
    }
    setState(() {
      _startErrorText = startError;
      _endErrorText = endError;
    });
    return startError == null && endError == null;
  }

  DateTime? _parseDate(String? text) {
    final MaterialLocalizations localizations = MaterialLocalizations.of(context);
    return localizations.parseCompactDate(text);
  }

  String? _validateDate(DateTime? date) {
    if (date == null) {
      return widget.errorFormatText ?? MaterialLocalizations.of(context).invalidDateFormatLabel;
    } else if (!_isDaySelectable(date)) {
      return widget.errorInvalidText ?? MaterialLocalizations.of(context).dateOutOfRangeLabel;
    }
    return null;
  }

  bool _isDaySelectable(DateTime day) {
    if (day.isBefore(widget.firstDate) || day.isAfter(widget.lastDate)) {
      return false;
    }
    if (widget.selectableDayPredicate == null) {
      return true;
    }
    return widget.selectableDayPredicate!(day, _startDate, _endDate);
  }

  void _updateController(TextEditingController controller, String text, bool selectText) {
    TextEditingValue textEditingValue = controller.value.copyWith(text: text);
    if (selectText) {
      textEditingValue = textEditingValue.copyWith(
        selection: TextSelection(baseOffset: 0, extentOffset: text.length),
      );
    }
    controller.value = textEditingValue;
  }

  void _handleStartChanged(String text) {
    setState(() {
      _startInputText = text;
      _startDate = _parseDate(text);
      widget.onStartDateChanged?.call(_startDate);
    });
    if (widget.autovalidate) {
      validate();
    }
  }

  void _handleEndChanged(String text) {
    setState(() {
      _endInputText = text;
      _endDate = _parseDate(text);
      widget.onEndDateChanged?.call(_endDate);
    });
    if (widget.autovalidate) {
      validate();
    }
  }

  @override
  Widget build(BuildContext context) {
    final ThemeData theme = Theme.of(context);
    final bool useMaterial3 = theme.useMaterial3;
    final MaterialLocalizations localizations = MaterialLocalizations.of(context);
    final InputDecorationTheme inputTheme = theme.inputDecorationTheme;
    final InputBorder inputBorder =
        inputTheme.border ??
        (useMaterial3 ? const OutlineInputBorder() : const UnderlineInputBorder());

    return Row(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: <Widget>[
        Expanded(
          child: TextField(
            controller: _startController,
            decoration: InputDecoration(
              border: inputBorder,
              filled: inputTheme.filled,
              hintText: widget.fieldStartHintText ?? localizations.dateHelpText,
              labelText: widget.fieldStartLabelText ?? localizations.dateRangeStartLabel,
              errorText: _startErrorText,
            ),
            keyboardType: widget.keyboardType,
            onChanged: _handleStartChanged,
            autofocus: widget.autofocus,
          ),
        ),
        const SizedBox(width: 8),
        Expanded(
          child: TextField(
            controller: _endController,
            decoration: InputDecoration(
              border: inputBorder,
              filled: inputTheme.filled,
              hintText: widget.fieldEndHintText ?? localizations.dateHelpText,
              labelText: widget.fieldEndLabelText ?? localizations.dateRangeEndLabel,
              errorText: _endErrorText,
            ),
            keyboardType: widget.keyboardType,
            onChanged: _handleEndChanged,
          ),
        ),
      ],
    );
  }
}
