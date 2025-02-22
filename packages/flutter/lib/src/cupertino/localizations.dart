// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'bottom_tab_bar.dart';
/// @docImport 'date_picker.dart';
/// @docImport 'search_field.dart';
library;

import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';

import 'debug.dart';

// Examples can assume:
// late BuildContext context;

/// Determines the order of the columns inside [CupertinoDatePicker] in
/// time and date time mode.
enum DatePickerDateTimeOrder {
  /// Order of the columns, from left to right: date, hour, minute, am/pm.
  ///
  /// Example: Fri Aug 31 | 02 | 08 | PM.
  date_time_dayPeriod,

  /// Order of the columns, from left to right: date, am/pm, hour, minute.
  ///
  /// Example: Fri Aug 31 | PM | 02 | 08.
  date_dayPeriod_time,

  /// Order of the columns, from left to right: hour, minute, am/pm, date.
  ///
  /// Example: 02 | 08 | PM | Fri Aug 31.
  time_dayPeriod_date,

  /// Order of the columns, from left to right: am/pm, hour, minute, date.
  ///
  /// Example: PM | 02 | 08 | Fri Aug 31.
  dayPeriod_time_date,
}

/// Determines the order of the columns inside [CupertinoDatePicker] in date mode.
enum DatePickerDateOrder {
  /// Order of the columns, from left to right: day, month, year.
  ///
  /// Example: 12 | March | 1996.
  dmy,

  /// Order of the columns, from left to right: month, day, year.
  ///
  /// Example: March | 12 | 1996.
  mdy,

  /// Order of the columns, from left to right: year, month, day.
  ///
  /// Example: 1996 | March | 12.
  ymd,

  /// Order of the columns, from left to right: year, day, month.
  ///
  /// Example: 1996 | 12 | March.
  ydm,
}

/// Defines the localized resource values used by the Cupertino widgets.
///
/// See also:
///
///  * [DefaultCupertinoLocalizations], the default, English-only, implementation
///    of this interface.
abstract class CupertinoLocalizations {
  /// Year that is shown in [CupertinoDatePicker] spinner corresponding to the
  /// given year index.
  ///
  /// Examples: datePickerYear(1) in:
  ///
  ///  - US English: 2018
  ///  - Korean: 2018년
  // The global version uses date symbols data from the intl package.
  String datePickerYear(int yearIndex);

  /// Month that is shown in [CupertinoDatePicker] spinner corresponding to
  /// the given month index.
  ///
  /// Examples: datePickerMonth(1) in:
  ///
  ///  - US English: January
  ///  - Korean: 1월
  ///  - Russian: января
  // The global version uses date symbols data from the intl package.
  String datePickerMonth(int monthIndex);

  /// Month that is shown in [CupertinoDatePicker] spinner corresponding to
  /// the given month index in [CupertinoDatePickerMode.monthYear] mode.
  ///
  /// This is distinct from [datePickerMonth] because in some languages, like Russian,
  /// the name of a month takes a different form depending
  /// on whether it is preceded by a day or whether it stands alone.
  ///
  /// Examples: datePickerMonth(1) in:
  ///
  ///  - US English: January
  ///  - Korean: 1월
  ///  - Russian: Январь
  // The global version uses date symbols data from the intl package.
  String datePickerStandaloneMonth(int monthIndex);

  /// Day of month that is shown in [CupertinoDatePicker] spinner corresponding
  /// to the given day index.
  ///
  /// If weekDay is provided then it will also show weekday name alongside the numerical day.
  ///
  /// Examples: datePickerDayOfMonth(1) in:
  ///
  ///  - US English: 1
  ///  - Korean: 1일
  /// Examples: datePickerDayOfMonth(1, 1) in:
  ///
  ///  - US English: Mon 1
  // The global version uses date symbols data from the intl package.
  String datePickerDayOfMonth(int dayIndex, [int? weekDay]);

  /// The medium-width date format that is shown in [CupertinoDatePicker]
  /// spinner. Abbreviates month and days of week.
  ///
  /// Examples:
  ///
  /// - US English: Wed Sep 27
  /// - Russian: ср сент. 27
  // The global version is based on intl package's DateFormat.MMMEd.
  String datePickerMediumDate(DateTime date);

  /// Hour that is shown in [CupertinoDatePicker] spinner corresponding
  /// to the given hour value.
  ///
  /// Examples: datePickerHour(1) in:
  ///
  ///  - US English: 1
  ///  - Arabic: ٠١
  // The global version uses date symbols data from the intl package.
  String datePickerHour(int hour);

  /// Semantics label for the given hour value in [CupertinoDatePicker].
  // The global version uses the translated string from the arb file.
  String? datePickerHourSemanticsLabel(int hour);

  /// Minute that is shown in [CupertinoDatePicker] spinner corresponding
  /// to the given minute value.
  ///
  /// Examples: datePickerMinute(1) in:
  ///
  ///  - US English: 01
  ///  - Arabic: ٠١
  // The global version uses date symbols data from the intl package.
  String datePickerMinute(int minute);

  /// Semantics label for the given minute value in [CupertinoDatePicker].
  // The global version uses the translated string from the arb file.
  String? datePickerMinuteSemanticsLabel(int minute);

  /// The order of the date elements that will be shown in [CupertinoDatePicker].
  // The global version uses the translated string from the arb file.
  DatePickerDateOrder get datePickerDateOrder;

  /// The order of the time elements that will be shown in [CupertinoDatePicker].
  // The global version uses the translated string from the arb file.
  DatePickerDateTimeOrder get datePickerDateTimeOrder;

  /// The abbreviation for ante meridiem (before noon) shown in the time picker.
  // The global version uses the translated string from the arb file.
  String get anteMeridiemAbbreviation;

  /// The abbreviation for post meridiem (after noon) shown in the time picker.
  // The global version uses the translated string from the arb file.
  String get postMeridiemAbbreviation;

  /// Label shown in date pickers when the date is today.
  // The global version uses the translated string from the arb file.
  String get todayLabel;

  /// The term used by the system to announce dialog alerts.
  // The global version uses the translated string from the arb file.
  String get alertDialogLabel;

  /// The accessibility label used on a tab in a [CupertinoTabBar].
  ///
  /// This message describes the index of the selected tab and how many tabs
  /// there are, e.g. 'tab, 1 of 2' in United States English.
  ///
  /// `tabIndex` and `tabCount` must be greater than or equal to one.
  String tabSemanticsLabel({required int tabIndex, required int tabCount});

  /// Hour that is shown in [CupertinoTimerPicker] corresponding to
  /// the given hour value.
  ///
  /// Examples: timerPickerHour(1) in:
  ///
  ///  - US English: 1
  ///  - Arabic: ١
  // The global version uses date symbols data from the intl package.
  String timerPickerHour(int hour);

  /// Minute that is shown in [CupertinoTimerPicker] corresponding to
  /// the given minute value.
  ///
  /// Examples: timerPickerMinute(1) in:
  ///
  ///  - US English: 1
  ///  - Arabic: ١
  // The global version uses date symbols data from the intl package.
  String timerPickerMinute(int minute);

  /// Second that is shown in [CupertinoTimerPicker] corresponding to
  /// the given second value.
  ///
  /// Examples: timerPickerSecond(1) in:
  ///
  ///  - US English: 1
  ///  - Arabic: ١
  // The global version uses date symbols data from the intl package.
  String timerPickerSecond(int second);

  /// Label that appears next to the hour picker in
  /// [CupertinoTimerPicker] when selected hour value is `hour`.
  /// This function will deal with pluralization based on the `hour` parameter.
  // The global version uses the translated string from the arb file.
  String? timerPickerHourLabel(int hour);

  /// All possible hour labels that appears next to the hour picker in
  /// [CupertinoTimerPicker]
  List<String> get timerPickerHourLabels;

  /// Label that appears next to the minute picker in
  /// [CupertinoTimerPicker] when selected minute value is `minute`.
  /// This function will deal with pluralization based on the `minute` parameter.
  // The global version uses the translated string from the arb file.
  String? timerPickerMinuteLabel(int minute);

  /// All possible minute labels that appears next to the minute picker in
  /// [CupertinoTimerPicker]
  List<String> get timerPickerMinuteLabels;

  /// Label that appears next to the minute picker in
  /// [CupertinoTimerPicker] when selected minute value is `second`.
  /// This function will deal with pluralization based on the `second` parameter.
  // The global version uses the translated string from the arb file.
  String? timerPickerSecondLabel(int second);

  /// All possible second labels that appears next to the second picker in
  /// [CupertinoTimerPicker]
  List<String> get timerPickerSecondLabels;

  /// The term used for cutting.
  // The global version uses the translated string from the arb file.
  String get cutButtonLabel;

  /// The term used for copying.
  // The global version uses the translated string from the arb file.
  String get copyButtonLabel;

  /// The term used for pasting.
  // The global version uses the translated string from the arb file.
  String get pasteButtonLabel;

  /// The term used for clearing a field.
  // The global version uses the translated string from the arb file.
  String get clearButtonLabel;

  /// Label that appears in the Cupertino toolbar when the spell checker
  /// couldn't find any replacements for the current word.
  // The global version uses the translated string from the arb file.
  String get noSpellCheckReplacementsLabel;

  /// The term used for selecting everything.
  // The global version uses the translated string from the arb file.
  String get selectAllButtonLabel;

  /// The term used for looking up a selection.
  // The global version uses the translated string from the arb file.
  String get lookUpButtonLabel;

  /// The term used for launching a web search on a selection.
  // The global version uses the translated string from the arb file.
  String get searchWebButtonLabel;

  /// The term used for launching a web search on a selection.
  // The global version uses the translated string from the arb file.
  String get shareButtonLabel;

  /// The default placeholder used in [CupertinoSearchTextField].
  // The global version uses the translated string from the arb file.
  String get searchTextFieldPlaceholderLabel;

  /// Label read out by accessibility tools (VoiceOver) for a modal
  /// barrier to indicate that a tap dismisses the barrier.
  ///
  /// A modal barrier can for example be found behind an alert or popup to block
  /// user interaction with elements behind it.
  String get modalBarrierDismissLabel;

  /// Label read out by accessibility tools (VoiceOver) for a context menu to
  /// indicate that a tap outside dismisses the context menu.
  String get menuDismissLabel;

  /// The label for the cancel button in modal views, used in [CupertinoNavigationBar] and [CupertinoSliverNavigationBar].
  String get cancelButtonLabel;

  /// The label for the back button, used in [CupertinoNavigationBar] and [CupertinoSliverNavigationBar].
  String get backButtonLabel;

  /// The `CupertinoLocalizations` from the closest [Localizations] instance
  /// that encloses the given context.
  ///
  /// If no [CupertinoLocalizations] are available in the given `context`, this
  /// method throws an exception.
  ///
  /// This method is just a convenient shorthand for:
  /// `Localizations.of<CupertinoLocalizations>(context, CupertinoLocalizations)!`.
  ///
  /// References to the localized resources defined by this class are typically
  /// written in terms of this method. For example:
  ///
  /// ```dart
  /// CupertinoLocalizations.of(context).anteMeridiemAbbreviation;
  /// ```
  static CupertinoLocalizations of(BuildContext context) {
    assert(debugCheckHasCupertinoLocalizations(context));
    return Localizations.of<CupertinoLocalizations>(context, CupertinoLocalizations)!;
  }
}

class _CupertinoLocalizationsDelegate extends LocalizationsDelegate<CupertinoLocalizations> {
  const _CupertinoLocalizationsDelegate();

  @override
  bool isSupported(Locale locale) => locale.languageCode == 'en';

  @override
  Future<CupertinoLocalizations> load(Locale locale) => DefaultCupertinoLocalizations.load(locale);

  @override
  bool shouldReload(_CupertinoLocalizationsDelegate old) => false;

  @override
  String toString() => 'DefaultCupertinoLocalizations.delegate(en_US)';
}

/// US English strings for the Cupertino widgets.
class DefaultCupertinoLocalizations implements CupertinoLocalizations {
  /// Constructs an object that defines the cupertino widgets' localized strings
  /// for US English (only).
  ///
  /// [LocalizationsDelegate] implementations typically call the static [load]
  /// function, rather than constructing this class directly.
  const DefaultCupertinoLocalizations();

  /// Short version of days of week.
  static const List<String> _shortWeekdays = <String>[
    'Mon',
    'Tue',
    'Wed',
    'Thu',
    'Fri',
    'Sat',
    'Sun',
  ];

  static const List<String> _shortMonths = <String>[
    'Jan',
    'Feb',
    'Mar',
    'Apr',
    'May',
    'Jun',
    'Jul',
    'Aug',
    'Sep',
    'Oct',
    'Nov',
    'Dec',
  ];

  static const List<String> _months = <String>[
    'January',
    'February',
    'March',
    'April',
    'May',
    'June',
    'July',
    'August',
    'September',
    'October',
    'November',
    'December',
  ];

  @override
  String datePickerYear(int yearIndex) => yearIndex.toString();

  @override
  String datePickerMonth(int monthIndex) => _months[monthIndex - 1];

  @override
  String datePickerStandaloneMonth(int monthIndex) => _months[monthIndex - 1];

  @override
  String datePickerDayOfMonth(int dayIndex, [int? weekDay]) {
    if (weekDay != null) {
      return ' ${_shortWeekdays[weekDay - DateTime.monday]} $dayIndex ';
    }

    return dayIndex.toString();
  }

  @override
  String datePickerHour(int hour) => hour.toString();

  @override
  String datePickerHourSemanticsLabel(int hour) => "$hour o'clock";

  @override
  String datePickerMinute(int minute) => minute.toString().padLeft(2, '0');

  @override
  String datePickerMinuteSemanticsLabel(int minute) {
    if (minute == 1) {
      return '1 minute';
    }
    return '$minute minutes';
  }

  @override
  String datePickerMediumDate(DateTime date) {
    return '${_shortWeekdays[date.weekday - DateTime.monday]} '
        '${_shortMonths[date.month - DateTime.january]} '
        '${date.day.toString().padRight(2)}';
  }

  @override
  DatePickerDateOrder get datePickerDateOrder => DatePickerDateOrder.mdy;

  @override
  DatePickerDateTimeOrder get datePickerDateTimeOrder =>
      DatePickerDateTimeOrder.date_time_dayPeriod;

  @override
  String get anteMeridiemAbbreviation => 'AM';

  @override
  String get postMeridiemAbbreviation => 'PM';

  @override
  String get todayLabel => 'Today';

  @override
  String get alertDialogLabel => 'Alert';

  @override
  String tabSemanticsLabel({required int tabIndex, required int tabCount}) {
    assert(tabIndex >= 1);
    assert(tabCount >= 1);
    return 'Tab $tabIndex of $tabCount';
  }

  @override
  String timerPickerHour(int hour) => hour.toString();

  @override
  String timerPickerMinute(int minute) => minute.toString();

  @override
  String timerPickerSecond(int second) => second.toString();

  @override
  String timerPickerHourLabel(int hour) => hour == 1 ? 'hour' : 'hours';

  @override
  List<String> get timerPickerHourLabels => const <String>['hour', 'hours'];

  @override
  String timerPickerMinuteLabel(int minute) => 'min.';

  @override
  List<String> get timerPickerMinuteLabels => const <String>['min.'];

  @override
  String timerPickerSecondLabel(int second) => 'sec.';

  @override
  List<String> get timerPickerSecondLabels => const <String>['sec.'];

  @override
  String get cutButtonLabel => 'Cut';

  @override
  String get copyButtonLabel => 'Copy';

  @override
  String get pasteButtonLabel => 'Paste';

  @override
  String get clearButtonLabel => 'Clear';

  @override
  String get noSpellCheckReplacementsLabel => 'No Replacements Found';

  @override
  String get selectAllButtonLabel => 'Select All';

  @override
  String get lookUpButtonLabel => 'Look Up';

  @override
  String get searchWebButtonLabel => 'Search Web';

  @override
  String get shareButtonLabel => 'Share...';

  @override
  String get searchTextFieldPlaceholderLabel => 'Search';

  @override
  String get modalBarrierDismissLabel => 'Dismiss';

  @override
  String get menuDismissLabel => 'Dismiss menu';

  @override
  String get cancelButtonLabel => 'Cancel';

  @override
  String get backButtonLabel => 'Back';

  /// Creates an object that provides US English resource values for the
  /// cupertino library widgets.
  ///
  /// The [locale] parameter is ignored.
  ///
  /// This method is typically used to create a [LocalizationsDelegate].
  static Future<CupertinoLocalizations> load(Locale locale) {
    return SynchronousFuture<CupertinoLocalizations>(const DefaultCupertinoLocalizations());
  }

  /// A [LocalizationsDelegate] that uses [DefaultCupertinoLocalizations.load]
  /// to create an instance of this class.
  static const LocalizationsDelegate<CupertinoLocalizations> delegate =
      _CupertinoLocalizationsDelegate();
}
