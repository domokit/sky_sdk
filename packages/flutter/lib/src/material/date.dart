// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'calendar_date_picker.dart';
/// @docImport 'date_picker.dart';
/// @docImport 'text_field.dart';
library;

import 'package:flutter/foundation.dart';

import 'material_localizations.dart';

/// Utility functions for working with dates.
abstract final class DateUtils {
  /// Returns a [DateTime] with the date of the original, but time set to
  /// midnight.
  static DateTime dateOnly(DateTime date) {
    return DateTime(date.year, date.month, date.day);
  }

  /// Returns a [DateTimeRange] with the dates of the original, but with times
  /// set to midnight.
  ///
  /// See also:
  ///  * [dateOnly], which does the same thing for a single date.
  static DateTimeRange datesOnly(DateTimeRange range) {
    return DateTimeRange(start: dateOnly(range.start), end: dateOnly(range.end));
  }

  /// Returns true if the two [DateTime] objects have the same day, month, and
  /// year, or are both null.
  static bool isSameDay(DateTime? dateA, DateTime? dateB) {
    return dateA?.year == dateB?.year && dateA?.month == dateB?.month && dateA?.day == dateB?.day;
  }

  /// Returns true if the two [DateTime] objects have the same month and
  /// year, or are both null.
  static bool isSameMonth(DateTime? dateA, DateTime? dateB) {
    return dateA?.year == dateB?.year && dateA?.month == dateB?.month;
  }

  /// Determines the number of months between two [DateTime] objects.
  ///
  /// For example:
  ///
  /// ```dart
  /// DateTime date1 = DateTime(2019, 6, 15);
  /// DateTime date2 = DateTime(2020, 1, 15);
  /// int delta = DateUtils.monthDelta(date1, date2);
  /// ```
  ///
  /// The value for `delta` would be `7`.
  static int monthDelta(DateTime startDate, DateTime endDate) {
    return (endDate.year - startDate.year) * 12 + endDate.month - startDate.month;
  }

  /// Returns a [DateTime] that is [monthDate] with the added number
  /// of months and the day set to 1 and time set to midnight.
  ///
  /// For example:
  ///
  /// ```dart
  /// DateTime date = DateTime(2019, 1, 15);
  /// DateTime futureDate = DateUtils.addMonthsToMonthDate(date, 3);
  /// ```
  ///
  /// `date` would be January 15, 2019.
  /// `futureDate` would be April 1, 2019 since it adds 3 months.
  static DateTime addMonthsToMonthDate(DateTime monthDate, int monthsToAdd) {
    return DateTime(monthDate.year, monthDate.month + monthsToAdd);
  }

  /// Returns a [DateTime] with the added number of days and time set to
  /// midnight.
  static DateTime addDaysToDate(DateTime date, int days) {
    return DateTime(date.year, date.month, date.day + days);
  }

  /// Computes the offset from the first day of the week that the first day of
  /// the [month] falls on.
  ///
  /// For example, September 1, 2017 falls on a Friday, which in the calendar
  /// localized for United States English appears as:
  ///
  ///     S M T W T F S
  ///     _ _ _ _ _ 1 2
  ///
  /// The offset for the first day of the months is the number of leading blanks
  /// in the calendar, i.e. 5.
  ///
  /// The same date localized for the Russian calendar has a different offset,
  /// because the first day of week is Monday rather than Sunday:
  ///
  ///     M T W T F S S
  ///     _ _ _ _ 1 2 3
  ///
  /// So the offset is 4, rather than 5.
  ///
  /// This code consolidates the following:
  ///
  /// - [DateTime.weekday] provides a 1-based index into days of week, with 1
  ///   falling on Monday.
  /// - [MaterialLocalizations.firstDayOfWeekIndex] provides a 0-based index
  ///   into the [MaterialLocalizations.narrowWeekdays] list.
  /// - [MaterialLocalizations.narrowWeekdays] list provides localized names of
  ///   days of week, always starting with Sunday and ending with Saturday.
  static int firstDayOffset(int year, int month, MaterialLocalizations localizations) {
    // 0-based day of week for the month and year, with 0 representing Monday.
    final int weekdayFromMonday = DateTime(year, month).weekday - 1;

    // 0-based start of week depending on the locale, with 0 representing Sunday.
    int firstDayOfWeekIndex = localizations.firstDayOfWeekIndex;

    // firstDayOfWeekIndex recomputed to be Monday-based, in order to compare with
    // weekdayFromMonday.
    firstDayOfWeekIndex = (firstDayOfWeekIndex - 1) % 7;

    // Number of days between the first day of week appearing on the calendar,
    // and the day corresponding to the first of the month.
    return (weekdayFromMonday - firstDayOfWeekIndex) % 7;
  }

  /// Returns the number of days in a month, according to the proleptic
  /// Gregorian calendar.
  ///
  /// This applies the leap year logic introduced by the Gregorian reforms of
  /// 1582. It will not give valid results for dates prior to that time.
  static int getDaysInMonth(int year, int month) {
    if (month == DateTime.february) {
      final bool isLeapYear = (year % 4 == 0) && (year % 100 != 0) || (year % 400 == 0);
      return isLeapYear ? 29 : 28;
    }
    const List<int> daysInMonth = <int>[31, -1, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31];
    return daysInMonth[month - 1];
  }
}

/// Mode of date entry method for the date picker dialog.
///
/// In [calendar] mode, a calendar grid is displayed and the user taps the
/// day they wish to select. In [input] mode, a [TextField] is displayed and
/// the user types in the date they wish to select.
///
/// [calendarOnly] and [inputOnly] are variants of the above that don't
/// allow the user to change to the mode.
///
/// See also:
///
///  * [showDatePicker] and [showDateRangePicker], which use this to control
///    the initial entry mode of their dialogs.
enum DatePickerEntryMode {
  /// User picks a date from calendar grid. Can switch to [input] by activating
  /// a mode button in the dialog.
  calendar,

  /// User can input the date by typing it into a text field.
  ///
  /// Can switch to [calendar] by activating a mode button in the dialog.
  input,

  /// User can only pick a date from calendar grid.
  ///
  /// There is no user interface to switch to another mode.
  calendarOnly,

  /// User can only input the date by typing it into a text field.
  ///
  /// There is no user interface to switch to another mode.
  inputOnly,
}

/// Initial display of a calendar date picker.
///
/// Either a grid of available years or a monthly calendar.
///
/// See also:
///
///  * [showDatePicker], which shows a dialog that contains a Material Design
///    date picker.
///  * [CalendarDatePicker], widget which implements the Material Design date picker.
enum DatePickerMode {
  /// Choosing a month and day.
  day,

  /// Choosing a year.
  year,
}

/// Signature for predicating dates for enabled date selections.
///
/// See [showDatePicker], which has a [SelectableDayPredicate] parameter used
/// to specify allowable days in the date picker.
typedef SelectableDayPredicate = bool Function(DateTime day);

/// Encapsulates a start and end [DateTime] that represent the range of dates.
///
/// The range includes the [start] and [end] dates. The [start] and [end] dates
/// may be equal to indicate a date range of a single day. The [start] date must
/// not be after the [end] date.
///
/// See also:
///  * [showDateRangePicker], which displays a dialog that allows the user to
///    select a date range.
@immutable
class DateTimeRange {
  /// Creates a date range for the given start and end [DateTime].
  DateTimeRange({required this.start, required this.end}) : assert(!start.isAfter(end));

  /// The start of the range of dates.
  final DateTime start;

  /// The end of the range of dates.
  final DateTime end;

  /// Returns a [Duration] of the time between [start] and [end].
  ///
  /// See [DateTime.difference] for more details.
  Duration get duration => end.difference(start);

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is DateTimeRange && other.start == start && other.end == end;
  }

  @override
  int get hashCode => Object.hash(start, end);

  @override
  String toString() => '$start - $end';
}
