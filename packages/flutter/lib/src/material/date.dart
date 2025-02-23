// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'calendar_date_picker.dart';
/// @docImport 'date_picker.dart';
/// @docImport 'text_field.dart';
library;

import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';

import 'material_localizations.dart';

/// Controls the calendar system used in the date picker.
///
/// A [CalendarDelegate] defines how dates are interpreted, formatted, and
/// navigated within the picker. Different calendar systems (e.g., Gregorian,
/// Nepali, Hijri, Buddhist) can be supported by providing custom implementations.
///
/// {@tool dartpad}
/// This example demonstrates how a [CalendarDelegate] is used to implement a
/// custom calendar system in the date picker.
///
/// ** See code in examples/api/lib/material/date_picker/custom_calendar_date_picker.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [GregorianCalendarDelegate], the default implementation for the Gregorian calendar.
///  * [CalendarDatePicker], which uses this delegate to manage calendar-specific behavior.
abstract class CalendarDelegate {
  /// Creates a calendar delegate.
  const CalendarDelegate();

  /// Returns a [DateTime] representing the current date and time.
  DateTime now();

  /// {@macro flutter.material.date.dateOnly}
  DateTime dateOnly(covariant DateTime date);

  /// {@macro flutter.material.date.datesOnly}
  DateTimeRange datesOnly(covariant DateTimeRange range);

  /// {@macro flutter.material.date.isSameDay}
  bool isSameDay(covariant DateTime? dateA, covariant DateTime? dateB);

  /// {@macro flutter.material.date.isSameMonth}
  bool isSameMonth(covariant DateTime? dateA, covariant DateTime? dateB);

  /// {@macro flutter.material.date.monthDelta}
  int monthDelta(covariant DateTime startDate, covariant DateTime endDate);

  /// {@macro flutter.material.date.addMonthsToMonthDate}
  DateTime addMonthsToMonthDate(covariant DateTime monthDate, int monthsToAdd);

  /// {@macro flutter.material.date.addDaysToDate}
  DateTime addDaysToDate(covariant DateTime date, int days);

  /// {@macro flutter.material.date.firstDayOffset}
  int firstDayOffset(int year, int month, MaterialLocalizations localizations);

  /// Returns the number of days in a month, according to the calendar system.
  int getDaysInMonth(int year, int month);

  /// Returns a [DateTime] with the given [year] and [month].
  DateTime getMonth(int year, int month);

  /// Returns a [DateTime] with the given [year], [month], and [day].
  DateTime getDay(int year, int month, int day);

  /// Formats the month and the year of the given [date].
  ///
  /// The returned string does not contain the day of the month. This appears
  /// in the date picker invoked using [showDatePicker].
  String formatMonthYear(covariant DateTime date, MaterialLocalizations localizations);

  /// Full unabbreviated year format, e.g. 2017 rather than 17.
  String formatYear(int year, MaterialLocalizations localizations);

  /// Formats the date using a medium-width format.
  ///
  /// Abbreviates month and days of week. This appears in the header of the date
  /// picker invoked using [showDatePicker].
  ///
  /// Examples:
  ///
  /// - US English: Wed, Sep 27
  /// - Russian: ср, сент. 27
  String formatMediumDate(covariant DateTime date, MaterialLocalizations localizations);

  /// Formats the month and day of the given [date].
  ///
  /// Examples:
  ///
  /// - US English: Feb 21
  /// - Russian: 21 февр.
  String formatShortMonthDay(covariant DateTime date, MaterialLocalizations localizations);

  /// Formats the date using a short-width format.
  ///
  /// Includes the abbreviation of the month, the day and year.
  ///
  /// Examples:
  ///
  /// - US English: Feb 21, 2019
  /// - Russian: 21 февр. 2019 г.
  String formatShortDate(covariant DateTime date, MaterialLocalizations localizations);

  /// Formats day of week, month, day of month and year in a long-width format.
  ///
  /// Does not abbreviate names. Appears in spoken announcements of the date
  /// picker invoked using [showDatePicker], when accessibility mode is on.
  ///
  /// Examples:
  ///
  /// - US English: Wednesday, September 27, 2017
  /// - Russian: Среда, Сентябрь 27, 2017
  String formatFullDate(covariant DateTime date, MaterialLocalizations localizations);

  /// Formats the date in a compact format.
  ///
  /// Usually just the numeric values for the for day, month and year are used.
  ///
  /// Examples:
  ///
  /// - US English: 02/21/2019
  /// - Russian: 21.02.2019
  ///
  /// See also:
  ///   * [parseCompactDate], which will convert a compact date string to a [DateTime].
  String formatCompactDate(covariant DateTime date, MaterialLocalizations localizations);

  /// Converts the given compact date formatted string into a [DateTime].
  ///
  /// The format of the string must be a valid compact date format for the
  /// given locale. If the text doesn't represent a valid date, `null` will be
  /// returned.
  ///
  /// See also:
  ///   * [formatCompactDate], which will convert a [DateTime] into a string in the compact format.
  DateTime? parseCompactDate(String? inputString, MaterialLocalizations localizations);

  /// The help text used on an empty [InputDatePickerFormField] to indicate
  /// to the user the date format being asked for.
  String dateHelpText(MaterialLocalizations localizations);
}

/// A [CalendarDelegate] implementation for the Gregorian calendar system.
///
/// The Gregorian calendar is the most widely used civil calendar worldwide.
/// This delegate provides standard date interpretation, formatting, and
/// navigation based on the Gregorian system.
///
/// This delegate is the default calendar system for [CalendarDatePicker].
///
/// See also:
/// * [CalendarDelegate], the base class for defining custom calendars.
/// * [CalendarDatePicker], which uses this delegate for date selection.
class GregorianCalendarDelegate extends CalendarDelegate {
  /// Creates a calendar delegate that uses the Gregorian calendar and the
  /// conventions of the current [MaterialLocalizations].
  const GregorianCalendarDelegate();

  @override
  DateTime now() => DateTime.now();

  @override
  DateTime dateOnly(DateTime date) => DateUtils.dateOnly(date);

  @override
  DateTimeRange datesOnly(DateTimeRange range) => DateUtils.datesOnly(range);

  @override
  bool isSameDay(DateTime? dateA, DateTime? dateB) => DateUtils.isSameDay(dateA, dateB);

  @override
  bool isSameMonth(DateTime? dateA, DateTime? dateB) => DateUtils.isSameMonth(dateA, dateB);

  @override
  int monthDelta(DateTime startDate, DateTime endDate) => DateUtils.monthDelta(startDate, endDate);

  @override
  DateTime addMonthsToMonthDate(DateTime monthDate, int monthsToAdd) {
    return DateUtils.addMonthsToMonthDate(monthDate, monthsToAdd);
  }

  @override
  DateTime addDaysToDate(DateTime date, int days) => DateUtils.addDaysToDate(date, days);

  @override
  int firstDayOffset(int year, int month, MaterialLocalizations localizations) {
    return DateUtils.firstDayOffset(year, month, localizations);
  }

  /// {@macro flutter.material.date.getDaysInMonth}
  @override
  int getDaysInMonth(int year, int month) => DateUtils.getDaysInMonth(year, month);

  @override
  DateTime getMonth(int year, int month) => DateTime(year, month);

  @override
  DateTime getDay(int year, int month, int day) => DateTime(year, month, day);

  @override
  String formatMonthYear(DateTime date, MaterialLocalizations localizations) {
    return localizations.formatMonthYear(date);
  }

  @override
  String formatYear(int year, MaterialLocalizations localizations) {
    return localizations.formatYear(DateTime(year));
  }

  @override
  String formatMediumDate(DateTime date, MaterialLocalizations localizations) {
    return localizations.formatMediumDate(date);
  }

  @override
  String formatShortMonthDay(DateTime date, MaterialLocalizations localizations) {
    return localizations.formatShortMonthDay(date);
  }

  @override
  String formatShortDate(DateTime date, MaterialLocalizations localizations) {
    return localizations.formatShortDate(date);
  }

  @override
  String formatFullDate(DateTime date, MaterialLocalizations localizations) {
    return localizations.formatFullDate(date);
  }

  @override
  String formatCompactDate(DateTime date, MaterialLocalizations localizations) {
    return localizations.formatCompactDate(date);
  }

  @override
  DateTime? parseCompactDate(String? inputString, MaterialLocalizations localizations) {
    return localizations.parseCompactDate(inputString);
  }

  @override
  String dateHelpText(MaterialLocalizations localizations) {
    return localizations.dateHelpText;
  }
}

/// Utility functions for working with dates.
abstract final class DateUtils {
  /// {@template flutter.material.date.dateOnly}
  /// Returns a [DateTime] with the date of the original, but time set to
  /// midnight.
  /// {@endtemplate}
  static DateTime dateOnly(DateTime date) {
    return DateTime(date.year, date.month, date.day);
  }

  /// {@template flutter.material.date.datesOnly}
  /// Returns a [DateTimeRange] with the dates of the original, but with times
  /// set to midnight.
  ///
  /// See also:
  ///  * [dateOnly], which does the same thing for a single date.
  /// {@endtemplate}
  static DateTimeRange datesOnly(DateTimeRange range) {
    return DateTimeRange(start: dateOnly(range.start), end: dateOnly(range.end));
  }

  /// {@template flutter.material.date.isSameDay}
  /// Returns true if the two [DateTime] objects have the same day, month, and
  /// year, or are both null.
  /// {@endtemplate}
  static bool isSameDay(DateTime? dateA, DateTime? dateB) {
    return dateA?.year == dateB?.year && dateA?.month == dateB?.month && dateA?.day == dateB?.day;
  }

  /// {@template flutter.material.date.isSameMonth}
  /// Returns true if the two [DateTime] objects have the same month and
  /// year, or are both null.
  /// {@endtemplate}
  static bool isSameMonth(DateTime? dateA, DateTime? dateB) {
    return dateA?.year == dateB?.year && dateA?.month == dateB?.month;
  }

  /// {@template flutter.material.date.monthDelta}
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
  /// {@endtemplate}
  static int monthDelta(DateTime startDate, DateTime endDate) {
    return (endDate.year - startDate.year) * 12 + endDate.month - startDate.month;
  }

  /// {@template flutter.material.date.addMonthsToMonthDate}
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
  /// {@endtemplate}
  static DateTime addMonthsToMonthDate(DateTime monthDate, int monthsToAdd) {
    return DateTime(monthDate.year, monthDate.month + monthsToAdd);
  }

  /// {@template flutter.material.date.addDaysToDate}
  /// Returns a [DateTime] with the added number of days and time set to
  /// midnight.
  /// {@endtemplate}
  static DateTime addDaysToDate(DateTime date, int days) {
    return DateTime(date.year, date.month, date.day + days);
  }

  /// {@template flutter.material.date.firstDayOffset}
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
  /// {@endtemplate}
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

  /// {@template flutter.material.date.getDaysInMonth}
  /// Returns the number of days in a month, according to the proleptic
  /// Gregorian calendar.
  ///
  /// This applies the leap year logic introduced by the Gregorian reforms of
  /// 1582. It will not give valid results for dates prior to that time.
  /// {@endtemplate}
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
@optionalTypeArgs
class DateTimeRange<T extends DateTime> {
  /// Creates a date range for the given start and end [T].
  DateTimeRange({required this.start, required this.end}) : assert(!start.isAfter(end));

  /// The start of the range of dates.
  final T start;

  /// The end of the range of dates.
  final T end;

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
