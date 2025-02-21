// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample demonstrating how to use a custom [CalendarDelegate]
/// with [CalendarDatePicker] to implement a hypothetical calendar system
/// where even-numbered months have 21 days, odd-numbered months have 28 days,
/// and every month starts on a Monday.

void main() => runApp(const CalendarDatePickerApp());

class CalendarDatePickerApp extends StatelessWidget {
  const CalendarDatePickerApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: CalendarDatePickerExample());
  }
}

class CalendarDatePickerExample extends StatefulWidget {
  const CalendarDatePickerExample({super.key});

  @override
  State<CalendarDatePickerExample> createState() => _CalendarDatePickerExampleState();
}

class _CalendarDatePickerExampleState extends State<CalendarDatePickerExample> {
  DateTime? selectedDate;

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('Custom Calendar')),
      body: Column(
        spacing: 16,
        children: <Widget>[
          CalendarDatePicker(
            initialDate: DateTime(2025, 2, 8),
            firstDate: DateTime(2025),
            lastDate: DateTime(2026),
            onDateChanged: (DateTime pickedDate) {
              setState(() {
                selectedDate = pickedDate;
              });
            },
            calendarDelegate: const CustomCalendarDelegate(),
          ),
          const Divider(height: 1),
          Text(
            selectedDate != null
                ? '${selectedDate!.day}/${selectedDate!.month}/${selectedDate!.year}'
                : 'No date selected',
          ),
        ],
      ),
    );
  }
}

/// A custom calendar system where even-numbered months have 21 days,
/// odd-numbered months have 28 days, and every month starts on a Monday.
///
/// This hypothetical calendar follows a fixed structure:
/// - **Even-numbered months (2, 4, 6, etc.)** always have **21 days**.
/// - **Odd-numbered months (1, 3, 5, etc.)** always have **28 days**.
/// - **The first day of every month is always a Monday**, ensuring a consistent weekly alignment.
class CustomCalendarDelegate extends CalendarDelegate {
  const CustomCalendarDelegate();

  @override
  int getDaysInMonth(int year, int month) {
    return month.isEven ? 21 : 28;
  }

  @override
  int firstDayOffset(int year, int month, MaterialLocalizations localizations) {
    return 1;
  }

  // ------------------------------------------------------------------------
  // All the implementations below are based on the Gregorian calendar system.

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
