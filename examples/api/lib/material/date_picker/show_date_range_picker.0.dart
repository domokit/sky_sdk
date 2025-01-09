// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [showDateRangePicker].

void main() => runApp(const DatePickerApp());

class DatePickerApp extends StatelessWidget {
  const DatePickerApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(
      restorationScopeId: 'app',
      home: DatePickerExample(restorationId: 'main'),
    );
  }
}

class DatePickerExample extends StatefulWidget {
  const DatePickerExample({super.key, this.restorationId});

  final String? restorationId;

  @override
  State<DatePickerExample> createState() => _DatePickerExampleState();
}

/// RestorationProperty objects can be used because of RestorationMixin.
class _DatePickerExampleState extends State<DatePickerExample> with RestorationMixin {
  // In this example, the restoration ID for the mixin is passed in through
  // the [StatefulWidget]'s constructor.
  @override
  String? get restorationId => widget.restorationId;

  final RestorableDateTimeN _startDate = RestorableDateTimeN(DateTime(2021));
  final RestorableDateTimeN _endDate = RestorableDateTimeN(DateTime(2021, 1, 5));
  late final RestorableRouteFuture<DateTimeRange?> _restorableDateRangePickerRouteFuture =
      RestorableRouteFuture<DateTimeRange?>(
        onComplete: _selectDateRange,
        onPresent: (NavigatorState navigator, Object? arguments) {
          return navigator.restorablePush(
            _dateRangePickerRoute,
            arguments: <String, dynamic>{
              'initialStartDate': _startDate.value?.millisecondsSinceEpoch,
              'initialEndDate': _endDate.value?.millisecondsSinceEpoch,
            },
          );
        },
      );

  void _selectDateRange(DateTimeRange? newSelectedDate) {
    if (newSelectedDate != null) {
      setState(() {
        _startDate.value = newSelectedDate.start;
        _endDate.value = newSelectedDate.end;
      });
    }
  }

  @override
  void restoreState(RestorationBucket? oldBucket, bool initialRestore) {
    registerForRestoration(_startDate, 'start_date');
    registerForRestoration(_endDate, 'end_date');
    registerForRestoration(_restorableDateRangePickerRouteFuture, 'date_picker_route_future');
  }

  @pragma('vm:entry-point')
  static Route<DateTimeRange?> _dateRangePickerRoute(BuildContext context, Object? arguments) {
    return DialogRoute<DateTimeRange?>(
      context: context,
      builder: (BuildContext context) {
        return DateRangePickerDialog(
          restorationId: 'date_picker_dialog',
          initialDateRange: _initialDateTimeRange(arguments! as Map<dynamic, dynamic>),
          firstDate: DateTime(2021),
          currentDate: DateTime(2021, 1, 25),
          lastDate: DateTime(2022),
        );
      },
    );
  }

  static DateTimeRange? _initialDateTimeRange(Map<dynamic, dynamic> arguments) {
    if (arguments['initialStartDate'] != null && arguments['initialEndDate'] != null) {
      return DateTimeRange(
        start: DateTime.fromMillisecondsSinceEpoch(arguments['initialStartDate'] as int),
        end: DateTime.fromMillisecondsSinceEpoch(arguments['initialEndDate'] as int),
      );
    }

    return null;
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Center(
        child: OutlinedButton(
          onPressed: () {
            _restorableDateRangePickerRouteFuture.present();
          },
          child: const Text('Open Date Range Picker'),
        ),
      ),
    );
  }
}
