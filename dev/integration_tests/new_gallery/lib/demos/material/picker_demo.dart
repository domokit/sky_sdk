// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:intl/intl.dart';

import '../../gallery_localizations.dart';
import 'material_demo_types.dart';

// BEGIN pickerDemo

class PickerDemo extends StatefulWidget {
  const PickerDemo({super.key, required this.type});

  final PickerDemoType type;

  @override
  State<PickerDemo> createState() => _PickerDemoState();
}

class _PickerDemoState extends State<PickerDemo> with RestorationMixin {
  final RestorableDateTime _fromDate = RestorableDateTime(DateTime.now());
  final RestorableTimeOfDay _fromTime = RestorableTimeOfDay(TimeOfDay.fromDateTime(DateTime.now()));
  final RestorableDateTime _startDate = RestorableDateTime(DateTime.now());
  final RestorableDateTime _endDate = RestorableDateTime(DateTime.now());

  late RestorableRouteFuture<DateTime?> _restorableDatePickerRouteFuture;
  late RestorableRouteFuture<DateTimeRange?> _restorableDateRangePickerRouteFuture;
  late RestorableRouteFuture<TimeOfDay?> _restorableTimePickerRouteFuture;

  void _selectDate(DateTime? selectedDate) {
    if (selectedDate != null && selectedDate != _fromDate.value) {
      setState(() {
        _fromDate.value = selectedDate;
      });
    }
  }

  void _selectDateRange(DateTimeRange? newSelectedDate) {
    if (newSelectedDate != null) {
      setState(() {
        _startDate.value = newSelectedDate.start;
        _endDate.value = newSelectedDate.end;
      });
    }
  }

  void _selectTime(TimeOfDay? selectedTime) {
    if (selectedTime != null && selectedTime != _fromTime.value) {
      setState(() {
        _fromTime.value = selectedTime;
      });
    }
  }

  static Route<DateTime> _datePickerRoute(BuildContext context, Object? arguments) {
    return DialogRoute<DateTime>(
      context: context,
      builder: (BuildContext context) {
        return DatePickerDialog(
          restorationId: 'date_picker_dialog',
          initialDate: DateTime.fromMillisecondsSinceEpoch(arguments! as int),
          firstDate: DateTime(2015),
          lastDate: DateTime(2100),
        );
      },
    );
  }

  static Route<TimeOfDay> _timePickerRoute(BuildContext context, Object? arguments) {
    final List<Object> args = arguments! as List<Object>;
    final TimeOfDay initialTime = TimeOfDay(hour: args[0] as int, minute: args[1] as int);

    return DialogRoute<TimeOfDay>(
      context: context,
      builder: (BuildContext context) {
        return TimePickerDialog(restorationId: 'time_picker_dialog', initialTime: initialTime);
      },
    );
  }

  static Route<DateTimeRange> _dateRangePickerRoute(BuildContext context, Object? arguments) {
    return DialogRoute<DateTimeRange>(
      context: context,
      builder: (BuildContext context) {
        return DateRangePickerDialog(
          restorationId: 'date_rage_picker_dialog',
          firstDate: DateTime(DateTime.now().year - 5),
          lastDate: DateTime(DateTime.now().year + 5),
        );
      },
    );
  }

  @override
  void initState() {
    super.initState();
    _restorableDatePickerRouteFuture = RestorableRouteFuture<DateTime?>(
      onComplete: _selectDate,
      onPresent: (NavigatorState navigator, Object? arguments) {
        return navigator.restorablePush(
          _datePickerRoute,
          arguments: _fromDate.value.millisecondsSinceEpoch,
        );
      },
    );
    _restorableDateRangePickerRouteFuture = RestorableRouteFuture<DateTimeRange?>(
      onComplete: _selectDateRange,
      onPresent:
          (NavigatorState navigator, Object? arguments) =>
              navigator.restorablePush(_dateRangePickerRoute),
    );

    _restorableTimePickerRouteFuture = RestorableRouteFuture<TimeOfDay?>(
      onComplete: _selectTime,
      onPresent:
          (NavigatorState navigator, Object? arguments) => navigator.restorablePush(
            _timePickerRoute,
            arguments: <int>[_fromTime.value.hour, _fromTime.value.minute],
          ),
    );
  }

  @override
  String get restorationId => 'picker_demo';

  @override
  void restoreState(RestorationBucket? oldBucket, bool initialRestore) {
    registerForRestoration(_fromDate, 'from_date');
    registerForRestoration(_fromTime, 'from_time');
    registerForRestoration(_startDate, 'start_date');
    registerForRestoration(_endDate, 'end_date');
    registerForRestoration(_restorableDatePickerRouteFuture, 'date_picker_route');
    registerForRestoration(_restorableDateRangePickerRouteFuture, 'date_range_picker_route');
    registerForRestoration(_restorableTimePickerRouteFuture, 'time_picker_route');
  }

  String get _title {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    return switch (widget.type) {
      PickerDemoType.date => localizations.demoDatePickerTitle,
      PickerDemoType.time => localizations.demoTimePickerTitle,
      PickerDemoType.range => localizations.demoDateRangePickerTitle,
    };
  }

  String get _labelText {
    final DateFormat yMMMd = DateFormat.yMMMd();
    return switch (widget.type) {
      PickerDemoType.date => yMMMd.format(_fromDate.value),
      PickerDemoType.time => _fromTime.value.format(context),
      PickerDemoType.range => '${yMMMd.format(_startDate.value)} - ${yMMMd.format(_endDate.value)}',
    };
  }

  @override
  Widget build(BuildContext context) {
    return Navigator(
      onGenerateRoute: (RouteSettings settings) {
        return MaterialPageRoute<void>(
          builder:
              (BuildContext context) => Scaffold(
                appBar: AppBar(automaticallyImplyLeading: false, title: Text(_title)),
                body: Center(
                  child: Column(
                    mainAxisSize: MainAxisSize.min,
                    children: <Widget>[
                      Text(_labelText),
                      const SizedBox(height: 16),
                      ElevatedButton(
                        onPressed:
                            () =>
                                switch (widget.type) {
                                  PickerDemoType.date => _restorableDatePickerRouteFuture,
                                  PickerDemoType.time => _restorableTimePickerRouteFuture,
                                  PickerDemoType.range => _restorableDateRangePickerRouteFuture,
                                }.present(),
                        child: Text(GalleryLocalizations.of(context)!.demoPickersShowPicker),
                      ),
                    ],
                  ),
                ),
              ),
        );
      },
    );
  }
}

// END
