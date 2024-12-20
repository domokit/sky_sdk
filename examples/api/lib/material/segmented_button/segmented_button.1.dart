// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [SegmentedButton.styleFrom].

void main() {
  runApp(const SegmentedButtonApp());
}

class SegmentedButtonApp extends StatelessWidget {
  const SegmentedButtonApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: Scaffold(body: Center(child: SegmentedButtonExample())));
  }
}

class SegmentedButtonExample extends StatefulWidget {
  const SegmentedButtonExample({super.key});

  @override
  State<SegmentedButtonExample> createState() => _SegmentedButtonExampleState();
}

enum Calendar { day, week, month, year }

class _SegmentedButtonExampleState extends State<SegmentedButtonExample> {
  Calendar calendarView = Calendar.week;

  @override
  Widget build(BuildContext context) {
    return SegmentedButton<Calendar>(
      style: SegmentedButton.styleFrom(
        backgroundColor: Colors.grey[200],
        foregroundColor: Colors.red,
        selectedForegroundColor: Colors.white,
        selectedBackgroundColor: Colors.green,
      ),
      segments: const <ButtonSegment<Calendar>>[
        ButtonSegment<Calendar>(
          value: Calendar.day,
          label: Text('Day'),
          icon: Icon(Icons.calendar_view_day),
        ),
        ButtonSegment<Calendar>(
          value: Calendar.week,
          label: Text('Week'),
          icon: Icon(Icons.calendar_view_week),
        ),
        ButtonSegment<Calendar>(
          value: Calendar.month,
          label: Text('Month'),
          icon: Icon(Icons.calendar_view_month),
        ),
        ButtonSegment<Calendar>(
          value: Calendar.year,
          label: Text('Year'),
          icon: Icon(Icons.calendar_today),
        ),
      ],
      selected: <Calendar>{calendarView},
      onSelectionChanged: (Set<Calendar> newSelection) {
        setState(() {
          // By default there is only a single segment that can be
          // selected at one time, so its value is always the first
          // item in the selected set.
          calendarView = newSelection.first;
        });
      },
    );
  }
}
