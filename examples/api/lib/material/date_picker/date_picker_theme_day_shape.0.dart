// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [DatePickerThemeData].

void main() => runApp(const DatePickerApp());

class DatePickerApp extends StatelessWidget {
  const DatePickerApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      theme: ThemeData(
        datePickerTheme: DatePickerThemeData(
          todayBackgroundColor: const WidgetStatePropertyAll<Color>(Colors.amber),
          todayForegroundColor: const WidgetStatePropertyAll<Color>(Colors.black),
          todayBorder: const BorderSide(width: 2),
          dayShape: WidgetStatePropertyAll<OutlinedBorder>(
            RoundedRectangleBorder(borderRadius: BorderRadius.circular(8.0)),
          ),
          shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16.0)),
        ),
      ),
      home: const DatePickerExample(),
    );
  }
}

class DatePickerExample extends StatefulWidget {
  const DatePickerExample({super.key});

  @override
  State<DatePickerExample> createState() => _DatePickerExampleState();
}

class _DatePickerExampleState extends State<DatePickerExample> {
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Center(
        child: OutlinedButton(
          onPressed: () {
            showDatePicker(
              context: context,
              initialDate: DateTime(2021, 1, 20),
              currentDate: DateTime(2021, 1, 15),
              firstDate: DateTime(2021),
              lastDate: DateTime(2022),
            );
          },
          child: const Text('Open Date Picker'),
        ),
      ),
    );
  }
}
