// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import '../utils.dart';
import 'use_cases.dart';

class DatePickerUseCase extends UseCase {
  @override
  String get name => 'DatePicker';

  @override
  String get route => '/date-picker';

  @override
  Widget build(BuildContext context) => const _MainWidget();
}

class _MainWidget extends StatefulWidget {
  const _MainWidget();

  @override
  State<_MainWidget> createState() => _MainWidgetState();
}

class _MainWidgetState extends State<_MainWidget> {
  String pageTitle = getUseCaseName(DatePickerUseCase());

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        backgroundColor: Theme.of(context).colorScheme.inversePrimary,
        title: Semantics(headingLevel: 1, child: Text('$pageTitle Demo')),
      ),
      body: Center(
        child: TextButton(
          onPressed:
              () => showDatePicker(
                context: context,
                initialEntryMode: DatePickerEntryMode.calendarOnly,
                initialDate: DateTime.now(),
                firstDate: DateTime.now().subtract(const Duration(days: 365)),
                lastDate: DateTime.now().add(const Duration(days: 365)),
              ),
          child: const Text('Show Date Picker'),
        ),
      ),
    );
  }
}
