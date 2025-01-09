// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter_localizations/flutter_localizations.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Test correct month form for CupertinoDatePicker in monthYear mode', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoPageScaffold(
          child: Center(
            child: CupertinoDatePicker(
              initialDateTime: DateTime(2023, 5),
              onDateTimeChanged: (_) {},
              mode: CupertinoDatePickerMode.monthYear,
            ),
          ),
        ),
        supportedLocales: const <Locale>[Locale('ru', 'RU')],
        localizationsDelegates: GlobalCupertinoLocalizations.delegates,
      ),
    );

    expect(find.text('Май'), findsWidgets);
  });

  testWidgets('Test correct month form for CupertinoDatePicker in date mode', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoPageScaffold(
          child: Center(
            child: CupertinoDatePicker(
              initialDateTime: DateTime(2023, 5),
              onDateTimeChanged: (_) {},
              mode: CupertinoDatePickerMode.date,
            ),
          ),
        ),
        supportedLocales: const <Locale>[Locale('ru', 'RU')],
        localizationsDelegates: GlobalCupertinoLocalizations.delegates,
      ),
    );

    expect(find.text('мая'), findsWidgets);
  });
}
