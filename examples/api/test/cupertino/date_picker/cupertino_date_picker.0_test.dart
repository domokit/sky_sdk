// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_api_samples/cupertino/date_picker/cupertino_date_picker.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

const Offset _kRowOffset = Offset(0.0, -50.0);

void main() {
  testWidgets('Can change date, time and dateTime using CupertinoDatePicker', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.DatePickerApp());
    // Open the date picker.
    await tester.tap(find.text('10-26-2016'));
    await tester.pumpAndSettle();

    // Drag month, day and year wheels to change the picked date.
    await tester.drag(
      find.text('October'),
      _kRowOffset,
      touchSlopY: 0,
      warnIfMissed: false,
    ); // see top of file
    await tester.drag(
      find.textContaining('26').last,
      _kRowOffset,
      touchSlopY: 0,
      warnIfMissed: false,
    ); // see top of file
    await tester.drag(
      find.text('2016'),
      _kRowOffset,
      touchSlopY: 0,
      warnIfMissed: false,
    ); // see top of file

    await tester.pump();
    await tester.pump(const Duration(milliseconds: 500));

    // Close the date picker.
    await tester.tapAt(const Offset(1.0, 1.0));
    await tester.pumpAndSettle();

    expect(find.text('12-28-2018'), findsOneWidget);

    // Open the time picker.
    await tester.tap(find.text('22:35'));
    await tester.pumpAndSettle();

    // Drag hour and minute wheels to change the picked time.
    await tester.drag(
      find.text('22'),
      const Offset(0.0, 50.0),
      touchSlopY: 0,
      warnIfMissed: false,
    ); // see top of file
    await tester.drag(
      find.text('35'),
      const Offset(0.0, 50.0),
      touchSlopY: 0,
      warnIfMissed: false,
    ); // see top of file

    await tester.pump();
    await tester.pump(const Duration(milliseconds: 500));

    // Close the time picker.
    await tester.tapAt(const Offset(1.0, 1.0));
    await tester.pumpAndSettle();

    expect(find.text('20:33'), findsOneWidget);

    // Open the dateTime picker.
    await tester.tap(find.text('8-3-2016 17:45'));
    await tester.pumpAndSettle();

    // Drag hour and minute wheels to change the picked time.
    await tester.drag(
      find.text('17'),
      const Offset(0.0, 50.0),
      touchSlopY: 0,
      warnIfMissed: false,
    ); // see top of file
    await tester.drag(
      find.text('45'),
      const Offset(0.0, 50.0),
      touchSlopY: 0,
      warnIfMissed: false,
    ); // see top of file

    await tester.pump();
    await tester.pump(const Duration(milliseconds: 500));

    // Close the dateTime picker.
    await tester.tapAt(const Offset(1.0, 1.0));
    await tester.pumpAndSettle();

    expect(find.text('8-3-2016 15:43'), findsOneWidget);
  });
}
