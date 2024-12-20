// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// reduced-test-set:
//   This file is run as part of a reduced test set in CI on Mac and Windows
//   machines.
@Tags(<String>['reduced-test-set'])
library;

import 'dart:math' as math;
import 'dart:ui';

import 'package:flutter/cupertino.dart';
import 'package:flutter/foundation.dart' show isCanvasKit;
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

import '../impeller_test_helpers.dart';

// TODO(yjbanov): on the web text rendered with perspective produces flaky goldens: https://github.com/flutter/flutter/issues/110785
final bool skipPerspectiveTextGoldens = isBrowser && !isCanvasKit;

// A number of the hit tests below say "warnIfMissed: false". This is because
// the way the CupertinoPicker works, the hits don't actually reach the labels,
// the scroll view intercepts them.

// scrolling by this offset will move the picker to the next item
const Offset _kRowOffset = Offset(0.0, -50.0);

void main() {
  group('Countdown timer picker', () {
    testWidgets('initialTimerDuration falls within limit', (WidgetTester tester) async {
      expect(() {
        CupertinoTimerPicker(
          onTimerDurationChanged: (_) {},
          initialTimerDuration: const Duration(days: 1),
        );
      }, throwsAssertionError);

      expect(() {
        CupertinoTimerPicker(
          onTimerDurationChanged: (_) {},
          initialTimerDuration: const Duration(seconds: -1),
        );
      }, throwsAssertionError);
    });

    testWidgets('minuteInterval is positive and is a factor of 60', (WidgetTester tester) async {
      expect(() {
        CupertinoTimerPicker(onTimerDurationChanged: (_) {}, minuteInterval: 0);
      }, throwsAssertionError);
      expect(() {
        CupertinoTimerPicker(onTimerDurationChanged: (_) {}, minuteInterval: -1);
      }, throwsAssertionError);
      expect(() {
        CupertinoTimerPicker(onTimerDurationChanged: (_) {}, minuteInterval: 7);
      }, throwsAssertionError);
    });

    testWidgets('secondInterval is positive and is a factor of 60', (WidgetTester tester) async {
      expect(() {
        CupertinoTimerPicker(onTimerDurationChanged: (_) {}, secondInterval: 0);
      }, throwsAssertionError);
      expect(() {
        CupertinoTimerPicker(onTimerDurationChanged: (_) {}, secondInterval: -1);
      }, throwsAssertionError);
      expect(() {
        CupertinoTimerPicker(onTimerDurationChanged: (_) {}, secondInterval: 7);
      }, throwsAssertionError);
    });

    testWidgets('background color default value', (WidgetTester tester) async {
      await tester.pumpWidget(
        CupertinoApp(home: CupertinoTimerPicker(onTimerDurationChanged: (_) {})),
      );

      final Iterable<CupertinoPicker> pickers = tester.allWidgets.whereType<CupertinoPicker>();
      expect(pickers.any((CupertinoPicker picker) => picker.backgroundColor != null), false);
    });

    testWidgets('background color can be null', (WidgetTester tester) async {
      await tester.pumpWidget(
        CupertinoApp(home: CupertinoTimerPicker(onTimerDurationChanged: (_) {})),
      );

      expect(tester.takeException(), isNull);
    });

    testWidgets('specified background color is applied', (WidgetTester tester) async {
      await tester.pumpWidget(
        CupertinoApp(
          home: CupertinoTimerPicker(
            onTimerDurationChanged: (_) {},
            backgroundColor: CupertinoColors.black,
          ),
        ),
      );

      final Iterable<CupertinoPicker> pickers = tester.allWidgets.whereType<CupertinoPicker>();
      expect(
        pickers.any((CupertinoPicker picker) => picker.backgroundColor != CupertinoColors.black),
        false,
      );
    });

    testWidgets('specified item extent value is applied', (WidgetTester tester) async {
      await tester.pumpWidget(
        CupertinoApp(home: CupertinoTimerPicker(itemExtent: 42, onTimerDurationChanged: (_) {})),
      );

      final Iterable<CupertinoPicker> pickers = tester.allWidgets.whereType<CupertinoPicker>();
      expect(pickers.any((CupertinoPicker picker) => picker.itemExtent != 42), false);
    });

    testWidgets('columns are ordered correctly when text direction is ltr', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        CupertinoApp(
          home: CupertinoTimerPicker(
            onTimerDurationChanged: (_) {},
            initialTimerDuration: const Duration(hours: 12, minutes: 30, seconds: 59),
          ),
        ),
      );

      Offset lastOffset = tester.getTopLeft(find.text('12'));

      expect(tester.getTopLeft(find.text('hours')).dx > lastOffset.dx, true);
      lastOffset = tester.getTopLeft(find.text('hours'));

      expect(tester.getTopLeft(find.text('30')).dx > lastOffset.dx, true);
      lastOffset = tester.getTopLeft(find.text('30'));

      expect(tester.getTopLeft(find.text('min.')).dx > lastOffset.dx, true);
      lastOffset = tester.getTopLeft(find.text('min.'));

      expect(tester.getTopLeft(find.text('59')).dx > lastOffset.dx, true);
      lastOffset = tester.getTopLeft(find.text('59'));

      expect(tester.getTopLeft(find.text('sec.')).dx > lastOffset.dx, true);
    });

    testWidgets('columns are ordered correctly when text direction is rtl', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        CupertinoApp(
          home: Directionality(
            textDirection: TextDirection.rtl,
            child: CupertinoTimerPicker(
              onTimerDurationChanged: (_) {},
              initialTimerDuration: const Duration(hours: 12, minutes: 30, seconds: 59),
            ),
          ),
        ),
      );

      Offset lastOffset = tester.getTopLeft(find.text('12'));

      expect(tester.getTopLeft(find.text('hours')).dx > lastOffset.dx, false);
      lastOffset = tester.getTopLeft(find.text('hours'));

      expect(tester.getTopLeft(find.text('30')).dx > lastOffset.dx, false);
      lastOffset = tester.getTopLeft(find.text('30'));

      expect(tester.getTopLeft(find.text('min.')).dx > lastOffset.dx, false);
      lastOffset = tester.getTopLeft(find.text('min.'));

      expect(tester.getTopLeft(find.text('59')).dx > lastOffset.dx, false);
      lastOffset = tester.getTopLeft(find.text('59'));

      expect(tester.getTopLeft(find.text('sec.')).dx > lastOffset.dx, false);
    });

    testWidgets('width of picker is consistent', (WidgetTester tester) async {
      await tester.pumpWidget(
        CupertinoApp(
          home: SizedBox(
            height: 400.0,
            width: 400.0,
            child: CupertinoTimerPicker(
              onTimerDurationChanged: (_) {},
              initialTimerDuration: const Duration(hours: 12, minutes: 30, seconds: 59),
            ),
          ),
        ),
      );

      // Distance between the first column and the last column.
      final double distance =
          tester.getCenter(find.text('sec.')).dx - tester.getCenter(find.text('12')).dx;

      await tester.pumpWidget(
        CupertinoApp(
          home: SizedBox(
            height: 400.0,
            width: 800.0,
            child: CupertinoTimerPicker(
              onTimerDurationChanged: (_) {},
              initialTimerDuration: const Duration(hours: 12, minutes: 30, seconds: 59),
            ),
          ),
        ),
      );

      // Distance between the first and the last column should be the same.
      expect(
        tester.getCenter(find.text('sec.')).dx - tester.getCenter(find.text('12')).dx,
        distance,
      );
    });
  });

  testWidgets('showDayOfWeek is only supported in date mode', (WidgetTester tester) async {
    expect(
      () => CupertinoDatePicker(
        mode: CupertinoDatePickerMode.date,
        onDateTimeChanged: (DateTime _) {},
        showDayOfWeek: true,
      ),
      returnsNormally,
    );

    expect(
      () => CupertinoDatePicker(
        mode: CupertinoDatePickerMode.time,
        onDateTimeChanged: (DateTime _) {},
        showDayOfWeek: true,
      ),
      throwsA(
        isA<AssertionError>().having(
          (AssertionError e) => e.message ?? 'Unknown error',
          'message',
          contains('showDayOfWeek is only supported in date mode'),
        ),
      ),
    );

    expect(
      () => CupertinoDatePicker(
        mode: CupertinoDatePickerMode.monthYear,
        onDateTimeChanged: (DateTime _) {},
        showDayOfWeek: true,
      ),
      throwsA(
        isA<AssertionError>().having(
          (AssertionError e) => e.message ?? 'Unknown error',
          'message',
          contains('showDayOfWeek is only supported in date mode'),
        ),
      ),
    );

    expect(
      () => CupertinoDatePicker(onDateTimeChanged: (DateTime _) {}, showDayOfWeek: true),
      throwsA(
        isA<AssertionError>().having(
          (AssertionError e) => e.message ?? 'Unknown error',
          'message',
          contains('showDayOfWeek is only supported in date mode'),
        ),
      ),
    );
  });

  testWidgets('picker honors minuteInterval and secondInterval', (WidgetTester tester) async {
    late Duration duration;
    await tester.pumpWidget(
      CupertinoApp(
        home: SizedBox(
          height: 400.0,
          width: 400.0,
          child: CupertinoTimerPicker(
            minuteInterval: 10,
            secondInterval: 12,
            initialTimerDuration: const Duration(hours: 10, minutes: 40, seconds: 48),
            onTimerDurationChanged: (Duration d) {
              duration = d;
            },
          ),
        ),
      ),
    );
    await tester.drag(find.text('40'), _kRowOffset, warnIfMissed: false); // see top of file
    await tester.pump();
    await tester.drag(find.text('48'), -_kRowOffset, warnIfMissed: false); // see top of file
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 500));

    expect(duration, const Duration(hours: 10, minutes: 50, seconds: 36));
  });

  group('Date picker', () {
    testWidgets('initial date is set to default value', (WidgetTester tester) async {
      final CupertinoDatePicker picker = CupertinoDatePicker(onDateTimeChanged: (_) {});
      expect(picker.initialDateTime, isNotNull);
    });

    testWidgets('background color default value', (WidgetTester tester) async {
      await tester.pumpWidget(CupertinoApp(home: CupertinoDatePicker(onDateTimeChanged: (_) {})));

      final Iterable<CupertinoPicker> pickers = tester.allWidgets.whereType<CupertinoPicker>();
      expect(pickers.any((CupertinoPicker picker) => picker.backgroundColor != null), false);
    });

    testWidgets('background color can be null', (WidgetTester tester) async {
      await tester.pumpWidget(CupertinoApp(home: CupertinoDatePicker(onDateTimeChanged: (_) {})));

      expect(tester.takeException(), isNull);
    });

    testWidgets('specified background color is applied', (WidgetTester tester) async {
      await tester.pumpWidget(
        CupertinoApp(
          home: CupertinoDatePicker(
            onDateTimeChanged: (_) {},
            backgroundColor: CupertinoColors.black,
          ),
        ),
      );

      final Iterable<CupertinoPicker> pickers = tester.allWidgets.whereType<CupertinoPicker>();
      expect(
        pickers.any((CupertinoPicker picker) => picker.backgroundColor != CupertinoColors.black),
        false,
      );
    });

    testWidgets('specified item extent value is applied', (WidgetTester tester) async {
      await tester.pumpWidget(
        CupertinoApp(home: CupertinoDatePicker(itemExtent: 55, onDateTimeChanged: (_) {})),
      );

      final Iterable<CupertinoPicker> pickers = tester.allWidgets.whereType<CupertinoPicker>();
      expect(pickers.any((CupertinoPicker picker) => picker.itemExtent != 55), false);
    });

    testWidgets('initial date honors minuteInterval', (WidgetTester tester) async {
      late DateTime newDateTime;
      await tester.pumpWidget(
        CupertinoApp(
          home: Center(
            child: SizedBox(
              width: 400,
              height: 400,
              child: CupertinoDatePicker(
                onDateTimeChanged: (DateTime d) => newDateTime = d,
                initialDateTime: DateTime(2018, 10, 10, 10, 3),
                minuteInterval: 3,
              ),
            ),
          ),
        ),
      );

      // Drag the minute picker to the next slot (03 -> 06).
      // The `initialDateTime` and the `minuteInterval` values are specifically chosen
      // so that `find.text` finds exactly one widget.
      await tester.drag(find.text('03'), _kRowOffset, warnIfMissed: false); // see top of file
      await tester.pump();

      expect(newDateTime.minute, 6);
    });

    test('initial date honors minimumDate & maximumDate', () {
      expect(() {
        CupertinoDatePicker(
          onDateTimeChanged: (DateTime d) {},
          initialDateTime: DateTime(2018, 10, 10),
          minimumDate: DateTime(2018, 10, 11),
        );
      }, throwsAssertionError);

      expect(() {
        CupertinoDatePicker(
          onDateTimeChanged: (DateTime d) {},
          initialDateTime: DateTime(2018, 10, 10),
          maximumDate: DateTime(2018, 10, 9),
        );
      }, throwsAssertionError);
    });

    testWidgets('changing initialDateTime after first build does not do anything', (
      WidgetTester tester,
    ) async {
      late DateTime selectedDateTime;
      await tester.pumpWidget(
        CupertinoApp(
          home: Center(
            child: SizedBox(
              height: 400.0,
              width: 400.0,
              child: CupertinoDatePicker(
                onDateTimeChanged: (DateTime dateTime) => selectedDateTime = dateTime,
                initialDateTime: DateTime(2018, 1, 1, 10, 30),
              ),
            ),
          ),
        ),
      );

      await tester.drag(
        find.text('10'),
        const Offset(0.0, 32.0),
        pointer: 1,
        touchSlopY: 0,
        warnIfMissed: false,
      ); // see top of file
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 500));

      expect(selectedDateTime, DateTime(2018, 1, 1, 9, 30));

      await tester.pumpWidget(
        CupertinoApp(
          home: Center(
            child: SizedBox(
              height: 400.0,
              width: 400.0,
              child: CupertinoDatePicker(
                onDateTimeChanged: (DateTime dateTime) => selectedDateTime = dateTime,
                // Change the initial date, but it shouldn't affect the present state.
                initialDateTime: DateTime(2016, 4, 5, 15),
              ),
            ),
          ),
        ),
      );

      await tester.drag(
        find.text('9'),
        const Offset(0.0, 32.0),
        pointer: 1,
        touchSlopY: 0,
        warnIfMissed: false,
      ); // see top of file
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 500));

      // Moving up an hour is still based on the original initial date time.
      expect(selectedDateTime, DateTime(2018, 1, 1, 8, 30));
    });

    testWidgets('date picker has expected string', (WidgetTester tester) async {
      await tester.pumpWidget(
        CupertinoApp(
          home: Center(
            child: SizedBox(
              height: 400.0,
              width: 400.0,
              child: CupertinoDatePicker(
                mode: CupertinoDatePickerMode.date,
                onDateTimeChanged: (_) {},
                initialDateTime: DateTime(2018, 9, 15),
              ),
            ),
          ),
        ),
      );

      expect(find.text('September'), findsOneWidget);
      expect(find.text('9'), findsOneWidget);
      expect(find.text('2018'), findsOneWidget);
    });

    testWidgets('datetime picker has expected string', (WidgetTester tester) async {
      await tester.pumpWidget(
        CupertinoApp(
          home: Center(
            child: SizedBox(
              height: 400.0,
              width: 400.0,
              child: CupertinoDatePicker(
                onDateTimeChanged: (_) {},
                initialDateTime: DateTime(2018, 9, 15, 3, 14),
              ),
            ),
          ),
        ),
      );

      expect(find.text('Sat Sep 15'), findsOneWidget);
      expect(find.text('3'), findsOneWidget);
      expect(find.text('14'), findsOneWidget);
      expect(find.text('AM'), findsOneWidget);
    });

    testWidgets('monthYear picker has expected string', (WidgetTester tester) async {
      await tester.pumpWidget(
        CupertinoApp(
          home: Center(
            child: SizedBox(
              height: 400.0,
              width: 400.0,
              child: CupertinoDatePicker(
                mode: CupertinoDatePickerMode.monthYear,
                onDateTimeChanged: (_) {},
                initialDateTime: DateTime(2018, 9),
              ),
            ),
          ),
        ),
      );

      expect(find.text('September'), findsOneWidget);
      expect(find.text('2018'), findsOneWidget);
    });

    testWidgets('width of picker in date and time mode is consistent', (WidgetTester tester) async {
      await tester.pumpWidget(
        CupertinoApp(
          home: Directionality(
            textDirection: TextDirection.ltr,
            child: CupertinoDatePicker(
              onDateTimeChanged: (_) {},
              initialDateTime: DateTime(2018, 1, 1, 10, 30),
            ),
          ),
        ),
      );

      // Distance between the first column and the last column.
      final double distance =
          tester.getCenter(find.text('Mon Jan 1 ')).dx - tester.getCenter(find.text('AM')).dx;

      await tester.pumpWidget(
        CupertinoApp(
          home: Center(
            child: SizedBox(
              height: 400.0,
              width: 800.0,
              child: CupertinoDatePicker(
                onDateTimeChanged: (_) {},
                initialDateTime: DateTime(2018, 1, 1, 10, 30),
              ),
            ),
          ),
        ),
      );

      // Distance between the first and the last column should be the same.
      expect(
        tester.getCenter(find.text('Mon Jan 1 ')).dx - tester.getCenter(find.text('AM')).dx,
        distance,
      );
    });

    testWidgets('width of picker in date mode is consistent', (WidgetTester tester) async {
      await tester.pumpWidget(
        CupertinoApp(
          home: Center(
            child: SizedBox(
              height: 400.0,
              width: 400.0,
              child: CupertinoDatePicker(
                mode: CupertinoDatePickerMode.date,
                onDateTimeChanged: (_) {},
                initialDateTime: DateTime(2018, 1, 1, 10, 30),
              ),
            ),
          ),
        ),
      );

      // Distance between the first column and the last column.
      final double distance =
          tester.getCenter(find.text('January')).dx - tester.getCenter(find.text('2018')).dx;

      await tester.pumpWidget(
        CupertinoApp(
          home: Center(
            child: SizedBox(
              height: 400.0,
              width: 800.0,
              child: CupertinoDatePicker(
                mode: CupertinoDatePickerMode.date,
                onDateTimeChanged: (_) {},
                initialDateTime: DateTime(2018, 1, 1, 10, 30),
              ),
            ),
          ),
        ),
      );

      // Distance between the first and the last column should be the same.
      expect(
        tester.getCenter(find.text('January')).dx - tester.getCenter(find.text('2018')).dx,
        distance,
      );
    });

    testWidgets('width of picker in time mode is consistent', (WidgetTester tester) async {
      await tester.pumpWidget(
        CupertinoApp(
          home: Center(
            child: SizedBox(
              height: 400.0,
              width: 400.0,
              child: CupertinoDatePicker(
                mode: CupertinoDatePickerMode.time,
                onDateTimeChanged: (_) {},
                initialDateTime: DateTime(2018, 1, 1, 10, 30),
              ),
            ),
          ),
        ),
      );

      // Distance between the first column and the last column.
      final double distance =
          tester.getCenter(find.text('10')).dx - tester.getCenter(find.text('AM')).dx;

      await tester.pumpWidget(
        CupertinoApp(
          home: Center(
            child: SizedBox(
              height: 400.0,
              width: 800.0,
              child: CupertinoDatePicker(
                mode: CupertinoDatePickerMode.time,
                onDateTimeChanged: (_) {},
                initialDateTime: DateTime(2018, 1, 1, 10, 30),
              ),
            ),
          ),
        ),
      );

      // Distance between the first and the last column should be the same.
      expect(
        tester.getCenter(find.text('10')).dx - tester.getCenter(find.text('AM')).dx,
        moreOrLessEquals(distance),
      );
    });

    testWidgets('width of picker in monthYear mode is consistent', (WidgetTester tester) async {
      await tester.pumpWidget(
        CupertinoApp(
          home: Center(
            child: SizedBox(
              height: 400.0,
              width: 400.0,
              child: CupertinoDatePicker(
                mode: CupertinoDatePickerMode.monthYear,
                onDateTimeChanged: (_) {},
                initialDateTime: DateTime(2018),
              ),
            ),
          ),
        ),
      );

      // Distance between the first column and the last column.
      final double distance =
          tester.getCenter(find.text('January')).dx - tester.getCenter(find.text('2018')).dx;

      await tester.pumpWidget(
        CupertinoApp(
          home: Center(
            child: SizedBox(
              height: 400.0,
              width: 800.0,
              child: CupertinoDatePicker(
                mode: CupertinoDatePickerMode.monthYear,
                onDateTimeChanged: (_) {},
                initialDateTime: DateTime(2018),
              ),
            ),
          ),
        ),
      );

      // Distance between the first and the last column should be the same.
      expect(
        tester.getCenter(find.text('January')).dx - tester.getCenter(find.text('2018')).dx,
        distance,
      );
    });

    testWidgets('wheel does not bend outwards', (WidgetTester tester) async {
      final Widget dateWidget = CupertinoDatePicker(
        mode: CupertinoDatePickerMode.date,
        onDateTimeChanged: (_) {},
        initialDateTime: DateTime(2018, 1, 1, 10, 30),
      );

      const String centerMonth = 'January';
      const List<String> visibleMonthsExceptTheCenter = <String>[
        'September',
        'October',
        'November',
        'December',
        'February',
        'March',
        'April',
        'May',
      ];

      await tester.pumpWidget(
        CupertinoApp(
          home: CupertinoPageScaffold(
            child: Center(child: SizedBox(height: 200.0, width: 300.0, child: dateWidget)),
          ),
        ),
      );

      // The wheel does not bend outwards.
      for (final String month in visibleMonthsExceptTheCenter) {
        expect(
          tester.getBottomLeft(find.text(centerMonth)).dx,
          lessThan(tester.getBottomLeft(find.text(month)).dx),
        );
      }

      await tester.pumpWidget(
        CupertinoApp(
          home: CupertinoPageScaffold(
            child: Center(child: SizedBox(height: 200.0, width: 3000.0, child: dateWidget)),
          ),
        ),
      );

      // The wheel does not bend outwards at large widths.
      for (final String month in visibleMonthsExceptTheCenter) {
        expect(
          tester.getBottomLeft(find.text(centerMonth)).dx,
          lessThan(tester.getBottomLeft(find.text(month)).dx),
        );
      }
    });

    testWidgets('non-selectable dates are greyed out, '
        'when minimum date is unconstrained', (WidgetTester tester) async {
      final DateTime maximum = DateTime(2018, 6, 15);
      await tester.pumpWidget(
        CupertinoApp(
          home: Center(
            child: SizedBox(
              height: 400.0,
              width: 400.0,
              child: CupertinoDatePicker(
                mode: CupertinoDatePickerMode.date,
                maximumDate: maximum,
                onDateTimeChanged: (_) {},
                initialDateTime: DateTime(2018, 6, 15),
              ),
            ),
          ),
        ),
      );

      // unconstrained bounds are not affected.
      expect(
        tester.widget<Text>(find.text('14')).style!.color,
        isNot(isSameColorAs(CupertinoColors.inactiveGray.color)),
      );

      // the selected day is not affected.
      expect(
        tester.widget<Text>(find.text('15')).style!.color,
        isNot(isSameColorAs(CupertinoColors.inactiveGray.color)),
      );

      // out of bounds and should be greyed out.
      expect(
        tester.widget<Text>(find.text('16')).style!.color,
        isSameColorAs(CupertinoColors.inactiveGray.color),
      );
    });

    testWidgets('non-selectable dates are greyed out, '
        'when maximum date is unconstrained', (WidgetTester tester) async {
      final DateTime minimum = DateTime(2018, 6, 15);
      await tester.pumpWidget(
        CupertinoApp(
          home: Center(
            child: SizedBox(
              height: 400.0,
              width: 400.0,
              child: CupertinoDatePicker(
                mode: CupertinoDatePickerMode.date,
                minimumDate: minimum,
                onDateTimeChanged: (_) {},
                initialDateTime: DateTime(2018, 6, 15),
              ),
            ),
          ),
        ),
      );

      // out of bounds and should be greyed out.
      expect(
        tester.widget<Text>(find.text('14')).style!.color,
        isSameColorAs(CupertinoColors.inactiveGray.color),
      );

      // the selected day is not affected.
      expect(
        tester.widget<Text>(find.text('15')).style!.color,
        isNot(isSameColorAs(CupertinoColors.inactiveGray.color)),
      );

      // unconstrained bounds are not affected.
      expect(
        tester.widget<Text>(find.text('16')).style!.color,
        isNot(isSameColorAs(CupertinoColors.inactiveGray.color)),
      );
    });

    testWidgets('non-selectable dates are greyed out, '
        'months should be taken into account when greying out days', (WidgetTester tester) async {
      final DateTime minimum = DateTime(2018, 5, 15);
      final DateTime maximum = DateTime(2018, 7, 15);
      await tester.pumpWidget(
        CupertinoApp(
          home: Center(
            child: SizedBox(
              height: 400.0,
              width: 400.0,
              child: CupertinoDatePicker(
                mode: CupertinoDatePickerMode.date,
                minimumDate: minimum,
                maximumDate: maximum,
                onDateTimeChanged: (_) {},
                initialDateTime: DateTime(2018, 6, 15),
              ),
            ),
          ),
        ),
      );

      // days of a different min/max month are not affected.
      expect(
        tester.widget<Text>(find.text('14')).style!.color,
        isNot(isSameColorAs(CupertinoColors.inactiveGray.color)),
      );
      expect(
        tester.widget<Text>(find.text('16')).style!.color,
        isNot(isSameColorAs(CupertinoColors.inactiveGray.color)),
      );
    });

    testWidgets('non-selectable dates are greyed out, '
        'years should be taken into account when greying out days', (WidgetTester tester) async {
      final DateTime minimum = DateTime(2017, 6, 15);
      final DateTime maximum = DateTime(2019, 6, 15);
      await tester.pumpWidget(
        CupertinoApp(
          home: Center(
            child: SizedBox(
              height: 400.0,
              width: 400.0,
              child: CupertinoDatePicker(
                mode: CupertinoDatePickerMode.date,
                minimumDate: minimum,
                maximumDate: maximum,
                onDateTimeChanged: (_) {},
                initialDateTime: DateTime(2018, 6, 15),
              ),
            ),
          ),
        ),
      );

      // days of a different min/max year are not affected.
      expect(
        tester.widget<Text>(find.text('14')).style!.color,
        isNot(isSameColorAs(CupertinoColors.inactiveGray.color)),
      );
      expect(
        tester.widget<Text>(find.text('16')).style!.color,
        isNot(isSameColorAs(CupertinoColors.inactiveGray.color)),
      );
    });

    testWidgets('picker automatically scrolls away from invalid date on month change', (
      WidgetTester tester,
    ) async {
      late DateTime date;
      await tester.pumpWidget(
        CupertinoApp(
          home: Center(
            child: SizedBox(
              height: 400.0,
              width: 400.0,
              child: CupertinoDatePicker(
                mode: CupertinoDatePickerMode.date,
                onDateTimeChanged: (DateTime newDate) {
                  date = newDate;
                },
                initialDateTime: DateTime(2018, 3, 30),
              ),
            ),
          ),
        ),
      );

      await tester.drag(
        find.text('March'),
        const Offset(0, 32.0),
        touchSlopY: 0.0,
        warnIfMissed: false,
      ); // see top of file

      // Momentarily, the 2018 and the incorrect 30 of February is aligned.
      expect(tester.getTopLeft(find.text('2018')).dy, tester.getTopLeft(find.text('30')).dy);
      await tester.pump(); // Once to trigger the post frame animate call.
      await tester.pump(); // Once to start the DrivenScrollActivity.
      await tester.pump(const Duration(milliseconds: 500));

      expect(date, DateTime(2018, 2, 28));
      expect(tester.getTopLeft(find.text('2018')).dy, tester.getTopLeft(find.text('28')).dy);
    });

    testWidgets('date picker automatically scrolls away from invalid date, '
        "and onDateTimeChanged doesn't report these dates", (WidgetTester tester) async {
      late DateTime date;
      // 2016 is a leap year.
      final DateTime minimum = DateTime(2016, 2, 29);
      final DateTime maximum = DateTime(2018, 12, 31);
      await tester.pumpWidget(
        CupertinoApp(
          home: Center(
            child: SizedBox(
              height: 400.0,
              width: 400.0,
              child: CupertinoDatePicker(
                mode: CupertinoDatePickerMode.date,
                minimumDate: minimum,
                maximumDate: maximum,
                onDateTimeChanged: (DateTime newDate) {
                  date = newDate;
                  // Callback doesn't transiently go into invalid dates.
                  expect(newDate.isAtSameMomentAs(minimum) || newDate.isAfter(minimum), isTrue);
                  expect(newDate.isAtSameMomentAs(maximum) || newDate.isBefore(maximum), isTrue);
                },
                initialDateTime: DateTime(2017, 2, 28),
              ),
            ),
          ),
        ),
      );

      // 2017 has 28 days in Feb so 29 is greyed out.
      expect(
        tester.widget<Text>(find.text('29')).style!.color,
        isSameColorAs(CupertinoColors.inactiveGray.color),
      );

      await tester.drag(
        find.text('2017'),
        const Offset(0.0, 32.0),
        touchSlopY: 0.0,
        warnIfMissed: false,
      ); // see top of file
      await tester.pump();
      await tester.pumpAndSettle(); // Now the autoscrolling should happen.

      expect(date, DateTime(2016, 2, 29));

      // 2016 has 29 days in Feb so 29 is not greyed out.
      expect(
        tester.widget<Text>(find.text('29')).style!.color,
        isNot(isSameColorAs(CupertinoColors.inactiveGray.color)),
      );

      await tester.drag(
        find.text('2016'),
        const Offset(0.0, -32.0),
        touchSlopY: 0.0,
        warnIfMissed: false,
      ); // see top of file
      await tester.pump(); // Once to trigger the post frame animate call.
      await tester.pumpAndSettle();

      expect(date, DateTime(2017, 2, 28));

      expect(
        tester.widget<Text>(find.text('29')).style!.color,
        isSameColorAs(CupertinoColors.inactiveGray.color),
      );
    });

    testWidgets('dateTime picker automatically scrolls away from invalid date, '
        "and onDateTimeChanged doesn't report these dates", (WidgetTester tester) async {
      late DateTime date;
      final DateTime minimum = DateTime(2019, 11, 11, 3, 30);
      final DateTime maximum = DateTime(2019, 11, 11, 14, 59, 59);
      await tester.pumpWidget(
        CupertinoApp(
          home: Center(
            child: SizedBox(
              height: 400.0,
              width: 400.0,
              child: CupertinoDatePicker(
                minimumDate: minimum,
                maximumDate: maximum,
                onDateTimeChanged: (DateTime newDate) {
                  date = newDate;
                  // Callback doesn't transiently go into invalid dates.
                  expect(minimum.isAfter(newDate), isFalse);
                  expect(maximum.isBefore(newDate), isFalse);
                },
                initialDateTime: DateTime(2019, 11, 11, 4),
              ),
            ),
          ),
        ),
      );

      // 3:00 is valid but 2:00 should be invalid.
      expect(
        tester.widget<Text>(find.text('3')).style!.color,
        isNot(isSameColorAs(CupertinoColors.inactiveGray.color)),
      );

      expect(
        tester.widget<Text>(find.text('2')).style!.color,
        isSameColorAs(CupertinoColors.inactiveGray.color),
      );

      // 'PM' is greyed out.
      expect(
        tester.widget<Text>(find.text('PM')).style!.color,
        isSameColorAs(CupertinoColors.inactiveGray.color),
      );

      await tester.drag(
        find.text('AM'),
        const Offset(0.0, -32.0),
        touchSlopY: 0.0,
        warnIfMissed: false,
      ); // see top of file
      await tester.pump();
      await tester.pumpAndSettle(); // Now the autoscrolling should happen.

      expect(date, DateTime(2019, 11, 11, 14, 59));

      // 3'o clock and 'AM' are now greyed out.
      expect(
        tester.widget<Text>(find.text('AM')).style!.color,
        isSameColorAs(CupertinoColors.inactiveGray.color),
      );
      expect(
        tester.widget<Text>(find.text('3')).style!.color,
        isSameColorAs(CupertinoColors.inactiveGray.color),
      );

      await tester.drag(
        find.text('PM'),
        const Offset(0.0, 32.0),
        touchSlopY: 0.0,
        warnIfMissed: false,
      ); // see top of file
      await tester.pump(); // Once to trigger the post frame animate call.
      await tester.pumpAndSettle();

      // Returns to min date.
      expect(date, DateTime(2019, 11, 11, 3, 30));
    });

    testWidgets('time picker automatically scrolls away from invalid date, '
        "and onDateTimeChanged doesn't report these dates", (WidgetTester tester) async {
      late DateTime date;
      final DateTime minimum = DateTime(2019, 11, 11, 3, 30);
      final DateTime maximum = DateTime(2019, 11, 11, 14, 59, 59);
      await tester.pumpWidget(
        CupertinoApp(
          home: Center(
            child: SizedBox(
              height: 400.0,
              width: 400.0,
              child: CupertinoDatePicker(
                mode: CupertinoDatePickerMode.time,
                minimumDate: minimum,
                maximumDate: maximum,
                onDateTimeChanged: (DateTime newDate) {
                  date = newDate;
                  // Callback doesn't transiently go into invalid dates.
                  expect(minimum.isAfter(newDate), isFalse);
                  expect(maximum.isBefore(newDate), isFalse);
                },
                initialDateTime: DateTime(2019, 11, 11, 4),
              ),
            ),
          ),
        ),
      );

      // 3:00 is valid but 2:00 should be invalid.
      expect(
        tester.widget<Text>(find.text('3')).style!.color,
        isNot(isSameColorAs(CupertinoColors.inactiveGray.color)),
      );

      expect(
        tester.widget<Text>(find.text('2')).style!.color,
        isSameColorAs(CupertinoColors.inactiveGray.color),
      );

      // 'PM' is greyed out.
      expect(
        tester.widget<Text>(find.text('PM')).style!.color,
        isSameColorAs(CupertinoColors.inactiveGray.color),
      );

      await tester.drag(
        find.text('AM'),
        const Offset(0.0, -32.0),
        touchSlopY: 0.0,
        warnIfMissed: false,
      ); // see top of file
      await tester.pump();
      await tester.pumpAndSettle(); // Now the autoscrolling should happen.

      expect(date, DateTime(2019, 11, 11, 14, 59));

      // 3'o clock and 'AM' are now greyed out.
      expect(
        tester.widget<Text>(find.text('AM')).style!.color,
        isSameColorAs(CupertinoColors.inactiveGray.color),
      );
      expect(
        tester.widget<Text>(find.text('3')).style!.color,
        isSameColorAs(CupertinoColors.inactiveGray.color),
      );

      await tester.drag(
        find.text('PM'),
        const Offset(0.0, 32.0),
        touchSlopY: 0.0,
        warnIfMissed: false,
      ); // see top of file
      await tester.pump(); // Once to trigger the post frame animate call.
      await tester.pumpAndSettle();

      // Returns to min date.
      expect(date, DateTime(2019, 11, 11, 3, 30));
    });

    testWidgets('monthYear picker automatically scrolls away from invalid date, '
        "and onDateTimeChanged doesn't report these dates", (WidgetTester tester) async {
      late DateTime date;
      final DateTime minimum = DateTime(2016, 2);
      final DateTime maximum = DateTime(2018, 12);
      await tester.pumpWidget(
        CupertinoApp(
          home: Center(
            child: SizedBox(
              height: 400.0,
              width: 400.0,
              child: CupertinoDatePicker(
                mode: CupertinoDatePickerMode.monthYear,
                minimumDate: minimum,
                maximumDate: maximum,
                onDateTimeChanged: (DateTime newDate) {
                  date = newDate;
                  // Callback doesn't transiently go into invalid dates.
                  expect(newDate.isAtSameMomentAs(minimum) || newDate.isAfter(minimum), isTrue);
                  expect(newDate.isAtSameMomentAs(maximum) || newDate.isBefore(maximum), isTrue);
                },
                initialDateTime: DateTime(2017, 2),
              ),
            ),
          ),
        ),
      );

      await tester.drag(
        find.text('2017'),
        const Offset(0.0, 100.0),
        touchSlopY: 0.0,
        warnIfMissed: false,
      ); // see top of file
      await tester.pump();
      await tester.pumpAndSettle(); // Now the autoscrolling should happen.

      expect(date, DateTime(2016, 2));

      await tester.drag(
        find.text('2016'),
        const Offset(0.0, -100.0),
        touchSlopY: 0.0,
        warnIfMissed: false,
      ); // see top of file
      await tester.pump(); // Once to trigger the post frame animate call.
      await tester.pumpAndSettle();

      expect(date, DateTime(2018, 12));

      await tester.drag(
        find.text('2016'),
        const Offset(0.0, 32.0),
        touchSlopY: 0.0,
        warnIfMissed: false,
      ); // see top of file
      await tester.pump(); // Once to trigger the post frame animate call.
      await tester.pumpAndSettle();

      expect(date, DateTime(2017, 12));
    });

    testWidgets('picker automatically scrolls away from invalid date on day change', (
      WidgetTester tester,
    ) async {
      late DateTime date;
      await tester.pumpWidget(
        CupertinoApp(
          home: Center(
            child: SizedBox(
              height: 400.0,
              width: 400.0,
              child: CupertinoDatePicker(
                mode: CupertinoDatePickerMode.date,
                onDateTimeChanged: (DateTime newDate) {
                  date = newDate;
                },
                initialDateTime: DateTime(2018, 2, 27), // 2018 has 28 days in Feb.
              ),
            ),
          ),
        ),
      );

      await tester.drag(
        find.text('27'),
        const Offset(0.0, -32.0),
        pointer: 1,
        touchSlopY: 0.0,
        warnIfMissed: false,
      ); // see top of file
      await tester.pump();
      expect(date, DateTime(2018, 2, 28));

      await tester.drag(
        find.text('28'),
        const Offset(0.0, -32.0),
        pointer: 1,
        touchSlopY: 0.0,
        warnIfMissed: false,
      ); // see top of file
      await tester.pump(); // Once to trigger the post frame animate call.

      // Callback doesn't transiently go into invalid dates.
      expect(date, DateTime(2018, 2, 28));
      // Momentarily, the invalid 29th of Feb is dragged into the middle.
      expect(tester.getTopLeft(find.text('2018')).dy, tester.getTopLeft(find.text('29')).dy);

      await tester.pump(); // Once to start the DrivenScrollActivity.
      await tester.pump(const Duration(milliseconds: 500));

      expect(date, DateTime(2018, 2, 28));
      expect(tester.getTopLeft(find.text('2018')).dy, tester.getTopLeft(find.text('28')).dy);
    });

    testWidgets(
      'date picker should only take into account the date part of minimumDate and maximumDate',
      (WidgetTester tester) async {
        // Regression test for https://github.com/flutter/flutter/issues/49606.
        late DateTime date;
        final DateTime minDate = DateTime(2020, 1, 1, 12);
        await tester.pumpWidget(
          CupertinoApp(
            home: Center(
              child: SizedBox(
                height: 400.0,
                width: 400.0,
                child: CupertinoDatePicker(
                  mode: CupertinoDatePickerMode.date,
                  minimumDate: minDate,
                  onDateTimeChanged: (DateTime newDate) {
                    date = newDate;
                  },
                  initialDateTime: DateTime(2020, 1, 12),
                ),
              ),
            ),
          ),
        );

        // Scroll to 2019.
        await tester.drag(
          find.text('2020'),
          const Offset(0.0, 32.0),
          touchSlopY: 0.0,
          warnIfMissed: false,
        ); // see top of file
        await tester.pump();
        await tester.pumpAndSettle();
        expect(date.year, minDate.year);
        expect(date.month, minDate.month);
        expect(date.day, minDate.day);
      },
    );

    testWidgets(
      'date picker does not display previous day of minimumDate if it is set at midnight',
      (WidgetTester tester) async {
        // Regression test for https://github.com/flutter/flutter/issues/72932
        final DateTime minDate = DateTime(2019, 12, 31);
        await tester.pumpWidget(
          CupertinoApp(
            home: Center(
              child: SizedBox(
                height: 400.0,
                width: 400.0,
                child: CupertinoDatePicker(
                  minimumDate: minDate,
                  onDateTimeChanged: (DateTime newDate) {},
                  initialDateTime: minDate.add(const Duration(days: 1)),
                ),
              ),
            ),
          ),
        );

        expect(find.text('Mon Dec 30'), findsNothing);
      },
    );

    group('Picker handles initial noon/midnight times', () {
      testWidgets('midnight', (WidgetTester tester) async {
        late DateTime date;
        await tester.pumpWidget(
          CupertinoApp(
            home: Center(
              child: SizedBox(
                height: 400.0,
                width: 400.0,
                child: CupertinoDatePicker(
                  mode: CupertinoDatePickerMode.time,
                  onDateTimeChanged: (DateTime newDate) {
                    date = newDate;
                  },
                  initialDateTime: DateTime(2019, 1, 1, 0, 15),
                ),
              ),
            ),
          ),
        );

        // 0:15 -> 0:16
        await tester.drag(find.text('15'), _kRowOffset, warnIfMissed: false); // see top of file
        await tester.pump();
        await tester.pump(const Duration(milliseconds: 500));

        expect(date, DateTime(2019, 1, 1, 0, 16));
      });

      testWidgets('noon', (WidgetTester tester) async {
        late DateTime date;
        await tester.pumpWidget(
          CupertinoApp(
            home: Center(
              child: SizedBox(
                height: 400.0,
                width: 400.0,
                child: CupertinoDatePicker(
                  mode: CupertinoDatePickerMode.time,
                  onDateTimeChanged: (DateTime newDate) {
                    date = newDate;
                  },
                  initialDateTime: DateTime(2019, 1, 1, 12, 15),
                ),
              ),
            ),
          ),
        );

        // 12:15 -> 12:16
        await tester.drag(find.text('15'), _kRowOffset, warnIfMissed: false); // see top of file
        await tester.pump();
        await tester.pump(const Duration(milliseconds: 500));

        expect(date, DateTime(2019, 1, 1, 12, 16));
      });

      testWidgets('noon in 24 hour time', (WidgetTester tester) async {
        late DateTime date;
        await tester.pumpWidget(
          CupertinoApp(
            home: Center(
              child: SizedBox(
                height: 400.0,
                width: 400.0,
                child: CupertinoDatePicker(
                  use24hFormat: true,
                  mode: CupertinoDatePickerMode.time,
                  onDateTimeChanged: (DateTime newDate) {
                    date = newDate;
                  },
                  initialDateTime: DateTime(2019, 1, 1, 12, 25),
                ),
              ),
            ),
          ),
        );

        // 12:25 -> 12:26
        await tester.drag(find.text('25'), _kRowOffset, warnIfMissed: false); // see top of file
        await tester.pump();
        await tester.pump(const Duration(milliseconds: 500));

        expect(date, DateTime(2019, 1, 1, 12, 26));
      });
    });

    testWidgets('picker persists am/pm value when scrolling hours', (WidgetTester tester) async {
      late DateTime date;
      await tester.pumpWidget(
        CupertinoApp(
          home: Center(
            child: SizedBox(
              height: 400.0,
              width: 400.0,
              child: CupertinoDatePicker(
                mode: CupertinoDatePickerMode.time,
                onDateTimeChanged: (DateTime newDate) {
                  date = newDate;
                },
                initialDateTime: DateTime(2019, 1, 1, 3),
              ),
            ),
          ),
        ),
      );

      // 3:00 -> 15:00
      await tester.drag(find.text('AM'), _kRowOffset, warnIfMissed: false); // see top of file
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 500));

      expect(date, DateTime(2019, 1, 1, 15));

      // 15:00 -> 16:00
      await tester.drag(find.text('3'), _kRowOffset, warnIfMissed: false); // see top of file
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 500));

      expect(date, DateTime(2019, 1, 1, 16));

      // 16:00 -> 4:00
      await tester.drag(find.text('PM'), -_kRowOffset, warnIfMissed: false); // see top of file
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 500));

      expect(date, DateTime(2019, 1, 1, 4));

      // 4:00 -> 3:00
      await tester.drag(find.text('4'), -_kRowOffset, warnIfMissed: false); // see top of file
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 500));

      expect(date, DateTime(2019, 1, 1, 3));
    });

    testWidgets(
      'picker automatically scrolls the am/pm column when the hour column changes enough',
      (WidgetTester tester) async {
        late DateTime date;
        await tester.pumpWidget(
          CupertinoApp(
            home: Center(
              child: SizedBox(
                height: 400.0,
                width: 400.0,
                child: CupertinoDatePicker(
                  mode: CupertinoDatePickerMode.time,
                  onDateTimeChanged: (DateTime newDate) {
                    date = newDate;
                  },
                  initialDateTime: DateTime(2018, 1, 1, 11, 59),
                ),
              ),
            ),
          ),
        );

        const Offset deltaOffset = Offset(0.0, -18.0);

        // 11:59 -> 12:59
        await tester.drag(find.text('11'), _kRowOffset, warnIfMissed: false); // see top of file
        await tester.pump();
        await tester.pump(const Duration(milliseconds: 500));

        expect(date, DateTime(2018, 1, 1, 12, 59));

        // 12:59 -> 11:59
        await tester.drag(find.text('12'), -_kRowOffset, warnIfMissed: false); // see top of file
        await tester.pump();
        await tester.pump(const Duration(milliseconds: 500));

        expect(date, DateTime(2018, 1, 1, 11, 59));

        // 11:59 -> 9:59
        await tester.drag(
          find.text('11'),
          -((_kRowOffset - deltaOffset) * 2 + deltaOffset),
          warnIfMissed: false,
        ); // see top of file
        await tester.pump();
        await tester.pump(const Duration(milliseconds: 500));

        expect(date, DateTime(2018, 1, 1, 9, 59));

        // 9:59 -> 15:59
        await tester.drag(
          find.text('9'),
          (_kRowOffset - deltaOffset) * 6 + deltaOffset,
          warnIfMissed: false,
        ); // see top of file
        await tester.pump();
        await tester.pump(const Duration(milliseconds: 500));

        expect(date, DateTime(2018, 1, 1, 15, 59));
      },
    );

    testWidgets('date picker given too narrow space horizontally shows message', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        CupertinoApp(
          home: Center(
            child: SizedBox(
              // This is too small to draw the picker out fully.
              width: 100,
              child: CupertinoDatePicker(
                initialDateTime: DateTime(2019, 1, 1, 4),
                onDateTimeChanged: (_) {},
              ),
            ),
          ),
        ),
      );

      final dynamic exception = tester.takeException();
      expect(exception, isFlutterError);
      expect(
        exception.toString(),
        contains('Insufficient horizontal space to render the CupertinoDatePicker'),
      );
    });

    testWidgets('DatePicker golden tests', (WidgetTester tester) async {
      Widget buildApp(CupertinoDatePickerMode mode) {
        return CupertinoApp(
          home: Center(
            child: SizedBox(
              width: 500,
              height: 400,
              child: RepaintBoundary(
                child: CupertinoDatePicker(
                  key: ValueKey<CupertinoDatePickerMode>(mode),
                  mode: mode,
                  initialDateTime: DateTime(2019, 1, 1, 4, 12, 30),
                  onDateTimeChanged: (_) {},
                ),
              ),
            ),
          ),
        );
      }

      await tester.pumpWidget(buildApp(CupertinoDatePickerMode.time));
      if (!skipPerspectiveTextGoldens) {
        await expectLater(
          find.byType(CupertinoDatePicker),
          matchesGoldenFile('date_picker_test.time.initial.png'),
        );
      }

      await tester.pumpWidget(buildApp(CupertinoDatePickerMode.date));
      if (!skipPerspectiveTextGoldens) {
        await expectLater(
          find.byType(CupertinoDatePicker),
          matchesGoldenFile('date_picker_test.date.initial.png'),
        );
      }

      await tester.pumpWidget(buildApp(CupertinoDatePickerMode.monthYear));
      if (!skipPerspectiveTextGoldens) {
        await expectLater(
          find.byType(CupertinoDatePicker),
          matchesGoldenFile('date_picker_test.monthyear.initial.png'),
        );
      }

      await tester.pumpWidget(buildApp(CupertinoDatePickerMode.dateAndTime));
      if (!skipPerspectiveTextGoldens) {
        await expectLater(
          find.byType(CupertinoDatePicker),
          matchesGoldenFile('date_picker_test.datetime.initial.png'),
        );
      }

      // Slightly drag the hour component to make the current hour off-center.
      await tester.drag(
        find.text('4'),
        Offset(0, _kRowOffset.dy / 2),
        warnIfMissed: false,
      ); // see top of file
      await tester.pump();

      if (!skipPerspectiveTextGoldens) {
        await expectLater(
          find.byType(CupertinoDatePicker),
          matchesGoldenFile('date_picker_test.datetime.drag.png'),
        );
      }
    }, skip: impellerEnabled); // https://github.com/flutter/flutter/issues/143616

    testWidgets('DatePicker displays the date in correct order', (WidgetTester tester) async {
      await tester.pumpWidget(
        CupertinoApp(
          home: Center(
            child: SizedBox(
              height: 400.0,
              width: 400.0,
              child: CupertinoDatePicker(
                dateOrder: DatePickerDateOrder.ydm,
                mode: CupertinoDatePickerMode.date,
                onDateTimeChanged: (DateTime newDate) {},
                initialDateTime: DateTime(2018, 1, 14, 10, 30),
              ),
            ),
          ),
        ),
      );

      expect(
        tester.getTopLeft(find.text('2018')).dx,
        lessThan(tester.getTopLeft(find.text('14')).dx),
      );

      expect(
        tester.getTopLeft(find.text('14')).dx,
        lessThan(tester.getTopLeft(find.text('January')).dx),
      );
    });

    testWidgets('monthYear DatePicker displays the date in correct order', (
      WidgetTester tester,
    ) async {
      Widget buildApp(DatePickerDateOrder order) {
        return CupertinoApp(
          home: Center(
            child: SizedBox(
              height: 400.0,
              width: 400.0,
              child: CupertinoDatePicker(
                key: ValueKey<DatePickerDateOrder>(order),
                dateOrder: order,
                mode: CupertinoDatePickerMode.monthYear,
                onDateTimeChanged: (DateTime newDate) {},
                initialDateTime: DateTime(2018, 1, 14, 10, 30),
              ),
            ),
          ),
        );
      }

      await tester.pumpWidget(buildApp(DatePickerDateOrder.dmy));
      expect(
        tester.getTopLeft(find.text('January')).dx,
        lessThan(tester.getTopLeft(find.text('2018')).dx),
      );

      await tester.pumpWidget(buildApp(DatePickerDateOrder.mdy));
      expect(
        tester.getTopLeft(find.text('January')).dx,
        lessThan(tester.getTopLeft(find.text('2018')).dx),
      );

      await tester.pumpWidget(buildApp(DatePickerDateOrder.ydm));
      expect(
        tester.getTopLeft(find.text('2018')).dx,
        lessThan(tester.getTopLeft(find.text('January')).dx),
      );

      await tester.pumpWidget(buildApp(DatePickerDateOrder.ymd));
      expect(
        tester.getTopLeft(find.text('2018')).dx,
        lessThan(tester.getTopLeft(find.text('January')).dx),
      );
    });

    testWidgets('DatePicker displays hours and minutes correctly in RTL', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        CupertinoApp(
          home: Directionality(
            textDirection: TextDirection.rtl,
            child: Center(
              child: SizedBox(
                width: 500,
                height: 400,
                child: CupertinoDatePicker(
                  initialDateTime: DateTime(2019, 1, 1, 4),
                  onDateTimeChanged: (_) {},
                ),
              ),
            ),
          ),
        ),
      );

      final double hourLeft = tester.getTopLeft(find.text('4')).dx;
      final double minuteLeft = tester.getTopLeft(find.text('00')).dx;
      expect(hourLeft, lessThan(minuteLeft));
    });
  });

  testWidgets('TimerPicker golden tests', (WidgetTester tester) async {
    await tester.pumpWidget(
      CupertinoApp(
        // Also check if the picker respects the theme.
        theme: const CupertinoThemeData(
          textTheme: CupertinoTextThemeData(
            pickerTextStyle: TextStyle(color: Color(0xFF663311), fontSize: 21),
          ),
        ),
        home: Center(
          child: SizedBox(
            width: 320,
            height: 216,
            child: RepaintBoundary(
              child: CupertinoTimerPicker(
                mode: CupertinoTimerPickerMode.hm,
                initialTimerDuration: const Duration(hours: 23, minutes: 59),
                onTimerDurationChanged: (_) {},
              ),
            ),
          ),
        ),
      ),
    );

    if (!skipPerspectiveTextGoldens) {
      await expectLater(
        find.byType(CupertinoTimerPicker),
        matchesGoldenFile('timer_picker_test.datetime.initial.png'),
      );
    }

    // Slightly drag the minute component to make the current minute off-center.
    await tester.drag(
      find.text('59'),
      Offset(0, _kRowOffset.dy / 2),
      warnIfMissed: false,
    ); // see top of file
    await tester.pump();

    if (!skipPerspectiveTextGoldens) {
      await expectLater(
        find.byType(CupertinoTimerPicker),
        matchesGoldenFile('timer_picker_test.datetime.drag.png'),
      );
    }
  }, skip: impellerEnabled); // https://github.com/flutter/flutter/issues/143616

  testWidgets('TimerPicker only changes hour label after scrolling stops', (
    WidgetTester tester,
  ) async {
    Duration? duration;
    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: SizedBox(
            width: 320,
            height: 216,
            child: CupertinoTimerPicker(
              mode: CupertinoTimerPickerMode.hm,
              initialTimerDuration: const Duration(hours: 2, minutes: 30),
              onTimerDurationChanged: (Duration d) {
                duration = d;
              },
            ),
          ),
        ),
      ),
    );

    expect(duration, isNull);
    expect(find.text('hour'), findsNothing);
    expect(find.text('hours'), findsOneWidget);

    await tester.drag(
      find.text('2'),
      Offset(0, -_kRowOffset.dy),
      warnIfMissed: false,
    ); // see top of file
    // Duration should change but not the label.
    expect(duration!.inHours, 1);
    expect(find.text('hour'), findsNothing);
    expect(find.text('hours'), findsOneWidget);
    await tester.pumpAndSettle();

    // Now the label should change.
    expect(duration!.inHours, 1);
    expect(find.text('hours'), findsNothing);
    expect(find.text('hour'), findsOneWidget);
  });

  testWidgets('TimerPicker has intrinsic width and height', (WidgetTester tester) async {
    const Key key = Key('key');

    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoTimerPicker(
          key: key,
          mode: CupertinoTimerPickerMode.hm,
          initialTimerDuration: const Duration(hours: 2, minutes: 30),
          onTimerDurationChanged: (Duration d) {},
        ),
      ),
    );

    expect(
      tester.getSize(find.descendant(of: find.byKey(key), matching: find.byType(Row))),
      const Size(320, 216),
    );

    // Different modes shouldn't share state.
    await tester.pumpWidget(const Placeholder());
    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoTimerPicker(
          key: key,
          mode: CupertinoTimerPickerMode.ms,
          initialTimerDuration: const Duration(minutes: 30, seconds: 3),
          onTimerDurationChanged: (Duration d) {},
        ),
      ),
    );

    expect(
      tester.getSize(find.descendant(of: find.byKey(key), matching: find.byType(Row))),
      const Size(320, 216),
    );

    // Different modes shouldn't share state.
    await tester.pumpWidget(const Placeholder());
    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoTimerPicker(
          key: key,
          initialTimerDuration: const Duration(hours: 5, minutes: 17, seconds: 19),
          onTimerDurationChanged: (Duration d) {},
        ),
      ),
    );

    expect(
      tester.getSize(find.descendant(of: find.byKey(key), matching: find.byType(Row))),
      const Size(342, 216),
    );
  });

  testWidgets('scrollController can be removed or added', (WidgetTester tester) async {
    final SemanticsHandle handle = tester.ensureSemantics();
    late int lastSelectedItem;
    void onSelectedItemChanged(int index) {
      lastSelectedItem = index;
    }

    final FixedExtentScrollController scrollController1 = FixedExtentScrollController();
    addTearDown(scrollController1.dispose);
    await tester.pumpWidget(
      _buildPicker(controller: scrollController1, onSelectedItemChanged: onSelectedItemChanged),
    );

    tester.binding.pipelineOwner.semanticsOwner!.performAction(1, SemanticsAction.increase);
    await tester.pumpAndSettle();
    expect(lastSelectedItem, 1);

    await tester.pumpWidget(_buildPicker(onSelectedItemChanged: onSelectedItemChanged));

    tester.binding.pipelineOwner.semanticsOwner!.performAction(1, SemanticsAction.increase);
    await tester.pumpAndSettle();
    expect(lastSelectedItem, 2);

    final FixedExtentScrollController scrollController2 = FixedExtentScrollController();
    addTearDown(scrollController2.dispose);
    await tester.pumpWidget(
      _buildPicker(controller: scrollController2, onSelectedItemChanged: onSelectedItemChanged),
    );

    tester.binding.pipelineOwner.semanticsOwner!.performAction(1, SemanticsAction.increase);
    await tester.pumpAndSettle();
    expect(lastSelectedItem, 3);

    handle.dispose();
  });

  testWidgets('CupertinoDataPicker does not provide invalid MediaQuery', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/47989.
    Brightness brightness = Brightness.light;
    late StateSetter setState;

    await tester.pumpWidget(
      CupertinoApp(
        theme: const CupertinoThemeData(
          textTheme: CupertinoTextThemeData(
            dateTimePickerTextStyle: TextStyle(
              color: CupertinoDynamicColor.withBrightness(
                color: Color(0xFFFFFFFF),
                darkColor: Color(0xFF000000),
              ),
            ),
          ),
        ),
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter stateSetter) {
            setState = stateSetter;
            return MediaQuery(
              data: MediaQuery.of(context).copyWith(platformBrightness: brightness),
              child: CupertinoDatePicker(
                initialDateTime: DateTime(2019),
                mode: CupertinoDatePickerMode.date,
                onDateTimeChanged: (DateTime date) {},
              ),
            );
          },
        ),
      ),
    );

    expect(
      tester.widget<Text>(find.text('2019')).style!.color,
      isSameColorAs(const Color(0xFFFFFFFF)),
    );

    setState(() {
      brightness = Brightness.dark;
    });
    await tester.pump();

    expect(
      tester.widget<Text>(find.text('2019')).style!.color,
      isSameColorAs(const Color(0xFF000000)),
    );
  });

  testWidgets('picker exports semantics', (WidgetTester tester) async {
    final SemanticsHandle handle = tester.ensureSemantics();
    debugResetSemanticsIdCounter();
    int? lastSelectedItem;
    await tester.pumpWidget(
      _buildPicker(
        onSelectedItemChanged: (int index) {
          lastSelectedItem = index;
        },
      ),
    );

    expect(
      tester.getSemantics(find.byType(CupertinoPicker)),
      matchesSemantics(
        children: <Matcher>[
          matchesSemantics(
            hasIncreaseAction: true,
            increasedValue: '1',
            value: '0',
            textDirection: TextDirection.ltr,
          ),
        ],
      ),
    );

    tester.binding.pipelineOwner.semanticsOwner!.performAction(1, SemanticsAction.increase);
    await tester.pumpAndSettle();

    expect(
      tester.getSemantics(find.byType(CupertinoPicker)),
      matchesSemantics(
        children: <Matcher>[
          matchesSemantics(
            hasIncreaseAction: true,
            hasDecreaseAction: true,
            increasedValue: '2',
            decreasedValue: '0',
            value: '1',
            textDirection: TextDirection.ltr,
          ),
        ],
      ),
    );
    expect(lastSelectedItem, 1);
    handle.dispose();
  });

  // Regression test for https://github.com/flutter/flutter/issues/98567
  testWidgets('picker semantics action test', (WidgetTester tester) async {
    final SemanticsHandle handle = tester.ensureSemantics();
    debugResetSemanticsIdCounter();
    final DateTime initialDate = DateTime(2018, 6, 8);
    late DateTime? date;
    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: SizedBox(
            height: 400.0,
            width: 400.0,
            child: CupertinoDatePicker(
              onDateTimeChanged: (DateTime newDate) => date = newDate,
              initialDateTime: initialDate,
              maximumDate: initialDate.add(const Duration(days: 2)),
              minimumDate: initialDate.subtract(const Duration(days: 2)),
            ),
          ),
        ),
      ),
    );

    tester.binding.pipelineOwner.semanticsOwner!.performAction(4, SemanticsAction.decrease);
    await tester.pumpAndSettle();

    expect(date, DateTime(2018, 6, 7));

    handle.dispose();
  });

  testWidgets('DatePicker adapts to MaterialApp dark mode', (WidgetTester tester) async {
    Widget buildDatePicker(Brightness brightness) {
      return MaterialApp(
        theme: ThemeData(brightness: brightness),
        home: CupertinoDatePicker(
          mode: CupertinoDatePickerMode.date,
          onDateTimeChanged: (DateTime neData) {},
          initialDateTime: DateTime(2018, 10, 10),
        ),
      );
    }

    // CupertinoDatePicker with light theme.
    await tester.pumpWidget(buildDatePicker(Brightness.light));
    RenderParagraph paragraph = tester.renderObject(find.text('October').first);
    expect(paragraph.text.style!.color, CupertinoColors.label);
    // Text style should not return unresolved color.
    expect(paragraph.text.style!.color.toString().contains('UNRESOLVED'), isFalse);

    // CupertinoDatePicker with dark theme.
    await tester.pumpWidget(buildDatePicker(Brightness.dark));
    paragraph = tester.renderObject(find.text('October').first);
    expect(paragraph.text.style!.color, CupertinoColors.label);
    // Text style should not return unresolved color.
    expect(paragraph.text.style!.color.toString().contains('UNRESOLVED'), isFalse);
  });

  testWidgets('TimerPicker adapts to MaterialApp dark mode', (WidgetTester tester) async {
    Widget buildTimerPicker(Brightness brightness) {
      return MaterialApp(
        theme: ThemeData(brightness: brightness),
        home: CupertinoTimerPicker(
          mode: CupertinoTimerPickerMode.hm,
          onTimerDurationChanged: (Duration newDuration) {},
          initialTimerDuration: const Duration(hours: 12, minutes: 30, seconds: 59),
        ),
      );
    }

    // CupertinoTimerPicker with light theme.
    await tester.pumpWidget(buildTimerPicker(Brightness.light));
    RenderParagraph paragraph = tester.renderObject(find.text('hours'));
    expect(paragraph.text.style!.color, CupertinoColors.label);
    // Text style should not return unresolved color.
    expect(paragraph.text.style!.color.toString().contains('UNRESOLVED'), isFalse);

    // CupertinoTimerPicker with light theme.
    await tester.pumpWidget(buildTimerPicker(Brightness.dark));
    paragraph = tester.renderObject(find.text('hours'));
    expect(paragraph.text.style!.color, CupertinoColors.label);
    // Text style should not return unresolved color.
    expect(paragraph.text.style!.color.toString().contains('UNRESOLVED'), isFalse);
  });

  testWidgets('TimerPicker minDate - maxDate with minuteInterval', (WidgetTester tester) async {
    late DateTime date;
    final DateTime minimum = DateTime(2022, 6, 14, 3, 31);
    final DateTime initial = DateTime(2022, 6, 14, 3, 40);
    final DateTime maximum = DateTime(2022, 6, 14, 3, 49);
    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: SizedBox(
            height: 400.0,
            width: 400.0,
            child: CupertinoDatePicker(
              initialDateTime: initial,
              minimumDate: minimum,
              maximumDate: maximum,
              minuteInterval: 5,
              use24hFormat: true,
              onDateTimeChanged: (DateTime newDate) {
                date = newDate;
              },
            ),
          ),
        ),
      ),
    );

    // Drag picker minutes to min date
    await tester.drag(
      find.text('40'),
      const Offset(0.0, 32.0),
      touchSlopY: 0.0,
      warnIfMissed: false,
    );
    await tester.pumpAndSettle();

    // Returns to min date.
    expect(date, DateTime(2022, 6, 14, 3, 35));

    // Drag picker minutes to max date
    await tester.drag(
      find.text('50'),
      const Offset(0.0, -64.0),
      touchSlopY: 0.0,
      warnIfMissed: false,
    );
    await tester.pumpAndSettle();

    // Returns to max date.
    expect(date, DateTime(2022, 6, 14, 3, 45));
  });

  testWidgets('date picker has expected day of week', (WidgetTester tester) async {
    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: SizedBox(
            height: 400.0,
            width: 400.0,
            child: CupertinoDatePicker(
              mode: CupertinoDatePickerMode.date,
              onDateTimeChanged: (_) {},
              initialDateTime: DateTime(2018, 9, 15),
              showDayOfWeek: true,
            ),
          ),
        ),
      ),
    );

    expect(find.text('September'), findsOneWidget);
    expect(find.textContaining('Sat').last, findsOneWidget);
    expect(find.textContaining('15').last, findsOneWidget);
    expect(find.text('2018'), findsOneWidget);
  });

  testWidgets('CupertinoDatePicker selectionOverlayBuilder with monthYear mode', (
    WidgetTester tester,
  ) async {
    final Widget selectionOverlay = Container(color: const Color(0x12345678));

    // For mode = CupertinoDatePickerMode.monthYear
    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoDatePicker(
            mode: CupertinoDatePickerMode.monthYear,
            onDateTimeChanged: (DateTime date) {},
            initialDateTime: DateTime(2018, 9, 15),
            selectionOverlayBuilder: (
              BuildContext context, {
              required int selectedIndex,
              required int columnCount,
            }) {
              return selectionOverlay;
            },
          ),
        ),
      ),
    );

    // Find the selection overlay.
    expect(find.byWidget(selectionOverlay), findsExactly(2));
  });

  testWidgets('CupertinoDatePicker selectionOverlayBuilder with date mode', (
    WidgetTester tester,
  ) async {
    final Widget selectionOverlay = Container(color: const Color(0x12345678));

    // For mode = CupertinoDatePickerMode.date
    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoDatePicker(
            mode: CupertinoDatePickerMode.date,
            onDateTimeChanged: (DateTime date) {},
            initialDateTime: DateTime(2018, 9, 15),
            selectionOverlayBuilder: (
              BuildContext context, {
              required int selectedIndex,
              required int columnCount,
            }) {
              return selectionOverlay;
            },
          ),
        ),
      ),
    );

    // Find the selection overlay.
    expect(find.byWidget(selectionOverlay), findsExactly(3));
  });

  testWidgets('CupertinoDatePicker selectionOverlayBuilder with time mode', (
    WidgetTester tester,
  ) async {
    final Widget selectionOverlay = Container(color: const Color(0x12345678));

    // For mode = CupertinoDatePickerMode.time
    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoDatePicker(
            mode: CupertinoDatePickerMode.time,
            onDateTimeChanged: (DateTime date) {},
            initialDateTime: DateTime(2018, 9, 15),
            selectionOverlayBuilder: (
              BuildContext context, {
              required int selectedIndex,
              required int columnCount,
            }) {
              return selectionOverlay;
            },
          ),
        ),
      ),
    );

    // Find the selection overlay.
    expect(find.byWidget(selectionOverlay), findsExactly(3));
  });

  testWidgets('CupertinoDatePicker selectionOverlayBuilder with dateAndTime mode', (
    WidgetTester tester,
  ) async {
    final Widget selectionOverlay = Container(color: const Color(0x12345678));

    // For mode = CupertinoDatePickerMode.dateAndTime
    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoDatePicker(
            onDateTimeChanged: (DateTime date) {},
            initialDateTime: DateTime(2018, 9, 15),
            selectionOverlayBuilder: (
              BuildContext context, {
              required int selectedIndex,
              required int columnCount,
            }) {
              return selectionOverlay;
            },
          ),
        ),
      ),
    );

    // Find the selection overlay.
    expect(find.byWidget(selectionOverlay), findsExactly(4));
  });

  testWidgets('CupertinoTimerPicker selectionOverlayBuilder with hms mode', (
    WidgetTester tester,
  ) async {
    final Widget selectionOverlay = Container(color: const Color(0x12345678));

    // For mode = CupertinoTimerPickerMode.hms
    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoTimerPicker(
            onTimerDurationChanged: (Duration duration) {},
            initialTimerDuration: const Duration(hours: 1, minutes: 1, seconds: 1),
            selectionOverlayBuilder: (
              BuildContext context, {
              required int selectedIndex,
              required int columnCount,
            }) {
              return selectionOverlay;
            },
          ),
        ),
      ),
    );

    // Find the selection overlay.
    expect(find.byWidget(selectionOverlay), findsExactly(3));
  });

  testWidgets('CupertinoTimerPicker selectionOverlayBuilder with ms mode', (
    WidgetTester tester,
  ) async {
    final Widget selectionOverlay = Container(color: const Color(0x12345678));

    // For mode = CupertinoTimerPickerMode.ms
    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoTimerPicker(
            onTimerDurationChanged: (Duration duration) {},
            mode: CupertinoTimerPickerMode.ms,
            initialTimerDuration: const Duration(hours: 1, minutes: 1, seconds: 1),
            selectionOverlayBuilder: (
              BuildContext context, {
              required int selectedIndex,
              required int columnCount,
            }) {
              return selectionOverlay;
            },
          ),
        ),
      ),
    );

    // Find the selection overlay.
    expect(find.byWidget(selectionOverlay), findsExactly(2));
  });

  testWidgets('CupertinoTimerPicker selectionOverlayBuilder with hm mode', (
    WidgetTester tester,
  ) async {
    final Widget selectionOverlay = Container(color: const Color(0x12345678));

    // For mode = CupertinoTimerPickerMode.hm
    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoTimerPicker(
            onTimerDurationChanged: (Duration duration) {},
            mode: CupertinoTimerPickerMode.hm,
            initialTimerDuration: const Duration(hours: 1, minutes: 1, seconds: 1),
            selectionOverlayBuilder: (
              BuildContext context, {
              required int selectedIndex,
              required int columnCount,
            }) {
              return selectionOverlay;
            },
          ),
        ),
      ),
    );

    // Find the selection overlay.
    expect(find.byWidget(selectionOverlay), findsExactly(2));
  });

  testWidgets('CupertinoDatePicker selectionOverlayBuilder returns null', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoDatePicker(
            onDateTimeChanged: (DateTime date) {},
            initialDateTime: DateTime(2018, 9, 15),
            selectionOverlayBuilder: (
              BuildContext context, {
              required int selectedIndex,
              required int columnCount,
            }) {
              return null;
            },
          ),
        ),
      ),
    );

    expect(find.byType(CupertinoPicker), isNot(paints..rrect()));
  });

  testWidgets('CupertinoTimerPicker selectionOverlayBuilder returns null', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoTimerPicker(
            onTimerDurationChanged: (Duration duration) {},
            mode: CupertinoTimerPickerMode.hm,
            initialTimerDuration: const Duration(hours: 1, minutes: 1, seconds: 1),
            selectionOverlayBuilder: (
              BuildContext context, {
              required int selectedIndex,
              required int columnCount,
            }) {
              return null;
            },
          ),
        ),
      ),
    );

    expect(find.byType(CupertinoPicker), isNot(paints..rrect()));
  });

  testWidgets('CupertinoTimerPicker selectionOverlayBuilder is explicitly passed null', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoTimerPicker(
            onTimerDurationChanged: (Duration duration) {},
            mode: CupertinoTimerPickerMode.hm,
            initialTimerDuration: const Duration(hours: 1, minutes: 1, seconds: 1),
          ),
        ),
      ),
    );

    expect(find.byType(CupertinoPickerDefaultSelectionOverlay), findsExactly(2));
  });

  testWidgets('CupertinoDatePicker selectionOverlayBuilder is explicitly passed null', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoDatePicker(
            onDateTimeChanged: (DateTime date) {},
            initialDateTime: DateTime(2018, 9, 15),
          ),
        ),
      ),
    );

    expect(find.byType(CupertinoPickerDefaultSelectionOverlay), findsExactly(4));
  });

  testWidgets('CupertinoDatePicker accommodates widest text using table codepoints', (
    WidgetTester tester,
  ) async {
    // |---------|
    // |  0x2002 | // EN SPACE - 1/2 Advance
    // |  0x2005 | // FOUR-PER-EM SPACE - 1/4 Advance
    // |---------|
    final List<String> testWords = <String>[
      '\u2002' * 10, // Output: 10 * 1/2 = 5
      '\u2005' * 20, // Output: 20 * 1/4 = 5
    ];

    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoDatePicker(
            onDateTimeChanged: (DateTime date) {},
            initialDateTime: DateTime(2018, 9, 15),
          ),
        ),
      ),
    );

    final BuildContext context = tester.element(find.byType(CupertinoDatePicker));

    const TextStyle textStyle = TextStyle(
      fontSize: 21,
      letterSpacing: 0.4,
      fontWeight: FontWeight.normal,
      color: CupertinoColors.label,
    );

    final List<double> widths =
        testWords.map((String word) => getColumnWidth(word, textStyle, context)).toList();

    final double largestWidth = widths.reduce(math.max);

    final double testWidth = CupertinoDatePicker.getColumnWidth(
      texts: testWords,
      context: context,
      textStyle: textStyle,
    );

    expect(testWidth, equals(largestWidth));
    expect(widths.indexOf(largestWidth), equals(1));
  }, skip: isBrowser); // https://github.com/flutter/flutter/issues/39998
}

Widget _buildPicker({
  FixedExtentScrollController? controller,
  required ValueChanged<int> onSelectedItemChanged,
}) {
  return Directionality(
    textDirection: TextDirection.ltr,
    child: CupertinoPicker(
      scrollController: controller,
      itemExtent: 100.0,
      onSelectedItemChanged: onSelectedItemChanged,
      children: List<Widget>.generate(100, (int index) {
        return Center(child: SizedBox(width: 400.0, height: 100.0, child: Text(index.toString())));
      }),
    ),
  );
}

double getColumnWidth(String text, TextStyle textStyle, BuildContext context) {
  return TextPainter.computeMaxIntrinsicWidth(
    text: TextSpan(text: text, style: textStyle),
    textDirection: Directionality.of(context),
  );
}
