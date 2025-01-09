// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

import '../widgets/feedback_tester.dart';

void main() {
  TestWidgetsFlutterBinding.ensureInitialized();

  final Finder nextMonthIcon = find.byWidgetPredicate(
    (Widget w) => w is IconButton && (w.tooltip?.startsWith('Next month') ?? false),
  );
  final Finder previousMonthIcon = find.byWidgetPredicate(
    (Widget w) => w is IconButton && (w.tooltip?.startsWith('Previous month') ?? false),
  );

  Widget calendarDatePicker({
    Key? key,
    DateTime? initialDate,
    DateTime? firstDate,
    DateTime? lastDate,
    DateTime? currentDate,
    ValueChanged<DateTime>? onDateChanged,
    ValueChanged<DateTime>? onDisplayedMonthChanged,
    DatePickerMode initialCalendarMode = DatePickerMode.day,
    SelectableDayPredicate? selectableDayPredicate,
    TextDirection textDirection = TextDirection.ltr,
    ThemeData? theme,
    bool? useMaterial3,
  }) {
    return MaterialApp(
      theme: theme ?? ThemeData(useMaterial3: useMaterial3),
      home: Material(
        child: Directionality(
          textDirection: textDirection,
          child: CalendarDatePicker(
            key: key,
            initialDate: initialDate,
            firstDate: firstDate ?? DateTime(2001),
            lastDate: lastDate ?? DateTime(2031, DateTime.december, 31),
            currentDate: currentDate ?? DateTime(2016, DateTime.january, 3),
            onDateChanged: onDateChanged ?? (DateTime date) {},
            onDisplayedMonthChanged: onDisplayedMonthChanged,
            initialCalendarMode: initialCalendarMode,
            selectableDayPredicate: selectableDayPredicate,
          ),
        ),
      ),
    );
  }

  Widget yearPicker({
    Key? key,
    DateTime? selectedDate,
    DateTime? initialDate,
    DateTime? firstDate,
    DateTime? lastDate,
    DateTime? currentDate,
    ValueChanged<DateTime>? onChanged,
    TextDirection textDirection = TextDirection.ltr,
  }) {
    return MaterialApp(
      home: Material(
        child: Directionality(
          textDirection: textDirection,
          child: YearPicker(
            key: key,
            selectedDate: selectedDate ?? DateTime(2016, DateTime.january, 15),
            firstDate: firstDate ?? DateTime(2001),
            lastDate: lastDate ?? DateTime(2031, DateTime.december, 31),
            currentDate: currentDate ?? DateTime(2016, DateTime.january, 3),
            onChanged: onChanged ?? (DateTime date) {},
          ),
        ),
      ),
    );
  }

  group('CalendarDatePicker', () {
    testWidgets('Can select a day', (WidgetTester tester) async {
      DateTime? selectedDate;
      await tester.pumpWidget(
        calendarDatePicker(
          initialDate: DateTime(2016, DateTime.january, 15),
          onDateChanged: (DateTime date) => selectedDate = date,
        ),
      );
      await tester.tap(find.text('12'));
      expect(selectedDate, equals(DateTime(2016, DateTime.january, 12)));
    });

    testWidgets('Can select a day with nothing first selected', (WidgetTester tester) async {
      DateTime? selectedDate;
      await tester.pumpWidget(
        calendarDatePicker(onDateChanged: (DateTime date) => selectedDate = date),
      );
      await tester.tap(find.text('12'));
      expect(selectedDate, equals(DateTime(2016, DateTime.january, 12)));
    });

    testWidgets('Can select a month', (WidgetTester tester) async {
      DateTime? displayedMonth;
      await tester.pumpWidget(
        calendarDatePicker(
          initialDate: DateTime(2016, DateTime.january, 15),
          onDisplayedMonthChanged: (DateTime date) => displayedMonth = date,
        ),
      );
      expect(find.text('January 2016'), findsOneWidget);

      // Go back two months
      await tester.tap(previousMonthIcon);
      await tester.pumpAndSettle();
      expect(find.text('December 2015'), findsOneWidget);
      expect(displayedMonth, equals(DateTime(2015, DateTime.december)));
      await tester.tap(previousMonthIcon);
      await tester.pumpAndSettle();
      expect(find.text('November 2015'), findsOneWidget);
      expect(displayedMonth, equals(DateTime(2015, DateTime.november)));

      // Go forward a month
      await tester.tap(nextMonthIcon);
      await tester.pumpAndSettle();
      expect(find.text('December 2015'), findsOneWidget);
      expect(displayedMonth, equals(DateTime(2015, DateTime.december)));
    });

    testWidgets('Can select a month with nothing first selected', (WidgetTester tester) async {
      DateTime? displayedMonth;
      await tester.pumpWidget(
        calendarDatePicker(onDisplayedMonthChanged: (DateTime date) => displayedMonth = date),
      );
      expect(find.text('January 2016'), findsOneWidget);

      // Go back two months
      await tester.tap(previousMonthIcon);
      await tester.pumpAndSettle();
      expect(find.text('December 2015'), findsOneWidget);
      expect(displayedMonth, equals(DateTime(2015, DateTime.december)));
      await tester.tap(previousMonthIcon);
      await tester.pumpAndSettle();
      expect(find.text('November 2015'), findsOneWidget);
      expect(displayedMonth, equals(DateTime(2015, DateTime.november)));

      // Go forward a month
      await tester.tap(nextMonthIcon);
      await tester.pumpAndSettle();
      expect(find.text('December 2015'), findsOneWidget);
      expect(displayedMonth, equals(DateTime(2015, DateTime.december)));
    });

    testWidgets('Can select a year', (WidgetTester tester) async {
      DateTime? displayedMonth;
      await tester.pumpWidget(
        calendarDatePicker(
          initialDate: DateTime(2016, DateTime.january, 15),
          onDisplayedMonthChanged: (DateTime date) => displayedMonth = date,
        ),
      );

      await tester.tap(find.text('January 2016')); // Switch to year mode.
      await tester.pumpAndSettle();
      await tester.tap(find.text('2018'));
      await tester.pumpAndSettle();
      expect(find.text('January 2018'), findsOneWidget);
      expect(displayedMonth, equals(DateTime(2018)));
    });

    testWidgets('Can select a year with nothing first selected', (WidgetTester tester) async {
      DateTime? displayedMonth;
      await tester.pumpWidget(
        calendarDatePicker(onDisplayedMonthChanged: (DateTime date) => displayedMonth = date),
      );

      await tester.tap(find.text('January 2016')); // Switch to year mode.
      await tester.pumpAndSettle();
      await tester.tap(find.text('2018'));
      await tester.pumpAndSettle();
      expect(find.text('January 2018'), findsOneWidget);
      expect(displayedMonth, equals(DateTime(2018)));
    });

    testWidgets('Selecting date does not change displayed month', (WidgetTester tester) async {
      DateTime? selectedDate;
      DateTime? displayedMonth;
      await tester.pumpWidget(
        calendarDatePicker(
          initialDate: DateTime(2020, DateTime.march, 15),
          onDateChanged: (DateTime date) => selectedDate = date,
          onDisplayedMonthChanged: (DateTime date) => displayedMonth = date,
        ),
      );

      await tester.tap(nextMonthIcon);
      await tester.pumpAndSettle();
      expect(find.text('April 2020'), findsOneWidget);
      expect(displayedMonth, equals(DateTime(2020, DateTime.april)));

      await tester.tap(find.text('25'));
      await tester.pumpAndSettle();
      expect(find.text('April 2020'), findsOneWidget);
      expect(displayedMonth, equals(DateTime(2020, DateTime.april)));
      expect(selectedDate, equals(DateTime(2020, DateTime.april, 25)));
      // There isn't a 31 in April so there shouldn't be one if it is showing April.
      expect(find.text('31'), findsNothing);
    });

    testWidgets('Changing year does change selected date', (WidgetTester tester) async {
      DateTime? selectedDate;
      await tester.pumpWidget(
        calendarDatePicker(
          initialDate: DateTime(2016, DateTime.january, 15),
          onDateChanged: (DateTime date) => selectedDate = date,
        ),
      );
      await tester.tap(find.text('4'));
      expect(selectedDate, equals(DateTime(2016, DateTime.january, 4)));
      await tester.tap(find.text('January 2016'));
      await tester.pumpAndSettle();
      await tester.tap(find.text('2018'));
      await tester.pumpAndSettle();
      expect(selectedDate, equals(DateTime(2018, DateTime.january, 4)));
    });

    testWidgets('Changing year for february 29th', (WidgetTester tester) async {
      DateTime? selectedDate;
      await tester.pumpWidget(
        calendarDatePicker(
          initialDate: DateTime(2020, DateTime.february, 29),
          onDateChanged: (DateTime date) => selectedDate = date,
        ),
      );
      await tester.tap(find.text('February 2020'));
      await tester.pumpAndSettle();
      await tester.tap(find.text('2018'));
      await tester.pumpAndSettle();
      expect(selectedDate, equals(DateTime(2018, DateTime.february, 28)));
      await tester.tap(find.text('February 2018'));
      await tester.pumpAndSettle();
      await tester.tap(find.text('2020'));
      await tester.pumpAndSettle();
      // Changing back to 2020 the 29th is not selected anymore.
      expect(selectedDate, equals(DateTime(2020, DateTime.february, 28)));
    });

    testWidgets('Changing year does not change the month', (WidgetTester tester) async {
      DateTime? displayedMonth;
      await tester.pumpWidget(
        calendarDatePicker(
          initialDate: DateTime(2016, DateTime.january, 15),
          onDisplayedMonthChanged: (DateTime date) => displayedMonth = date,
        ),
      );
      await tester.tap(nextMonthIcon);
      await tester.pumpAndSettle();
      await tester.tap(nextMonthIcon);
      await tester.pumpAndSettle();
      await tester.tap(find.text('March 2016'));
      await tester.pumpAndSettle();
      await tester.tap(find.text('2018'));
      await tester.pumpAndSettle();
      expect(find.text('March 2018'), findsOneWidget);
      expect(displayedMonth, equals(DateTime(2018, DateTime.march)));
    });

    testWidgets('Can select a year and then a day', (WidgetTester tester) async {
      DateTime? selectedDate;
      await tester.pumpWidget(
        calendarDatePicker(
          initialDate: DateTime(2016, DateTime.january, 15),
          onDateChanged: (DateTime date) => selectedDate = date,
        ),
      );
      await tester.tap(find.text('January 2016')); // Switch to year mode.
      await tester.pumpAndSettle();
      await tester.tap(find.text('2017'));
      await tester.pumpAndSettle();
      await tester.tap(find.text('19'));
      expect(selectedDate, equals(DateTime(2017, DateTime.january, 19)));
    });

    testWidgets('Cannot select a day outside bounds', (WidgetTester tester) async {
      final DateTime validDate = DateTime(2017, DateTime.january, 15);
      DateTime? selectedDate;
      await tester.pumpWidget(
        calendarDatePicker(
          initialDate: validDate,
          firstDate: validDate,
          lastDate: validDate,
          onDateChanged: (DateTime date) => selectedDate = date,
        ),
      );

      // Earlier than firstDate. Should be ignored.
      await tester.tap(find.text('10'));
      expect(selectedDate, isNull);

      // Later than lastDate. Should be ignored.
      await tester.tap(find.text('20'));
      expect(selectedDate, isNull);

      // This one is just right.
      await tester.tap(find.text('15'));
      expect(selectedDate, validDate);
    });

    testWidgets('Cannot navigate to a month outside bounds', (WidgetTester tester) async {
      DateTime? displayedMonth;
      await tester.pumpWidget(
        calendarDatePicker(
          firstDate: DateTime(2016, DateTime.december, 15),
          initialDate: DateTime(2017, DateTime.january, 15),
          lastDate: DateTime(2017, DateTime.february, 15),
          onDisplayedMonthChanged: (DateTime date) => displayedMonth = date,
        ),
      );

      await tester.tap(nextMonthIcon);
      await tester.pumpAndSettle();
      expect(displayedMonth, equals(DateTime(2017, DateTime.february)));
      // Shouldn't be possible to keep going forward into March.
      expect(nextMonthIcon, findsNothing);

      await tester.tap(previousMonthIcon);
      await tester.pumpAndSettle();
      await tester.tap(previousMonthIcon);
      await tester.pumpAndSettle();
      expect(displayedMonth, equals(DateTime(2016, DateTime.december)));
      // Shouldn't be possible to keep going backward into November.
      expect(previousMonthIcon, findsNothing);
    });

    testWidgets('Cannot select disabled year', (WidgetTester tester) async {
      DateTime? displayedMonth;
      await tester.pumpWidget(
        calendarDatePicker(
          firstDate: DateTime(2018, DateTime.june, 9),
          initialDate: DateTime(2018, DateTime.july, 4),
          lastDate: DateTime(2018, DateTime.december, 15),
          onDisplayedMonthChanged: (DateTime date) => displayedMonth = date,
        ),
      );
      await tester.tap(find.text('July 2018')); // Switch to year mode.
      await tester.pumpAndSettle();
      await tester.tap(find.text('2016')); // Disabled, doesn't change the year.
      await tester.pumpAndSettle();
      await tester.tap(find.text('2020')); // Disabled, doesn't change the year.
      await tester.pumpAndSettle();

      await tester.tap(find.text('2018'));
      await tester.pumpAndSettle();
      // Nothing should have changed.
      expect(displayedMonth, isNull);
    });

    testWidgets('Selecting firstDate year respects firstDate', (WidgetTester tester) async {
      DateTime? selectedDate;
      DateTime? displayedMonth;
      await tester.pumpWidget(
        calendarDatePicker(
          firstDate: DateTime(2016, DateTime.june, 9),
          initialDate: DateTime(2018, DateTime.may, 4),
          lastDate: DateTime(2019, DateTime.january, 15),
          onDateChanged: (DateTime date) => selectedDate = date,
          onDisplayedMonthChanged: (DateTime date) => displayedMonth = date,
        ),
      );
      await tester.tap(find.text('May 2018'));
      await tester.pumpAndSettle();
      await tester.tap(find.text('2016'));
      await tester.pumpAndSettle();
      // Month should be clamped to June as the range starts at June 2016.
      expect(find.text('June 2016'), findsOneWidget);
      expect(displayedMonth, DateTime(2016, DateTime.june));
      expect(selectedDate, DateTime(2016, DateTime.june, 9));
    });

    testWidgets('Selecting lastDate year respects lastDate', (WidgetTester tester) async {
      DateTime? selectedDate;
      DateTime? displayedMonth;
      await tester.pumpWidget(
        calendarDatePicker(
          firstDate: DateTime(2016, DateTime.june, 9),
          initialDate: DateTime(2018, DateTime.may, 4),
          lastDate: DateTime(2019, DateTime.january, 15),
          onDateChanged: (DateTime date) => selectedDate = date,
          onDisplayedMonthChanged: (DateTime date) => displayedMonth = date,
        ),
      );
      // Selected date is now 2018-05-04 (initialDate).
      await tester.tap(find.text('May 2018'));
      // Selected date is still 2018-05-04.
      await tester.pumpAndSettle();
      await tester.tap(find.text('2019'));
      // Selected date would become 2019-05-04 but gets clamped to the month of lastDate, so 2019-01-04.
      await tester.pumpAndSettle();
      expect(find.text('January 2019'), findsOneWidget);
      expect(displayedMonth, DateTime(2019));
      expect(selectedDate, DateTime(2019, DateTime.january, 4));
    });

    testWidgets('Selecting lastDate year respects lastDate', (WidgetTester tester) async {
      DateTime? selectedDate;
      DateTime? displayedMonth;
      await tester.pumpWidget(
        calendarDatePicker(
          firstDate: DateTime(2016, DateTime.june, 9),
          initialDate: DateTime(2018, DateTime.may, 15),
          lastDate: DateTime(2019, DateTime.january, 4),
          onDateChanged: (DateTime date) => selectedDate = date,
          onDisplayedMonthChanged: (DateTime date) => displayedMonth = date,
        ),
      );
      // Selected date is now 2018-05-15 (initialDate).
      await tester.tap(find.text('May 2018'));
      // Selected date is still 2018-05-15.
      await tester.pumpAndSettle();
      await tester.tap(find.text('2019'));
      // Selected date would become 2019-05-15 but gets clamped to the month of lastDate, so 2019-01-15.
      // Day is now beyond the lastDate so that also gets clamped, to 2019-01-04.
      await tester.pumpAndSettle();
      expect(find.text('January 2019'), findsOneWidget);
      expect(displayedMonth, DateTime(2019));
      expect(selectedDate, DateTime(2019, DateTime.january, 4));
    });

    testWidgets('Only predicate days are selectable', (WidgetTester tester) async {
      DateTime? selectedDate;
      await tester.pumpWidget(
        calendarDatePicker(
          firstDate: DateTime(2017, DateTime.january, 10),
          initialDate: DateTime(2017, DateTime.january, 16),
          lastDate: DateTime(2017, DateTime.january, 20),
          onDateChanged: (DateTime date) => selectedDate = date,
          selectableDayPredicate: (DateTime date) => date.day.isEven,
        ),
      );
      await tester.tap(find.text('13')); // Odd, doesn't work.
      expect(selectedDate, isNull);
      await tester.tap(find.text('10')); // Even, works.
      expect(selectedDate, DateTime(2017, DateTime.january, 10));
      await tester.tap(find.text('17')); // Odd, doesn't work.
      expect(selectedDate, DateTime(2017, DateTime.january, 10));
    });

    testWidgets('Can select initial calendar picker mode', (WidgetTester tester) async {
      await tester.pumpWidget(
        calendarDatePicker(
          initialDate: DateTime(2014, DateTime.january, 15),
          initialCalendarMode: DatePickerMode.year,
        ),
      );
      // 2018 wouldn't be available if the year picker wasn't showing.
      // The initial current year is 2014.
      await tester.tap(find.text('2018'));
      await tester.pumpAndSettle();
      expect(find.text('January 2018'), findsOneWidget);
    });

    testWidgets('Material2 - currentDate is highlighted', (WidgetTester tester) async {
      await tester.pumpWidget(
        calendarDatePicker(
          useMaterial3: false,
          initialDate: DateTime(2016, DateTime.january, 15),
          currentDate: DateTime(2016, 1, 2),
        ),
      );
      const Color todayColor = Color(0xff2196f3); // default primary color
      expect(
        Material.of(tester.element(find.text('2'))),
        // The current day should be painted with a circle outline.
        paints..circle(color: todayColor, style: PaintingStyle.stroke, strokeWidth: 1.0),
      );
    });

    testWidgets('Material3 - currentDate is highlighted', (WidgetTester tester) async {
      await tester.pumpWidget(
        calendarDatePicker(
          useMaterial3: true,
          initialDate: DateTime(2016, DateTime.january, 15),
          currentDate: DateTime(2016, 1, 2),
        ),
      );
      const Color todayColor = Color(0xff6750a4); // default primary color
      expect(
        Material.of(tester.element(find.text('2'))),
        // The current day should be painted with a circle outline.
        paints..circle(color: todayColor, style: PaintingStyle.stroke, strokeWidth: 1.0),
      );
    });

    testWidgets('Material2 - currentDate is highlighted even if it is disabled', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        calendarDatePicker(
          useMaterial3: false,
          firstDate: DateTime(2016, 1, 3),
          lastDate: DateTime(2016, 1, 31),
          currentDate: DateTime(2016, 1, 2), // not between first and last
          initialDate: DateTime(2016, 1, 5),
        ),
      );
      const Color disabledColor = Color(0x61000000); // default disabled color
      expect(
        Material.of(tester.element(find.text('2'))),
        // The current day should be painted with a circle outline.
        paints..circle(color: disabledColor, style: PaintingStyle.stroke, strokeWidth: 1.0),
      );
    });

    testWidgets('Material3 - currentDate is highlighted even if it is disabled', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        calendarDatePicker(
          useMaterial3: true,
          firstDate: DateTime(2016, 1, 3),
          lastDate: DateTime(2016, 1, 31),
          currentDate: DateTime(2016, 1, 2), // not between first and last
          initialDate: DateTime(2016, 1, 5),
        ),
      );
      const Color disabledColor = Color(0x616750a4); // default disabled color
      expect(
        Material.of(tester.element(find.text('2'))),
        // The current day should be painted with a circle outline.
        paints..circle(color: disabledColor, style: PaintingStyle.stroke, strokeWidth: 1.0),
      );
    });

    testWidgets('Selecting date does not switch picker to year selection', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        calendarDatePicker(
          initialDate: DateTime(2020, DateTime.may, 10),
          initialCalendarMode: DatePickerMode.year,
        ),
      );
      await tester.tap(find.text('2017'));
      await tester.pumpAndSettle();
      expect(find.text('May 2017'), findsOneWidget);
      await tester.tap(find.text('10'));
      await tester.pumpAndSettle();
      expect(find.text('May 2017'), findsOneWidget);
      expect(find.text('2017'), findsNothing);
    });

    testWidgets('Selecting disabled date does not change current selection', (
      WidgetTester tester,
    ) async {
      DateTime day(int day) => DateTime(2020, DateTime.may, day);

      DateTime selection = day(2);
      await tester.pumpWidget(
        calendarDatePicker(
          initialDate: selection,
          firstDate: day(2),
          lastDate: day(3),
          onDateChanged: (DateTime date) {
            selection = date;
          },
        ),
      );

      await tester.tap(find.text('3'));
      await tester.pumpAndSettle();
      expect(selection, day(3));
      await tester.tap(find.text('4'));
      await tester.pumpAndSettle();
      expect(selection, day(3));
      await tester.tap(find.text('5'));
      await tester.pumpAndSettle();
      expect(selection, day(3));
    });

    for (final bool useMaterial3 in <bool>[false, true]) {
      testWidgets(
        'Updates to initialDate parameter are not reflected in the state (useMaterial3=$useMaterial3)',
        (WidgetTester tester) async {
          final Key pickerKey = UniqueKey();
          final DateTime initialDate = DateTime(2020, 1, 21);
          final DateTime updatedDate = DateTime(1976, 2, 23);
          final DateTime firstDate = DateTime(1970);
          final DateTime lastDate = DateTime(2099, 31, 12);
          final Color selectedColor =
              useMaterial3
                  ? const Color(0xff6750a4)
                  : const Color(0xff2196f3); // default primary color

          await tester.pumpWidget(
            calendarDatePicker(
              key: pickerKey,
              useMaterial3: useMaterial3,
              initialDate: initialDate,
              firstDate: firstDate,
              lastDate: lastDate,
              onDateChanged: (DateTime value) {},
            ),
          );
          await tester.pumpAndSettle();

          // Month should show as January 2020.
          expect(find.text('January 2020'), findsOneWidget);
          // Selected date should be painted with a colored circle.
          expect(
            Material.of(tester.element(find.text('21'))),
            paints..circle(color: selectedColor, style: PaintingStyle.fill),
          );

          // Change to the updated initialDate.
          // This should have no effect, the initialDate is only the _initial_ date.
          await tester.pumpWidget(
            calendarDatePicker(
              key: pickerKey,
              useMaterial3: useMaterial3,
              initialDate: updatedDate,
              firstDate: firstDate,
              lastDate: lastDate,
              onDateChanged: (DateTime value) {},
            ),
          );
          // Wait for the page scroll animation to finish.
          await tester.pumpAndSettle(const Duration(milliseconds: 200));

          // Month should show as January 2020 still.
          expect(find.text('January 2020'), findsOneWidget);
          expect(find.text('February 1976'), findsNothing);
          // Selected date should be painted with a colored circle.
          expect(
            Material.of(tester.element(find.text('21'))),
            paints..circle(color: selectedColor, style: PaintingStyle.fill),
          );
        },
      );
    }

    testWidgets('Updates to initialCalendarMode parameter is not reflected in the state', (
      WidgetTester tester,
    ) async {
      final Key pickerKey = UniqueKey();

      await tester.pumpWidget(
        calendarDatePicker(
          key: pickerKey,
          initialDate: DateTime(2016, DateTime.january, 15),
          initialCalendarMode: DatePickerMode.year,
        ),
      );
      await tester.pumpAndSettle();

      // Should be in year mode.
      expect(find.text('January 2016'), findsOneWidget); // Day/year selector
      expect(find.text('15'), findsNothing); // day 15 in grid
      expect(find.text('2016'), findsOneWidget); // 2016 in year grid

      await tester.pumpWidget(
        calendarDatePicker(key: pickerKey, initialDate: DateTime(2016, DateTime.january, 15)),
      );
      await tester.pumpAndSettle();

      // Should be in year mode still; updating an _initial_ parameter has no effect.
      expect(find.text('January 2016'), findsOneWidget); // Day/year selector
      expect(find.text('15'), findsNothing); // day 15 in grid
      expect(find.text('2016'), findsOneWidget); // 2016 in year grid
    });

    testWidgets('Dragging more than half the width should not cause a jump', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        calendarDatePicker(initialDate: DateTime(2016, DateTime.january, 15)),
      );
      await tester.pumpAndSettle();
      final TestGesture gesture = await tester.startGesture(
        tester.getCenter(find.byType(PageView)),
      );
      // This initial drag is required for the PageView to recognize the gesture, as it uses DragStartBehavior.start.
      // It does not count towards the drag distance.
      await gesture.moveBy(const Offset(100, 0));
      // Dragging for a bit less than half the width should reveal the previous month.
      await gesture.moveBy(const Offset(800 / 2 - 1, 0));
      await tester.pumpAndSettle();
      expect(find.text('January 2016'), findsOneWidget);
      expect(find.text('1'), findsNWidgets(2));
      // Dragging a bit over the half should still show both.
      await gesture.moveBy(const Offset(2, 0));
      await tester.pumpAndSettle();
      expect(find.text('December 2015'), findsOneWidget);
      expect(find.text('1'), findsNWidgets(2));
    });

    group('Keyboard navigation', () {
      testWidgets('Can toggle to year mode', (WidgetTester tester) async {
        await tester.pumpWidget(
          calendarDatePicker(initialDate: DateTime(2016, DateTime.january, 15)),
        );
        expect(find.text('2016'), findsNothing);
        expect(find.text('January 2016'), findsOneWidget);
        // Navigate to the year selector and activate it.
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.sendKeyEvent(LogicalKeyboardKey.space);
        await tester.pumpAndSettle();
        // The years should be visible.
        expect(find.text('2016'), findsOneWidget);
        expect(find.text('January 2016'), findsOneWidget);
      });

      testWidgets('Can navigate next/previous months', (WidgetTester tester) async {
        await tester.pumpWidget(
          calendarDatePicker(initialDate: DateTime(2016, DateTime.january, 15)),
        );
        expect(find.text('January 2016'), findsOneWidget);
        // Navigate to the previous month button and activate it twice.
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.sendKeyEvent(LogicalKeyboardKey.space);
        await tester.pumpAndSettle();
        await tester.sendKeyEvent(LogicalKeyboardKey.space);
        await tester.pumpAndSettle();
        // Should be showing Nov 2015
        expect(find.text('November 2015'), findsOneWidget);

        // Navigate to the next month button and activate it four times.
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.sendKeyEvent(LogicalKeyboardKey.space);
        await tester.pumpAndSettle();
        await tester.sendKeyEvent(LogicalKeyboardKey.space);
        await tester.pumpAndSettle();
        await tester.sendKeyEvent(LogicalKeyboardKey.space);
        await tester.pumpAndSettle();
        await tester.sendKeyEvent(LogicalKeyboardKey.space);
        await tester.pumpAndSettle();
        // Should be on Mar 2016.
        expect(find.text('March 2016'), findsOneWidget);
      });

      testWidgets('Can navigate date grid with arrow keys', (WidgetTester tester) async {
        DateTime? selectedDate;
        await tester.pumpWidget(
          calendarDatePicker(
            initialDate: DateTime(2016, DateTime.january, 15),
            onDateChanged: (DateTime date) => selectedDate = date,
          ),
        );
        // Navigate to the grid.
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);

        // Navigate from Jan 15 to Jan 18 with arrow keys.
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
        await tester.pumpAndSettle();

        // Activate it.
        await tester.sendKeyEvent(LogicalKeyboardKey.space);
        await tester.pumpAndSettle();

        // Should have selected Jan 18.
        expect(selectedDate, DateTime(2016, DateTime.january, 18));
      });

      testWidgets('Navigating with arrow keys scrolls months', (WidgetTester tester) async {
        DateTime? selectedDate;
        await tester.pumpWidget(
          calendarDatePicker(
            initialDate: DateTime(2016, DateTime.january, 15),
            onDateChanged: (DateTime date) => selectedDate = date,
          ),
        );
        // Navigate to the grid.
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.pumpAndSettle();

        // Navigate from Jan 15 to Dec 31 with arrow keys
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
        await tester.pumpAndSettle();

        // Should have scrolled to Dec 2015.
        expect(find.text('December 2015'), findsOneWidget);

        // Navigate from Dec 31 to Nov 26 with arrow keys.
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
        await tester.pumpAndSettle();

        // Should have scrolled to Nov 2015.
        expect(find.text('November 2015'), findsOneWidget);

        // Activate it
        await tester.sendKeyEvent(LogicalKeyboardKey.space);
        await tester.pumpAndSettle();

        // Should have selected Jan 18.
        expect(selectedDate, DateTime(2015, DateTime.november, 26));
      });

      testWidgets('RTL text direction reverses the horizontal arrow key navigation', (
        WidgetTester tester,
      ) async {
        DateTime? selectedDate;
        await tester.pumpWidget(
          calendarDatePicker(
            initialDate: DateTime(2016, DateTime.january, 15),
            onDateChanged: (DateTime date) => selectedDate = date,
            textDirection: TextDirection.rtl,
          ),
        );
        // Navigate to the grid.
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.pumpAndSettle();

        // Navigate from Jan 15 to 19 with arrow keys.
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
        await tester.pumpAndSettle();

        // Activate it.
        await tester.sendKeyEvent(LogicalKeyboardKey.space);
        await tester.pumpAndSettle();

        // Should have selected Jan 19.
        expect(selectedDate, DateTime(2016, DateTime.january, 19));
      });
    });

    group('Haptic feedback', () {
      const Duration hapticFeedbackInterval = Duration(milliseconds: 10);
      late FeedbackTester feedback;

      setUp(() {
        feedback = FeedbackTester();
      });

      tearDown(() {
        feedback.dispose();
      });

      testWidgets('Selecting date vibrates', (WidgetTester tester) async {
        await tester.pumpWidget(
          calendarDatePicker(initialDate: DateTime(2016, DateTime.january, 15)),
        );
        await tester.tap(find.text('10'));
        await tester.pump(hapticFeedbackInterval);
        expect(feedback.hapticCount, 1);
        await tester.tap(find.text('12'));
        await tester.pump(hapticFeedbackInterval);
        expect(feedback.hapticCount, 2);
        await tester.tap(find.text('14'));
        await tester.pump(hapticFeedbackInterval);
        expect(feedback.hapticCount, 3);
      });

      testWidgets('Tapping unselectable date does not vibrate', (WidgetTester tester) async {
        await tester.pumpWidget(
          calendarDatePicker(
            initialDate: DateTime(2016, DateTime.january, 10),
            selectableDayPredicate: (DateTime date) => date.day.isEven,
          ),
        );
        await tester.tap(find.text('11'));
        await tester.pump(hapticFeedbackInterval);
        expect(feedback.hapticCount, 0);
        await tester.tap(find.text('13'));
        await tester.pump(hapticFeedbackInterval);
        expect(feedback.hapticCount, 0);
        await tester.tap(find.text('15'));
        await tester.pump(hapticFeedbackInterval);
        expect(feedback.hapticCount, 0);
      });

      testWidgets('Changing modes and year vibrates', (WidgetTester tester) async {
        await tester.pumpWidget(
          calendarDatePicker(initialDate: DateTime(2016, DateTime.january, 15)),
        );
        await tester.tap(find.text('January 2016'));
        await tester.pump(hapticFeedbackInterval);
        expect(feedback.hapticCount, 1);
        await tester.tap(find.text('2018'));
        await tester.pump(hapticFeedbackInterval);
        expect(feedback.hapticCount, 2);
      });
    });

    group('Semantics', () {
      testWidgets('day mode', (WidgetTester tester) async {
        final SemanticsHandle semantics = tester.ensureSemantics();

        await tester.pumpWidget(
          calendarDatePicker(initialDate: DateTime(2016, DateTime.january, 15)),
        );

        // Year mode drop down button.
        expect(
          tester.getSemantics(find.text('January 2016')),
          matchesSemantics(
            label: 'Select year\nJanuary 2016',
            isButton: true,
            hasTapAction: true,
            hasFocusAction: true,
            isFocusable: true,
          ),
        );

        // Prev/Next month buttons.
        expect(
          tester.getSemantics(previousMonthIcon),
          matchesSemantics(
            tooltip: 'Previous month',
            isButton: true,
            hasTapAction: true,
            hasFocusAction: true,
            isEnabled: true,
            hasEnabledState: true,
            isFocusable: true,
          ),
        );
        expect(
          tester.getSemantics(nextMonthIcon),
          matchesSemantics(
            tooltip: 'Next month',
            isButton: true,
            hasTapAction: true,
            hasFocusAction: true,
            isEnabled: true,
            hasEnabledState: true,
            isFocusable: true,
          ),
        );

        // Day grid.
        expect(
          tester.getSemantics(find.text('1')),
          matchesSemantics(
            label: '1, Friday, January 1, 2016',
            isButton: true,
            hasTapAction: true,
            hasFocusAction: true,
            isFocusable: true,
          ),
        );
        expect(
          tester.getSemantics(find.text('2')),
          matchesSemantics(
            label: '2, Saturday, January 2, 2016',
            isButton: true,
            hasTapAction: true,
            hasFocusAction: true,
            isFocusable: true,
          ),
        );
        expect(
          tester.getSemantics(find.text('3')),
          matchesSemantics(
            label: '3, Sunday, January 3, 2016, Today',
            isButton: true,
            hasTapAction: true,
            hasFocusAction: true,
            isFocusable: true,
          ),
        );
        expect(
          tester.getSemantics(find.text('4')),
          matchesSemantics(
            label: '4, Monday, January 4, 2016',
            isButton: true,
            hasTapAction: true,
            hasFocusAction: true,
            isFocusable: true,
          ),
        );
        expect(
          tester.getSemantics(find.text('5')),
          matchesSemantics(
            label: '5, Tuesday, January 5, 2016',
            isButton: true,
            hasTapAction: true,
            hasFocusAction: true,
            isFocusable: true,
          ),
        );
        expect(
          tester.getSemantics(find.text('6')),
          matchesSemantics(
            label: '6, Wednesday, January 6, 2016',
            isButton: true,
            hasTapAction: true,
            hasFocusAction: true,
            isFocusable: true,
          ),
        );
        expect(
          tester.getSemantics(find.text('7')),
          matchesSemantics(
            label: '7, Thursday, January 7, 2016',
            isButton: true,
            hasTapAction: true,
            hasFocusAction: true,
            isFocusable: true,
          ),
        );
        expect(
          tester.getSemantics(find.text('8')),
          matchesSemantics(
            label: '8, Friday, January 8, 2016',
            isButton: true,
            hasTapAction: true,
            hasFocusAction: true,
            isFocusable: true,
          ),
        );
        expect(
          tester.getSemantics(find.text('9')),
          matchesSemantics(
            label: '9, Saturday, January 9, 2016',
            isButton: true,
            hasTapAction: true,
            hasFocusAction: true,
            isFocusable: true,
          ),
        );
        expect(
          tester.getSemantics(find.text('10')),
          matchesSemantics(
            label: '10, Sunday, January 10, 2016',
            isButton: true,
            hasTapAction: true,
            hasFocusAction: true,
            isFocusable: true,
          ),
        );
        expect(
          tester.getSemantics(find.text('11')),
          matchesSemantics(
            label: '11, Monday, January 11, 2016',
            isButton: true,
            hasTapAction: true,
            hasFocusAction: true,
            isFocusable: true,
          ),
        );
        expect(
          tester.getSemantics(find.text('12')),
          matchesSemantics(
            label: '12, Tuesday, January 12, 2016',
            isButton: true,
            hasTapAction: true,
            hasFocusAction: true,
            isFocusable: true,
          ),
        );
        expect(
          tester.getSemantics(find.text('13')),
          matchesSemantics(
            label: '13, Wednesday, January 13, 2016',
            isButton: true,
            hasTapAction: true,
            hasFocusAction: true,
            isFocusable: true,
          ),
        );
        expect(
          tester.getSemantics(find.text('14')),
          matchesSemantics(
            label: '14, Thursday, January 14, 2016',
            isButton: true,
            hasTapAction: true,
            hasFocusAction: true,
            isFocusable: true,
          ),
        );
        expect(
          tester.getSemantics(find.text('15')),
          matchesSemantics(
            label: '15, Friday, January 15, 2016',
            isButton: true,
            hasTapAction: true,
            hasFocusAction: true,
            isSelected: true,
            isFocusable: true,
          ),
        );
        expect(
          tester.getSemantics(find.text('16')),
          matchesSemantics(
            label: '16, Saturday, January 16, 2016',
            isButton: true,
            hasTapAction: true,
            hasFocusAction: true,
            isFocusable: true,
          ),
        );
        expect(
          tester.getSemantics(find.text('17')),
          matchesSemantics(
            label: '17, Sunday, January 17, 2016',
            isButton: true,
            hasTapAction: true,
            hasFocusAction: true,
            isFocusable: true,
          ),
        );
        expect(
          tester.getSemantics(find.text('18')),
          matchesSemantics(
            label: '18, Monday, January 18, 2016',
            isButton: true,
            hasTapAction: true,
            hasFocusAction: true,
            isFocusable: true,
          ),
        );
        expect(
          tester.getSemantics(find.text('19')),
          matchesSemantics(
            label: '19, Tuesday, January 19, 2016',
            isButton: true,
            hasTapAction: true,
            hasFocusAction: true,
            isFocusable: true,
          ),
        );
        expect(
          tester.getSemantics(find.text('20')),
          matchesSemantics(
            label: '20, Wednesday, January 20, 2016',
            isButton: true,
            hasTapAction: true,
            hasFocusAction: true,
            isFocusable: true,
          ),
        );
        expect(
          tester.getSemantics(find.text('21')),
          matchesSemantics(
            label: '21, Thursday, January 21, 2016',
            isButton: true,
            hasTapAction: true,
            hasFocusAction: true,
            isFocusable: true,
          ),
        );
        expect(
          tester.getSemantics(find.text('22')),
          matchesSemantics(
            label: '22, Friday, January 22, 2016',
            isButton: true,
            hasTapAction: true,
            hasFocusAction: true,
            isFocusable: true,
          ),
        );
        expect(
          tester.getSemantics(find.text('23')),
          matchesSemantics(
            label: '23, Saturday, January 23, 2016',
            isButton: true,
            hasTapAction: true,
            hasFocusAction: true,
            isFocusable: true,
          ),
        );
        expect(
          tester.getSemantics(find.text('24')),
          matchesSemantics(
            label: '24, Sunday, January 24, 2016',
            isButton: true,
            hasTapAction: true,
            hasFocusAction: true,
            isFocusable: true,
          ),
        );
        expect(
          tester.getSemantics(find.text('25')),
          matchesSemantics(
            label: '25, Monday, January 25, 2016',
            isButton: true,
            hasTapAction: true,
            hasFocusAction: true,
            isFocusable: true,
          ),
        );
        expect(
          tester.getSemantics(find.text('26')),
          matchesSemantics(
            label: '26, Tuesday, January 26, 2016',
            isButton: true,
            hasTapAction: true,
            hasFocusAction: true,
            isFocusable: true,
          ),
        );
        expect(
          tester.getSemantics(find.text('27')),
          matchesSemantics(
            label: '27, Wednesday, January 27, 2016',
            isButton: true,
            hasTapAction: true,
            hasFocusAction: true,
            isFocusable: true,
          ),
        );
        expect(
          tester.getSemantics(find.text('28')),
          matchesSemantics(
            label: '28, Thursday, January 28, 2016',
            isButton: true,
            hasTapAction: true,
            hasFocusAction: true,
            isFocusable: true,
          ),
        );
        expect(
          tester.getSemantics(find.text('29')),
          matchesSemantics(
            label: '29, Friday, January 29, 2016',
            isButton: true,
            hasTapAction: true,
            hasFocusAction: true,
            isFocusable: true,
          ),
        );
        expect(
          tester.getSemantics(find.text('30')),
          matchesSemantics(
            label: '30, Saturday, January 30, 2016',
            isButton: true,
            hasTapAction: true,
            hasFocusAction: true,
            isFocusable: true,
          ),
        );
        semantics.dispose();
      });

      testWidgets('calendar year mode', (WidgetTester tester) async {
        final SemanticsHandle semantics = tester.ensureSemantics();

        await tester.pumpWidget(
          calendarDatePicker(
            initialDate: DateTime(2016, DateTime.january, 15),
            initialCalendarMode: DatePickerMode.year,
          ),
        );

        // Year mode drop down button.
        expect(
          tester.getSemantics(find.text('January 2016')),
          matchesSemantics(
            label: 'Select year\nJanuary 2016',
            isButton: true,
            hasTapAction: true,
            hasFocusAction: true,
            isFocusable: true,
          ),
        );

        // Year grid only shows 2010 - 2024.
        for (int year = 2010; year <= 2024; year++) {
          expect(
            tester.getSemantics(find.text('$year')),
            matchesSemantics(
              label: '$year',
              hasTapAction: true,
              hasFocusAction: true,
              isSelected: year == 2016,
              isFocusable: true,
              isButton: true,
            ),
          );
        }
        semantics.dispose();
      });

      // This is a regression test for https://github.com/flutter/flutter/issues/143439.
      testWidgets(
        'Selected date Semantics announcement on onDateChanged',
        (WidgetTester tester) async {
          final SemanticsHandle semantics = tester.ensureSemantics();
          const DefaultMaterialLocalizations localizations = DefaultMaterialLocalizations();
          final DateTime initialDate = DateTime(2016, DateTime.january, 15);
          DateTime? selectedDate;

          await tester.pumpWidget(
            calendarDatePicker(
              initialDate: initialDate,
              onDateChanged: (DateTime value) {
                selectedDate = value;
              },
            ),
          );

          final bool isToday = DateUtils.isSameDay(initialDate, selectedDate);
          final String semanticLabelSuffix = isToday ? ', ${localizations.currentDateLabel}' : '';

          // The initial date should be announced.
          expect(
            tester.takeAnnouncements().last.message,
            '${localizations.formatFullDate(initialDate)}$semanticLabelSuffix',
          );

          // Select a new date.
          await tester.tap(find.text('20'));
          await tester.pumpAndSettle();

          // The selected date should be announced.
          expect(
            tester.takeAnnouncements().last.message,
            '${localizations.selectedDateLabel} ${localizations.formatFullDate(selectedDate!)}$semanticLabelSuffix',
          );

          // Select the initial date.
          await tester.tap(find.text('15'));

          // The initial date should be announced as selected.
          expect(
            tester.takeAnnouncements().first.message,
            '${localizations.selectedDateLabel} ${localizations.formatFullDate(initialDate)}$semanticLabelSuffix',
          );

          semantics.dispose();
        },
        variant: TargetPlatformVariant.desktop(),
      );
    });

    // This is a regression test for https://github.com/flutter/flutter/issues/141350.
    testWidgets('Default day selection overlay', (WidgetTester tester) async {
      final ThemeData theme = ThemeData();
      await tester.pumpWidget(
        calendarDatePicker(
          firstDate: DateTime(2016, DateTime.december, 15),
          initialDate: DateTime(2017, DateTime.january, 15),
          lastDate: DateTime(2017, DateTime.february, 15),
          onDisplayedMonthChanged: (DateTime date) {},
          theme: theme,
        ),
      );

      RenderObject inkFeatures = tester.allRenderObjects.firstWhere(
        (RenderObject object) => object.runtimeType.toString() == '_RenderInkFeatures',
      );
      expect(
        inkFeatures,
        isNot(
          paints..circle(radius: 35.0, color: theme.colorScheme.onSurfaceVariant.withOpacity(0.08)),
        ),
      );
      expect(inkFeatures, paintsExactlyCountTimes(#clipPath, 0));

      final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
      await gesture.addPointer();
      await gesture.moveTo(tester.getCenter(find.text('25')));
      await tester.pumpAndSettle();
      inkFeatures = tester.allRenderObjects.firstWhere(
        (RenderObject object) => object.runtimeType.toString() == '_RenderInkFeatures',
      );
      expect(
        inkFeatures,
        paints
          ..circle()
          ..circle(radius: 35.0, color: theme.colorScheme.onSurfaceVariant.withOpacity(0.08)),
      );
      expect(inkFeatures, paintsExactlyCountTimes(#clipPath, 1));

      final Rect expectedClipRect = Rect.fromCircle(
        center: const Offset(400.0, 241.0),
        radius: 35.0,
      );
      final Path expectedClipPath = Path()..addRect(expectedClipRect);
      expect(
        inkFeatures,
        paints..clipPath(
          pathMatcher: coversSameAreaAs(
            expectedClipPath,
            areaToCompare: expectedClipRect,
            sampleSize: 100,
          ),
        ),
      );
    });
  });

  group('YearPicker', () {
    testWidgets('Current year is visible in year picker', (WidgetTester tester) async {
      await tester.pumpWidget(yearPicker());
      expect(find.text('2016'), findsOneWidget);
    });

    testWidgets('Can select a year', (WidgetTester tester) async {
      DateTime? selectedDate;
      await tester.pumpWidget(yearPicker(onChanged: (DateTime date) => selectedDate = date));
      await tester.pumpAndSettle();
      await tester.tap(find.text('2018'));
      await tester.pumpAndSettle();
      expect(selectedDate, equals(DateTime(2018)));
    });

    testWidgets('Cannot select disabled year', (WidgetTester tester) async {
      DateTime? selectedYear;
      await tester.pumpWidget(
        yearPicker(
          firstDate: DateTime(2018, DateTime.june, 9),
          selectedDate: DateTime(2018, DateTime.july, 4),
          lastDate: DateTime(2018, DateTime.december, 15),
          onChanged: (DateTime date) => selectedYear = date,
        ),
      );
      await tester.tap(find.text('2016')); // Disabled, doesn't change the year.
      await tester.pumpAndSettle();
      expect(selectedYear, isNull);
      await tester.tap(find.text('2020')); // Disabled, doesn't change the year.
      await tester.pumpAndSettle();
      expect(selectedYear, isNull);
      await tester.tap(find.text('2018'));
      await tester.pumpAndSettle();
      expect(selectedYear, equals(DateTime(2018, DateTime.july)));
    });

    testWidgets('Selecting year with no selected month uses earliest month', (
      WidgetTester tester,
    ) async {
      DateTime? selectedYear;
      await tester.pumpWidget(
        yearPicker(
          firstDate: DateTime(2018, DateTime.june, 9),
          lastDate: DateTime(2019, DateTime.december, 15),
          onChanged: (DateTime date) => selectedYear = date,
        ),
      );
      await tester.tap(find.text('2018'));
      expect(selectedYear, equals(DateTime(2018, DateTime.june)));
      await tester.pumpWidget(
        yearPicker(
          firstDate: DateTime(2018, DateTime.june, 9),
          lastDate: DateTime(2019, DateTime.december, 15),
          selectedDate: DateTime(2018, DateTime.june),
          onChanged: (DateTime date) => selectedYear = date,
        ),
      );
      await tester.tap(find.text('2019'));
      expect(selectedYear, equals(DateTime(2019, DateTime.june)));
    });

    testWidgets('Selecting year with no selected month uses January', (WidgetTester tester) async {
      DateTime? selectedYear;
      await tester.pumpWidget(
        yearPicker(
          firstDate: DateTime(2018, DateTime.june, 9),
          lastDate: DateTime(2019, DateTime.december, 15),
          onChanged: (DateTime date) => selectedYear = date,
        ),
      );
      await tester.tap(find.text('2019'));
      expect(selectedYear, equals(DateTime(2019))); // january implied
      await tester.pumpWidget(
        yearPicker(
          firstDate: DateTime(2018, DateTime.june, 9),
          lastDate: DateTime(2019, DateTime.december, 15),
          selectedDate: DateTime(2018),
          onChanged: (DateTime date) => selectedYear = date,
        ),
      );
      await tester.tap(find.text('2018'));
      expect(selectedYear, equals(DateTime(2018, DateTime.june)));
    });
  });
}
