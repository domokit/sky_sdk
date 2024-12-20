// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_localizations/flutter_localizations.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Material2 - can localize the header in all known formats - portrait', (
    WidgetTester tester,
  ) async {
    // Ensure picker is displayed in portrait mode.
    tester.view.physicalSize = const Size(400, 800);
    tester.view.devicePixelRatio = 1;
    addTearDown(tester.view.reset);

    final Finder timeSelectorSeparatorFinder =
        find
            .descendant(
              of: find.byWidgetPredicate(
                (Widget w) => '${w.runtimeType}' == '_TimeSelectorSeparator',
              ),
              matching: find.byType(Text),
            )
            .first;
    final Finder hourControlFinder = find.byWidgetPredicate(
      (Widget w) => '${w.runtimeType}' == '_HourControl',
    );
    final Finder minuteControlFinder = find.byWidgetPredicate(
      (Widget w) => '${w.runtimeType}' == '_MinuteControl',
    );
    final Finder dayPeriodControlFinder = find.byWidgetPredicate(
      (Widget w) => '${w.runtimeType}' == '_DayPeriodControl',
    );

    // TODO(yjbanov): also test `HH.mm` (in_ID), `a h:mm` (ko_KR) and `HH:mm น.` (th_TH) when we have .arb files for them
    final List<Locale> locales = <Locale>[
      const Locale('en', 'US'), //'h:mm a'
      const Locale('en', 'GB'), //'HH:mm'
      const Locale('es', 'ES'), //'H:mm'
      const Locale('fr', 'CA'), //'HH \'h\' mm'
      const Locale('zh', 'ZH'), //'ah:mm'
      const Locale('fa', 'IR'), //'H:mm' but RTL
    ];

    for (final Locale locale in locales) {
      final Offset center = await startPicker(
        tester,
        (TimeOfDay? time) {},
        locale: locale,
        useMaterial3: false,
      );
      final Text stringFragmentText = tester.widget(timeSelectorSeparatorFinder);
      final double hourLeftOffset = tester.getTopLeft(hourControlFinder).dx;
      final double minuteLeftOffset = tester.getTopLeft(minuteControlFinder).dx;
      final double stringFragmentLeftOffset = tester.getTopLeft(timeSelectorSeparatorFinder).dx;

      if (locale == const Locale('en', 'US')) {
        final double dayPeriodLeftOffset = tester.getTopLeft(dayPeriodControlFinder).dx;
        expect(stringFragmentText.data, ':');
        expect(hourLeftOffset, lessThan(stringFragmentLeftOffset));
        expect(stringFragmentLeftOffset, lessThan(minuteLeftOffset));
        expect(minuteLeftOffset, lessThan(dayPeriodLeftOffset));
      } else if (locale == const Locale('en', 'GB')) {
        expect(stringFragmentText.data, ':');
        expect(hourLeftOffset, lessThan(stringFragmentLeftOffset));
        expect(stringFragmentLeftOffset, lessThan(minuteLeftOffset));
        expect(dayPeriodControlFinder, findsNothing);
      } else if (locale == const Locale('es', 'ES')) {
        expect(stringFragmentText.data, ':');
        expect(hourLeftOffset, lessThan(stringFragmentLeftOffset));
        expect(stringFragmentLeftOffset, lessThan(minuteLeftOffset));
        expect(dayPeriodControlFinder, findsNothing);
      } else if (locale == const Locale('fr', 'CA')) {
        expect(stringFragmentText.data, 'h');
        expect(hourLeftOffset, lessThan(stringFragmentLeftOffset));
        expect(stringFragmentLeftOffset, lessThan(minuteLeftOffset));
        expect(dayPeriodControlFinder, findsNothing);
      } else if (locale == const Locale('zh', 'ZH')) {
        final double dayPeriodLeftOffset = tester.getTopLeft(dayPeriodControlFinder).dx;
        expect(stringFragmentText.data, ':');
        expect(dayPeriodLeftOffset, lessThan(hourLeftOffset));
        expect(hourLeftOffset, lessThan(stringFragmentLeftOffset));
        expect(stringFragmentLeftOffset, lessThan(minuteLeftOffset));
      } else if (locale == const Locale('fa', 'IR')) {
        // Even though this is an RTL locale, the hours and minutes positions should remain the same.
        expect(stringFragmentText.data, ':');
        expect(hourLeftOffset, lessThan(stringFragmentLeftOffset));
        expect(stringFragmentLeftOffset, lessThan(minuteLeftOffset));
        expect(dayPeriodControlFinder, findsNothing);
      }
      await tester.tapAt(Offset(center.dx, center.dy - 50.0));
      await finishPicker(tester);
    }
  });

  testWidgets('Material3 - can localize the header in all known formats - portrait', (
    WidgetTester tester,
  ) async {
    // Ensure picker is displayed in portrait mode.
    tester.view.physicalSize = const Size(400, 800);
    tester.view.devicePixelRatio = 1;
    addTearDown(tester.view.reset);

    final Finder timeSelectorSeparatorFinder =
        find
            .descendant(
              of: find.byWidgetPredicate(
                (Widget w) => '${w.runtimeType}' == '_TimeSelectorSeparator',
              ),
              matching: find.byType(Text),
            )
            .first;
    final Finder hourControlFinder = find.byWidgetPredicate(
      (Widget w) => '${w.runtimeType}' == '_HourControl',
    );
    final Finder minuteControlFinder = find.byWidgetPredicate(
      (Widget w) => '${w.runtimeType}' == '_MinuteControl',
    );
    final Finder dayPeriodControlFinder = find.byWidgetPredicate(
      (Widget w) => '${w.runtimeType}' == '_DayPeriodControl',
    );

    // TODO(yjbanov): also test `HH.mm` (in_ID), `a h:mm` (ko_KR) and `HH:mm น.` (th_TH) when we have .arb files for them
    final List<Locale> locales = <Locale>[
      const Locale('en', 'US'), //'h:mm a'
      const Locale('en', 'GB'), //'HH:mm'
      const Locale('es', 'ES'), //'H:mm'
      const Locale('fr', 'CA'), //'HH \'h\' mm'
      const Locale('zh', 'ZH'), //'ah:mm'
      const Locale('fa', 'IR'), //'H:mm' but RTL
    ];

    for (final Locale locale in locales) {
      final Offset center = await startPicker(
        tester,
        (TimeOfDay? time) {},
        locale: locale,
        useMaterial3: true,
      );
      final Text stringFragmentText = tester.widget(timeSelectorSeparatorFinder);
      final double hourLeftOffset = tester.getTopLeft(hourControlFinder).dx;
      final double minuteLeftOffset = tester.getTopLeft(minuteControlFinder).dx;
      final double stringFragmentLeftOffset = tester.getTopLeft(timeSelectorSeparatorFinder).dx;

      if (locale == const Locale('en', 'US')) {
        final double dayPeriodLeftOffset = tester.getTopLeft(dayPeriodControlFinder).dx;
        expect(stringFragmentText.data, ':');
        expect(hourLeftOffset, lessThan(stringFragmentLeftOffset));
        expect(stringFragmentLeftOffset, lessThan(minuteLeftOffset));
        expect(minuteLeftOffset, lessThan(dayPeriodLeftOffset));
      } else if (locale == const Locale('en', 'GB')) {
        expect(stringFragmentText.data, ':');
        expect(hourLeftOffset, lessThan(stringFragmentLeftOffset));
        expect(stringFragmentLeftOffset, lessThan(minuteLeftOffset));
        expect(dayPeriodControlFinder, findsNothing);
      } else if (locale == const Locale('es', 'ES')) {
        expect(stringFragmentText.data, ':');
        expect(hourLeftOffset, lessThan(stringFragmentLeftOffset));
        expect(stringFragmentLeftOffset, lessThan(minuteLeftOffset));
        expect(dayPeriodControlFinder, findsNothing);
      } else if (locale == const Locale('fr', 'CA')) {
        expect(stringFragmentText.data, 'h');
        expect(hourLeftOffset, lessThan(stringFragmentLeftOffset));
        expect(stringFragmentLeftOffset, lessThan(minuteLeftOffset));
        expect(dayPeriodControlFinder, findsNothing);
      } else if (locale == const Locale('zh', 'ZH')) {
        final double dayPeriodLeftOffset = tester.getTopLeft(dayPeriodControlFinder).dx;
        expect(stringFragmentText.data, ':');
        expect(dayPeriodLeftOffset, lessThan(hourLeftOffset));
        expect(hourLeftOffset, lessThan(stringFragmentLeftOffset));
        expect(stringFragmentLeftOffset, lessThan(minuteLeftOffset));
      } else if (locale == const Locale('fa', 'IR')) {
        // Even though this is an RTL locale, the hours and minutes positions should remain the same.
        expect(stringFragmentText.data, ':');
        expect(hourLeftOffset, lessThan(stringFragmentLeftOffset));
        expect(stringFragmentLeftOffset, lessThan(minuteLeftOffset));
        expect(dayPeriodControlFinder, findsNothing);
      }
      await tester.tapAt(Offset(center.dx, center.dy - 50.0));
      await finishPicker(tester);
    }
  });

  testWidgets('Material2 - can localize the header in all known formats - landscape', (
    WidgetTester tester,
  ) async {
    // Ensure picker is displayed in landscape mode.
    tester.view.physicalSize = const Size(800, 400);
    tester.view.devicePixelRatio = 1;
    addTearDown(tester.view.reset);

    final Finder timeSelectorSeparatorFinder =
        find
            .descendant(
              of: find.byWidgetPredicate(
                (Widget w) => '${w.runtimeType}' == '_TimeSelectorSeparator',
              ),
              matching: find.byType(Text),
            )
            .first;
    final Finder hourControlFinder = find.byWidgetPredicate(
      (Widget w) => '${w.runtimeType}' == '_HourControl',
    );
    final Finder minuteControlFinder = find.byWidgetPredicate(
      (Widget w) => '${w.runtimeType}' == '_MinuteControl',
    );
    final Finder dayPeriodControlFinder = find.byWidgetPredicate(
      (Widget w) => '${w.runtimeType}' == '_DayPeriodControl',
    );

    // TODO(yjbanov): also test `HH.mm` (in_ID), `a h:mm` (ko_KR) and `HH:mm น.` (th_TH) when we have .arb files for them
    final List<Locale> locales = <Locale>[
      const Locale('en', 'US'), //'h:mm a'
      const Locale('en', 'GB'), //'HH:mm'
      const Locale('es', 'ES'), //'H:mm'
      const Locale('fr', 'CA'), //'HH \'h\' mm'
      const Locale('zh', 'ZH'), //'ah:mm'
      const Locale('fa', 'IR'), //'H:mm' but RTL
    ];

    for (final Locale locale in locales) {
      final Offset center = await startPicker(
        tester,
        (TimeOfDay? time) {},
        locale: locale,
        useMaterial3: false,
      );
      final Text stringFragmentText = tester.widget(timeSelectorSeparatorFinder);
      final double hourLeftOffset = tester.getTopLeft(hourControlFinder).dx;
      final double hourTopOffset = tester.getTopLeft(hourControlFinder).dy;
      final double minuteLeftOffset = tester.getTopLeft(minuteControlFinder).dx;
      final double stringFragmentLeftOffset = tester.getTopLeft(timeSelectorSeparatorFinder).dx;

      if (locale == const Locale('en', 'US')) {
        final double dayPeriodLeftOffset = tester.getTopLeft(dayPeriodControlFinder).dx;
        final double dayPeriodTopOffset = tester.getTopLeft(dayPeriodControlFinder).dy;
        expect(stringFragmentText.data, ':');
        expect(hourLeftOffset, lessThan(stringFragmentLeftOffset));
        expect(stringFragmentLeftOffset, lessThan(minuteLeftOffset));
        expect(hourLeftOffset, dayPeriodLeftOffset);
        expect(hourTopOffset, lessThan(dayPeriodTopOffset));
      } else if (locale == const Locale('en', 'GB')) {
        expect(stringFragmentText.data, ':');
        expect(hourLeftOffset, lessThan(stringFragmentLeftOffset));
        expect(stringFragmentLeftOffset, lessThan(minuteLeftOffset));
        expect(dayPeriodControlFinder, findsNothing);
      } else if (locale == const Locale('es', 'ES')) {
        expect(stringFragmentText.data, ':');
        expect(hourLeftOffset, lessThan(stringFragmentLeftOffset));
        expect(stringFragmentLeftOffset, lessThan(minuteLeftOffset));
        expect(dayPeriodControlFinder, findsNothing);
      } else if (locale == const Locale('fr', 'CA')) {
        expect(stringFragmentText.data, 'h');
        expect(hourLeftOffset, lessThan(stringFragmentLeftOffset));
        expect(stringFragmentLeftOffset, lessThan(minuteLeftOffset));
        expect(dayPeriodControlFinder, findsNothing);
      } else if (locale == const Locale('zh', 'ZH')) {
        final double dayPeriodLeftOffset = tester.getTopLeft(dayPeriodControlFinder).dx;
        final double dayPeriodTopOffset = tester.getTopLeft(dayPeriodControlFinder).dy;
        expect(stringFragmentText.data, ':');
        expect(hourLeftOffset, lessThan(stringFragmentLeftOffset));
        expect(stringFragmentLeftOffset, lessThan(minuteLeftOffset));
        expect(hourLeftOffset, dayPeriodLeftOffset);
        expect(hourTopOffset, greaterThan(dayPeriodTopOffset));
      } else if (locale == const Locale('fa', 'IR')) {
        // Even though this is an RTL locale, the hours and minutes positions should remain the same.
        expect(stringFragmentText.data, ':');
        expect(hourLeftOffset, lessThan(stringFragmentLeftOffset));
        expect(stringFragmentLeftOffset, lessThan(minuteLeftOffset));
        expect(dayPeriodControlFinder, findsNothing);
      }
      await tester.tapAt(Offset(center.dx, center.dy - 50.0));
      await finishPicker(tester);
    }
  });

  testWidgets('Material3 - can localize the header in all known formats - landscape', (
    WidgetTester tester,
  ) async {
    // Ensure picker is displayed in landscape mode.
    tester.view.physicalSize = const Size(800, 400);
    tester.view.devicePixelRatio = 1;
    addTearDown(tester.view.reset);

    final Finder timeSelectorSeparatorFinder =
        find
            .descendant(
              of: find.byWidgetPredicate(
                (Widget w) => '${w.runtimeType}' == '_TimeSelectorSeparator',
              ),
              matching: find.byType(Text),
            )
            .first;
    final Finder hourControlFinder = find.byWidgetPredicate(
      (Widget w) => '${w.runtimeType}' == '_HourControl',
    );
    final Finder minuteControlFinder = find.byWidgetPredicate(
      (Widget w) => '${w.runtimeType}' == '_MinuteControl',
    );
    final Finder dayPeriodControlFinder = find.byWidgetPredicate(
      (Widget w) => '${w.runtimeType}' == '_DayPeriodControl',
    );

    // TODO(yjbanov): also test `HH.mm` (in_ID), `a h:mm` (ko_KR) and `HH:mm น.` (th_TH) when we have .arb files for them
    final List<Locale> locales = <Locale>[
      const Locale('en', 'US'), //'h:mm a'
      const Locale('en', 'GB'), //'HH:mm'
      const Locale('es', 'ES'), //'H:mm'
      const Locale('fr', 'CA'), //'HH \'h\' mm'
      const Locale('zh', 'ZH'), //'ah:mm'
      const Locale('fa', 'IR'), //'H:mm' but RTL
    ];

    for (final Locale locale in locales) {
      final Offset center = await startPicker(
        tester,
        (TimeOfDay? time) {},
        locale: locale,
        useMaterial3: true,
      );
      final Text stringFragmentText = tester.widget(timeSelectorSeparatorFinder);
      final double hourLeftOffset = tester.getTopLeft(hourControlFinder).dx;
      final double hourTopOffset = tester.getTopLeft(hourControlFinder).dy;
      final double minuteLeftOffset = tester.getTopLeft(minuteControlFinder).dx;
      final double stringFragmentLeftOffset = tester.getTopLeft(timeSelectorSeparatorFinder).dx;

      if (locale == const Locale('en', 'US')) {
        final double dayPeriodLeftOffset = tester.getTopLeft(dayPeriodControlFinder).dx;
        final double dayPeriodTopOffset = tester.getTopLeft(dayPeriodControlFinder).dy;
        expect(stringFragmentText.data, ':');
        expect(hourLeftOffset, lessThan(stringFragmentLeftOffset));
        expect(stringFragmentLeftOffset, lessThan(minuteLeftOffset));
        expect(hourLeftOffset, dayPeriodLeftOffset);
        expect(hourTopOffset, lessThan(dayPeriodTopOffset));
      } else if (locale == const Locale('en', 'GB')) {
        expect(stringFragmentText.data, ':');
        expect(hourLeftOffset, lessThan(stringFragmentLeftOffset));
        expect(stringFragmentLeftOffset, lessThan(minuteLeftOffset));
        expect(dayPeriodControlFinder, findsNothing);
      } else if (locale == const Locale('es', 'ES')) {
        expect(stringFragmentText.data, ':');
        expect(hourLeftOffset, lessThan(stringFragmentLeftOffset));
        expect(stringFragmentLeftOffset, lessThan(minuteLeftOffset));
        expect(dayPeriodControlFinder, findsNothing);
      } else if (locale == const Locale('fr', 'CA')) {
        expect(stringFragmentText.data, 'h');
        expect(hourLeftOffset, lessThan(stringFragmentLeftOffset));
        expect(stringFragmentLeftOffset, lessThan(minuteLeftOffset));
        expect(dayPeriodControlFinder, findsNothing);
      } else if (locale == const Locale('zh', 'ZH')) {
        final double dayPeriodLeftOffset = tester.getTopLeft(dayPeriodControlFinder).dx;
        final double dayPeriodTopOffset = tester.getTopLeft(dayPeriodControlFinder).dy;
        expect(stringFragmentText.data, ':');
        expect(hourLeftOffset, lessThan(stringFragmentLeftOffset));
        expect(stringFragmentLeftOffset, lessThan(minuteLeftOffset));
        expect(hourLeftOffset, dayPeriodLeftOffset);
        expect(hourTopOffset, greaterThan(dayPeriodTopOffset));
      } else if (locale == const Locale('fa', 'IR')) {
        // Even though this is an RTL locale, the hours and minutes positions should remain the same.
        expect(stringFragmentText.data, ':');
        expect(hourLeftOffset, lessThan(stringFragmentLeftOffset));
        expect(stringFragmentLeftOffset, lessThan(minuteLeftOffset));
        expect(dayPeriodControlFinder, findsNothing);
      }
      await tester.tapAt(Offset(center.dx, center.dy - 50.0));
      await finishPicker(tester);
    }
  });

  testWidgets('Material2 - can localize input mode in all known formats', (
    WidgetTester tester,
  ) async {
    final Finder hourControlFinder = find.byWidgetPredicate(
      (Widget w) => '${w.runtimeType}' == '_HourTextField',
    );
    final Finder minuteControlFinder = find.byWidgetPredicate(
      (Widget w) => '${w.runtimeType}' == '_MinuteTextField',
    );
    final Finder dayPeriodControlFinder = find.byWidgetPredicate(
      (Widget w) => '${w.runtimeType}' == '_DayPeriodControl',
    );
    final Finder timeSelectorSeparatorFinder =
        find
            .descendant(
              of: find.byWidgetPredicate(
                (Widget w) => '${w.runtimeType}' == '_TimeSelectorSeparator',
              ),
              matching: find.byType(Text),
            )
            .first;

    // TODO(yjbanov): also test `HH.mm` (in_ID), `a h:mm` (ko_KR) and `HH:mm น.` (th_TH) when we have .arb files for them
    final List<Locale> locales = <Locale>[
      const Locale('en', 'US'), //'h:mm a'
      const Locale('en', 'GB'), //'HH:mm'
      const Locale('es', 'ES'), //'H:mm'
      const Locale('fr', 'CA'), //'HH \'h\' mm'
      const Locale('zh', 'ZH'), //'ah:mm'
      const Locale('fa', 'IR'), //'H:mm' but RTL
    ];

    for (final Locale locale in locales) {
      await tester.pumpWidget(
        _TimePickerLauncher(
          onChanged: (TimeOfDay? time) {},
          locale: locale,
          entryMode: TimePickerEntryMode.input,
          useMaterial3: false,
        ),
      );
      await tester.tap(find.text('X'));
      await tester.pumpAndSettle(const Duration(seconds: 1));

      final Text stringFragmentText = tester.widget(timeSelectorSeparatorFinder);
      final double hourLeftOffset = tester.getTopLeft(hourControlFinder).dx;
      final double minuteLeftOffset = tester.getTopLeft(minuteControlFinder).dx;
      final double stringFragmentLeftOffset = tester.getTopLeft(timeSelectorSeparatorFinder).dx;

      if (locale == const Locale('en', 'US')) {
        final double dayPeriodLeftOffset = tester.getTopLeft(dayPeriodControlFinder).dx;
        expect(stringFragmentText.data, ':');
        expect(hourLeftOffset, lessThan(stringFragmentLeftOffset));
        expect(stringFragmentLeftOffset, lessThan(minuteLeftOffset));
        expect(minuteLeftOffset, lessThan(dayPeriodLeftOffset));
      } else if (locale == const Locale('en', 'GB')) {
        expect(stringFragmentText.data, ':');
        expect(hourLeftOffset, lessThan(stringFragmentLeftOffset));
        expect(stringFragmentLeftOffset, lessThan(minuteLeftOffset));
        expect(dayPeriodControlFinder, findsNothing);
      } else if (locale == const Locale('es', 'ES')) {
        expect(stringFragmentText.data, ':');
        expect(hourLeftOffset, lessThan(stringFragmentLeftOffset));
        expect(stringFragmentLeftOffset, lessThan(minuteLeftOffset));
        expect(dayPeriodControlFinder, findsNothing);
      } else if (locale == const Locale('fr', 'CA')) {
        expect(stringFragmentText.data, 'h');
        expect(hourLeftOffset, lessThan(stringFragmentLeftOffset));
        expect(stringFragmentLeftOffset, lessThan(minuteLeftOffset));
        expect(dayPeriodControlFinder, findsNothing);
      } else if (locale == const Locale('zh', 'ZH')) {
        final double dayPeriodLeftOffset = tester.getTopLeft(dayPeriodControlFinder).dx;
        expect(stringFragmentText.data, ':');
        expect(dayPeriodLeftOffset, lessThan(hourLeftOffset));
        expect(hourLeftOffset, lessThan(stringFragmentLeftOffset));
        expect(stringFragmentLeftOffset, lessThan(minuteLeftOffset));
      } else if (locale == const Locale('fa', 'IR')) {
        // Even though this is an RTL locale, the hours and minutes positions should remain the same.
        expect(stringFragmentText.data, ':');
        expect(hourLeftOffset, lessThan(stringFragmentLeftOffset));
        expect(stringFragmentLeftOffset, lessThan(minuteLeftOffset));
        expect(dayPeriodControlFinder, findsNothing);
      }
      await finishPicker(tester);
      expect(tester.takeException(), isNot(throwsFlutterError));
    }
  });

  testWidgets('Material3 - can localize input mode in all known formats', (
    WidgetTester tester,
  ) async {
    final Finder hourControlFinder = find.byWidgetPredicate(
      (Widget w) => '${w.runtimeType}' == '_HourTextField',
    );
    final Finder minuteControlFinder = find.byWidgetPredicate(
      (Widget w) => '${w.runtimeType}' == '_MinuteTextField',
    );
    final Finder dayPeriodControlFinder = find.byWidgetPredicate(
      (Widget w) => '${w.runtimeType}' == '_DayPeriodControl',
    );
    final Finder timeSelectorSeparatorFinder =
        find
            .descendant(
              of: find.byWidgetPredicate(
                (Widget w) => '${w.runtimeType}' == '_TimeSelectorSeparator',
              ),
              matching: find.byType(Text),
            )
            .first;

    // TODO(yjbanov): also test `HH.mm` (in_ID), `a h:mm` (ko_KR) and `HH:mm น.` (th_TH) when we have .arb files for them
    final List<Locale> locales = <Locale>[
      const Locale('en', 'US'), //'h:mm a'
      const Locale('en', 'GB'), //'HH:mm'
      const Locale('es', 'ES'), //'H:mm'
      const Locale('fr', 'CA'), //'HH \'h\' mm'
      const Locale('zh', 'ZH'), //'ah:mm'
      const Locale('fa', 'IR'), //'H:mm' but RTL
    ];

    for (final Locale locale in locales) {
      await tester.pumpWidget(
        _TimePickerLauncher(
          onChanged: (TimeOfDay? time) {},
          locale: locale,
          entryMode: TimePickerEntryMode.input,
          useMaterial3: true,
        ),
      );
      await tester.tap(find.text('X'));
      await tester.pumpAndSettle(const Duration(seconds: 1));

      final Text stringFragmentText = tester.widget(timeSelectorSeparatorFinder);
      final double hourLeftOffset = tester.getTopLeft(hourControlFinder).dx;
      final double minuteLeftOffset = tester.getTopLeft(minuteControlFinder).dx;
      final double stringFragmentLeftOffset = tester.getTopLeft(timeSelectorSeparatorFinder).dx;

      if (locale == const Locale('en', 'US')) {
        final double dayPeriodLeftOffset = tester.getTopLeft(dayPeriodControlFinder).dx;
        expect(stringFragmentText.data, ':');
        expect(hourLeftOffset, lessThan(stringFragmentLeftOffset));
        expect(stringFragmentLeftOffset, lessThan(minuteLeftOffset));
        expect(minuteLeftOffset, lessThan(dayPeriodLeftOffset));
      } else if (locale == const Locale('en', 'GB')) {
        expect(stringFragmentText.data, ':');
        expect(hourLeftOffset, lessThan(stringFragmentLeftOffset));
        expect(stringFragmentLeftOffset, lessThan(minuteLeftOffset));
        expect(dayPeriodControlFinder, findsNothing);
      } else if (locale == const Locale('es', 'ES')) {
        expect(stringFragmentText.data, ':');
        expect(hourLeftOffset, lessThan(stringFragmentLeftOffset));
        expect(stringFragmentLeftOffset, lessThan(minuteLeftOffset));
        expect(dayPeriodControlFinder, findsNothing);
      } else if (locale == const Locale('fr', 'CA')) {
        expect(stringFragmentText.data, 'h');
        expect(hourLeftOffset, lessThan(stringFragmentLeftOffset));
        expect(stringFragmentLeftOffset, lessThan(minuteLeftOffset));
        expect(dayPeriodControlFinder, findsNothing);
      } else if (locale == const Locale('zh', 'ZH')) {
        final double dayPeriodLeftOffset = tester.getTopLeft(dayPeriodControlFinder).dx;
        expect(stringFragmentText.data, ':');
        expect(dayPeriodLeftOffset, lessThan(hourLeftOffset));
        expect(hourLeftOffset, lessThan(stringFragmentLeftOffset));
        expect(stringFragmentLeftOffset, lessThan(minuteLeftOffset));
      } else if (locale == const Locale('fa', 'IR')) {
        // Even though this is an RTL locale, the hours and minutes positions should remain the same.
        expect(stringFragmentText.data, ':');
        expect(hourLeftOffset, lessThan(stringFragmentLeftOffset));
        expect(stringFragmentLeftOffset, lessThan(minuteLeftOffset));
        expect(dayPeriodControlFinder, findsNothing);
      }
      await finishPicker(tester);
      expect(tester.takeException(), isNot(throwsFlutterError));
    }
  });

  testWidgets('Material2 uses single-ring 24-hour dial for all locales', (
    WidgetTester tester,
  ) async {
    const List<Locale> locales = <Locale>[
      Locale('en', 'US'), // h
      Locale('en', 'GB'), // HH
      Locale('es', 'ES'), // H
    ];
    for (final Locale locale in locales) {
      // Tap along the segment stretching from the center to the edge at
      // 12:00 AM position. Because there's only one ring, in the M2
      // DatePicker no matter where you tap the time will be the same.
      for (int i = 1; i < 10; i++) {
        TimeOfDay? result;
        final Offset center = await startPicker(
          tester,
          (TimeOfDay? time) {
            result = time;
          },
          locale: locale,
          useMaterial3: false,
        );
        final Size size = tester.getSize(find.byKey(const Key('time-picker-dial')));
        final double dy = (size.height / 2.0 / 10) * i;
        await tester.tapAt(Offset(center.dx, center.dy - dy));
        await finishPicker(tester);
        expect(result, equals(const TimeOfDay(hour: 0, minute: 0)));
      }
    }
  });

  testWidgets('Material3 uses a  double-ring 24-hour dial for 24 hour locales', (
    WidgetTester tester,
  ) async {
    Future<void> testLocale(
      Locale locale,
      int startFactor,
      int endFactor,
      TimeOfDay expectedTime,
    ) async {
      // For locales that display 24 hour time, factors 1-5 put the tap on the
      // inner ring's "12" (the inner ring goes from 12-23). Otherwise the offset
      // should land on the outer ring's "00".
      for (int factor = startFactor; factor < endFactor; factor += 1) {
        TimeOfDay? result;
        final Offset center = await startPicker(
          tester,
          (TimeOfDay? time) {
            result = time;
          },
          locale: locale,
          useMaterial3: true,
        );
        final Size size = tester.getSize(find.byKey(const Key('time-picker-dial')));
        final double dy = (size.height / 2.0 / 10) * factor;
        await tester.tapAt(Offset(center.dx, center.dy - dy));
        await finishPicker(tester);
        expect(
          result,
          equals(expectedTime),
          reason: 'Failed for locale=$locale with factor=$factor',
        );
      }
    }

    await testLocale(
      const Locale('en', 'US'),
      1,
      10,
      const TimeOfDay(hour: 0, minute: 0),
    ); // 12 hour
    await testLocale(
      const Locale('en', 'ES'),
      1,
      10,
      const TimeOfDay(hour: 0, minute: 0),
    ); // 12 hour
    await testLocale(
      const Locale('en', 'GB'),
      1,
      5,
      const TimeOfDay(hour: 12, minute: 0),
    ); // 24 hour, inner ring
    await testLocale(
      const Locale('en', 'GB'),
      6,
      10,
      const TimeOfDay(hour: 0, minute: 0),
    ); // 24 hour, outer ring
  });

  const List<String> labels12To11 = <String>[
    '12',
    '1',
    '2',
    '3',
    '4',
    '5',
    '6',
    '7',
    '8',
    '9',
    '10',
    '11',
  ];
  const List<String> labels00To22TwoDigit = <String>[
    '00',
    '02',
    '04',
    '06',
    '08',
    '10',
    '12',
    '14',
    '16',
    '18',
    '20',
    '22',
  ]; // Material 2
  const List<String> labels00To23TwoDigit = <String>[
    // Material 3
    '00',
    '1',
    '2',
    '3',
    '4',
    '5',
    '6',
    '7',
    '8',
    '9',
    '10',
    '11',
    '12',
    '13',
    '14',
    '15',
    '16',
    '17',
    '18',
    '19',
    '20',
    '21',
    '22',
    '23',
  ];

  Future<void> mediaQueryBoilerplate(
    WidgetTester tester, {
    required bool alwaysUse24HourFormat,
    required bool useMaterial3,
  }) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: useMaterial3),
        builder: (BuildContext context, Widget? child) {
          return MediaQuery(
            data: MediaQueryData(alwaysUse24HourFormat: alwaysUse24HourFormat),
            child: child!,
          );
        },
        home: Material(
          child: Directionality(
            textDirection: TextDirection.ltr,
            child: Navigator(
              onGenerateRoute: (RouteSettings settings) {
                return MaterialPageRoute<void>(
                  builder: (BuildContext context) {
                    return TextButton(
                      onPressed: () {
                        showTimePicker(
                          context: context,
                          initialTime: const TimeOfDay(hour: 7, minute: 0),
                        );
                      },
                      child: const Text('X'),
                    );
                  },
                );
              },
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.text('X'));
    await tester.pumpAndSettle();
  }

  testWidgets('Material2 respects MediaQueryData.alwaysUse24HourFormat == false', (
    WidgetTester tester,
  ) async {
    await mediaQueryBoilerplate(tester, alwaysUse24HourFormat: false, useMaterial3: false);

    final CustomPaint dialPaint = tester.widget(
      find.byKey(const ValueKey<String>('time-picker-dial')),
    );
    final dynamic dialPainter = dialPaint.painter;
    // ignore: avoid_dynamic_calls
    final List<dynamic> primaryLabels = dialPainter.primaryLabels as List<dynamic>;
    expect(
      primaryLabels.map<String>(
        // ignore: avoid_dynamic_calls
        (dynamic tp) => ((tp.painter as TextPainter).text! as TextSpan).text!,
      ),
      labels12To11,
    );

    // ignore: avoid_dynamic_calls
    final List<dynamic> selectedLabels = dialPainter.selectedLabels as List<dynamic>;
    expect(
      selectedLabels.map<String>(
        // ignore: avoid_dynamic_calls
        (dynamic tp) => ((tp.painter as TextPainter).text! as TextSpan).text!,
      ),
      labels12To11,
    );
  });

  testWidgets('Material3 respects MediaQueryData.alwaysUse24HourFormat == false', (
    WidgetTester tester,
  ) async {
    await mediaQueryBoilerplate(tester, alwaysUse24HourFormat: false, useMaterial3: true);

    final CustomPaint dialPaint = tester.widget(
      find.byKey(const ValueKey<String>('time-picker-dial')),
    );
    final dynamic dialPainter = dialPaint.painter;
    // ignore: avoid_dynamic_calls
    final List<dynamic> primaryLabels = dialPainter.primaryLabels as List<dynamic>;
    expect(
      primaryLabels.map<String>(
        // ignore: avoid_dynamic_calls
        (dynamic tp) => ((tp.painter as TextPainter).text! as TextSpan).text!,
      ),
      labels12To11,
    );

    // ignore: avoid_dynamic_calls
    final List<dynamic> selectedLabels = dialPainter.selectedLabels as List<dynamic>;
    expect(
      selectedLabels.map<String>(
        // ignore: avoid_dynamic_calls
        (dynamic tp) => ((tp.painter as TextPainter).text! as TextSpan).text!,
      ),
      labels12To11,
    );
  });

  testWidgets('Material3 respects MediaQueryData.alwaysUse24HourFormat == true', (
    WidgetTester tester,
  ) async {
    await mediaQueryBoilerplate(tester, alwaysUse24HourFormat: true, useMaterial3: true);

    final CustomPaint dialPaint = tester.widget(
      find.byKey(const ValueKey<String>('time-picker-dial')),
    );
    final dynamic dialPainter = dialPaint.painter;
    // ignore: avoid_dynamic_calls
    final List<dynamic> primaryLabels = dialPainter.primaryLabels as List<dynamic>;
    expect(
      primaryLabels.map<String>(
        // ignore: avoid_dynamic_calls
        (dynamic tp) => ((tp.painter as TextPainter).text! as TextSpan).text!,
      ),
      labels00To23TwoDigit,
    );

    // ignore: avoid_dynamic_calls
    final List<dynamic> selectedLabels = dialPainter.selectedLabels as List<dynamic>;
    expect(
      selectedLabels.map<String>(
        // ignore: avoid_dynamic_calls
        (dynamic tp) => ((tp.painter as TextPainter).text! as TextSpan).text!,
      ),
      labels00To23TwoDigit,
    );
  });

  testWidgets('Material2 respects MediaQueryData.alwaysUse24HourFormat == true', (
    WidgetTester tester,
  ) async {
    await mediaQueryBoilerplate(tester, alwaysUse24HourFormat: true, useMaterial3: false);

    final CustomPaint dialPaint = tester.widget(
      find.byKey(const ValueKey<String>('time-picker-dial')),
    );
    final dynamic dialPainter = dialPaint.painter;
    // ignore: avoid_dynamic_calls
    final List<dynamic> primaryLabels = dialPainter.primaryLabels as List<dynamic>;
    expect(
      primaryLabels.map<String>(
        // ignore: avoid_dynamic_calls
        (dynamic tp) => ((tp.painter as TextPainter).text! as TextSpan).text!,
      ),
      labels00To22TwoDigit,
    );

    // ignore: avoid_dynamic_calls
    final List<dynamic> selectedLabels = dialPainter.selectedLabels as List<dynamic>;
    expect(
      selectedLabels.map<String>(
        // ignore: avoid_dynamic_calls
        (dynamic tp) => ((tp.painter as TextPainter).text! as TextSpan).text!,
      ),
      labels00To22TwoDigit,
    );
  });

  // Regression test for https://github.com/flutter/flutter/issues/156565
  testWidgets('AM/PM buttons should be aligned to LTR in Hindi language - Portrait', (
    WidgetTester tester,
  ) async {
    const Locale locale = Locale('hi', 'HI');

    final Offset centerPortrait = await startPicker(
      tester,
      (TimeOfDay? time) {},
      locale: locale,
      useMaterial3: false,
      orientation: Orientation.portrait,
    );

    final Finder amButtonPortrait = find.text('AM');
    final Finder pmButtonPortrait = find.text('PM');

    final Offset amButtonPositionPortrait = tester.getCenter(amButtonPortrait);
    final Offset pmButtonPositionPortrait = tester.getCenter(pmButtonPortrait);

    expect(amButtonPositionPortrait.dx, greaterThan(centerPortrait.dx));
    expect(pmButtonPositionPortrait.dx, greaterThan(centerPortrait.dx));
  });

  // Regression test for https://github.com/flutter/flutter/issues/156565
  testWidgets('AM/PM buttons should be aligned to LTR in Hindi language - Landscape', (
    WidgetTester tester,
  ) async {
    const Locale locale = Locale('hi', 'HI');

    final Offset centerLandscape = await startPicker(
      tester,
      (TimeOfDay? time) {},
      locale: locale,
      useMaterial3: false,
      orientation: Orientation.landscape,
    );

    final Finder amButtonLandscape = find.text('AM');
    final Finder pmButtonLandscape = find.text('PM');

    final Offset amButtonPositionLandscape = tester.getCenter(amButtonLandscape);
    final Offset pmButtonPositionLandscape = tester.getCenter(pmButtonLandscape);

    expect(amButtonPositionLandscape.dy, greaterThan(centerLandscape.dy));
    expect(pmButtonPositionLandscape.dy, greaterThan(centerLandscape.dy));
  });
}

class _TimePickerLauncher extends StatelessWidget {
  const _TimePickerLauncher({
    this.onChanged,
    required this.locale,
    this.entryMode = TimePickerEntryMode.dial,
    this.useMaterial3,
    this.orientation,
  });

  final ValueChanged<TimeOfDay?>? onChanged;
  final Locale locale;
  final TimePickerEntryMode entryMode;
  final bool? useMaterial3;
  final Orientation? orientation;

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      theme: ThemeData(useMaterial3: useMaterial3),
      locale: locale,
      supportedLocales: <Locale>[locale],
      localizationsDelegates: GlobalMaterialLocalizations.delegates,
      home: Material(
        child: Center(
          child: Builder(
            builder: (BuildContext context) {
              return ElevatedButton(
                child: const Text('X'),
                onPressed: () async {
                  onChanged?.call(
                    await showTimePicker(
                      context: context,
                      initialEntryMode: entryMode,
                      orientation: orientation,
                      initialTime: const TimeOfDay(hour: 7, minute: 0),
                    ),
                  );
                },
              );
            },
          ),
        ),
      ),
    );
  }
}

Future<Offset> startPicker(
  WidgetTester tester,
  ValueChanged<TimeOfDay?> onChanged, {
  Locale locale = const Locale('en', 'US'),
  bool? useMaterial3,
  Orientation? orientation,
}) async {
  await tester.pumpWidget(
    _TimePickerLauncher(
      onChanged: onChanged,
      locale: locale,
      useMaterial3: useMaterial3,
      orientation: orientation,
    ),
  );
  await tester.tap(find.text('X'));
  await tester.pumpAndSettle(const Duration(seconds: 1));
  return tester.getCenter(find.byKey(const Key('time-picker-dial')));
}

Future<void> finishPicker(WidgetTester tester) async {
  final MaterialLocalizations materialLocalizations = MaterialLocalizations.of(
    tester.element(find.byType(ElevatedButton)),
  );
  await tester.tap(find.text(materialLocalizations.okButtonLabel));
  await tester.pumpAndSettle(const Duration(seconds: 1));
}
