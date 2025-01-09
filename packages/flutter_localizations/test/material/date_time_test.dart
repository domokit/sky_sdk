// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter/material.dart';
import 'package:flutter_localizations/flutter_localizations.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:intl/intl.dart';

void main() {
  group(GlobalMaterialLocalizations, () {
    test('uses exact locale when exists', () async {
      final GlobalMaterialLocalizations localizations =
          await GlobalMaterialLocalizations.delegate.load(const Locale('pt', 'PT'))
              as GlobalMaterialLocalizations;
      expect(localizations.formatDecimal(10000), '10\u00A0000');
    });

    test('falls back to language code when exact locale is missing', () async {
      final GlobalMaterialLocalizations localizations =
          await GlobalMaterialLocalizations.delegate.load(const Locale('pt', 'XX'))
              as GlobalMaterialLocalizations;
      expect(localizations.formatDecimal(10000), '10.000');
    });

    test('fails when neither language code nor exact locale are available', () async {
      await expectLater(() async {
        await GlobalMaterialLocalizations.delegate.load(const Locale('xx', 'XX'));
      }, throwsAssertionError);
    });

    group('formatHour', () {
      Future<String> formatHour(WidgetTester tester, Locale locale, TimeOfDay timeOfDay) async {
        final Completer<String> completer = Completer<String>();
        await tester.pumpWidget(
          MaterialApp(
            supportedLocales: <Locale>[locale],
            locale: locale,
            localizationsDelegates: GlobalMaterialLocalizations.delegates,
            home: Builder(
              builder: (BuildContext context) {
                completer.complete(MaterialLocalizations.of(context).formatHour(timeOfDay));
                return Container();
              },
            ),
          ),
        );
        return completer.future;
      }

      testWidgets('formats h', (WidgetTester tester) async {
        expect(
          await formatHour(tester, const Locale('en', 'US'), const TimeOfDay(hour: 10, minute: 0)),
          '10',
        );
        expect(
          await formatHour(tester, const Locale('en', 'US'), const TimeOfDay(hour: 20, minute: 0)),
          '8',
        );
      });

      testWidgets('formats HH', (WidgetTester tester) async {
        expect(
          await formatHour(tester, const Locale('de'), const TimeOfDay(hour: 9, minute: 0)),
          '09',
        );
        expect(
          await formatHour(tester, const Locale('de'), const TimeOfDay(hour: 20, minute: 0)),
          '20',
        );

        expect(
          await formatHour(tester, const Locale('en', 'GB'), const TimeOfDay(hour: 9, minute: 0)),
          '09',
        );
        expect(
          await formatHour(tester, const Locale('en', 'GB'), const TimeOfDay(hour: 20, minute: 0)),
          '20',
        );
      });

      testWidgets('formats H', (WidgetTester tester) async {
        expect(
          await formatHour(tester, const Locale('es'), const TimeOfDay(hour: 9, minute: 0)),
          '9',
        );
        expect(
          await formatHour(tester, const Locale('es'), const TimeOfDay(hour: 20, minute: 0)),
          '20',
        );

        expect(
          await formatHour(tester, const Locale('fa'), const TimeOfDay(hour: 9, minute: 0)),
          '۹',
        );
        expect(
          await formatHour(tester, const Locale('fa'), const TimeOfDay(hour: 20, minute: 0)),
          '۲۰',
        );
      });
    });

    group('formatMinute', () {
      test('formats English', () async {
        final GlobalMaterialLocalizations localizations =
            await GlobalMaterialLocalizations.delegate.load(const Locale('en', 'US'))
                as GlobalMaterialLocalizations;
        expect(localizations.formatMinute(const TimeOfDay(hour: 1, minute: 32)), '32');
      });
    });

    group('formatTimeOfDay', () {
      Future<String> formatTimeOfDay(
        WidgetTester tester,
        Locale locale,
        TimeOfDay timeOfDay,
      ) async {
        final Completer<String> completer = Completer<String>();
        await tester.pumpWidget(
          MaterialApp(
            supportedLocales: <Locale>[locale],
            locale: locale,
            localizationsDelegates: GlobalMaterialLocalizations.delegates,
            home: Builder(
              builder: (BuildContext context) {
                completer.complete(MaterialLocalizations.of(context).formatTimeOfDay(timeOfDay));
                return Container();
              },
            ),
          ),
        );
        return completer.future;
      }

      testWidgets('formats ${TimeOfDayFormat.h_colon_mm_space_a}', (WidgetTester tester) async {
        expect(
          await formatTimeOfDay(tester, const Locale('en'), const TimeOfDay(hour: 9, minute: 32)),
          '9:32 AM',
        );
        expect(
          await formatTimeOfDay(tester, const Locale('en'), const TimeOfDay(hour: 20, minute: 32)),
          '8:32 PM',
        );
      });

      testWidgets('formats ${TimeOfDayFormat.HH_colon_mm}', (WidgetTester tester) async {
        expect(
          await formatTimeOfDay(tester, const Locale('de'), const TimeOfDay(hour: 9, minute: 32)),
          '09:32',
        );
        expect(
          await formatTimeOfDay(
            tester,
            const Locale('en', 'ZA'),
            const TimeOfDay(hour: 9, minute: 32),
          ),
          '09:32',
        );
      });

      testWidgets('formats ${TimeOfDayFormat.H_colon_mm}', (WidgetTester tester) async {
        expect(
          await formatTimeOfDay(tester, const Locale('es'), const TimeOfDay(hour: 9, minute: 32)),
          '9:32',
        );
        expect(
          await formatTimeOfDay(tester, const Locale('es'), const TimeOfDay(hour: 20, minute: 32)),
          '20:32',
        );

        expect(
          await formatTimeOfDay(tester, const Locale('ja'), const TimeOfDay(hour: 9, minute: 32)),
          '9:32',
        );
        expect(
          await formatTimeOfDay(tester, const Locale('ja'), const TimeOfDay(hour: 20, minute: 32)),
          '20:32',
        );
      });

      testWidgets('formats ${TimeOfDayFormat.HH_dot_mm}', (WidgetTester tester) async {
        expect(
          await formatTimeOfDay(tester, const Locale('fi'), const TimeOfDay(hour: 20, minute: 32)),
          '20.32',
        );
        expect(
          await formatTimeOfDay(tester, const Locale('fi'), const TimeOfDay(hour: 9, minute: 32)),
          '09.32',
        );
        expect(
          await formatTimeOfDay(tester, const Locale('da'), const TimeOfDay(hour: 9, minute: 32)),
          '09.32',
        );
      });

      testWidgets('formats ${TimeOfDayFormat.frenchCanadian}', (WidgetTester tester) async {
        expect(
          await formatTimeOfDay(
            tester,
            const Locale('fr', 'CA'),
            const TimeOfDay(hour: 9, minute: 32),
          ),
          '09 h 32',
        );
      });

      testWidgets('formats ${TimeOfDayFormat.a_space_h_colon_mm}', (WidgetTester tester) async {
        expect(
          await formatTimeOfDay(tester, const Locale('zh'), const TimeOfDay(hour: 9, minute: 32)),
          '上午 9:32',
        );
        expect(
          await formatTimeOfDay(tester, const Locale('ta'), const TimeOfDay(hour: 9, minute: 32)),
          '9:32 AM',
        );
      });
    });

    group('date formatters', () {
      Future<Map<DateType, String>> formatDate(
        WidgetTester tester,
        Locale locale,
        DateTime dateTime,
      ) async {
        final Completer<Map<DateType, String>> completer = Completer<Map<DateType, String>>();
        await tester.pumpWidget(
          MaterialApp(
            supportedLocales: <Locale>[locale],
            locale: locale,
            localizationsDelegates: GlobalMaterialLocalizations.delegates,
            home: Builder(
              builder: (BuildContext context) {
                final MaterialLocalizations localizations = MaterialLocalizations.of(context);
                completer.complete(<DateType, String>{
                  DateType.year: localizations.formatYear(dateTime),
                  DateType.medium: localizations.formatMediumDate(dateTime),
                  DateType.full: localizations.formatFullDate(dateTime),
                  DateType.monthYear: localizations.formatMonthYear(dateTime),
                });
                return Container();
              },
            ),
          ),
        );
        return completer.future;
      }

      testWidgets('formats dates in English', (WidgetTester tester) async {
        final Map<DateType, String> formatted = await formatDate(
          tester,
          const Locale('en'),
          DateTime(2018, 8),
        );
        expect(formatted[DateType.year], '2018');
        expect(formatted[DateType.medium], 'Wed, Aug 1');
        expect(formatted[DateType.full], 'Wednesday, August 1, 2018');
        expect(formatted[DateType.monthYear], 'August 2018');
      });

      testWidgets('formats dates in German', (WidgetTester tester) async {
        final Map<DateType, String> formatted = await formatDate(
          tester,
          const Locale('de'),
          DateTime(2018, 8),
        );
        expect(formatted[DateType.year], '2018');
        expect(formatted[DateType.medium], 'Mi., 1. Aug.');
        expect(formatted[DateType.full], 'Mittwoch, 1. August 2018');
        expect(formatted[DateType.monthYear], 'August 2018');
      });

      testWidgets('formats dates in Serbian', (WidgetTester tester) async {
        final Map<DateType, String> formatted = await formatDate(
          tester,
          const Locale('sr'),
          DateTime(2018, 8),
        );
        expect(formatted[DateType.year], '2018.');
        expect(formatted[DateType.medium], 'сре 1. авг');
        expect(formatted[DateType.full], 'среда, 1. август 2018.');
        expect(formatted[DateType.monthYear], 'август 2018.');
      });

      testWidgets('formats dates in Serbian (Latin)', (WidgetTester tester) async {
        final Map<DateType, String> formatted = await formatDate(
          tester,
          const Locale.fromSubtags(languageCode: 'sr', scriptCode: 'Latn'),
          DateTime(2018, 8),
        );
        expect(formatted[DateType.year], '2018.');
        expect(formatted[DateType.medium], 'sre 1. avg');
        expect(formatted[DateType.full], 'sreda, 1. avgust 2018.');
        expect(formatted[DateType.monthYear], 'avgust 2018.');
      });
    });
  });

  // Regression test for https://github.com/flutter/flutter/issues/67644.
  testWidgets('en_US is initialized correctly by Flutter when DateFormat is used', (
    WidgetTester tester,
  ) async {
    late DateFormat dateFormat;

    await tester.pumpWidget(
      MaterialApp(
        locale: const Locale('en', 'US'),
        localizationsDelegates: GlobalMaterialLocalizations.delegates,
        home: Builder(
          builder: (BuildContext context) {
            dateFormat = DateFormat('EEE, d MMM yyyy HH:mm:ss', 'en_US');
            return Container();
          },
        ),
      ),
    );

    expect(dateFormat.locale, 'en_US');
  });

  testWidgets('cy is initialized correctly by Flutter when DateFormat is used', (
    WidgetTester tester,
  ) async {
    late DateFormat dateFormat;

    await tester.pumpWidget(
      MaterialApp(
        locale: const Locale('cy'),
        localizationsDelegates: GlobalMaterialLocalizations.delegates,
        home: Builder(
          builder: (BuildContext context) {
            dateFormat = DateFormat.yMMMd('cy');
            return Container();
          },
        ),
      ),
    );

    expect(dateFormat.locale, 'cy');
    expect(dateFormat.format(DateTime(2023, 4, 10, 2, 32)), equals('10 Ebr 2023'));
  });
}

enum DateType { year, medium, full, monthYear }
