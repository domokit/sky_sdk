// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('English translations exist for all CupertinoLocalization properties', (
    WidgetTester tester,
  ) async {
    const CupertinoLocalizations localizations = DefaultCupertinoLocalizations();

    expect(localizations.datePickerYear(2018), isNotNull);
    expect(localizations.datePickerMonth(1), isNotNull);
    expect(localizations.datePickerDayOfMonth(1), isNotNull);
    expect(localizations.datePickerDayOfMonth(1, 1), isNotNull);
    expect(localizations.datePickerHour(0), isNotNull);
    expect(localizations.datePickerHourSemanticsLabel(0), isNotNull);
    expect(localizations.datePickerMinute(0), isNotNull);
    expect(localizations.datePickerMinuteSemanticsLabel(0), isNotNull);
    expect(localizations.datePickerMediumDate(DateTime.now()), isNotNull);
    expect(localizations.datePickerDateOrder, isNotNull);
    expect(localizations.datePickerDateTimeOrder, isNotNull);

    expect(localizations.anteMeridiemAbbreviation, isNotNull);
    expect(localizations.postMeridiemAbbreviation, isNotNull);

    expect(localizations.timerPickerHour(0), isNotNull);
    expect(localizations.timerPickerMinute(0), isNotNull);
    expect(localizations.timerPickerSecond(0), isNotNull);
    expect(localizations.timerPickerHourLabel(0), isNotNull);
    expect(localizations.timerPickerMinuteLabel(0), isNotNull);
    expect(localizations.timerPickerSecondLabel(0), isNotNull);

    expect(localizations.modalBarrierDismissLabel, isNotNull);
    expect(localizations.searchTextFieldPlaceholderLabel, isNotNull);
    expect(localizations.noSpellCheckReplacementsLabel, isNotNull);
    expect(localizations.clearButtonLabel, isNotNull);
    expect(localizations.cancelButtonLabel, isNotNull);
    expect(localizations.backButtonLabel, isNotNull);
  });

  testWidgets('CupertinoLocalizations.of throws', (WidgetTester tester) async {
    final GlobalKey noLocalizationsAvailable = GlobalKey();
    final GlobalKey localizationsAvailable = GlobalKey();

    await tester.pumpWidget(
      Container(
        key: noLocalizationsAvailable,
        child: CupertinoApp(home: Container(key: localizationsAvailable)),
      ),
    );

    expect(
      () => CupertinoLocalizations.of(noLocalizationsAvailable.currentContext!),
      throwsA(
        isAssertionError.having(
          (AssertionError e) => e.message,
          'message',
          contains('No CupertinoLocalizations found'),
        ),
      ),
    );

    expect(
      CupertinoLocalizations.of(localizationsAvailable.currentContext!),
      isA<CupertinoLocalizations>(),
    );
  });
}
