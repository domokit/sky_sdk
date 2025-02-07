// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:io';

import 'package:flutter/material.dart';
import 'package:flutter_localizations/flutter_localizations.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:path/path.dart' as path;

import '../test_utils.dart';

final String rootDirectoryPath = Directory.current.path;

void main() {
  for (final String language in kMaterialSupportedLanguages) {
    testWidgets('translations exist for $language', (WidgetTester tester) async {
      final Locale locale = Locale(language);

      expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);

      final MaterialLocalizations localizations = await GlobalMaterialLocalizations.delegate.load(
        locale,
      );

      expect(localizations.openAppDrawerTooltip, isNotNull);
      expect(localizations.backButtonTooltip, isNotNull);
      expect(localizations.closeButtonTooltip, isNotNull);
      expect(localizations.nextMonthTooltip, isNotNull);
      expect(localizations.previousMonthTooltip, isNotNull);
      expect(localizations.nextPageTooltip, isNotNull);
      expect(localizations.previousPageTooltip, isNotNull);
      expect(localizations.firstPageTooltip, isNotNull);
      expect(localizations.lastPageTooltip, isNotNull);
      expect(localizations.showMenuTooltip, isNotNull);
      expect(localizations.licensesPageTitle, isNotNull);
      expect(localizations.rowsPerPageTitle, isNotNull);
      expect(localizations.cancelButtonLabel, isNotNull);
      expect(localizations.closeButtonLabel, isNotNull);
      expect(localizations.continueButtonLabel, isNotNull);
      expect(localizations.copyButtonLabel, isNotNull);
      expect(localizations.cutButtonLabel, isNotNull);
      expect(localizations.okButtonLabel, isNotNull);
      expect(localizations.pasteButtonLabel, isNotNull);
      expect(localizations.selectAllButtonLabel, isNotNull);
      expect(localizations.viewLicensesButtonLabel, isNotNull);
      expect(localizations.drawerLabel, isNotNull);
      expect(localizations.popupMenuLabel, isNotNull);
      expect(localizations.dialogLabel, isNotNull);
      expect(localizations.alertDialogLabel, isNotNull);
      expect(localizations.collapsedIconTapHint, isNotNull);
      expect(localizations.expandedIconTapHint, isNotNull);
      expect(localizations.expansionTileExpandedHint, isNotNull);
      expect(localizations.expansionTileCollapsedHint, isNotNull);
      expect(localizations.collapsedHint, isNotNull);
      expect(localizations.expandedHint, isNotNull);
      expect(localizations.refreshIndicatorSemanticLabel, isNotNull);
      expect(localizations.selectedDateLabel, isNotNull);

      // Regression test for https://github.com/flutter/flutter/issues/136090
      expect(localizations.remainingTextFieldCharacterCount(0), isNot(contains('TBD')));

      expect(localizations.remainingTextFieldCharacterCount(0), isNotNull);
      expect(localizations.remainingTextFieldCharacterCount(1), isNotNull);
      expect(localizations.remainingTextFieldCharacterCount(10), isNotNull);
      expect(
        localizations.remainingTextFieldCharacterCount(0),
        isNot(contains(r'$remainingCount')),
      );
      expect(
        localizations.remainingTextFieldCharacterCount(1),
        isNot(contains(r'$remainingCount')),
      );
      expect(
        localizations.remainingTextFieldCharacterCount(10),
        isNot(contains(r'$remainingCount')),
      );

      expect(localizations.aboutListTileTitle('FOO'), isNotNull);
      expect(localizations.aboutListTileTitle('FOO'), contains('FOO'));

      expect(localizations.selectedRowCountTitle(0), isNotNull);
      expect(localizations.selectedRowCountTitle(1), isNotNull);
      expect(localizations.selectedRowCountTitle(2), isNotNull);
      expect(localizations.selectedRowCountTitle(100), isNotNull);
      expect(localizations.selectedRowCountTitle(0), isNot(contains(r'$selectedRowCount')));
      expect(localizations.selectedRowCountTitle(1), isNot(contains(r'$selectedRowCount')));
      expect(localizations.selectedRowCountTitle(2), isNot(contains(r'$selectedRowCount')));
      expect(localizations.selectedRowCountTitle(100), isNot(contains(r'$selectedRowCount')));

      expect(localizations.pageRowsInfoTitle(1, 10, 100, true), isNotNull);
      expect(localizations.pageRowsInfoTitle(1, 10, 100, false), isNotNull);
      expect(localizations.pageRowsInfoTitle(1, 10, 100, true), isNot(contains(r'$firstRow')));
      expect(localizations.pageRowsInfoTitle(1, 10, 100, true), isNot(contains(r'$lastRow')));
      expect(localizations.pageRowsInfoTitle(1, 10, 100, true), isNot(contains(r'$rowCount')));
      expect(localizations.pageRowsInfoTitle(1, 10, 100, false), isNot(contains(r'$firstRow')));
      expect(localizations.pageRowsInfoTitle(1, 10, 100, false), isNot(contains(r'$lastRow')));
      expect(localizations.pageRowsInfoTitle(1, 10, 100, false), isNot(contains(r'$rowCount')));

      expect(localizations.tabLabel(tabIndex: 2, tabCount: 5), isNotNull);
      expect(localizations.tabLabel(tabIndex: 2, tabCount: 5), isNot(contains(r'$tabIndex')));
      expect(localizations.tabLabel(tabIndex: 2, tabCount: 5), isNot(contains(r'$tabCount')));
      expect(() => localizations.tabLabel(tabIndex: 0, tabCount: 5), throwsAssertionError);
      expect(() => localizations.tabLabel(tabIndex: 2, tabCount: 0), throwsAssertionError);

      expect(localizations.formatHour(const TimeOfDay(hour: 10, minute: 0)), isNotNull);
      expect(localizations.formatMinute(const TimeOfDay(hour: 10, minute: 0)), isNotNull);
      expect(localizations.formatYear(DateTime(2018, 8)), isNotNull);
      expect(localizations.formatMediumDate(DateTime(2018, 8)), isNotNull);
      expect(localizations.formatFullDate(DateTime(2018, 8)), isNotNull);
      expect(localizations.formatMonthYear(DateTime(2018, 8)), isNotNull);
      expect(localizations.narrowWeekdays, isNotNull);
      expect(localizations.narrowWeekdays.length, 7);
      expect(localizations.formatDecimal(123), isNotNull);
      expect(localizations.formatTimeOfDay(const TimeOfDay(hour: 10, minute: 0)), isNotNull);
    });
  }

  testWidgets('translations spot check', (WidgetTester tester) async {
    Locale locale = const Locale.fromSubtags(languageCode: 'zh');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    MaterialLocalizations localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationZh>());
    expect(localizations.firstPageTooltip, '第一页');
    expect(localizations.lastPageTooltip, '最后一页');

    locale = const Locale.fromSubtags(languageCode: 'zu');
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    expect(localizations.firstPageTooltip, 'Ikhasi lokuqala');
    expect(localizations.lastPageTooltip, 'Ikhasi lokugcina');
  });

  testWidgets('translations spot check expansionTileExpandedHint', (WidgetTester tester) async {
    const Locale locale = Locale.fromSubtags(languageCode: 'en');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    final MaterialLocalizations localizations = await GlobalMaterialLocalizations.delegate.load(
      locale,
    );
    expect(localizations, isA<MaterialLocalizationEn>());
    expect(localizations.expansionTileExpandedHint, 'double tap to collapse');
  });

  testWidgets('spot check selectedRowCount translations', (WidgetTester tester) async {
    MaterialLocalizations localizations = await GlobalMaterialLocalizations.delegate.load(
      const Locale('en'),
    );
    expect(localizations.selectedRowCountTitle(0), 'No items selected');
    expect(localizations.selectedRowCountTitle(1), '1 item selected');
    expect(localizations.selectedRowCountTitle(2), '2 items selected');
    expect(localizations.selectedRowCountTitle(3), '3 items selected');
    expect(localizations.selectedRowCountTitle(5), '5 items selected');
    expect(localizations.selectedRowCountTitle(10), '10 items selected');
    expect(localizations.selectedRowCountTitle(15), '15 items selected');
    expect(localizations.selectedRowCountTitle(29), '29 items selected');
    expect(localizations.selectedRowCountTitle(10000), '10,000 items selected');
    expect(localizations.selectedRowCountTitle(10019), '10,019 items selected');
    expect(localizations.selectedRowCountTitle(123456789), '123,456,789 items selected');

    localizations = await GlobalMaterialLocalizations.delegate.load(const Locale('es'));
    expect(localizations.selectedRowCountTitle(0), 'No se han seleccionado elementos');
    expect(localizations.selectedRowCountTitle(1), '1 elemento seleccionado');
    expect(localizations.selectedRowCountTitle(2), '2 elementos seleccionados');
    expect(localizations.selectedRowCountTitle(3), '3 elementos seleccionados');
    expect(localizations.selectedRowCountTitle(5), '5 elementos seleccionados');
    expect(localizations.selectedRowCountTitle(10), '10 elementos seleccionados');
    expect(localizations.selectedRowCountTitle(15), '15 elementos seleccionados');
    expect(localizations.selectedRowCountTitle(29), '29 elementos seleccionados');
    expect(localizations.selectedRowCountTitle(10000), '10.000 elementos seleccionados');
    expect(localizations.selectedRowCountTitle(10019), '10.019 elementos seleccionados');
    expect(localizations.selectedRowCountTitle(123456789), '123.456.789 elementos seleccionados');

    localizations = await GlobalMaterialLocalizations.delegate.load(const Locale('ro'));
    expect(localizations.selectedRowCountTitle(0), 'Nu există elemente selectate');
    expect(localizations.selectedRowCountTitle(1), 'Un articol selectat');
    expect(localizations.selectedRowCountTitle(2), '2 articole selectate');
    expect(localizations.selectedRowCountTitle(3), '3 articole selectate');
    expect(localizations.selectedRowCountTitle(5), '5 articole selectate');
    expect(localizations.selectedRowCountTitle(10), '10 articole selectate');
    expect(localizations.selectedRowCountTitle(15), '15 articole selectate');
    expect(localizations.selectedRowCountTitle(29), '29 de articole selectate');
    expect(localizations.selectedRowCountTitle(10000), '10.000 de articole selectate');
    expect(localizations.selectedRowCountTitle(10019), '10.019 articole selectate');
    expect(localizations.selectedRowCountTitle(123456789), '123.456.789 de articole selectate');

    localizations = await GlobalMaterialLocalizations.delegate.load(const Locale('km'));
    expect(localizations.selectedRowCountTitle(0), 'បាន​ជ្រើស​រើស​ធាតុ 0');
    expect(localizations.selectedRowCountTitle(1), 'បាន​ជ្រើស​រើស​ធាតុ 1');
    expect(localizations.selectedRowCountTitle(2), 'បាន​ជ្រើស​រើស​ធាតុ 2');
    expect(localizations.selectedRowCountTitle(10000), 'បាន​ជ្រើស​រើស​ធាតុ 10,000');
    expect(localizations.selectedRowCountTitle(123456789), 'បាន​ជ្រើស​រើស​ធាតុ 123,456,789');
  });

  testWidgets('spot check formatMediumDate(), formatFullDate() translations', (
    WidgetTester tester,
  ) async {
    MaterialLocalizations localizations = await GlobalMaterialLocalizations.delegate.load(
      const Locale('en'),
    );
    expect(localizations.formatMediumDate(DateTime(2015, 7, 23)), 'Thu, Jul 23');
    expect(localizations.formatFullDate(DateTime(2015, 7, 23)), 'Thursday, July 23, 2015');

    localizations = await GlobalMaterialLocalizations.delegate.load(const Locale('en', 'GB'));
    expect(localizations.formatMediumDate(DateTime(2015, 7, 23)), 'Thu, 23 Jul');
    expect(localizations.formatFullDate(DateTime(2015, 7, 23)), 'Thursday, 23 July 2015');

    localizations = await GlobalMaterialLocalizations.delegate.load(const Locale('es'));
    expect(localizations.formatMediumDate(DateTime(2015, 7, 23)), 'jue, 23 jul');
    expect(localizations.formatFullDate(DateTime(2015, 7, 23)), 'jueves, 23 de julio de 2015');

    localizations = await GlobalMaterialLocalizations.delegate.load(const Locale('de'));
    expect(localizations.formatMediumDate(DateTime(2015, 7, 23)), 'Do., 23. Juli');
    expect(localizations.formatFullDate(DateTime(2015, 7, 23)), 'Donnerstag, 23. Juli 2015');
  });

  testWidgets('Chinese resolution', (WidgetTester tester) async {
    Locale locale = const Locale.fromSubtags(languageCode: 'zh');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    MaterialLocalizations localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationZh>());

    locale = const Locale('zh', 'TW');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationZhHantTw>());

    locale = const Locale.fromSubtags(languageCode: 'zh', countryCode: 'HK');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationZhHantHk>());

    locale = const Locale.fromSubtags(languageCode: 'zh', countryCode: 'TW');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationZhHantTw>());

    locale = const Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Hans');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationZhHans>());

    locale = const Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Hant');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationZhHant>());

    locale = const Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Hans', countryCode: 'US');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationZhHans>());

    locale = const Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Hans', countryCode: 'CN');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationZhHans>());

    locale = const Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Hans', countryCode: 'TW');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationZhHans>());

    locale = const Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Hant', countryCode: 'CN');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationZhHant>());

    locale = const Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Hant', countryCode: 'US');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationZhHant>());

    locale = const Locale.fromSubtags(languageCode: 'zh', countryCode: 'US');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationZh>());

    locale = const Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Latn', countryCode: 'US');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationZh>());

    locale = const Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Latn', countryCode: 'TW');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationZh>());

    locale = const Locale.fromSubtags(languageCode: 'en', countryCode: 'TW');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationEn>());

    locale = const Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Cyrl', countryCode: 'RU');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationZh>());

    locale = const Locale.fromSubtags(languageCode: 'zh', countryCode: 'RU');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationZh>());

    locale = const Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Cyrl');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationZh>());
  });

  testWidgets('Serbian resolution', (WidgetTester tester) async {
    Locale locale = const Locale.fromSubtags(languageCode: 'sr');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    MaterialLocalizations localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationSr>());

    locale = const Locale.fromSubtags(languageCode: 'sr', scriptCode: 'Cyrl');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationSrCyrl>());

    locale = const Locale.fromSubtags(languageCode: 'sr', scriptCode: 'Latn');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationSrLatn>());

    locale = const Locale.fromSubtags(languageCode: 'sr', countryCode: 'SR');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationSr>());

    locale = const Locale.fromSubtags(languageCode: 'sr', scriptCode: 'Cyrl', countryCode: 'SR');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationSrCyrl>());

    locale = const Locale.fromSubtags(languageCode: 'sr', scriptCode: 'Latn', countryCode: 'SR');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationSrLatn>());

    locale = const Locale.fromSubtags(languageCode: 'sr', scriptCode: 'Cyrl', countryCode: 'US');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationSrCyrl>());

    locale = const Locale.fromSubtags(languageCode: 'sr', scriptCode: 'Latn', countryCode: 'US');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationSrLatn>());

    locale = const Locale.fromSubtags(languageCode: 'sr', countryCode: 'US');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationSr>());
  });

  testWidgets('Misc resolution', (WidgetTester tester) async {
    Locale locale = const Locale.fromSubtags(languageCode: 'en');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    MaterialLocalizations localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationEn>());

    locale = const Locale.fromSubtags(languageCode: 'en', scriptCode: 'Cyrl');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationEn>());

    locale = const Locale.fromSubtags(languageCode: 'en', countryCode: 'US');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationEn>());

    locale = const Locale.fromSubtags(languageCode: 'en', countryCode: 'AU');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationEnAu>());

    locale = const Locale.fromSubtags(languageCode: 'en', countryCode: 'GB');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationEnGb>());

    locale = const Locale.fromSubtags(languageCode: 'en', countryCode: 'SG');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationEnSg>());

    locale = const Locale.fromSubtags(languageCode: 'en', countryCode: 'MX');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationEn>());

    locale = const Locale.fromSubtags(languageCode: 'en', scriptCode: 'Hant');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationEn>());

    locale = const Locale.fromSubtags(languageCode: 'en', scriptCode: 'Hant', countryCode: 'US');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationEn>());

    locale = const Locale.fromSubtags(languageCode: 'en', scriptCode: 'Hans', countryCode: 'CN');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationEn>());

    locale = const Locale.fromSubtags(languageCode: 'es');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationEs>());

    locale = const Locale.fromSubtags(languageCode: 'es', countryCode: '419');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationEs419>());

    locale = const Locale.fromSubtags(languageCode: 'es', countryCode: 'MX');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationEsMx>());

    locale = const Locale.fromSubtags(languageCode: 'es', countryCode: 'US');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationEsUs>());

    locale = const Locale.fromSubtags(languageCode: 'es', countryCode: 'AR');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationEsAr>());

    locale = const Locale.fromSubtags(languageCode: 'es', countryCode: 'ES');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationEs>());

    locale = const Locale.fromSubtags(languageCode: 'es', scriptCode: 'Latn');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationEs>());

    locale = const Locale.fromSubtags(languageCode: 'es', scriptCode: 'Latn', countryCode: 'US');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationEsUs>());

    locale = const Locale.fromSubtags(languageCode: 'fr');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationFr>());

    locale = const Locale.fromSubtags(languageCode: 'fr', countryCode: 'CA');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationFrCa>());

    locale = const Locale.fromSubtags(languageCode: 'de');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationDe>());

    locale = const Locale.fromSubtags(languageCode: 'de', countryCode: 'CH');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationDeCh>());

    locale = const Locale.fromSubtags(languageCode: 'th');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationTh>());

    locale = const Locale.fromSubtags(languageCode: 'ru');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationRu>());
  });

  testWidgets('Chinese translations spot check', (WidgetTester tester) async {
    Locale locale = const Locale.fromSubtags(languageCode: 'zh');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    MaterialLocalizations localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationZh>());
    expect(localizations.alertDialogLabel, '提醒');
    expect(localizations.anteMeridiemAbbreviation, '上午');
    expect(localizations.closeButtonLabel, '关闭');
    expect(localizations.okButtonLabel, '确定');

    locale = const Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Hans');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationZhHans>());
    expect(localizations.alertDialogLabel, '提醒');
    expect(localizations.anteMeridiemAbbreviation, '上午');
    expect(localizations.closeButtonLabel, '关闭');
    expect(localizations.okButtonLabel, '确定');

    locale = const Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Hant');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationZhHant>());
    expect(localizations.alertDialogLabel, '通知');
    expect(localizations.anteMeridiemAbbreviation, '上午');
    expect(localizations.closeButtonLabel, '關閉');
    expect(localizations.okButtonLabel, '確定');

    locale = const Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Hant', countryCode: 'TW');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationZhHantTw>());
    expect(localizations.alertDialogLabel, '快訊');
    expect(localizations.anteMeridiemAbbreviation, '上午');
    expect(localizations.closeButtonLabel, '關閉');
    expect(localizations.okButtonLabel, '確定');

    locale = const Locale.fromSubtags(languageCode: 'zh', scriptCode: 'Hant', countryCode: 'HK');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    localizations = await GlobalMaterialLocalizations.delegate.load(locale);
    expect(localizations, isA<MaterialLocalizationZhHantHk>());
    expect(localizations.alertDialogLabel, '通知');
    expect(localizations.anteMeridiemAbbreviation, '上午');
    expect(localizations.closeButtonLabel, '關閉');
    expect(localizations.okButtonLabel, '確定');
  });

  // Regression test for https://github.com/flutter/flutter/issues/36704.
  testWidgets('kn arb file should be properly Unicode escaped', (WidgetTester tester) async {
    final File file = File(path.join(rootDirectoryPath, 'lib', 'src', 'l10n', 'material_kn.arb'));

    final Map<String, dynamic> bundle =
        json.decode(file.readAsStringSync()) as Map<String, dynamic>;

    // Encodes the arb resource values if they have not already been
    // encoded.
    encodeBundleTranslations(bundle);

    // Generates the encoded arb output file in as a string.
    final String encodedArbFile = generateArbString(bundle);

    // After encoding the bundles, the generated string should match
    // the existing material_kn.arb.
    if (Platform.isWindows) {
      // On Windows, the character '\n' can output the two-character sequence
      // '\r\n' (and when reading the file back, '\r\n' is translated back
      // into a single '\n' character).
      expect(file.readAsStringSync().replaceAll('\r\n', '\n'), encodedArbFile);
    } else {
      expect(file.readAsStringSync(), encodedArbFile);
    }
  });

  // Regression test for https://github.com/flutter/flutter/issues/110451.
  testWidgets('Finnish translation for tab label', (WidgetTester tester) async {
    const Locale locale = Locale('fi');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    final MaterialLocalizations localizations = await GlobalMaterialLocalizations.delegate.load(
      locale,
    );
    expect(localizations, isA<MaterialLocalizationFi>());
    expect(localizations.tabLabel(tabIndex: 1, tabCount: 2), 'Välilehti 1 kautta 2');
  });

  // Regression test for https://github.com/flutter/flutter/issues/138728.
  testWidgets('Share button label on Material', (WidgetTester tester) async {
    const Locale locale = Locale('en');
    expect(GlobalMaterialLocalizations.delegate.isSupported(locale), isTrue);
    final MaterialLocalizations localizations = await GlobalMaterialLocalizations.delegate.load(
      locale,
    );
    expect(localizations, isA<MaterialLocalizationEn>());
    expect(localizations.shareButtonLabel, 'Share');
  });

  // Regression test for https://github.com/flutter/flutter/issues/141764
  testWidgets('zh-CN translation for look up label', (WidgetTester tester) async {
    const Locale locale = Locale('zh');
    expect(GlobalCupertinoLocalizations.delegate.isSupported(locale), isTrue);
    final MaterialLocalizations localizations = await GlobalMaterialLocalizations.delegate.load(
      locale,
    );
    expect(localizations, isA<MaterialLocalizationZh>());
    expect(localizations.lookUpButtonLabel, '查询');
  });

  // Regression test for https://github.com/flutter/flutter/pull/151364
  testWidgets('ko-KR translation for cut, copy, paste label in ButtonLabel', (
    WidgetTester tester,
  ) async {
    const Locale locale = Locale('ko');
    expect(GlobalCupertinoLocalizations.delegate.isSupported(locale), isTrue);
    final MaterialLocalizations localizations = await GlobalMaterialLocalizations.delegate.load(
      locale,
    );
    expect(localizations, isA<MaterialLocalizationKo>());
    expect(localizations.cutButtonLabel, '잘라내기');
    expect(localizations.copyButtonLabel, '복사');
    expect(localizations.pasteButtonLabel, '붙여넣기');
  });

  // Regression test for https://github.com/flutter/flutter/issues/156954
  testWidgets('Italian translation for dateHelpText', (WidgetTester tester) async {
    const Locale locale = Locale('it');
    expect(GlobalCupertinoLocalizations.delegate.isSupported(locale), isTrue);
    final MaterialLocalizations localizations = await GlobalMaterialLocalizations.delegate.load(
      locale,
    );
    expect(localizations, isA<MaterialLocalizationIt>());
    expect(localizations.dateHelpText, 'gg/mm/aaaa');
  });
}
