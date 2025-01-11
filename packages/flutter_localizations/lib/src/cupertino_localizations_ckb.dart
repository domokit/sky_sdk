// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter/cupertino.dart';
import 'package:intl/intl.dart' as intl;

import 'cupertino_localizations.dart';
import 'utils/date_localizations.dart' as util;

/// A custom number format for Kurdish Sorani that converts Western digits to Eastern Arabic digits.
class _KurdishSoraniNumberFormat implements intl.NumberFormat {
  const _KurdishSoraniNumberFormat();

  static const _KurdishSoraniNumberFormat instance = _KurdishSoraniNumberFormat();

  static const Map<String, String> _westernToEasternDigits = <String, String>{
    '0': '٠',
    '1': '١',
    '2': '٢',
    '3': '٣',
    '4': '٤',
    '5': '٥',
    '6': '٦',
    '7': '٧',
    '8': '٨',
    '9': '٩',
  };

  @override
  String format(dynamic number) {
    final String westernStr = number.toString();
    final StringBuffer sb = StringBuffer();
    for (int i = 0; i < westernStr.length; i++) {
      final String digit = westernStr[i];
      final String? eastern = _westernToEasternDigits[digit];
      sb.write(eastern ?? digit);
    }
    return sb.toString();
  }

  @override
  dynamic noSuchMethod(Invocation invocation) => super.noSuchMethod(invocation);
}

/// A custom date format for Kurdish Sorani that uses Eastern Arabic digits.
class _KurdishSoraniDateFormat extends intl.DateFormat {
  _KurdishSoraniDateFormat(String pattern, String _) : super(pattern, 'ar');

  @override
  String format(DateTime date) {
    if (pattern == 'HH' || pattern == 'H') {
      // Convert to 12-hour format for hour display
      final int hour = date.hour > 12 ? date.hour - 12 : (date.hour == 0 ? 12 : date.hour);
      return _KurdishSoraniNumberFormat.instance.format(hour);
    }
    final String formatted = super.format(date);
    return _KurdishSoraniNumberFormat.instance.format(formatted);
  }
}

/// Cupertino Localizations for Kurdish (Sorani).
///
/// See also:
///
///  * [GlobalCupertinoLocalizations], which provides cupertino localizations for
///    many languages.
///  * [CupertinoLocalizations], which provides localizations for Cupertino widgets.
class CupertinoLocalizationCkb extends GlobalCupertinoLocalizations {
  /// Create an instance of the translation bundle for Kurdish (Sorani).
  factory CupertinoLocalizationCkb({
    String localeName = 'ckb',
    required intl.DateFormat fullYearFormat,
    required intl.DateFormat dayFormat,
    required intl.DateFormat mediumDateFormat,
    required intl.DateFormat singleDigitHourFormat,
    required intl.DateFormat singleDigitMinuteFormat,
    required intl.DateFormat doubleDigitMinuteFormat,
    required intl.DateFormat singleDigitSecondFormat,
    required intl.NumberFormat decimalFormat,
    required intl.DateFormat weekdayFormat,
  }) {
    return CupertinoLocalizationCkb._(
      localeName: localeName,
      fullYearFormat: _KurdishSoraniDateFormat(fullYearFormat.pattern!, localeName),
      dayFormat: _KurdishSoraniDateFormat(dayFormat.pattern!, localeName),
      mediumDateFormat: _KurdishSoraniDateFormat(mediumDateFormat.pattern!, localeName),
      singleDigitHourFormat: _KurdishSoraniDateFormat(singleDigitHourFormat.pattern!, localeName),
      singleDigitMinuteFormat: _KurdishSoraniDateFormat(
        singleDigitMinuteFormat.pattern!,
        localeName,
      ),
      doubleDigitMinuteFormat: _KurdishSoraniDateFormat(
        doubleDigitMinuteFormat.pattern!,
        localeName,
      ),
      singleDigitSecondFormat: _KurdishSoraniDateFormat(
        singleDigitSecondFormat.pattern!,
        localeName,
      ),
      decimalFormat: _KurdishSoraniNumberFormat.instance,
      weekdayFormat: _KurdishSoraniDateFormat(weekdayFormat.pattern!, localeName),
    );
  }

  const CupertinoLocalizationCkb._({
    required super.localeName,
    required super.fullYearFormat,
    required super.dayFormat,
    required super.mediumDateFormat,
    required super.singleDigitHourFormat,
    required super.singleDigitMinuteFormat,
    required super.doubleDigitMinuteFormat,
    required super.singleDigitSecondFormat,
    required super.weekdayFormat,
    required intl.NumberFormat decimalFormat,
  }) : super(decimalFormat: _KurdishSoraniNumberFormat.instance);

  /// The text direction for Kurdish Sorani, which is right-to-left.
  @override
  TextDirection get textDirection => TextDirection.rtl;

  @override
  String get alertDialogLabel => 'ئاگاداکردنەوە';

  @override
  String get anteMeridiemAbbreviation => 'پ.ن';

  @override
  String get copyButtonLabel => 'لەبەرگرتنەوە';

  @override
  String get cutButtonLabel => 'بڕین';

  @override
  String get datePickerDateOrderString => 'dmy';

  @override
  String get datePickerDateTimeOrderString => 'date_time_dayPeriod';

  @override
  String get datePickerHourSemanticsLabelOne => r'کاتژمێر $hour';

  @override
  String get datePickerHourSemanticsLabelOther => r'کاتژمێر $hour';

  @override
  String get datePickerMinuteSemanticsLabelOne => r'$minute خولەک';

  @override
  String get datePickerMinuteSemanticsLabelOther => r'$minute خولەک';

  @override
  String get modalBarrierDismissLabel => 'داخستن';

  @override
  String get pasteButtonLabel => 'لکاندن';

  @override
  String get postMeridiemAbbreviation => 'د.ن';

  @override
  String get searchTextFieldPlaceholderLabel => 'گەڕان';

  @override
  String get selectAllButtonLabel => 'هەموو هەڵبژێرە';

  @override
  String get tabSemanticsLabelRaw => r'تابی $tabIndex لە $tabCount';

  @override
  String get timerPickerHourLabelOne => 'کاتژمێر';

  @override
  String get timerPickerHourLabelOther => 'کاتژمێر';

  @override
  String get timerPickerMinuteLabelOne => 'خولەک';

  @override
  String get timerPickerMinuteLabelOther => 'خولەک';

  @override
  String get timerPickerSecondLabelOne => 'چرکە';

  @override
  String get timerPickerSecondLabelOther => 'چرکە';

  @override
  String get todayLabel => 'ئەمڕۆ';

  @override
  String get noSpellCheckReplacementsLabel => 'هیچ جێگرەوەیەک نەدۆزرایەوە';

  @override
  String get clearButtonLabel => 'سڕینەوە';

  @override
  String get lookUpButtonLabel => 'گەڕان';

  @override
  String get searchWebButtonLabel => 'گەڕان لە وێب';

  @override
  String get shareButtonLabel => 'هاوبەشکردن';

  @override
  String get menuDismissLabel => 'داخستنی مێنیو';

  @override
  String datePickerYear(int yearIndex) {
    return _KurdishSoraniNumberFormat.instance.format(yearIndex);
  }

  @override
  String datePickerMonth(int monthIndex) {
    return _KurdishSoraniNumberFormat.instance.format(monthIndex);
  }

  @override
  String datePickerDayOfMonth(int dayIndex, [int? weekDay]) {
    return _KurdishSoraniNumberFormat.instance.format(dayIndex);
  }

  @override
  String datePickerHour(int hour) {
    // Convert to 12-hour format and format using Kurdish Sorani numbers
    final int displayHour = hour > 12 ? hour - 12 : (hour == 0 ? 12 : hour);
    final String formattedHour = _KurdishSoraniNumberFormat.instance.format(displayHour);
    return 'کاتژمێر $formattedHour';
  }

  @override
  String datePickerHourSemanticsLabel(int hour) {
    final String hourString = _KurdishSoraniNumberFormat.instance.format(hour);
    return 'کاتژمێر $hourString';
  }

  @override
  String datePickerMinute(int minute) {
    return '${_KurdishSoraniNumberFormat.instance.format(minute)} خولەک';
  }

  @override
  String datePickerMinuteSemanticsLabel(int minute) {
    return '${_KurdishSoraniNumberFormat.instance.format(minute)} خولەک';
  }

  @override
  String timerPickerHour(int hour) {
    return _KurdishSoraniNumberFormat.instance.format(hour);
  }

  @override
  String timerPickerMinute(int minute) {
    return _KurdishSoraniNumberFormat.instance.format(minute);
  }

  @override
  String timerPickerSecond(int second) {
    return _KurdishSoraniNumberFormat.instance.format(second);
  }

  @override
  String tabSemanticsLabel({required int tabIndex, required int tabCount}) {
    return 'تابی ${_KurdishSoraniNumberFormat.instance.format(tabIndex)} لە ${_KurdishSoraniNumberFormat.instance.format(tabCount)}';
  }
}

/// A [LocalizationsDelegate] for Kurdish Sorani Cupertino localizations.
///
/// Most applications will use [GlobalCupertinoLocalizations.delegate] as the
/// [LocalizationsDelegate] for the locale specified in [GlobalCupertinoLocalizations].
///
/// See also:
///
///  * [CupertinoLocalizationCkb], which contains the default translations.
///  * [CupertinoLocalizations], which provides localizations for Cupertino widgets.
class KurdishSoraniCupertinoLocalizations extends LocalizationsDelegate<CupertinoLocalizations> {
  /// Creates a delegate instance.
  const KurdishSoraniCupertinoLocalizations();

  @override
  bool isSupported(Locale locale) => locale.languageCode == 'ckb';

  @override
  Future<CupertinoLocalizations> load(Locale locale) async {
    util.loadDateIntlDataIfNotLoaded();

    final String localeName = intl.Intl.canonicalizedLocale(locale.toString());
    assert(
      locale.toString() == localeName,
      'Flutter does not support the non-standard locale form $locale (which '
      'might be $localeName',
    );

    late intl.DateFormat fullYearFormat;
    late intl.DateFormat dayFormat;
    late intl.DateFormat weekdayFormat;
    late intl.DateFormat mediumDateFormat;
    late intl.DateFormat singleDigitHourFormat;
    late intl.DateFormat singleDigitMinuteFormat;
    late intl.DateFormat doubleDigitMinuteFormat;
    late intl.DateFormat singleDigitSecondFormat;
    late intl.NumberFormat decimalFormat;

    // Use Arabic as a fallback since it's closely related
    const String fallbackLocale = 'ar';
    void loadFormats(String locale) {
      fullYearFormat = intl.DateFormat.y(locale);
      dayFormat = intl.DateFormat.d(locale);
      weekdayFormat = intl.DateFormat.E(locale);
      mediumDateFormat = intl.DateFormat.MMMEd(locale);
      singleDigitHourFormat = intl.DateFormat('HH', locale);
      singleDigitMinuteFormat = intl.DateFormat.m(locale);
      doubleDigitMinuteFormat = intl.DateFormat('mm', locale);
      singleDigitSecondFormat = intl.DateFormat.s(locale);
      decimalFormat = intl.NumberFormat.decimalPattern(locale);
    }

    loadFormats(fallbackLocale);

    return CupertinoLocalizationCkb(
      localeName: localeName,
      fullYearFormat: fullYearFormat,
      dayFormat: dayFormat,
      mediumDateFormat: mediumDateFormat,
      singleDigitHourFormat: singleDigitHourFormat,
      singleDigitMinuteFormat: singleDigitMinuteFormat,
      doubleDigitMinuteFormat: doubleDigitMinuteFormat,
      singleDigitSecondFormat: singleDigitSecondFormat,
      decimalFormat: decimalFormat,
      weekdayFormat: weekdayFormat,
    );
  }

  @override
  bool shouldReload(KurdishSoraniCupertinoLocalizations old) => false;

  @override
  String toString() => 'KurdishSoraniCupertinoLocalizations.delegate';
}
