// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:intl/intl.dart';
library;

import 'package:flutter/cupertino.dart';
import 'package:flutter/foundation.dart';
import 'package:intl/intl.dart' as intl;

import 'l10n/generated_cupertino_localizations.dart';
import 'utils/date_localizations.dart' as util;
import 'widgets_localizations.dart';

// Examples can assume:
// import 'package:flutter_localizations/flutter_localizations.dart';
// import 'package:flutter/cupertino.dart';

/// Implementation of localized strings for Cupertino widgets using the `intl`
/// package for date and time formatting.
///
/// Further localization of strings beyond date time formatting are provided
/// by language specific subclasses of [GlobalCupertinoLocalizations].
///
/// ## Supported languages
///
/// This class supports locales with the following [Locale.languageCode]s:
///
/// {@macro flutter.localizations.cupertino.languages}
///
/// This list is available programmatically via [kCupertinoSupportedLanguages].
///
/// ## Sample code
///
/// To include the localizations provided by this class in a [CupertinoApp],
/// add [GlobalCupertinoLocalizations.delegates] to
/// [CupertinoApp.localizationsDelegates], and specify the locales your
/// app supports with [CupertinoApp.supportedLocales]:
///
/// ```dart
/// const CupertinoApp(
///   localizationsDelegates: GlobalCupertinoLocalizations.delegates,
///   supportedLocales: <Locale>[
///     Locale('en', 'US'), // American English
///     Locale('he', 'IL'), // Israeli Hebrew
///     // ...
///   ],
///   // ...
/// )
/// ```
///
/// See also:
///
///  * [DefaultCupertinoLocalizations], which provides US English localizations
///    for Cupertino widgets.
abstract class GlobalCupertinoLocalizations implements CupertinoLocalizations {
  /// Initializes an object that defines the Cupertino widgets' localized
  /// strings for the given `localeName`.
  ///
  /// The remaining '*Format' arguments uses the intl package to provide
  /// [DateFormat] configurations for the `localeName`.
  const GlobalCupertinoLocalizations({
    required String localeName,
    required intl.DateFormat fullYearFormat,
    required intl.DateFormat dayFormat,
    required intl.DateFormat weekdayFormat,
    required intl.DateFormat mediumDateFormat,
    required intl.DateFormat singleDigitHourFormat,
    required intl.DateFormat singleDigitMinuteFormat,
    required intl.DateFormat doubleDigitMinuteFormat,
    required intl.DateFormat singleDigitSecondFormat,
    required intl.NumberFormat decimalFormat,
  }) : _localeName = localeName,
       _fullYearFormat = fullYearFormat,
       _dayFormat = dayFormat,
       _weekdayFormat = weekdayFormat,
       _mediumDateFormat = mediumDateFormat,
       _singleDigitHourFormat = singleDigitHourFormat,
       _singleDigitMinuteFormat = singleDigitMinuteFormat,
       _doubleDigitMinuteFormat = doubleDigitMinuteFormat,
       _singleDigitSecondFormat = singleDigitSecondFormat,
       _decimalFormat = decimalFormat;

  final String _localeName;
  final intl.DateFormat _fullYearFormat;
  final intl.DateFormat _dayFormat;
  final intl.DateFormat _weekdayFormat;
  final intl.DateFormat _mediumDateFormat;
  final intl.DateFormat _singleDigitHourFormat;
  final intl.DateFormat _singleDigitMinuteFormat;
  final intl.DateFormat _doubleDigitMinuteFormat;
  final intl.DateFormat _singleDigitSecondFormat;
  final intl.NumberFormat _decimalFormat;

  @override
  String datePickerYear(int yearIndex) {
    return _fullYearFormat.format(DateTime.utc(yearIndex));
  }

  @override
  String datePickerMonth(int monthIndex) {
    // It doesn't actually have anything to do with _fullYearFormat. It's just
    // taking advantage of the fact that _fullYearFormat loaded the needed
    // locale's symbols.
    return _fullYearFormat.dateSymbols.MONTHS[monthIndex - 1];
  }

  @override
  String datePickerStandaloneMonth(int monthIndex) {
    // It doesn't actually have anything to do with _fullYearFormat. It's just
    // taking advantage of the fact that _fullYearFormat loaded the needed
    // locale's symbols.
    //
    // Because this will be used without specifying any day of month,
    // in most cases it should be capitalized (according to rules in specific language).
    return intl.toBeginningOfSentenceCase(
          _fullYearFormat.dateSymbols.STANDALONEMONTHS[monthIndex - 1],
        ) ??
        _fullYearFormat.dateSymbols.STANDALONEMONTHS[monthIndex - 1];
  }

  @override
  String datePickerDayOfMonth(int dayIndex, [int? weekDay]) {
    return weekDay != null
        ? '${_weekdayFormat.format(DateTime.utc(1, 1, weekDay))} ${_dayFormat.format(DateTime.utc(1, 1, dayIndex))}'
        // Year and month doesn't matter since we just want to day formatted.
        : _dayFormat.format(DateTime.utc(0, 0, dayIndex));
  }

  @override
  String datePickerMediumDate(DateTime date) {
    return _mediumDateFormat.format(date);
  }

  @override
  String datePickerHour(int hour) {
    return _singleDigitHourFormat.format(DateTime.utc(0, 0, 0, hour));
  }

  @override
  String datePickerMinute(int minute) {
    return _doubleDigitMinuteFormat.format(DateTime.utc(0, 0, 0, 0, minute));
  }

  /// Subclasses should provide the optional zero pluralization of [datePickerHourSemanticsLabel] based on the ARB file.
  @protected
  String? get datePickerHourSemanticsLabelZero => null;

  /// Subclasses should provide the optional one pluralization of [datePickerHourSemanticsLabel] based on the ARB file.
  @protected
  String? get datePickerHourSemanticsLabelOne => null;

  /// Subclasses should provide the optional two pluralization of [datePickerHourSemanticsLabel] based on the ARB file.
  @protected
  String? get datePickerHourSemanticsLabelTwo => null;

  /// Subclasses should provide the optional few pluralization of [datePickerHourSemanticsLabel] based on the ARB file.
  @protected
  String? get datePickerHourSemanticsLabelFew => null;

  /// Subclasses should provide the optional many pluralization of [datePickerHourSemanticsLabel] based on the ARB file.
  @protected
  String? get datePickerHourSemanticsLabelMany => null;

  /// Subclasses should provide the required other pluralization of [datePickerHourSemanticsLabel] based on the ARB file.
  @protected
  String? get datePickerHourSemanticsLabelOther;

  @override
  String? datePickerHourSemanticsLabel(int hour) {
    return intl.Intl.pluralLogic(
      hour,
      zero: datePickerHourSemanticsLabelZero,
      one: datePickerHourSemanticsLabelOne,
      two: datePickerHourSemanticsLabelTwo,
      few: datePickerHourSemanticsLabelFew,
      many: datePickerHourSemanticsLabelMany,
      other: datePickerHourSemanticsLabelOther,
      locale: _localeName,
    )?.replaceFirst(r'$hour', _decimalFormat.format(hour));
  }

  /// Subclasses should provide the optional zero pluralization of [datePickerMinuteSemanticsLabel] based on the ARB file.
  @protected
  String? get datePickerMinuteSemanticsLabelZero => null;

  /// Subclasses should provide the optional one pluralization of [datePickerMinuteSemanticsLabel] based on the ARB file.
  @protected
  String? get datePickerMinuteSemanticsLabelOne => null;

  /// Subclasses should provide the optional two pluralization of [datePickerMinuteSemanticsLabel] based on the ARB file.
  @protected
  String? get datePickerMinuteSemanticsLabelTwo => null;

  /// Subclasses should provide the optional few pluralization of [datePickerMinuteSemanticsLabel] based on the ARB file.
  @protected
  String? get datePickerMinuteSemanticsLabelFew => null;

  /// Subclasses should provide the optional many pluralization of [datePickerMinuteSemanticsLabel] based on the ARB file.
  @protected
  String? get datePickerMinuteSemanticsLabelMany => null;

  /// Subclasses should provide the required other pluralization of [datePickerMinuteSemanticsLabel] based on the ARB file.
  @protected
  String? get datePickerMinuteSemanticsLabelOther;

  @override
  String? datePickerMinuteSemanticsLabel(int minute) {
    return intl.Intl.pluralLogic(
      minute,
      zero: datePickerMinuteSemanticsLabelZero,
      one: datePickerMinuteSemanticsLabelOne,
      two: datePickerMinuteSemanticsLabelTwo,
      few: datePickerMinuteSemanticsLabelFew,
      many: datePickerMinuteSemanticsLabelMany,
      other: datePickerMinuteSemanticsLabelOther,
      locale: _localeName,
    )?.replaceFirst(r'$minute', _decimalFormat.format(minute));
  }

  /// A string describing the [DatePickerDateOrder] enum value.
  ///
  /// Subclasses should provide this string value based on the ARB file for
  /// the locale.
  ///
  /// See also:
  ///
  ///  * [datePickerDateOrder], which provides the [DatePickerDateOrder]
  ///    enum value for [CupertinoLocalizations] based on this string value
  @protected
  String get datePickerDateOrderString;

  @override
  DatePickerDateOrder get datePickerDateOrder {
    switch (datePickerDateOrderString) {
      case 'dmy':
        return DatePickerDateOrder.dmy;
      case 'mdy':
        return DatePickerDateOrder.mdy;
      case 'ymd':
        return DatePickerDateOrder.ymd;
      case 'ydm':
        return DatePickerDateOrder.ydm;
      default:
        assert(
          false,
          'Failed to load DatePickerDateOrder $datePickerDateOrderString for '
          "locale $_localeName.\nNon conforming string for $_localeName's "
          '.arb file',
        );
        return DatePickerDateOrder.mdy;
    }
  }

  /// A string describing the [DatePickerDateTimeOrder] enum value.
  ///
  /// Subclasses should provide this string value based on the ARB file for
  /// the locale.
  ///
  /// See also:
  ///
  ///  * [datePickerDateTimeOrder], which provides the [DatePickerDateTimeOrder]
  ///    enum value for [CupertinoLocalizations] based on this string value.
  @protected
  String get datePickerDateTimeOrderString;

  @override
  DatePickerDateTimeOrder get datePickerDateTimeOrder {
    switch (datePickerDateTimeOrderString) {
      case 'date_time_dayPeriod':
        return DatePickerDateTimeOrder.date_time_dayPeriod;
      case 'date_dayPeriod_time':
        return DatePickerDateTimeOrder.date_dayPeriod_time;
      case 'time_dayPeriod_date':
        return DatePickerDateTimeOrder.time_dayPeriod_date;
      case 'dayPeriod_time_date':
        return DatePickerDateTimeOrder.dayPeriod_time_date;
      default:
        assert(
          false,
          'Failed to load DatePickerDateTimeOrder $datePickerDateTimeOrderString '
          "for locale $_localeName.\nNon conforming string for $_localeName's "
          '.arb file',
        );
        return DatePickerDateTimeOrder.date_time_dayPeriod;
    }
  }

  /// The raw version of [tabSemanticsLabel], with `$tabIndex` and `$tabCount` verbatim
  /// in the string.
  @protected
  String get tabSemanticsLabelRaw;

  @override
  String tabSemanticsLabel({required int tabIndex, required int tabCount}) {
    assert(tabIndex >= 1);
    assert(tabCount >= 1);
    final String template = tabSemanticsLabelRaw;
    return template
        .replaceFirst(r'$tabIndex', _decimalFormat.format(tabIndex))
        .replaceFirst(r'$tabCount', _decimalFormat.format(tabCount));
  }

  @override
  String timerPickerHour(int hour) {
    return _singleDigitHourFormat.format(DateTime.utc(0, 0, 0, hour));
  }

  @override
  String timerPickerMinute(int minute) {
    return _singleDigitMinuteFormat.format(DateTime.utc(0, 0, 0, 0, minute));
  }

  @override
  String timerPickerSecond(int second) {
    return _singleDigitSecondFormat.format(DateTime.utc(0, 0, 0, 0, 0, second));
  }

  /// Subclasses should provide the optional zero pluralization of [timerPickerHourLabel] based on the ARB file.
  @protected
  String? get timerPickerHourLabelZero => null;

  /// Subclasses should provide the optional one pluralization of [timerPickerHourLabel] based on the ARB file.
  @protected
  String? get timerPickerHourLabelOne => null;

  /// Subclasses should provide the optional two pluralization of [timerPickerHourLabel] based on the ARB file.
  @protected
  String? get timerPickerHourLabelTwo => null;

  /// Subclasses should provide the optional few pluralization of [timerPickerHourLabel] based on the ARB file.
  @protected
  String? get timerPickerHourLabelFew => null;

  /// Subclasses should provide the optional many pluralization of [timerPickerHourLabel] based on the ARB file.
  @protected
  String? get timerPickerHourLabelMany => null;

  /// Subclasses should provide the required other pluralization of [timerPickerHourLabel] based on the ARB file.
  @protected
  String? get timerPickerHourLabelOther;

  @override
  String? timerPickerHourLabel(int hour) {
    return intl.Intl.pluralLogic(
      hour,
      zero: timerPickerHourLabelZero,
      one: timerPickerHourLabelOne,
      two: timerPickerHourLabelTwo,
      few: timerPickerHourLabelFew,
      many: timerPickerHourLabelMany,
      other: timerPickerHourLabelOther,
      locale: _localeName,
    )?.replaceFirst(r'$hour', _decimalFormat.format(hour));
  }

  @override
  List<String> get timerPickerHourLabels => <String>[
    if (timerPickerHourLabelZero != null) timerPickerHourLabelZero!,
    if (timerPickerHourLabelOne != null) timerPickerHourLabelOne!,
    if (timerPickerHourLabelTwo != null) timerPickerHourLabelTwo!,
    if (timerPickerHourLabelFew != null) timerPickerHourLabelFew!,
    if (timerPickerHourLabelMany != null) timerPickerHourLabelMany!,
    if (timerPickerHourLabelOther != null) timerPickerHourLabelOther!,
  ];

  /// Subclasses should provide the optional zero pluralization of [timerPickerMinuteLabel] based on the ARB file.
  @protected
  String? get timerPickerMinuteLabelZero => null;

  /// Subclasses should provide the optional one pluralization of [timerPickerMinuteLabel] based on the ARB file.
  @protected
  String? get timerPickerMinuteLabelOne => null;

  /// Subclasses should provide the optional two pluralization of [timerPickerMinuteLabel] based on the ARB file.
  @protected
  String? get timerPickerMinuteLabelTwo => null;

  /// Subclasses should provide the optional few pluralization of [timerPickerMinuteLabel] based on the ARB file.
  @protected
  String? get timerPickerMinuteLabelFew => null;

  /// Subclasses should provide the optional many pluralization of [timerPickerMinuteLabel] based on the ARB file.
  @protected
  String? get timerPickerMinuteLabelMany => null;

  /// Subclasses should provide the required other pluralization of [timerPickerMinuteLabel] based on the ARB file.
  @protected
  String? get timerPickerMinuteLabelOther;

  @override
  String? timerPickerMinuteLabel(int minute) {
    return intl.Intl.pluralLogic(
      minute,
      zero: timerPickerMinuteLabelZero,
      one: timerPickerMinuteLabelOne,
      two: timerPickerMinuteLabelTwo,
      few: timerPickerMinuteLabelFew,
      many: timerPickerMinuteLabelMany,
      other: timerPickerMinuteLabelOther,
      locale: _localeName,
    )?.replaceFirst(r'$minute', _decimalFormat.format(minute));
  }

  @override
  List<String> get timerPickerMinuteLabels => <String>[
    if (timerPickerMinuteLabelZero != null) timerPickerMinuteLabelZero!,
    if (timerPickerMinuteLabelOne != null) timerPickerMinuteLabelOne!,
    if (timerPickerMinuteLabelTwo != null) timerPickerMinuteLabelTwo!,
    if (timerPickerMinuteLabelFew != null) timerPickerMinuteLabelFew!,
    if (timerPickerMinuteLabelMany != null) timerPickerMinuteLabelMany!,
    if (timerPickerMinuteLabelOther != null) timerPickerMinuteLabelOther!,
  ];

  /// Subclasses should provide the optional zero pluralization of [timerPickerSecondLabel] based on the ARB file.
  @protected
  String? get timerPickerSecondLabelZero => null;

  /// Subclasses should provide the optional one pluralization of [timerPickerSecondLabel] based on the ARB file.
  @protected
  String? get timerPickerSecondLabelOne => null;

  /// Subclasses should provide the optional two pluralization of [timerPickerSecondLabel] based on the ARB file.
  @protected
  String? get timerPickerSecondLabelTwo => null;

  /// Subclasses should provide the optional few pluralization of [timerPickerSecondLabel] based on the ARB file.
  @protected
  String? get timerPickerSecondLabelFew => null;

  /// Subclasses should provide the optional many pluralization of [timerPickerSecondLabel] based on the ARB file.
  @protected
  String? get timerPickerSecondLabelMany => null;

  /// Subclasses should provide the required other pluralization of [timerPickerSecondLabel] based on the ARB file.
  @protected
  String? get timerPickerSecondLabelOther;

  @override
  String? timerPickerSecondLabel(int second) {
    return intl.Intl.pluralLogic(
      second,
      zero: timerPickerSecondLabelZero,
      one: timerPickerSecondLabelOne,
      two: timerPickerSecondLabelTwo,
      few: timerPickerSecondLabelFew,
      many: timerPickerSecondLabelMany,
      other: timerPickerSecondLabelOther,
      locale: _localeName,
    )?.replaceFirst(r'$second', _decimalFormat.format(second));
  }

  @override
  List<String> get timerPickerSecondLabels => <String>[
    if (timerPickerSecondLabelZero != null) timerPickerSecondLabelZero!,
    if (timerPickerSecondLabelOne != null) timerPickerSecondLabelOne!,
    if (timerPickerSecondLabelTwo != null) timerPickerSecondLabelTwo!,
    if (timerPickerSecondLabelFew != null) timerPickerSecondLabelFew!,
    if (timerPickerSecondLabelMany != null) timerPickerSecondLabelMany!,
    if (timerPickerSecondLabelOther != null) timerPickerSecondLabelOther!,
  ];

  /// A [LocalizationsDelegate] for [CupertinoLocalizations].
  ///
  /// Most internationalized apps will use [GlobalCupertinoLocalizations.delegates]
  /// as the value of [CupertinoApp.localizationsDelegates] to include
  /// the localizations for both the cupertino and widget libraries.
  static const LocalizationsDelegate<CupertinoLocalizations> delegate =
      _GlobalCupertinoLocalizationsDelegate();

  /// A value for [CupertinoApp.localizationsDelegates] that's typically used by
  /// internationalized apps.
  ///
  /// ## Sample code
  ///
  /// To include the localizations provided by this class and by
  /// [GlobalWidgetsLocalizations] in a [CupertinoApp],
  /// use [GlobalCupertinoLocalizations.delegates] as the value of
  /// [CupertinoApp.localizationsDelegates], and specify the locales your
  /// app supports with [CupertinoApp.supportedLocales]:
  ///
  /// ```dart
  /// const CupertinoApp(
  ///   localizationsDelegates: GlobalCupertinoLocalizations.delegates,
  ///   supportedLocales: <Locale>[
  ///     Locale('en', 'US'), // English
  ///     Locale('he', 'IL'), // Hebrew
  ///   ],
  ///   // ...
  /// )
  /// ```
  static const List<LocalizationsDelegate<dynamic>> delegates = <LocalizationsDelegate<dynamic>>[
    GlobalCupertinoLocalizations.delegate,
    GlobalWidgetsLocalizations.delegate,
  ];
}

class _GlobalCupertinoLocalizationsDelegate extends LocalizationsDelegate<CupertinoLocalizations> {
  const _GlobalCupertinoLocalizationsDelegate();

  @override
  bool isSupported(Locale locale) => kCupertinoSupportedLanguages.contains(locale.languageCode);

  static final Map<Locale, Future<CupertinoLocalizations>> _loadedTranslations =
      <Locale, Future<CupertinoLocalizations>>{};

  @override
  Future<CupertinoLocalizations> load(Locale locale) {
    assert(isSupported(locale));
    return _loadedTranslations.putIfAbsent(locale, () {
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
      // We don't want any additional decoration here. The am/pm is handled in
      // the date picker. We just want an hour number localized.
      late intl.DateFormat singleDigitHourFormat;
      late intl.DateFormat singleDigitMinuteFormat;
      late intl.DateFormat doubleDigitMinuteFormat;
      late intl.DateFormat singleDigitSecondFormat;
      late intl.NumberFormat decimalFormat;

      void loadFormats(String? locale) {
        fullYearFormat = intl.DateFormat.y(locale);
        dayFormat = intl.DateFormat.d(locale);
        weekdayFormat = intl.DateFormat.E(locale);
        mediumDateFormat = intl.DateFormat.MMMEd(locale);
        // TODO(xster): fix when https://github.com/dart-lang/intl/issues/207 is resolved.
        singleDigitHourFormat = intl.DateFormat('HH', locale);
        singleDigitMinuteFormat = intl.DateFormat.m(locale);
        doubleDigitMinuteFormat = intl.DateFormat('mm', locale);
        singleDigitSecondFormat = intl.DateFormat.s(locale);
        decimalFormat = intl.NumberFormat.decimalPattern(locale);
      }

      if (intl.DateFormat.localeExists(localeName)) {
        loadFormats(localeName);
      } else if (intl.DateFormat.localeExists(locale.languageCode)) {
        loadFormats(locale.languageCode);
      } else {
        loadFormats(null);
      }

      return SynchronousFuture<CupertinoLocalizations>(
        getCupertinoTranslation(
          locale,
          fullYearFormat,
          dayFormat,
          weekdayFormat,
          mediumDateFormat,
          singleDigitHourFormat,
          singleDigitMinuteFormat,
          doubleDigitMinuteFormat,
          singleDigitSecondFormat,
          decimalFormat,
        )!,
      );
    });
  }

  @override
  bool shouldReload(_GlobalCupertinoLocalizationsDelegate old) => false;

  @override
  String toString() =>
      'GlobalCupertinoLocalizations.delegate(${kCupertinoSupportedLanguages.length} locales)';
}
