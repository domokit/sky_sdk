// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'localizations_utils.dart';

String generateCupertinoHeader(String regenerateInstructions) {
  return '''
// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file has been automatically generated. Please do not edit it manually.
// To regenerate the file, use:
// $regenerateInstructions

import 'dart:collection';

import 'package:flutter/cupertino.dart';
import 'package:intl/intl.dart' as intl;

import '../cupertino_localizations.dart';

// The classes defined here encode all of the translations found in the
// `flutter_localizations/lib/src/l10n/*.arb` files.
//
// These classes are constructed by the [getCupertinoTranslation] method at the
// bottom of this file, and used by the [_GlobalCupertinoLocalizationsDelegate.load]
// method defined in `flutter_localizations/lib/src/cupertino_localizations.dart`.

// TODO(goderbauer): Extend the generator to properly format the output.
// dart format off''';
}

/// Returns the source of the constructor for a GlobalCupertinoLocalizations
/// subclass.
String generateCupertinoConstructor(LocaleInfo locale) {
  final String localeName = locale.originalString;
  return '''
  /// Create an instance of the translation bundle for ${describeLocale(localeName)}.
  ///
  /// For details on the meaning of the arguments, see [GlobalCupertinoLocalizations].
  const CupertinoLocalization${locale.camelCase()}({
    super.localeName = '$localeName',
    required super.fullYearFormat,
    required super.dayFormat,
    required super.weekdayFormat,
    required super.mediumDateFormat,
    required super.singleDigitHourFormat,
    required super.singleDigitMinuteFormat,
    required super.doubleDigitMinuteFormat,
    required super.singleDigitSecondFormat,
    required super.decimalFormat,
  });''';
}

const String cupertinoFactoryName = 'getCupertinoTranslation';

const String cupertinoFactoryDeclaration = '''
GlobalCupertinoLocalizations? getCupertinoTranslation(
  Locale locale,
  intl.DateFormat fullYearFormat,
  intl.DateFormat dayFormat,
  intl.DateFormat weekdayFormat,
  intl.DateFormat mediumDateFormat,
  intl.DateFormat singleDigitHourFormat,
  intl.DateFormat singleDigitMinuteFormat,
  intl.DateFormat doubleDigitMinuteFormat,
  intl.DateFormat singleDigitSecondFormat,
  intl.NumberFormat decimalFormat,
) {''';

const String cupertinoFactoryArguments =
    'fullYearFormat: fullYearFormat, dayFormat: dayFormat, weekdayFormat: weekdayFormat, mediumDateFormat: mediumDateFormat, singleDigitHourFormat: singleDigitHourFormat, singleDigitMinuteFormat: singleDigitMinuteFormat, doubleDigitMinuteFormat: doubleDigitMinuteFormat, singleDigitSecondFormat: singleDigitSecondFormat, decimalFormat: decimalFormat';

const String cupertinoSupportedLanguagesConstant = 'kCupertinoSupportedLanguages';

const String cupertinoSupportedLanguagesDocMacro = 'flutter.localizations.cupertino.languages';
