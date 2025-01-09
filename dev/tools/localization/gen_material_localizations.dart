// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'localizations_utils.dart';

String generateMaterialHeader(String regenerateInstructions) {
  return '''
// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file has been automatically generated. Please do not edit it manually.
// To regenerate the file, use:
// $regenerateInstructions

import 'dart:collection';

import 'package:flutter/material.dart';
import 'package:intl/intl.dart' as intl;

import '../material_localizations.dart';

// The classes defined here encode all of the translations found in the
// `flutter_localizations/lib/src/l10n/*.arb` files.
//
// These classes are constructed by the [getMaterialTranslation] method at the
// bottom of this file, and used by the [_MaterialLocalizationsDelegate.load]
// method defined in `flutter_localizations/lib/src/material_localizations.dart`.

// TODO(goderbauer): Extend the generator to properly format the output.
// dart format off''';
}

/// Returns the source of the constructor for a GlobalMaterialLocalizations
/// subclass.
String generateMaterialConstructor(LocaleInfo locale) {
  final String localeName = locale.originalString;
  return '''
  /// Create an instance of the translation bundle for ${describeLocale(localeName)}.
  ///
  /// For details on the meaning of the arguments, see [GlobalMaterialLocalizations].
  const MaterialLocalization${locale.camelCase()}({
    super.localeName = '$localeName',
    required super.fullYearFormat,
    required super.compactDateFormat,
    required super.shortDateFormat,
    required super.mediumDateFormat,
    required super.longDateFormat,
    required super.yearMonthFormat,
    required super.shortMonthDayFormat,
    required super.decimalFormat,
    required super.twoDigitZeroPaddedFormat,
  });''';
}

const String materialFactoryName = 'getMaterialTranslation';

const String materialFactoryDeclaration = '''
GlobalMaterialLocalizations? getMaterialTranslation(
  Locale locale,
  intl.DateFormat fullYearFormat,
  intl.DateFormat compactDateFormat,
  intl.DateFormat shortDateFormat,
  intl.DateFormat mediumDateFormat,
  intl.DateFormat longDateFormat,
  intl.DateFormat yearMonthFormat,
  intl.DateFormat shortMonthDayFormat,
  intl.NumberFormat decimalFormat,
  intl.NumberFormat twoDigitZeroPaddedFormat,
) {''';

const String materialFactoryArguments =
    'fullYearFormat: fullYearFormat, compactDateFormat: compactDateFormat, shortDateFormat: shortDateFormat, mediumDateFormat: mediumDateFormat, longDateFormat: longDateFormat, yearMonthFormat: yearMonthFormat, shortMonthDayFormat: shortMonthDayFormat, decimalFormat: decimalFormat, twoDigitZeroPaddedFormat: twoDigitZeroPaddedFormat';

const String materialSupportedLanguagesConstant = 'kMaterialSupportedLanguages';

const String materialSupportedLanguagesDocMacro = 'flutter.localizations.material.languages';
