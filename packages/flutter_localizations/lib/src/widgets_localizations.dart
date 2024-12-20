// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
///
/// @docImport 'material_localizations.dart';
library;

import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';

import 'l10n/generated_widgets_localizations.dart';

/// Localized values for widgets.
///
/// ## Supported languages
///
/// This class supports locales with the following [Locale.languageCode]s:
///
/// {@macro flutter.localizations.widgets.languages}
///
/// This list is available programmatically via [kWidgetsSupportedLanguages].
///
/// Besides localized strings, this class also maps [Locale] to [textDirection].
/// All locales are [TextDirection.ltr] except for locales with the following
/// [Locale.languageCode] values, which are [TextDirection.rtl]:
///
///   * ar - Arabic
///   * fa - Farsi
///   * he - Hebrew
///   * ps - Pashto
///   * sd - Sindhi
///   * ur - Urdu
///
abstract class GlobalWidgetsLocalizations implements WidgetsLocalizations {
  /// Construct an object that defines the localized values for the widgets
  /// library for the given [textDirection].
  const GlobalWidgetsLocalizations(this.textDirection);

  @override
  final TextDirection textDirection;

  /// A [LocalizationsDelegate] for [WidgetsLocalizations].
  ///
  /// Most internationalized apps will use [GlobalMaterialLocalizations.delegates]
  /// as the value of [MaterialApp.localizationsDelegates] to include
  /// the localizations for both the material and widget libraries.
  static const LocalizationsDelegate<WidgetsLocalizations> delegate =
      _WidgetsLocalizationsDelegate();
}

class _WidgetsLocalizationsDelegate extends LocalizationsDelegate<WidgetsLocalizations> {
  const _WidgetsLocalizationsDelegate();

  @override
  bool isSupported(Locale locale) => kWidgetsSupportedLanguages.contains(locale.languageCode);

  static final Map<Locale, Future<WidgetsLocalizations>> _loadedTranslations =
      <Locale, Future<WidgetsLocalizations>>{};

  @override
  Future<WidgetsLocalizations> load(Locale locale) {
    assert(isSupported(locale));
    return _loadedTranslations.putIfAbsent(locale, () {
      return SynchronousFuture<WidgetsLocalizations>(getWidgetsTranslation(locale)!);
    });
  }

  @override
  bool shouldReload(_WidgetsLocalizationsDelegate old) => false;

  @override
  String toString() =>
      'GlobalWidgetsLocalizations.delegate(${kWidgetsSupportedLanguages.length} locales)';
}
