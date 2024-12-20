// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:intl/date_symbol_data_custom.dart' as date_symbol_data_custom;
import 'package:intl/date_symbols.dart' as intl;

import '../l10n/generated_date_localizations.dart' as date_localizations;

/// Tracks if date i18n data has been loaded.
bool _dateIntlDataInitialized = false;

/// Loads i18n data for dates if it hasn't been loaded yet.
///
/// Only the first invocation of this function loads the data. Subsequent
/// invocations have no effect.
void loadDateIntlDataIfNotLoaded() {
  if (!_dateIntlDataInitialized) {
    date_localizations.dateSymbols.forEach((String locale, intl.DateSymbols symbols) {
      // Perform initialization.
      assert(date_localizations.datePatterns.containsKey(locale));
      date_symbol_data_custom.initializeDateFormattingCustom(
        locale: locale,
        symbols: symbols,
        patterns: date_localizations.datePatterns[locale],
      );
    });
    _dateIntlDataInitialized = true;
  }
}
