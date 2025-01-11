// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';

/// Returns the text direction for a given language code.
TextDirection getLanguageDirection(String languageCode) {
  // List of RTL language codes
  const Set<String> rtlLanguages = <String>{
    'ar', // Arabic
    'fa', // Persian/Farsi
    'he', // Hebrew
    'ps', // Pashto
    'ur', // Urdu
    'ckb', // Kurdish (Sorani)
  };

  return rtlLanguages.contains(languageCode) ? TextDirection.rtl : TextDirection.ltr;
}
