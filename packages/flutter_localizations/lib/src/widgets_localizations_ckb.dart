// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';

import 'widgets_localizations.dart';

/// Widgets Localizations for Kurdish (Sorani).
///
/// See also:
///
///  * [GlobalWidgetsLocalizations], which provides widgets localizations for
///    many languages.
class WidgetsLocalizationCkb extends GlobalWidgetsLocalizations {
  /// Create an instance of the translation bundle for Kurdish (Sorani).
  const WidgetsLocalizationCkb() : super(TextDirection.rtl);

  @override
  String get reorderItemToStart => 'گواستنەوە بۆ سەرەتا';

  @override
  String get reorderItemToEnd => 'گواستنەوە بۆ کۆتایی';

  @override
  String get reorderItemUp => 'گواستنەوە بۆ سەرەوە';

  @override
  String get reorderItemDown => 'گواستنەوە بۆ خوارەوە';

  @override
  String get reorderItemLeft => 'گواستنەوە بۆ چەپ';

  @override
  String get reorderItemRight => 'گواستنەوە بۆ ڕاست';
}
