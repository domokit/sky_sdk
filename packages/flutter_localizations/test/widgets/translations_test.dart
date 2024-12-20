// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_localizations/flutter_localizations.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  for (final String language in kWidgetsSupportedLanguages) {
    testWidgets('translations exist for $language', (WidgetTester tester) async {
      final Locale locale = Locale(language);

      expect(GlobalWidgetsLocalizations.delegate.isSupported(locale), isTrue);

      final WidgetsLocalizations localizations = await GlobalWidgetsLocalizations.delegate.load(
        locale,
      );

      expect(localizations.reorderItemDown, isNotNull);
      expect(localizations.reorderItemLeft, isNotNull);
      expect(localizations.reorderItemRight, isNotNull);
      expect(localizations.reorderItemToEnd, isNotNull);
      expect(localizations.reorderItemToStart, isNotNull);
      expect(localizations.reorderItemUp, isNotNull);
    });
  }
}
