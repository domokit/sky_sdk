// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/page_transitions_theme/page_transitions_theme.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('MaterialApp defines a custom PageTransitionsTheme', (WidgetTester tester) async {
    await tester.pumpWidget(const example.PageTransitionsThemeApp());

    final Finder homePage = find.byType(example.HomePage);
    expect(homePage, findsOneWidget);

    final PageTransitionsTheme theme = Theme.of(tester.element(homePage)).pageTransitionsTheme;
    expect(theme.builders, isNotNull);

    // Check defined page transitions builder for each platform.
    for (final TargetPlatform platform in TargetPlatform.values) {
      switch (platform) {
        case TargetPlatform.iOS:
          expect(theme.builders[platform], isA<CupertinoPageTransitionsBuilder>());
        case TargetPlatform.linux:
          expect(theme.builders[platform], isA<OpenUpwardsPageTransitionsBuilder>());
        case TargetPlatform.macOS:
          expect(theme.builders[platform], isA<FadeUpwardsPageTransitionsBuilder>());
        case TargetPlatform.android:
        case TargetPlatform.fuchsia:
        case TargetPlatform.windows:
          expect(theme.builders[platform], isNull);
      }
    }

    // Can navigate to the second page.
    expect(find.text('To SecondPage'), findsOneWidget);
    await tester.tap(find.text('To SecondPage'));
    await tester.pumpAndSettle();

    // Can navigate back to the home page.
    expect(find.text('Back to HomePage'), findsOneWidget);
    await tester.tap(find.text('Back to HomePage'));
    await tester.pumpAndSettle();
    expect(find.text('To SecondPage'), findsOneWidget);
  });
}
