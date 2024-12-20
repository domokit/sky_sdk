// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:a11y_assessments/use_cases/material_banner.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

import 'test_utils.dart';

void main() {
  testWidgets('material banner can run', (WidgetTester tester) async {
    await pumpsUseCase(tester, MaterialBannerUseCase());
    expect(find.text('Show a MaterialBanner'), findsOneWidget);

    await tester.tap(find.text('Show a MaterialBanner'));
    await tester.pumpAndSettle();
    expect(find.text('Hello, I am a Material Banner'), findsOneWidget);

    await tester.tap(find.text('DISMISS'));
    await tester.pumpAndSettle();
    expect(find.text('Hello, I am a Material Banner'), findsNothing);
  });

  testWidgets('dismiss button focused on banner open', (WidgetTester tester) async {
    await pumpsUseCase(tester, MaterialBannerUseCase());
    await tester.tap(find.text('Show a MaterialBanner'));
    await tester.pumpAndSettle();

    final TextButton dismissButtonFinder = tester.widget<TextButton>(find.byType(TextButton));
    expect(dismissButtonFinder.focusNode!.hasFocus, isTrue);
  });

  testWidgets('show button focused on banner close', (WidgetTester tester) async {
    await pumpsUseCase(tester, MaterialBannerUseCase());
    await tester.tap(find.text('Show a MaterialBanner'));
    await tester.pumpAndSettle();

    await tester.tap(find.byType(TextButton));

    final ElevatedButton showButtonFinder = tester.widget<ElevatedButton>(
      find.byType(ElevatedButton),
    );
    expect(showButtonFinder.focusNode!.hasFocus, isTrue);
  });

  testWidgets('material banner has one h1 tag', (WidgetTester tester) async {
    await pumpsUseCase(tester, MaterialBannerUseCase());
    final Finder findHeadingLevelOnes = find.bySemanticsLabel('MaterialBanner Demo');
    await tester.pumpAndSettle();
    expect(findHeadingLevelOnes, findsOne);
  });
}
