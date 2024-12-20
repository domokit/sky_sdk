// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:a11y_assessments/use_cases/slider.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

import 'test_utils.dart';

void main() {
  testWidgets('slider can run', (WidgetTester tester) async {
    await pumpsUseCase(tester, SliderUseCase());
    expect(find.byType(Slider), findsOneWidget);

    await tester.tapAt(tester.getCenter(find.byType(Slider)));
    await tester.pumpAndSettle();

    final MainWidgetState state = tester.state<MainWidgetState>(find.byType(MainWidget));
    expect(state.currentSliderValue, 60);
  });
  testWidgets('slider semantics wrapper exists', (WidgetTester tester) async {
    await pumpsUseCase(tester, SliderUseCase());
    final Finder semanticsWidget = find.bySemanticsLabel('Accessibility Test Slider');
    expect(semanticsWidget, findsOneWidget);
  });

  testWidgets('slider demo page has one h1 tag', (WidgetTester tester) async {
    await pumpsUseCase(tester, SliderUseCase());
    final Finder findHeadingLevelOnes = find.bySemanticsLabel('Slider demo');
    await tester.pumpAndSettle();
    expect(findHeadingLevelOnes, findsOne);
  });
}
