// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/animation/animation_controller/animated_digit.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('animated digit example', (WidgetTester tester) async {
    await tester.pumpWidget(const example.AnimatedDigitApp());

    Finder findVisibleDigit(int digit) {
      return find.descendant(of: find.byType(SlideTransition).last, matching: find.text('$digit'));
    }

    expect(findVisibleDigit(0), findsOneWidget);

    await tester.tap(find.byType(FloatingActionButton));
    await tester.pumpAndSettle();
    expect(findVisibleDigit(1), findsOneWidget);

    await tester.tap(find.byType(FloatingActionButton));
    await tester.pumpAndSettle();
    expect(findVisibleDigit(2), findsOneWidget);

    await tester.tap(find.byType(FloatingActionButton));
    await tester.pump(const Duration(milliseconds: 100)); // Animation duration is 300ms
    await tester.tap(find.byType(FloatingActionButton));
    await tester.pumpAndSettle();
    expect(findVisibleDigit(4), findsOneWidget);
  });
}
