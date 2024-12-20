// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/heroes/hero.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Has Hero animation', (WidgetTester tester) async {
    await tester.pumpWidget(const example.HeroApp());

    expect(find.text('Hero Sample'), findsOneWidget);
    await tester.tap(find.byType(Container));
    await tester.pump();

    Size heroSize = tester.getSize(find.byType(Container));

    // Jump 25% into the transition (total length = 300ms)
    await tester.pump(const Duration(milliseconds: 75)); // 25% of 300ms
    heroSize = tester.getSize(find.byType(Container));
    expect(heroSize.width.roundToDouble(), 103.0);
    expect(heroSize.height.roundToDouble(), 60.0);

    // Jump to 50% into the transition.
    await tester.pump(const Duration(milliseconds: 75)); // 25% of 300ms
    heroSize = tester.getSize(find.byType(Container));
    expect(heroSize.width.roundToDouble(), 189.0);
    expect(heroSize.height.roundToDouble(), 146.0);

    // Jump to 75% into the transition.
    await tester.pump(const Duration(milliseconds: 75)); // 25% of 300ms
    heroSize = tester.getSize(find.byType(Container));
    expect(heroSize.width.roundToDouble(), 199.0);
    expect(heroSize.height.roundToDouble(), 190.0);

    // Jump to 100% into the transition.
    await tester.pump(const Duration(milliseconds: 75)); // 25% of 300ms
    heroSize = tester.getSize(find.byType(Container));
    expect(heroSize, const Size(200.0, 200.0));

    expect(find.byIcon(Icons.arrow_back), findsOneWidget);
    await tester.tap(find.byIcon(Icons.arrow_back));
    await tester.pump();

    // Jump 25% into the transition (total length = 300ms)
    await tester.pump(const Duration(milliseconds: 75)); // 25% of 300ms
    heroSize = tester.getSize(find.byType(Container));
    expect(heroSize.width.roundToDouble(), 199.0);
    expect(heroSize.height.roundToDouble(), 190.0);

    // Jump to 50% into the transition.
    await tester.pump(const Duration(milliseconds: 75)); // 25% of 300ms
    heroSize = tester.getSize(find.byType(Container));
    expect(heroSize.width.roundToDouble(), 189.0);
    expect(heroSize.height.roundToDouble(), 146.0);

    // Jump to 75% into the transition.
    await tester.pump(const Duration(milliseconds: 75)); // 25% of 300ms
    heroSize = tester.getSize(find.byType(Container));
    expect(heroSize.width.roundToDouble(), 103.0);
    expect(heroSize.height.roundToDouble(), 60.0);

    // Jump to 100% into the transition.
    await tester.pump(const Duration(milliseconds: 75)); // 25% of 300ms
    heroSize = tester.getSize(find.byType(Container));
    expect(heroSize, const Size(50.0, 50.0));
  });
}
