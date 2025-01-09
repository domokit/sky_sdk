// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/animated_switcher/animated_switcher.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Increments counter on button tap', (WidgetTester tester) async {
    await tester.pumpWidget(const example.AnimatedSwitcherExampleApp());

    int counter = 0;

    expect(find.text('$counter'), findsOneWidget);

    while (counter < 10) {
      // Tap on the button to increment the counter.
      await tester.tap(
        find.ancestor(of: find.text('Increment'), matching: find.byType(ElevatedButton)),
      );
      await tester.pumpAndSettle();

      counter += 1;

      expect(find.text('${counter - 1}'), findsNothing);
      expect(find.text('$counter'), findsOneWidget);
    }
  });

  testWidgets('Animates counter change', (WidgetTester tester) async {
    await tester.pumpWidget(const example.AnimatedSwitcherExampleApp());

    // The animation duration defined in the example app.
    const Duration animationDuration = Duration(milliseconds: 500);

    final Finder zeroTransitionFinder = find.ancestor(
      of: find.text('0'),
      matching: find.byType(ScaleTransition),
    );
    final Finder oneTransitionFinder = find.ancestor(
      of: find.text('1'),
      matching: find.byType(ScaleTransition),
    );

    expect(zeroTransitionFinder, findsOneWidget);
    ScaleTransition zeroTransition = tester.widget(zeroTransitionFinder);
    expect(zeroTransition.scale.value, equals(1.0));

    expect(oneTransitionFinder, findsNothing);

    // Tap on the button to increment the counter.
    await tester.tap(
      find.ancestor(of: find.text('Increment'), matching: find.byType(ElevatedButton)),
    );
    await tester.pump();

    expect(zeroTransitionFinder, findsOneWidget);
    zeroTransition = tester.widget(zeroTransitionFinder);
    expect(zeroTransition.scale.value, equals(1.0));

    expect(oneTransitionFinder, findsOneWidget);
    ScaleTransition oneTransition = tester.widget(oneTransitionFinder);
    expect(oneTransition.scale.value, equals(0.0));

    // Advance animation to the middle.
    await tester.pump(animationDuration ~/ 2);

    expect(zeroTransitionFinder, findsOneWidget);
    zeroTransition = tester.widget(zeroTransitionFinder);
    expect(zeroTransition.scale.value, equals(0.5));

    expect(oneTransitionFinder, findsOneWidget);
    oneTransition = tester.widget(oneTransitionFinder);
    expect(oneTransition.scale.value, equals(0.5));

    // Advance animation to the end.
    await tester.pump(animationDuration ~/ 2);

    expect(zeroTransitionFinder, findsOneWidget);
    zeroTransition = tester.widget(zeroTransitionFinder);
    expect(zeroTransition.scale.value, equals(0.0));

    expect(oneTransitionFinder, findsOneWidget);
    oneTransition = tester.widget(oneTransitionFinder);
    expect(oneTransition.scale.value, equals(1.0));
  });
}
