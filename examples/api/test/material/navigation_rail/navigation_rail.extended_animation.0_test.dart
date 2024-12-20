// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/navigation_rail/navigation_rail.extended_animation.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Navigation rail animates itself between the normal and extended state', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.ExtendedAnimationExampleApp());

    expect(find.text('Tap on FloatingActionButton to expand'), findsOne);
    expect(find.widgetWithIcon(FloatingActionButton, Icons.add), findsOne);
    expect(find.byIcon(Icons.favorite), findsOne);
    expect(find.text('First'), findsOne);
    expect(find.byIcon(Icons.bookmark_border), findsOne);
    expect(find.text('Second'), findsOne);
    expect(find.byIcon(Icons.star_border), findsOne);
    expect(find.text('First'), findsOne);

    // The navigation rail should be in the normal state.
    expect(
      tester.getCenter(find.byType(FloatingActionButton)),
      offsetMoreOrLessEquals(const Offset(40, 36), epsilon: 0.1),
    );
    expect(find.widgetWithText(FloatingActionButton, 'CREATE'), findsNothing);

    // Expand the navigation rail.
    await tester.tap(find.byType(FloatingActionButton));
    await tester.pump();
    await tester.pump(kThemeAnimationDuration * 0.5);
    expect(
      tester.getCenter(find.byType(FloatingActionButton)),
      offsetMoreOrLessEquals(const Offset(128.1, 36), epsilon: 0.1),
    );

    await tester.pump(kThemeAnimationDuration * 0.5);
    expect(
      tester.getCenter(find.byType(FloatingActionButton)),
      offsetMoreOrLessEquals(const Offset(132, 36), epsilon: 0.1),
    );
    expect(find.widgetWithText(FloatingActionButton, 'CREATE'), findsOne);

    // Collapse the navigation rail.
    await tester.tap(find.byType(FloatingActionButton));
    await tester.pump();
    await tester.pump(kThemeAnimationDuration * 0.5);
    expect(
      tester.getCenter(find.byType(FloatingActionButton)),
      offsetMoreOrLessEquals(const Offset(128.1, 36), epsilon: 0.1),
    );

    await tester.pump(kThemeAnimationDuration * 0.5);
    expect(
      tester.getCenter(find.byType(FloatingActionButton)),
      offsetMoreOrLessEquals(const Offset(40, 36), epsilon: 0.1),
    );
    expect(find.widgetWithText(FloatingActionButton, 'CREATE'), findsNothing);
  });
}
