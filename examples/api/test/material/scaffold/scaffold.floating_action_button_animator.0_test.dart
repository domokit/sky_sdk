// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/scaffold/scaffold.floating_action_button_animator.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('FloatingActionButton animation can be customized', (WidgetTester tester) async {
    await tester.pumpWidget(const example.ScaffoldFloatingActionButtonAnimatorApp());

    expect(find.byType(FloatingActionButton), findsNothing);

    // Test default FloatingActionButtonAnimator.
    // Tap the toggle button to show the FAB.
    await tester.tap(find.text('Toggle FAB'));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 100)); // Advance animation by 100ms.
    // FAB is partially animated in.
    expect(tester.getTopLeft(find.byType(FloatingActionButton)).dx, closeTo(743.8, 0.1));

    await tester.pump(const Duration(milliseconds: 100)); // Advance animation by 100ms.
    // FAB is fully animated in.
    expect(tester.getTopLeft(find.byType(FloatingActionButton)).dx, equals(728.0));

    // Tap the toggle button to hide the FAB.
    await tester.tap(find.text('Toggle FAB'));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 100)); // Advance animation by 100ms.
    // FAB is partially animated out.
    expect(tester.getTopLeft(find.byType(FloatingActionButton)).dx, closeTo(747.1, 0.1));

    await tester.pump(const Duration(milliseconds: 100)); // Advance animation by 100ms.
    // FAB is fully animated out.
    expect(tester.getTopLeft(find.byType(FloatingActionButton)).dx, equals(756.0));

    await tester.pump(const Duration(milliseconds: 50)); // Advance animation by 50ms.
    // FAB is hidden.
    expect(find.byType(FloatingActionButton), findsNothing);

    // Select 'None' to disable animation.
    await tester.tap(find.text('None'));
    await tester.pump();

    // Test no animation FloatingActionButtonAnimator.
    await tester.tap(find.text('Toggle FAB'));
    await tester.pump();
    // FAB is immediately shown.
    expect(tester.getTopLeft(find.byType(FloatingActionButton)).dx, equals(728.0));

    // Tap the toggle button to hide the FAB.
    await tester.tap(find.text('Toggle FAB'));
    await tester.pump();
    // FAB is immediately hidden.
    expect(find.byType(FloatingActionButton), findsNothing);
  });
}
