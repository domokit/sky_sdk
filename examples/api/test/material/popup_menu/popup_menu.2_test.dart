// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/popup_menu/popup_menu.2.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Popup animation can be customized using AnimationStyle', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.PopupMenuApp());

    // Test the default popup animation.
    await tester.tap(find.byIcon(Icons.more_vert));
    await tester.pump();
    // Advance the animation by half of the default duration.
    await tester.pump(const Duration(milliseconds: 100));

    expect(
      tester.getSize(find.byType(Material).last),
      within(distance: 0.1, from: const Size(224.0, 130.0)),
    );

    // Let the animation finish.
    await tester.pumpAndSettle();

    expect(
      tester.getSize(find.byType(Material).last),
      within(distance: 0.1, from: const Size(224.0, 312.0)),
    );

    // Tap outside the popup menu to close it.
    await tester.tapAt(const Offset(1, 1));
    await tester.pumpAndSettle();

    // Test the custom animation curve and duration.
    await tester.tap(find.text('Custom'));
    await tester.pumpAndSettle();

    await tester.tap(find.byIcon(Icons.more_vert));
    await tester.pump();
    // Advance the animation by one third of the custom duration.
    await tester.pump(const Duration(milliseconds: 1000));

    expect(
      tester.getSize(find.byType(Material).last),
      within(distance: 0.1, from: const Size(224.0, 312.0)),
    );

    // Let the animation finish.
    await tester.pumpAndSettle();

    expect(
      tester.getSize(find.byType(Material).last),
      within(distance: 0.1, from: const Size(224.0, 312.0)),
    );

    // Tap outside the popup menu to close it.
    await tester.tapAt(const Offset(1, 1));
    await tester.pumpAndSettle();

    // Test the no animation style.
    await tester.tap(find.text('None'));
    await tester.pumpAndSettle();

    await tester.tap(find.byIcon(Icons.more_vert));
    // Advance the animation by only one frame.
    await tester.pump();

    // The popup menu is shown immediately.
    expect(
      tester.getSize(find.byType(Material).last),
      within(distance: 0.1, from: const Size(224.0, 312.0)),
    );
  });
}
