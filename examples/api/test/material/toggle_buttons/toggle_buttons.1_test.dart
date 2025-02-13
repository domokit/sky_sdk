// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/toggle_buttons/toggle_buttons.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('ToggleButtons allows multiple or no selection', (WidgetTester tester) async {
    final ThemeData theme = ThemeData();
    Finder findButton(String text) {
      return find.descendant(
        of: find.byType(ToggleButtons),
        matching: find.widgetWithText(TextButton, text),
      );
    }

    await tester.pumpWidget(const example.ToggleButtonsApp());

    TextButton toggleMButton = tester.widget<TextButton>(findButton('M'));
    TextButton toggleXLButton = tester.widget<TextButton>(findButton('XL'));

    // Initially, only M is selected.
    expect(
      toggleMButton.style!.backgroundColor!.resolve(enabled),
      theme.colorScheme.primary.withOpacity(0.12),
    );
    expect(
      toggleXLButton.style!.backgroundColor!.resolve(enabled),
      theme.colorScheme.surface.withOpacity(0.0),
    );

    // Tap on XL.
    await tester.tap(findButton('XL'));
    await tester.pumpAndSettle();

    // Now both M and XL are selected.
    toggleMButton = tester.widget<TextButton>(findButton('M'));
    toggleXLButton = tester.widget<TextButton>(findButton('XL'));

    expect(
      toggleMButton.style!.backgroundColor!.resolve(enabled),
      theme.colorScheme.primary.withOpacity(0.12),
    );
    expect(
      toggleXLButton.style!.backgroundColor!.resolve(enabled),
      theme.colorScheme.primary.withOpacity(0.12),
    );

    // Tap M to deselect it.
    await tester.tap(findButton('M'));
    await tester.pumpAndSettle();

    // Tap XL to deselect it.
    await tester.tap(findButton('XL'));
    await tester.pumpAndSettle();

    // Now neither M nor XL are selected.
    toggleMButton = tester.widget<TextButton>(findButton('M'));
    toggleXLButton = tester.widget<TextButton>(findButton('XL'));

    expect(
      toggleMButton.style!.backgroundColor!.resolve(enabled),
      theme.colorScheme.surface.withOpacity(0.0),
    );
    expect(
      toggleXLButton.style!.backgroundColor!.resolve(enabled),
      theme.colorScheme.surface.withOpacity(0.0),
    );
  });

  testWidgets('SegmentedButton allows multiple or no selection', (WidgetTester tester) async {
    final ThemeData theme = ThemeData();
    Finder findButton(String text) {
      return find.descendant(
        of: find.byType(SegmentedButton<example.ShirtSize>),
        matching: find.widgetWithText(TextButton, text),
      );
    }

    await tester.pumpWidget(const example.ToggleButtonsApp());

    Material segmentMButton = tester.widget<Material>(
      find.descendant(of: findButton('M'), matching: find.byType(Material)),
    );
    Material segmentXLButton = tester.widget<Material>(
      find.descendant(of: findButton('XL'), matching: find.byType(Material)),
    );

    // Initially, only M is selected.
    expect(segmentMButton.color, theme.colorScheme.secondaryContainer);
    expect(segmentXLButton.color, Colors.transparent);

    // Tap on XL.
    await tester.tap(findButton('XL'));
    await tester.pumpAndSettle();

    // // Now both M and XL are selected.
    segmentMButton = tester.widget<Material>(
      find.descendant(of: findButton('M'), matching: find.byType(Material)),
    );
    segmentXLButton = tester.widget<Material>(
      find.descendant(of: findButton('XL'), matching: find.byType(Material)),
    );

    expect(segmentMButton.color, theme.colorScheme.secondaryContainer);
    expect(segmentXLButton.color, theme.colorScheme.secondaryContainer);

    // Tap M to deselect it.
    await tester.tap(findButton('M'));
    await tester.pumpAndSettle();

    // Tap XL to deselect it.
    await tester.tap(findButton('XL'));
    await tester.pumpAndSettle();

    // Now neither M nor XL are selected.
    segmentMButton = tester.widget<Material>(
      find.descendant(of: findButton('M'), matching: find.byType(Material)),
    );
    segmentXLButton = tester.widget<Material>(
      find.descendant(of: findButton('XL'), matching: find.byType(Material)),
    );

    expect(segmentMButton.color, Colors.transparent);
    expect(segmentXLButton.color, Colors.transparent);
  });
}

Set<MaterialState> enabled = <MaterialState>{};
