// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/floating_action_button/floating_action_button.2.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('FloatingActionButton variants', (WidgetTester tester) async {
    await tester.pumpWidget(const example.FloatingActionButtonExampleApp());

    FloatingActionButton getFAB(Finder finder) {
      return tester.widget<FloatingActionButton>(finder);
    }

    final ColorScheme colorScheme = ThemeData(useMaterial3: true).colorScheme;

    // Test the FAB with surface color mapping.
    FloatingActionButton fab = getFAB(find.byType(FloatingActionButton).at(0));
    expect(fab.foregroundColor, colorScheme.primary);
    expect(fab.backgroundColor, colorScheme.surface);

    // Test the FAB with secondary color mapping.
    fab = getFAB(find.byType(FloatingActionButton).at(1));
    expect(fab.foregroundColor, colorScheme.onSecondaryContainer);
    expect(fab.backgroundColor, colorScheme.secondaryContainer);

    // Test the FAB with tertiary color mapping.
    fab = getFAB(find.byType(FloatingActionButton).at(2));
    expect(fab.foregroundColor, colorScheme.onTertiaryContainer);
    expect(fab.backgroundColor, colorScheme.tertiaryContainer);
  });
}
