// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/expansion_panel/expansion_panel_list.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('ExpansionPanel can be expanded', (WidgetTester tester) async {
    await tester.pumpWidget(const example.ExpansionPanelListExampleApp());

    // Verify the first tile is collapsed.
    expect(tester.widget<ExpandIcon>(find.byType(ExpandIcon).first).isExpanded, false);

    // Tap to expand the first tile.
    await tester.tap(find.byType(ExpandIcon).first);
    await tester.pumpAndSettle();

    // Verify that the first tile is expanded.
    expect(tester.widget<ExpandIcon>(find.byType(ExpandIcon).first).isExpanded, true);
  });

  testWidgets('Tap to delete a ExpansionPanel', (WidgetTester tester) async {
    const int index = 3;

    await tester.pumpWidget(const example.ExpansionPanelListExampleApp());

    expect(find.widgetWithText(ListTile, 'Panel $index'), findsOneWidget);
    expect(tester.widget<ExpandIcon>(find.byType(ExpandIcon).at(index)).isExpanded, false);

    // Tap to expand the tile at index 3.
    await tester.tap(find.byType(ExpandIcon).at(index));
    await tester.pumpAndSettle();

    expect(tester.widget<ExpandIcon>(find.byType(ExpandIcon).at(index)).isExpanded, true);

    // Tap to delete the tile at index 3.
    await tester.tap(find.byIcon(Icons.delete).at(index));
    await tester.pumpAndSettle();

    // Verify that the tile at index 3 is deleted.
    expect(find.widgetWithText(ListTile, 'Panel $index'), findsNothing);
  });

  testWidgets('ExpansionPanelList is scrollable', (WidgetTester tester) async {
    await tester.pumpWidget(const example.ExpansionPanelListExampleApp());

    expect(find.byType(SingleChildScrollView), findsOneWidget);

    // Expand all the tiles.
    for (int i = 0; i < 8; i++) {
      await tester.tap(find.byType(ExpandIcon).at(i));
    }
    await tester.pumpAndSettle();

    // Check panel 3 tile position.
    Offset tilePosition = tester.getBottomLeft(find.widgetWithText(ListTile, 'Panel 3'));
    expect(tilePosition.dy, 656.0);

    // Scroll up.
    await tester.drag(find.byType(SingleChildScrollView), const Offset(0, -300));
    await tester.pumpAndSettle();

    // Verify panel 3 tile position is updated after scrolling.
    tilePosition = tester.getBottomLeft(find.widgetWithText(ListTile, 'Panel 3'));
    expect(tilePosition.dy, 376.0);
  });
}
