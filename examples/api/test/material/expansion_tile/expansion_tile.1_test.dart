// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_api_samples/material/expansion_tile/expansion_tile.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Test the basics of ExpansionTileControllerApp', (WidgetTester tester) async {
    await tester.pumpWidget(const example.ExpansionTileControllerApp());

    expect(find.text('ExpansionTile Contents'), findsNothing);
    expect(find.text('Collapse This Tile'), findsNothing);

    await tester.tap(find.text('Expand/Collapse the Tile Above'));
    await tester.pumpAndSettle();
    expect(find.text('ExpansionTile Contents'), findsOneWidget);
    await tester.tap(find.text('Expand/Collapse the Tile Above'));
    await tester.pumpAndSettle();
    expect(find.text('ExpansionTile Contents'), findsNothing);

    await tester.tap(find.text('ExpansionTile with implicit controller.'));
    await tester.pumpAndSettle();
    expect(find.text('Collapse This Tile'), findsOneWidget);
    await tester.tap(find.text('Collapse This Tile'));
    await tester.pumpAndSettle();
    expect(find.text('Collapse This Tile'), findsNothing);
  });
}
