// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/expansion_tile/expansion_tile.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('When expansion tiles are expanded tile numbers are revealed', (
    WidgetTester tester,
  ) async {
    const int totalTiles = 3;

    await tester.pumpWidget(const example.ExpansionTileApp());

    expect(find.byType(ExpansionTile), findsNWidgets(totalTiles));

    const String tileOne = 'This is tile number 1';
    expect(find.text(tileOne), findsNothing);

    await tester.tap(find.text('ExpansionTile 1'));
    await tester.pumpAndSettle();
    expect(find.text(tileOne), findsOneWidget);

    const String tileTwo = 'This is tile number 2';
    expect(find.text(tileTwo), findsNothing);

    await tester.tap(find.text('ExpansionTile 2'));
    await tester.pumpAndSettle();
    expect(find.text(tileTwo), findsOneWidget);

    const String tileThree = 'This is tile number 3';
    expect(find.text(tileThree), findsNothing);

    await tester.tap(find.text('ExpansionTile 3'));
    await tester.pumpAndSettle();
    expect(find.text(tileThree), findsOneWidget);
  });
}
