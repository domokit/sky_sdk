// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/list_tile/list_tile.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('ListTile with Hero does not throw', (WidgetTester tester) async {
    const int totalTiles = 3;

    await tester.pumpWidget(const example.ListTileApp());

    expect(find.byType(ListTile), findsNWidgets(totalTiles));

    const String heroTransitionText = 'Tap here for Hero transition';
    const String goBackText = 'Tap here to go back';

    expect(find.text(heroTransitionText), findsOneWidget);
    expect(find.text(goBackText), findsNothing);

    // Tap on the ListTile widget to trigger the Hero transition.
    await tester.tap(find.text(heroTransitionText));
    await tester.pumpAndSettle();

    // The Hero transition is triggered and tap to go back text is displayed.
    expect(find.text(heroTransitionText), findsNothing);
    expect(find.text(goBackText), findsOneWidget);

    expect(tester.takeException(), null);
  });
}
