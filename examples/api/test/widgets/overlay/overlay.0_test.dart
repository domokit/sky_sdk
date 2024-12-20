// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/overlay/overlay.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Can use Overlay to highlight NavigationBar destination', (
    WidgetTester tester,
  ) async {
    const String explorePage = 'Explore page';
    const String commutePage = 'Commute page';
    const String savedPage = 'Saved page';

    await tester.pumpWidget(const example.OverlayApp());

    expect(find.text(explorePage), findsNothing);
    expect(find.text(commutePage), findsNothing);
    expect(find.text(savedPage), findsNothing);

    await tester.tap(find.widgetWithText(ElevatedButton, 'Explore'));
    await tester.pumpAndSettle();
    expect(find.text(explorePage), findsOneWidget);

    await tester.tap(find.widgetWithText(ElevatedButton, 'Commute'));
    await tester.pumpAndSettle();
    expect(find.text(commutePage), findsOneWidget);

    await tester.tap(find.widgetWithText(ElevatedButton, 'Saved'));
    await tester.pumpAndSettle();
    expect(find.text(savedPage), findsOneWidget);

    await tester.tap(find.widgetWithText(ElevatedButton, 'Remove Overlay'));
    await tester.pumpAndSettle();
    expect(find.text(explorePage), findsNothing);
    expect(find.text(commutePage), findsNothing);
    expect(find.text(savedPage), findsNothing);
  });

  testWidgets('Narrow layout does not overflow', (WidgetTester tester) async {
    // Set a narrow screen size.
    tester.view
      ..physicalSize = const Size(320, 480)
      ..devicePixelRatio = 1;
    addTearDown(tester.view.reset);

    await tester.pumpWidget(const example.OverlayApp());

    // Verify that no overflow errors occur.
    expect(tester.takeException(), isNull);
  });
}
