// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/expansion_tile/expansion_tile.2.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('ExpansionTile animation can be customized using AnimationStyle', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.ExpansionTileAnimationStyleApp());

    double getHeight(WidgetTester tester) {
      return tester.getSize(find.byType(ExpansionTile)).height;
    }

    expect(getHeight(tester), 58.0);

    // Test the default animation style.
    await tester.tap(find.text('ExpansionTile'));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 100));

    expect(getHeight(tester), closeTo(93.4, 0.1));

    await tester.pumpAndSettle();

    expect(getHeight(tester), 170.0);

    // Tap to collapse.
    await tester.tap(find.text('ExpansionTile'));
    await tester.pumpAndSettle();

    // Test the custom animation style.
    await tester.tap(find.text('Custom'));
    await tester.pumpAndSettle();

    await tester.tap(find.text('ExpansionTile'));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 100));

    expect(getHeight(tester), closeTo(59.2, 0.1));

    await tester.pumpAndSettle();

    expect(getHeight(tester), 170.0);

    // Tap to collapse.
    await tester.tap(find.text('ExpansionTile'));
    await tester.pumpAndSettle();

    // Test the no animation style.
    await tester.tap(find.text('None'));
    await tester.pumpAndSettle();

    await tester.tap(find.text('ExpansionTile'));
    await tester.pump();

    expect(getHeight(tester), 170.0);
  });
}
