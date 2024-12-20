// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/reorderable_list/reorderable_list_view.2.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Dragged Card is elevated', (WidgetTester tester) async {
    await tester.pumpWidget(const example.ReorderableApp());

    Card findCardOne() {
      return tester.widget<Card>(
        find.ancestor(of: find.text('Card 1'), matching: find.byType(Card)),
      );
    }

    // Card has default elevation when not dragged.
    expect(findCardOne().elevation, null);

    // Dragged card is elevated.
    final TestGesture drag = await tester.startGesture(tester.getCenter(find.text('Card 1')));
    await tester.pump(kLongPressTimeout + kPressTimeout);
    await tester.pumpAndSettle();
    expect(findCardOne().elevation, 6);

    // After the drag gesture ends, the card elevation has default value.
    await drag.moveTo(tester.getCenter(find.text('Card 4')));
    await drag.up();
    await tester.pumpAndSettle();
    expect(findCardOne().elevation, null);
  });
}
