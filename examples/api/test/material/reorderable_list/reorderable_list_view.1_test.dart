// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/reorderable_list/reorderable_list_view.1.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Dragged item color is updated', (WidgetTester tester) async {
    await tester.pumpWidget(const example.ReorderableApp());

    final ThemeData theme = Theme.of(tester.element(find.byType(MaterialApp)));

    // Dragged item is wrapped in a Material widget with correct color.
    final TestGesture drag = await tester.startGesture(tester.getCenter(find.text('Item 1')));
    await tester.pump(kLongPressTimeout + kPressTimeout);
    await tester.pumpAndSettle();
    final Material material = tester.widget<Material>(
      find.ancestor(of: find.text('Item 1'), matching: find.byType(Material)),
    );
    expect(material.color, theme.colorScheme.secondary);

    // Ends the drag gesture.
    await drag.moveTo(tester.getCenter(find.text('Item 4')));
    await drag.up();
    await tester.pumpAndSettle();
  });
}
