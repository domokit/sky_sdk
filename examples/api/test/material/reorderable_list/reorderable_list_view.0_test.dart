// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/reorderable_list/reorderable_list_view.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Content is reordered after a drag', (WidgetTester tester) async {
    await tester.pumpWidget(const example.ReorderableApp());

    bool item1IsBeforeItem2() {
      final Iterable<Text> texts = tester.widgetList<Text>(find.byType(Text));
      final List<String?> labels = texts.map((final Text text) => text.data).toList();
      return labels.indexOf('Item 1') < labels.indexOf('Item 2');
    }

    expect(item1IsBeforeItem2(), true);

    // Drag 'Item 1' after 'Item 4'.
    final TestGesture drag = await tester.startGesture(tester.getCenter(find.text('Item 1')));
    await tester.pump(kLongPressTimeout + kPressTimeout);
    await tester.pumpAndSettle();
    await drag.moveTo(tester.getCenter(find.text('Item 4')));
    await drag.up();
    await tester.pumpAndSettle();

    expect(item1IsBeforeItem2(), false);
  });
}
