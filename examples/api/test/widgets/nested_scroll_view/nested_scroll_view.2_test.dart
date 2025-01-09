// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/nested_scroll_view/nested_scroll_view.2.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Hides app bar after scrolling past first item', (WidgetTester tester) async {
    await tester.pumpWidget(const example.NestedScrollViewExampleApp());
    expect(find.text('Snapping Nested SliverAppBar'), findsOneWidget);
    expect(find.byType(NestedScrollView), findsOneWidget);
    expect(find.text('Item 0'), findsOneWidget);

    while (find.text('Item 0').evaluate().isNotEmpty) {
      await tester.sendEventToBinding(const PointerScrollEvent(scrollDelta: Offset(0.0, 1.0)));
      await tester.pump();
    }

    expect(find.text('Snapping Nested SliverAppBar'), findsNothing);
  });
}
