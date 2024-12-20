// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/nested_scroll_view/nested_scroll_view.1.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Hides app bar after scrolling down', (WidgetTester tester) async {
    await tester.pumpWidget(const example.NestedScrollViewExampleApp());
    expect(find.text('Floating Nested SliverAppBar'), findsOneWidget);
    expect(find.byType(NestedScrollView), findsOneWidget);
    expect(find.text('Item 0'), findsOneWidget);
    expect(find.text('Item 12'), findsNothing);
    expect(find.text('Item 12', skipOffstage: false), findsOneWidget);

    await tester.ensureVisible(find.text('Item 12', skipOffstage: false));
    await tester.pump();
    expect(find.text('Item 0'), findsNothing);
    expect(find.text('Floating Nested SliverAppBar'), findsNothing);
  });
}
