// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/sliver/sliver_floating_header.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('SliverFloatingHeader example', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SliverFloatingHeaderApp());

    final Finder headerText = find.text(
      'SliverFloatingHeader\nScroll down a little to show\nScroll up a little to hide',
    );
    final double headerHeight = tester.getSize(headerText).height;

    await tester.drag(find.byType(CustomScrollView), Offset(0, -2 * headerHeight));
    await tester.pumpAndSettle();
    expect(headerText, findsNothing);

    await tester.drag(find.byType(CustomScrollView), Offset(0, 0.5 * headerHeight));
    await tester.pumpAndSettle();
    expect(headerText, findsOneWidget);

    await tester.drag(find.byType(CustomScrollView), Offset(0, -0.5 * headerHeight));
    await tester.pumpAndSettle();
    expect(headerText, findsNothing);
  });
}
