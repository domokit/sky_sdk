// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/sliver_fill/sliver_fill_remaining.2.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Shows all elements', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SliverFillRemainingExampleApp());
    expect(find.text('SliverFillRemaining Sample'), findsOneWidget);
    expect(find.byType(CustomScrollView), findsOneWidget);
    expect(find.byType(SliverFixedExtentList), findsOneWidget);

    expect(
      find.byWidgetPredicate(
        (Widget widget) => (widget is Container) && widget.color == Colors.indigo[200],
        skipOffstage: false,
      ),
      findsNWidgets(3),
    );
    expect(
      find.byWidgetPredicate(
        (Widget widget) => (widget is Container) && widget.color == Colors.orange[200],
      ),
      findsNWidgets(2),
    );
    expect(find.byType(Container, skipOffstage: false), findsNWidgets(5));

    expect(find.byType(SliverFillRemaining), findsNothing);
    await tester.scrollUntilVisible(find.byType(SliverFillRemaining), 20);
    expect(find.byType(SliverFillRemaining), findsOneWidget);
    expect(find.byIcon(Icons.pan_tool), findsOneWidget);
    expect(tester.widget<Icon>(find.byIcon(Icons.pan_tool)).color, Colors.blueGrey);
  });
}
