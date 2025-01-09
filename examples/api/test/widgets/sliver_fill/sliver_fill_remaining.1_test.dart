// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/sliver_fill/sliver_fill_remaining.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Shows all elements', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SliverFillRemainingExampleApp());
    expect(find.text('SliverFillRemaining Sample'), findsOneWidget);
    expect(find.byType(CustomScrollView), findsOneWidget);
    expect(
      find.byWidgetPredicate(
        (Widget widget) => (widget is Container) && widget.color == Colors.amber[200],
      ),
      findsNWidgets(2),
    );
    expect(
      find.byWidgetPredicate(
        (Widget widget) => (widget is Container) && widget.color == Colors.blue[200],
      ),
      findsOneWidget,
    );
    expect(
      find.byWidgetPredicate(
        (Widget widget) => (widget is Container) && widget.color == Colors.orange[300],
      ),
      findsOneWidget,
    );
    expect(find.byType(SliverFixedExtentList), findsOneWidget);
    expect(find.byType(SliverFillRemaining), findsOneWidget);
    expect(find.byType(FlutterLogo), findsOneWidget);
  });

  testWidgets('Fills up all available space', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SliverFillRemainingExampleApp());

    final double listSpace = tester.getSize(find.byType(CustomScrollView)).height;
    double contentHeight = 0.0;
    for (final Widget widget in tester.widgetList(
      find.byWidgetPredicate(
        (Widget widget) =>
            (widget is Container) &&
            <Color>[
              Colors.orange[300]!,
              Colors.blue[200]!,
              Colors.amber[200]!,
            ].contains(widget.color),
      ),
    )) {
      contentHeight += tester.getSize(find.byWidget(widget)).height;
    }
    expectLater(contentHeight, equals(listSpace));
  });
}
