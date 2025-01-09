// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/sliver_fill/sliver_fill_remaining.3.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Shows all elements', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SliverFillRemainingExampleApp());
    expect(find.text('SliverFillRemaining Sample'), findsOneWidget);
    expect(find.byType(ElevatedButton), findsOneWidget);
    expect(find.text('Bottom Pinned Button!'), findsOneWidget);
    expect(find.byType(CustomScrollView), findsOneWidget);
    final CustomScrollView scroll = tester.widget(find.byType(CustomScrollView));
    expect(
      scroll.physics,
      isA<BouncingScrollPhysics>().having(
        (BouncingScrollPhysics bsp) => bsp.parent,
        'parent',
        isA<AlwaysScrollableScrollPhysics>(),
      ),
    );

    expect(
      find.byWidgetPredicate(
        (Widget widget) => (widget is Container) && widget.color == Colors.tealAccent[700],
      ),
      findsOneWidget,
    );
    expect(
      find.byWidgetPredicate(
        (Widget widget) => (widget is Container) && widget.color == Colors.teal[100],
      ),
      findsOneWidget,
    );
    expect(find.byType(Container), findsNWidgets(2));

    expect(find.byType(SliverFillRemaining), findsOneWidget);
    final SliverFillRemaining fill = tester.widget(find.byType(SliverFillRemaining));
    expect(fill.hasScrollBody, false);
    expect(fill.fillOverscroll, true);
  });
}
