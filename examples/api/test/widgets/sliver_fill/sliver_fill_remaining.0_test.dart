// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/sliver_fill/sliver_fill_remaining.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('shows how SliverFillRemaining takes up remaining space', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.SliverFillRemainingExampleApp());
    expect(find.text('SliverFillRemaining Sample'), findsOneWidget);
    expect(find.byType(CustomScrollView), findsOneWidget);

    expect(
      find.descendant(of: find.byType(SliverToBoxAdapter), matching: find.byType(Container)),
      findsOneWidget,
    );
    final Container upperContainer = tester.widget(
      find.descendant(of: find.byType(SliverToBoxAdapter), matching: find.byType(Container)),
    );
    expect(upperContainer.color, Colors.amber[300]);

    expect(find.byType(SliverFillRemaining), findsOneWidget);
    expect(
      find.descendant(of: find.byType(SliverFillRemaining), matching: find.byType(Container)),
      findsOneWidget,
    );
    expect(find.byIcon(Icons.sentiment_very_satisfied), findsOneWidget);
    final Icon lowerIcon = tester.widget(
      find.descendant(of: find.byType(SliverFillRemaining), matching: find.byType(Icon)),
    );
    expect(lowerIcon.color, Colors.blue[900]);

    final double total = tester.getSize(find.byType(CustomScrollView)).height;
    final double upperHeight =
        tester
            .getSize(
              find.descendant(
                of: find.byType(SliverToBoxAdapter),
                matching: find.byType(Container),
              ),
            )
            .height;
    final double lowerHeight =
        tester
            .getSize(
              find.descendant(of: find.byType(SliverFillRemaining), matching: find.byType(Icon)),
            )
            .height;
    expect(upperHeight + lowerHeight, equals(total));
  });
}
