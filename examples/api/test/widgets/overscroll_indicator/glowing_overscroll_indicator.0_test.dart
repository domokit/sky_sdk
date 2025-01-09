// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_api_samples/widgets/overscroll_indicator/glowing_overscroll_indicator.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Displays widget tree when the example app is run', (WidgetTester tester) async {
    await tester.pumpWidget(const example.GlowingOverscrollIndicatorExampleApp());

    expect(
      find.descendant(
        of: find.byType(Scaffold),
        matching: find.widgetWithText(AppBar, 'GlowingOverscrollIndicator Sample'),
      ),
      findsOne,
    );

    final Finder customScrollViewFinder = find.byType(CustomScrollView);
    final Finder sliverAppBarFinder = find.descendant(
      of: customScrollViewFinder,
      matching: find.widgetWithText(SliverAppBar, 'Custom PaintOffset'),
    );

    expect(sliverAppBarFinder, findsOne);

    expect(
      find.descendant(
        of: customScrollViewFinder,
        matching: find.widgetWithText(Center, 'Glow all day!'),
      ),
      findsOne,
    );

    expect(
      find.descendant(of: customScrollViewFinder, matching: find.byType(SliverToBoxAdapter)),
      findsOne,
    );

    expect(
      find.descendant(
        of: customScrollViewFinder,
        matching: find.widgetWithIcon(SliverFillRemaining, Icons.sunny),
      ),
      findsOne,
    );

    expect(
      find.descendant(
        of: customScrollViewFinder,
        matching: find.byType(GlowingOverscrollIndicator),
      ),
      findsOne,
    );

    // Check if GlowingOverscrollIndicator overlays the SliverAppBar.
    final RenderBox overscrollIndicator = tester.renderObject<RenderBox>(
      find.descendant(
        of: customScrollViewFinder,
        matching: find.byType(GlowingOverscrollIndicator),
      ),
    );
    final RenderSliver sliverAppBar = tester.renderObject<RenderSliver>(sliverAppBarFinder);
    final Matrix4 transform = overscrollIndicator.getTransformTo(sliverAppBar);
    final Offset? offset = MatrixUtils.getAsTranslation(transform);
    expect(offset?.dy, 0);
  });

  testWidgets('Triggers a notification listener when the screen is dragged', (
    WidgetTester tester,
  ) async {
    bool overscrollNotified = false;
    double leadingPaintOffset = 0.0;

    await tester.pumpWidget(
      NotificationListener<OverscrollIndicatorNotification>(
        onNotification: (OverscrollIndicatorNotification notification) {
          overscrollNotified = true;
          leadingPaintOffset = notification.paintOffset;
          return false;
        },
        child: const example.GlowingOverscrollIndicatorExampleApp(),
      ),
    );

    expect(leadingPaintOffset, 0);
    expect(overscrollNotified, isFalse);
    final BuildContext context = tester.element(find.byType(MaterialApp));

    await tester.drag(find.byType(CustomScrollView), const Offset(0, 500));
    await tester.pump();

    expect(leadingPaintOffset, MediaQuery.paddingOf(context).top + kToolbarHeight);
    expect(overscrollNotified, isTrue);
  });
}
