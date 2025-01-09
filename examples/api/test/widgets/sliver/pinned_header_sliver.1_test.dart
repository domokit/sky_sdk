// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/sliver/pinned_header_sliver.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('PinnedHeaderSliver iOS Settings example', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SettingsAppBarApp());

    // Verify that the app contains a header: a SliverPersistentHeader
    // with an AnimatedOpacity widget below it, and a Text('Settings') below that.
    expect(find.byType(PinnedHeaderSliver), findsOneWidget);
    expect(
      find.descendant(of: find.byType(PinnedHeaderSliver), matching: find.byType(AnimatedOpacity)),
      findsOneWidget,
    );
    expect(find.widgetWithText(AnimatedOpacity, 'Settings'), findsOneWidget);

    // Verify that the app contains a "Settings" title: a SliverToBoxAdapter
    // with a Text('Settings') widget below it.
    expect(find.widgetWithText(SliverToBoxAdapter, 'Settings'), findsOneWidget);

    final Finder headerOpacity = find.widgetWithText(AnimatedOpacity, 'Settings');
    expect(tester.widget<AnimatedOpacity>(headerOpacity).opacity, 0);

    // Scroll up: the header's opacity goes to 1 and the title disappears.
    await tester.timedDrag(
      find.byType(CustomScrollView),
      const Offset(0, -500),
      const Duration(milliseconds: 500),
    );
    await tester.pumpAndSettle();
    expect(tester.widget<AnimatedOpacity>(headerOpacity).opacity, 1);
    expect(find.widgetWithText(SliverToBoxAdapter, 'Settings'), findsNothing);

    // Scroll back down and we're back to where we started.
    await tester.timedDrag(
      find.byType(CustomScrollView),
      const Offset(0, 500),
      const Duration(milliseconds: 500),
    );
    await tester.pumpAndSettle();
    expect(tester.widget<AnimatedOpacity>(headerOpacity).opacity, 0);
    expect(find.widgetWithText(SliverToBoxAdapter, 'Settings'), findsOneWidget);
  });
}
