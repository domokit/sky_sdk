// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/app_bar/sliver_app_bar.3.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Visibility and interaction of crucial widgets', (WidgetTester tester) async {
    await tester.pumpWidget(const example.AppBarLargeApp());

    const String title = 'Large App Bar';

    expect(
      find.descendant(
        of: find.byType(CustomScrollView),
        matching: find.widgetWithText(SliverAppBar, title),
      ),
      findsOne,
    );

    expect(
      find.descendant(of: find.byType(SliverAppBar), matching: find.byType(IconButton)),
      findsExactly(2),
    );

    // Based on https://m3.material.io/components/top-app-bar/specs the title of
    // the SliverAppBar.large widget is formatted with the headlineMedium style.
    final BuildContext context = tester.element(find.byType(MaterialApp));
    final TextStyle expectedTitleStyle = Theme.of(context).textTheme.headlineMedium!;

    // There are two Text widgets: expanded and collapsed. The expanded is first.
    final Finder titleFinder = find.text(title).first;
    final TextStyle actualTitleStyle = DefaultTextStyle.of(tester.element(titleFinder)).style;

    expect(actualTitleStyle, expectedTitleStyle);

    // Scrolling the screen moves the title up.
    expect(tester.getBottomLeft(titleFinder).dy, 124.0);
    await tester.drag(titleFinder, const Offset(0.0, -200.0));
    await tester.pump();
    expect(tester.getBottomLeft(titleFinder).dy, 36.0);
  });
}
