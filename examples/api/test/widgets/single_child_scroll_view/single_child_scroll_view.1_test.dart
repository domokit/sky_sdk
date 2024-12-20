// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_api_samples/widgets/single_child_scroll_view/single_child_scroll_view.1.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('The flexible child should fill the space if the screen is big enough', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.SingleChildScrollViewExampleApp());

    final Finder fixedHeightFinder = find.widgetWithText(Container, 'Fixed Height Content');
    final Finder flexibleHeightFinder = find.widgetWithText(Container, 'Flexible Content');
    expect(tester.getTopLeft(fixedHeightFinder), Offset.zero);
    expect(tester.getSize(fixedHeightFinder), const Size(800, 120));
    expect(tester.getTopLeft(flexibleHeightFinder), const Offset(0, 120));
    expect(tester.getSize(flexibleHeightFinder), const Size(800, 480));

    await tester.fling(find.byType(SingleChildScrollView).last, const Offset(0, -100), 10.0);

    // The view should not scroll when the screen is big enough.
    expect(tester.getTopLeft(fixedHeightFinder), Offset.zero);
    expect(tester.getSize(fixedHeightFinder), const Size(800, 120));
    expect(tester.getTopLeft(flexibleHeightFinder), const Offset(0, 120));
    expect(tester.getSize(flexibleHeightFinder), const Size(800, 480));
  });

  testWidgets('The view should be scrollable when the screen is not big enough', (
    WidgetTester tester,
  ) async {
    tester.view
      ..physicalSize = const Size(400, 200)
      ..devicePixelRatio = 1;
    addTearDown(tester.view.reset);
    await tester.pumpWidget(const example.SingleChildScrollViewExampleApp());

    final Finder fixedHeightFinder = find.widgetWithText(Container, 'Fixed Height Content');
    final Finder flexibleHeightFinder = find.widgetWithText(Container, 'Flexible Content');
    expect(tester.getTopLeft(fixedHeightFinder), Offset.zero);
    expect(tester.getSize(fixedHeightFinder), const Size(400, 120));
    expect(tester.getTopLeft(flexibleHeightFinder), const Offset(0, 120));
    expect(tester.getSize(flexibleHeightFinder), const Size(400, 120));

    await tester.fling(find.byType(SingleChildScrollView).last, const Offset(0, -40), 10.0);

    // The view should scroll when the screen is not big enough.
    expect(tester.getTopLeft(fixedHeightFinder), const Offset(0, -40));
    expect(tester.getSize(fixedHeightFinder), const Size(400, 120));
    expect(tester.getTopLeft(flexibleHeightFinder), const Offset(0, 80));
    expect(tester.getSize(flexibleHeightFinder), const Size(400, 120));
  });
}
