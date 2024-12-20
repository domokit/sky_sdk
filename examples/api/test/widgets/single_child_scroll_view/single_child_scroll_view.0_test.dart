// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_api_samples/widgets/single_child_scroll_view/single_child_scroll_view.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('The children should be spaced out equally when the screen is big enough', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.SingleChildScrollViewExampleApp());

    expect(find.text('Fixed Height Content'), findsExactly(2));
    expect(tester.getTopLeft(find.byType(Container).first), const Offset(0, 90));
    expect(tester.getTopLeft(find.byType(Container).last), const Offset(0, 390));

    await tester.fling(find.byType(SingleChildScrollView).last, const Offset(0, -100), 10.0);

    // The view should not scroll when the screen is big enough.
    expect(tester.getTopLeft(find.byType(Container).first), const Offset(0, 90));
    expect(tester.getTopLeft(find.byType(Container).last), const Offset(0, 390));
  });

  testWidgets('The view should be scrollable when the screen is not big enough', (
    WidgetTester tester,
  ) async {
    tester.view
      ..physicalSize = const Size(400, 200)
      ..devicePixelRatio = 1;
    addTearDown(tester.view.reset);
    await tester.pumpWidget(const example.SingleChildScrollViewExampleApp());

    expect(find.text('Fixed Height Content'), findsExactly(2));
    expect(tester.getTopLeft(find.byType(Container).first), Offset.zero);
    expect(tester.getTopLeft(find.byType(Container).last), const Offset(0, 120));

    await tester.fling(find.byType(SingleChildScrollView).last, const Offset(0, -40), 10.0);

    // The view should scroll when the screen is not big enough.
    expect(tester.getTopLeft(find.byType(Container).first), const Offset(0, -40));
    expect(tester.getTopLeft(find.byType(Container).last), const Offset(0, 80));
  });
}
