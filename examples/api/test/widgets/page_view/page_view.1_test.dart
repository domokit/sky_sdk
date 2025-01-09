// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/page_view/page_view.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('tapping Reverse button should reverse PageView', (WidgetTester tester) async {
    await tester.pumpWidget(const example.PageViewExampleApp());
    final Finder pageView = find.byType(PageView);
    final Finder reverseFinder = find.text('Reverse items');
    final Finder firstItemFinder = find.byKey(const ValueKey<String>('1'));
    final Finder lastItemFinder = find.byKey(const ValueKey<String>('5'));
    expect(pageView, findsOneWidget);
    expect(reverseFinder, findsOneWidget);
    expect(firstItemFinder, findsOneWidget);
    expect(lastItemFinder, findsNothing);
    await tester.tap(reverseFinder);
    await tester.pump();
    expect(firstItemFinder, findsNothing);
    expect(lastItemFinder, findsOneWidget);
  });
}
