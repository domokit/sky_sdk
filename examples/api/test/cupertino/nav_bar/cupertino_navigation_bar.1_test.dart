// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter_api_samples/cupertino/nav_bar/cupertino_navigation_bar.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('CupertinoNavigationBar with bottom widget', (WidgetTester tester) async {
    await tester.pumpWidget(const example.NavBarApp());

    final Finder navBarFinder = find.byType(CupertinoNavigationBar);
    final Finder searchFieldFinder = find.byType(CupertinoSearchTextField);

    expect(navBarFinder, findsOneWidget);
    expect(searchFieldFinder, findsOneWidget);

    // The bottom widget is bounded by the navigation bar.
    expect(
      tester.getBottomLeft(searchFieldFinder).dy,
      lessThan(tester.getBottomLeft(navBarFinder).dy),
    );
  });
}
