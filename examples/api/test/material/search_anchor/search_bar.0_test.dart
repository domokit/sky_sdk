// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/search_anchor/search_bar.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Can open search view', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SearchBarApp());

    final Finder searchBarFinder = find.byType(SearchBar);
    final SearchBar searchBar = tester.widget<SearchBar>(searchBarFinder);
    expect(find.byIcon(Icons.search), findsOneWidget);
    expect(searchBar.trailing, isNotEmpty);
    expect(searchBar.trailing?.length, equals(1));
    final Finder trailingButtonFinder = find.widgetWithIcon(IconButton, Icons.wb_sunny_outlined);
    expect(trailingButtonFinder, findsOneWidget);

    await tester.tap(trailingButtonFinder);
    await tester.pumpAndSettle();

    expect(find.widgetWithIcon(IconButton, Icons.brightness_2_outlined), findsOneWidget);

    await tester.tap(searchBarFinder);
    await tester.pumpAndSettle();

    expect(find.text('item 0'), findsOneWidget);
    expect(find.text('item 1'), findsOneWidget);
    expect(find.text('item 2'), findsOneWidget);
    expect(find.text('item 3'), findsOneWidget);
    expect(find.text('item 4'), findsOneWidget);
  });
}
