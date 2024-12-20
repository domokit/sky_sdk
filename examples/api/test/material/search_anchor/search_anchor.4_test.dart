// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/search_anchor/search_anchor.4.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets(
    'can search and find options after waiting for fake network delay and debounce delay',
    (WidgetTester tester) async {
      await tester.pumpWidget(const example.SearchAnchorAsyncExampleApp());

      await tester.tap(find.byIcon(Icons.search));
      await tester.pumpAndSettle();

      expect(find.widgetWithText(ListTile, 'aardvark'), findsNothing);
      expect(find.widgetWithText(ListTile, 'bobcat'), findsNothing);
      expect(find.widgetWithText(ListTile, 'chameleon'), findsNothing);

      await tester.enterText(find.byType(SearchBar), 'a');
      await tester.pump(example.fakeAPIDuration);

      // No results yet, need to also wait for the debounce duration.
      expect(find.widgetWithText(ListTile, 'aardvark'), findsNothing);
      expect(find.widgetWithText(ListTile, 'bobcat'), findsNothing);
      expect(find.widgetWithText(ListTile, 'chameleon'), findsNothing);

      await tester.pump(example.debounceDuration);

      expect(find.widgetWithText(ListTile, 'aardvark'), findsOneWidget);
      expect(find.widgetWithText(ListTile, 'bobcat'), findsOneWidget);
      expect(find.widgetWithText(ListTile, 'chameleon'), findsOneWidget);

      await tester.enterText(find.byType(SearchBar), 'aa');
      await tester.pump(example.debounceDuration + example.fakeAPIDuration);

      expect(find.widgetWithText(ListTile, 'aardvark'), findsOneWidget);
      expect(find.widgetWithText(ListTile, 'bobcat'), findsNothing);
      expect(find.widgetWithText(ListTile, 'chameleon'), findsNothing);
    },
  );

  testWidgets('debounce is reset each time a character is entered', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SearchAnchorAsyncExampleApp());

    await tester.tap(find.byIcon(Icons.search));
    await tester.pumpAndSettle();

    await tester.enterText(find.byType(SearchBar), 'c');
    await tester.pump(example.debounceDuration - const Duration(milliseconds: 100));

    expect(find.widgetWithText(ListTile, 'aardvark'), findsNothing);
    expect(find.widgetWithText(ListTile, 'bobcat'), findsNothing);
    expect(find.widgetWithText(ListTile, 'chameleon'), findsNothing);

    await tester.enterText(find.byType(SearchBar), 'ch');
    await tester.pump(example.debounceDuration - const Duration(milliseconds: 100));

    expect(find.widgetWithText(ListTile, 'aardvark'), findsNothing);
    expect(find.widgetWithText(ListTile, 'bobcat'), findsNothing);
    expect(find.widgetWithText(ListTile, 'chameleon'), findsNothing);

    await tester.enterText(find.byType(SearchBar), 'cha');
    await tester.pump(example.debounceDuration - const Duration(milliseconds: 100));

    expect(find.widgetWithText(ListTile, 'aardvark'), findsNothing);
    expect(find.widgetWithText(ListTile, 'bobcat'), findsNothing);
    expect(find.widgetWithText(ListTile, 'chameleon'), findsNothing);

    await tester.enterText(find.byType(SearchBar), 'cham');
    await tester.pump(example.debounceDuration - const Duration(milliseconds: 100));

    // Despite the total elapsed time being greater than debounceDuration +
    // fakeAPIDuration, the search has not yet completed, because the debounce
    // was reset each time text input happened.
    expect(find.widgetWithText(ListTile, 'aardvark'), findsNothing);
    expect(find.widgetWithText(ListTile, 'bobcat'), findsNothing);
    expect(find.widgetWithText(ListTile, 'chameleon'), findsNothing);

    await tester.enterText(find.byType(SearchBar), 'chame');
    await tester.pump(example.debounceDuration + example.fakeAPIDuration);

    expect(find.widgetWithText(ListTile, 'aardvark'), findsNothing);
    expect(find.widgetWithText(ListTile, 'bobcat'), findsNothing);
    expect(find.widgetWithText(ListTile, 'chameleon'), findsOneWidget);
  });
}
