// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/autocomplete/raw_autocomplete.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Autocomplete example is visible', (WidgetTester tester) async {
    await tester.pumpWidget(const example.AutocompleteExampleApp());
    expect(find.text('RawAutocomplete Custom Type'), findsOneWidget);
    expect(find.byType(TextFormField), findsOneWidget);

    expect(find.text('Alice'), findsNothing);
    expect(find.text('Bob'), findsNothing);
    expect(find.text('Charlie'), findsNothing);
  });

  testWidgets('Options are shown correctly and selectable', (WidgetTester tester) async {
    await tester.pumpWidget(const example.AutocompleteExampleApp());
    await tester.tap(find.byType(TextFormField));
    await tester.pump();

    expect(find.byType(ListTile), findsNWidgets(3));
    expect(find.text('Alice'), findsOneWidget);
    expect(find.text('Bob'), findsOneWidget);
    expect(find.text('Charlie'), findsOneWidget);

    await tester.enterText(find.byType(TextFormField), 'b');
    await tester.pump();

    expect(find.byType(ListTile), findsOneWidget);
    expect(find.text('Alice'), findsNothing);
    expect(find.text('Bob'), findsOneWidget);
    expect(find.text('Charlie'), findsNothing);

    await tester.tap(find.text('Bob'));
    await tester.pump();

    expect(find.byType(ListTile), findsNothing);
    expect(
      find.descendant(of: find.byType(TextFormField), matching: find.text('Bob')),
      findsOneWidget,
    );
  });

  testWidgets('Finds users by email address', (WidgetTester tester) async {
    await tester.pumpWidget(const example.AutocompleteExampleApp());

    await tester.enterText(find.byType(TextFormField), '@');
    await tester.pump();

    expect(find.byType(ListTile), findsNWidgets(3));
    expect(find.text('Alice'), findsOneWidget);
    expect(find.text('Bob'), findsOneWidget);
    expect(find.text('Charlie'), findsOneWidget);

    await tester.enterText(find.byType(TextFormField), '@gmail');
    await tester.pump();

    expect(find.byType(ListTile), findsOneWidget);
    expect(find.text('Alice'), findsNothing);
    expect(find.text('Bob'), findsNothing);
    expect(find.text('Charlie'), findsOneWidget);

    await tester.tap(find.text('Charlie'));
    await tester.pump();

    expect(find.byType(ListTile), findsNothing);
    expect(
      find.descendant(of: find.byType(TextFormField), matching: find.text('Charlie')),
      findsOneWidget,
    );
  });
}
