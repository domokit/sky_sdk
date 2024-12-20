// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/autocomplete/raw_autocomplete.focus_node.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Autocomplete example is visible', (WidgetTester tester) async {
    await tester.pumpWidget(const example.AutocompleteExampleApp());
    expect(find.text('Split RawAutocomplete App'), findsOneWidget);
    expect(find.byType(AppBar), findsOneWidget);
    expect(find.byType(TextFormField), findsOneWidget);
    expect(
      find.descendant(of: find.byType(AppBar), matching: find.byType(TextFormField)),
      findsOneWidget,
    );

    expect(find.byType(RawAutocomplete<String>), findsOneWidget);
    expect(find.text('aardvark'), findsNothing);
    expect(find.text('bobcat'), findsNothing);
    expect(find.text('chameleon'), findsNothing);
    expect(
      find.ancestor(matching: find.byType(AppBar), of: find.byType(RawAutocomplete)),
      findsNothing,
    );
  });

  testWidgets('Options are shown correctly and selectable', (WidgetTester tester) async {
    await tester.pumpWidget(const example.AutocompleteExampleApp());
    await tester.tap(find.byType(TextFormField));
    await tester.pump();

    expect(find.byType(ListTile), findsNWidgets(3));
    expect(find.text('aardvark'), findsOneWidget);
    expect(find.text('bobcat'), findsOneWidget);
    expect(find.text('chameleon'), findsOneWidget);

    await tester.enterText(find.byType(TextFormField), 'b');
    await tester.pump();

    expect(find.byType(ListTile), findsOneWidget);
    expect(find.text('aardvark'), findsNothing);
    expect(find.text('bobcat'), findsOneWidget);
    expect(find.text('chameleon'), findsNothing);

    await tester.tap(find.text('bobcat'));
    await tester.pump();

    expect(find.byType(ListTile), findsNothing);
    expect(
      find.descendant(of: find.byType(TextFormField), matching: find.text('bobcat')),
      findsOneWidget,
    );
  });
}
