// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/autocomplete/raw_autocomplete.2.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Form is entirely visible and rejects invalid responses', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.AutocompleteExampleApp());
    expect(find.text('RawAutocomplete Form'), findsOneWidget);
    expect(find.byType(TextFormField), findsNWidgets(2));
    expect(find.byIcon(Icons.arrow_downward), findsOneWidget);
    expect(find.text('This is a regular DropdownButtonFormField'), findsOneWidget);
    expect(find.text('This is a regular TextFormField'), findsOneWidget);
    expect(find.text('This is a RawAutocomplete!'), findsOneWidget);
    expect(find.text('Submit'), findsOneWidget);

    expect(find.text('One'), findsNothing);
    expect(find.text('Two'), findsNothing);
    expect(find.text('Free'), findsNothing);
    expect(find.text('Four'), findsNothing);
    expect(find.text('aardvark'), findsNothing);
    expect(find.text('bobcat'), findsNothing);
    expect(find.text('chameleon'), findsNothing);

    expect(find.text('Must make a selection.'), findsNothing);
    expect(find.text("Can't be empty."), findsNothing);
    expect(find.text('Nothing selected.'), findsNothing);
    await tester.tap(find.text('Submit'));
    await tester.pump();
    expect(find.text('Must make a selection.'), findsOneWidget);
    expect(find.text("Can't be empty."), findsOneWidget);
    expect(find.text('Nothing selected.'), findsOneWidget);
  });

  testWidgets('Form accepts valid inputs', (WidgetTester tester) async {
    await tester.pumpWidget(const example.AutocompleteExampleApp());
    await tester.tap(find.byIcon(Icons.arrow_downward));
    await tester.pump();

    expect(find.text('One'), findsOneWidget);
    expect(find.text('Two'), findsOneWidget);
    expect(find.text('Free'), findsOneWidget);
    expect(find.text('Four'), findsOneWidget);
    await tester.tap(find.text('Free'));
    await tester.pump();
    expect(find.text('Two'), findsNothing);
    expect(find.text('Free'), findsOneWidget);

    expect(find.text('This is a regular TextFormField'), findsOneWidget);
    await tester.enterText(
      find.ancestor(
        of: find.text('This is a regular TextFormField'),
        matching: find.byType(TextFormField),
      ),
      'regular user input',
    );

    await tester.tap(
      find.ancestor(
        of: find.text('This is a RawAutocomplete!'),
        matching: find.byType(TextFormField),
      ),
    );
    await tester.pump();
    expect(find.text('aardvark'), findsOneWidget);
    expect(find.text('bobcat'), findsOneWidget);
    expect(find.text('chameleon'), findsOneWidget);
    await tester.tap(find.text('aardvark'));
    await tester.pump();

    expect(find.byType(AlertDialog), findsNothing);
    await tester.tap(find.text('Submit'));
    await tester.pump();
    expect(find.byType(AlertDialog), findsOneWidget);
    expect(find.text('Successfully submitted'), findsOneWidget);
    expect(find.text('DropdownButtonFormField: "Free"'), findsOneWidget);
    expect(find.text('TextFormField: "regular user input"'), findsOneWidget);
    expect(find.text('RawAutocomplete: "aardvark"'), findsOneWidget);
    expect(find.text('Ok'), findsOneWidget);

    await tester.tap(find.text('Ok'));
    await tester.pump();
    expect(find.byType(AlertDialog), findsNothing);
  });
}
