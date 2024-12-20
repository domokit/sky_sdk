// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/autocomplete/autocomplete.4.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets(
    'can search and find options after waiting for fake network delay and debounce delay',
    (WidgetTester tester) async {
      await tester.pumpWidget(const example.AutocompleteExampleApp());

      expect(find.text('aardvark'), findsNothing);
      expect(find.text('bobcat'), findsNothing);
      expect(find.text('chameleon'), findsNothing);

      await tester.enterText(find.byType(TextFormField), 'a');
      await tester.pump(example.fakeAPIDuration);

      // No results yet, need to also wait for the debounce duration.
      expect(find.text('aardvark'), findsNothing);
      expect(find.text('bobcat'), findsNothing);
      expect(find.text('chameleon'), findsNothing);

      await tester.pump(example.debounceDuration);

      expect(find.text('aardvark'), findsOneWidget);
      expect(find.text('bobcat'), findsOneWidget);
      expect(find.text('chameleon'), findsOneWidget);

      await tester.enterText(find.byType(TextFormField), 'aa');
      await tester.pump(example.debounceDuration + example.fakeAPIDuration);

      expect(find.text('aardvark'), findsOneWidget);
      expect(find.text('bobcat'), findsNothing);
      expect(find.text('chameleon'), findsNothing);
    },
  );

  testWidgets('debounce is reset each time a character is entered', (WidgetTester tester) async {
    await tester.pumpWidget(const example.AutocompleteExampleApp());

    await tester.enterText(find.byType(TextFormField), 'c');
    await tester.pump(example.debounceDuration - const Duration(milliseconds: 100));

    expect(find.text('aardvark'), findsNothing);
    expect(find.text('bobcat'), findsNothing);
    expect(find.text('chameleon'), findsNothing);

    await tester.enterText(find.byType(TextFormField), 'ch');
    await tester.pump(example.debounceDuration - const Duration(milliseconds: 100));

    expect(find.text('aardvark'), findsNothing);
    expect(find.text('bobcat'), findsNothing);
    expect(find.text('chameleon'), findsNothing);

    await tester.enterText(find.byType(TextFormField), 'cha');
    await tester.pump(example.debounceDuration - const Duration(milliseconds: 100));

    expect(find.text('aardvark'), findsNothing);
    expect(find.text('bobcat'), findsNothing);
    expect(find.text('chameleon'), findsNothing);

    await tester.enterText(find.byType(TextFormField), 'cham');
    await tester.pump(example.debounceDuration - const Duration(milliseconds: 100));

    // Despite the total elapsed time being greater than debounceDuration +
    // fakeAPIDuration, the search has not yet completed, because the debounce
    // was reset each time text input happened.
    expect(find.text('aardvark'), findsNothing);
    expect(find.text('bobcat'), findsNothing);
    expect(find.text('chameleon'), findsNothing);

    await tester.enterText(find.byType(TextFormField), 'chame');
    await tester.pump(example.debounceDuration + example.fakeAPIDuration);

    expect(find.text('aardvark'), findsNothing);
    expect(find.text('bobcat'), findsNothing);
    expect(find.text('chameleon'), findsOneWidget);
  });

  testWidgets('shows an error message for network errors', (WidgetTester tester) async {
    await tester.pumpWidget(const example.AutocompleteExampleApp());

    await tester.enterText(find.byType(TextFormField), 'chame');
    await tester.pump(example.debounceDuration + example.fakeAPIDuration);

    expect(find.text('aardvark'), findsNothing);
    expect(find.text('bobcat'), findsNothing);
    expect(find.text('chameleon'), findsOneWidget);
    InputDecorator inputDecorator = tester.widget(find.byType(InputDecorator));
    expect(inputDecorator.decoration.errorText, isNull);

    // Turn the network off.
    await tester.tap(find.byType(Switch));
    await tester.pump();

    await tester.enterText(find.byType(TextFormField), 'chamel');
    await tester.pump(example.debounceDuration + example.fakeAPIDuration);

    expect(find.text('aardvark'), findsNothing);
    expect(find.text('bobcat'), findsNothing);
    expect(find.text('chameleon'), findsNothing);
    inputDecorator = tester.widget(find.byType(InputDecorator));
    expect(inputDecorator.decoration.errorText, isNotNull);

    // Turn the network back on.
    await tester.tap(find.byType(Switch));
    await tester.pump();

    await tester.enterText(find.byType(TextFormField), 'chamele');
    await tester.pump(example.debounceDuration + example.fakeAPIDuration);

    expect(find.text('aardvark'), findsNothing);
    expect(find.text('bobcat'), findsNothing);
    expect(find.text('chameleon'), findsOneWidget);
    inputDecorator = tester.widget(find.byType(InputDecorator));
    expect(inputDecorator.decoration.errorText, isNull);
  });
}
