// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/radio_list_tile/radio_list_tile.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Radio aligns appropriately', (WidgetTester tester) async {
    await tester.pumpWidget(const example.RadioListTileApp());

    expect(find.byType(RadioListTile<example.Groceries>), findsNWidgets(3));

    Offset tileTopLeft = tester.getTopLeft(find.byType(RadioListTile<example.Groceries>).at(0));
    Offset radioTopLeft = tester.getTopLeft(find.byType(Radio<example.Groceries>).at(0));

    // The radio is centered vertically with the text.
    expect(radioTopLeft - tileTopLeft, const Offset(16.0, 16.0));

    tileTopLeft = tester.getTopLeft(find.byType(RadioListTile<example.Groceries>).at(1));
    radioTopLeft = tester.getTopLeft(find.byType(Radio<example.Groceries>).at(1));

    // The radio is centered vertically with the text.
    expect(radioTopLeft - tileTopLeft, const Offset(16.0, 30.0));

    tileTopLeft = tester.getTopLeft(find.byType(RadioListTile<example.Groceries>).at(2));
    radioTopLeft = tester.getTopLeft(find.byType(Radio<example.Groceries>).at(2));

    // The radio is aligned to the top vertically with the text.
    expect(radioTopLeft - tileTopLeft, const Offset(16.0, 8.0));
  });

  testWidgets('Radios can be checked', (WidgetTester tester) async {
    await tester.pumpWidget(const example.RadioListTileApp());

    expect(find.byType(RadioListTile<example.Groceries>), findsNWidgets(3));
    final Finder radioListTile = find.byType(RadioListTile<example.Groceries>);

    //  Initially the first radio is checked.
    expect(
      tester.widget<RadioListTile<example.Groceries>>(radioListTile.at(0)).groupValue,
      example.Groceries.pickles,
    );
    expect(
      tester.widget<RadioListTile<example.Groceries>>(radioListTile.at(1)).groupValue,
      example.Groceries.pickles,
    );
    expect(
      tester.widget<RadioListTile<example.Groceries>>(radioListTile.at(2)).groupValue,
      example.Groceries.pickles,
    );

    // Tap the second radio.
    await tester.tap(find.byType(Radio<example.Groceries>).at(1));
    await tester.pumpAndSettle();

    // The second radio is checked.
    expect(
      tester.widget<RadioListTile<example.Groceries>>(radioListTile.at(0)).groupValue,
      example.Groceries.tomato,
    );
    expect(
      tester.widget<RadioListTile<example.Groceries>>(radioListTile.at(1)).groupValue,
      example.Groceries.tomato,
    );
    expect(
      tester.widget<RadioListTile<example.Groceries>>(radioListTile.at(2)).groupValue,
      example.Groceries.tomato,
    );

    // Tap the third radio.
    await tester.tap(find.byType(Radio<example.Groceries>).at(2));
    await tester.pumpAndSettle();

    // The third radio is checked.
    expect(
      tester.widget<RadioListTile<example.Groceries>>(radioListTile.at(0)).groupValue,
      example.Groceries.lettuce,
    );
    expect(
      tester.widget<RadioListTile<example.Groceries>>(radioListTile.at(1)).groupValue,
      example.Groceries.lettuce,
    );
    expect(
      tester.widget<RadioListTile<example.Groceries>>(radioListTile.at(2)).groupValue,
      example.Groceries.lettuce,
    );
  });
}
