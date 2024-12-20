// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/checkbox_list_tile/checkbox_list_tile.1.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Checkbox aligns appropriately', (WidgetTester tester) async {
    await tester.pumpWidget(const example.CheckboxListTileApp());

    expect(find.byType(CheckboxListTile), findsNWidgets(3));

    Offset tileTopLeft = tester.getTopLeft(find.byType(CheckboxListTile).at(0));
    Offset checkboxTopLeft = tester.getTopLeft(find.byType(Checkbox).at(0));

    // The checkbox is centered vertically with the text.
    expect(checkboxTopLeft - tileTopLeft, const Offset(736.0, 16.0));

    tileTopLeft = tester.getTopLeft(find.byType(CheckboxListTile).at(1));
    checkboxTopLeft = tester.getTopLeft(find.byType(Checkbox).at(1));

    // The checkbox is centered vertically with the text.
    expect(checkboxTopLeft - tileTopLeft, const Offset(736.0, 30.0));

    tileTopLeft = tester.getTopLeft(find.byType(CheckboxListTile).at(2));
    checkboxTopLeft = tester.getTopLeft(find.byType(Checkbox).at(2));

    // The checkbox is aligned to the top vertically with the text.
    expect(checkboxTopLeft - tileTopLeft, const Offset(736.0, 8.0));
  });

  testWidgets('Checkboxes can be checked', (WidgetTester tester) async {
    await tester.pumpWidget(const example.CheckboxListTileApp());

    expect(find.byType(CheckboxListTile), findsNWidgets(3));

    // All checkboxes are checked.
    expect(tester.widget<Checkbox>(find.byType(Checkbox).at(0)).value, isTrue);
    expect(tester.widget<Checkbox>(find.byType(Checkbox).at(1)).value, isTrue);
    expect(tester.widget<Checkbox>(find.byType(Checkbox).at(2)).value, isTrue);

    // Tap the first checkbox.
    await tester.tap(find.byType(Checkbox).at(0));
    await tester.pumpAndSettle();

    // The first checkbox is unchecked.
    expect(tester.widget<Checkbox>(find.byType(Checkbox).at(0)).value, isFalse);
    expect(tester.widget<Checkbox>(find.byType(Checkbox).at(1)).value, isTrue);
    expect(tester.widget<Checkbox>(find.byType(Checkbox).at(2)).value, isTrue);

    // Tap the second checkbox.
    await tester.tap(find.byType(Checkbox).at(1));
    await tester.pumpAndSettle();

    // The first and second checkboxes are unchecked.
    expect(tester.widget<Checkbox>(find.byType(Checkbox).at(0)).value, isFalse);
    expect(tester.widget<Checkbox>(find.byType(Checkbox).at(1)).value, isFalse);
    expect(tester.widget<Checkbox>(find.byType(Checkbox).at(2)).value, isTrue);

    // Tap the third checkbox.
    await tester.tap(find.byType(Checkbox).at(2));
    await tester.pumpAndSettle();

    // All checkboxes are unchecked.
    expect(tester.widget<Checkbox>(find.byType(Checkbox).at(0)).value, isFalse);
    expect(tester.widget<Checkbox>(find.byType(Checkbox).at(1)).value, isFalse);
    expect(tester.widget<Checkbox>(find.byType(Checkbox).at(2)).value, isFalse);
  });
}
