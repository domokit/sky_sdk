// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/switch_list_tile/switch_list_tile.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Switch aligns appropriately', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SwitchListTileApp());

    expect(find.byType(SwitchListTile), findsNWidgets(3));

    Offset tileTopLeft = tester.getTopLeft(find.byType(SwitchListTile).at(0));
    Offset switchTopLeft = tester.getTopLeft(find.byType(Switch).at(0));

    // The switch is centered vertically with the text.
    expect(switchTopLeft - tileTopLeft, const Offset(716.0, 16.0));

    tileTopLeft = tester.getTopLeft(find.byType(SwitchListTile).at(1));
    switchTopLeft = tester.getTopLeft(find.byType(Switch).at(1));

    // The switch is centered vertically with the text.
    expect(switchTopLeft - tileTopLeft, const Offset(716.0, 30.0));

    tileTopLeft = tester.getTopLeft(find.byType(SwitchListTile).at(2));
    switchTopLeft = tester.getTopLeft(find.byType(Switch).at(2));

    // The switch is aligned to the top vertically with the text.
    expect(switchTopLeft - tileTopLeft, const Offset(716.0, 8.0));
  });

  testWidgets('Switches can be checked', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SwitchListTileApp());

    expect(find.byType(SwitchListTile), findsNWidgets(3));

    // All switches are on.
    expect(tester.widget<Switch>(find.byType(Switch).at(0)).value, isTrue);
    expect(tester.widget<Switch>(find.byType(Switch).at(1)).value, isTrue);
    expect(tester.widget<Switch>(find.byType(Switch).at(2)).value, isTrue);

    // Tap the first switch.
    await tester.tap(find.byType(Switch).at(0));
    await tester.pumpAndSettle();

    // The first switch is off.
    expect(tester.widget<Switch>(find.byType(Switch).at(0)).value, isFalse);
    expect(tester.widget<Switch>(find.byType(Switch).at(1)).value, isTrue);
    expect(tester.widget<Switch>(find.byType(Switch).at(2)).value, isTrue);

    // Tap the second switch.
    await tester.tap(find.byType(Switch).at(1));
    await tester.pumpAndSettle();

    // The first and second switches are off.
    expect(tester.widget<Switch>(find.byType(Switch).at(0)).value, isFalse);
    expect(tester.widget<Switch>(find.byType(Switch).at(1)).value, isFalse);
    expect(tester.widget<Switch>(find.byType(Switch).at(2)).value, isTrue);

    // Tap the third switch.
    await tester.tap(find.byType(Switch).at(2));
    await tester.pumpAndSettle();

    // All switches are off.
    expect(tester.widget<Switch>(find.byType(Switch).at(0)).value, isFalse);
    expect(tester.widget<Switch>(find.byType(Switch).at(1)).value, isFalse);
    expect(tester.widget<Switch>(find.byType(Switch).at(2)).value, isFalse);
  });
}
