// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/radio_list_tile/radio_list_tile.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Can update RadioListTile group value', (WidgetTester tester) async {
    await tester.pumpWidget(const example.RadioListTileApp());

    // Find the number of RadioListTiles.
    expect(find.byType(RadioListTile<example.SingingCharacter>), findsNWidgets(2));

    // The initial group value is lafayette for the first RadioListTile.
    RadioListTile<example.SingingCharacter> radioListTile = tester.widget(
      find.byType(RadioListTile<example.SingingCharacter>).first,
    );
    expect(radioListTile.groupValue, example.SingingCharacter.lafayette);

    // The initial group value is lafayette for the last RadioListTile.
    radioListTile = tester.widget(find.byType(RadioListTile<example.SingingCharacter>).last);
    expect(radioListTile.groupValue, example.SingingCharacter.lafayette);

    // Tap the last RadioListTile to change the group value to jefferson.
    await tester.tap(find.byType(RadioListTile<example.SingingCharacter>).last);
    await tester.pump();

    // The group value is now jefferson for the first RadioListTile.
    radioListTile = tester.widget(find.byType(RadioListTile<example.SingingCharacter>).first);
    expect(radioListTile.groupValue, example.SingingCharacter.jefferson);

    // The group value is now jefferson for the last RadioListTile.
    radioListTile = tester.widget(find.byType(RadioListTile<example.SingingCharacter>).last);
    expect(radioListTile.groupValue, example.SingingCharacter.jefferson);
  });
}
