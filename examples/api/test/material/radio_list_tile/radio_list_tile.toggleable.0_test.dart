// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/radio_list_tile/radio_list_tile.toggleable.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('RadioListTile is toggleable', (WidgetTester tester) async {
    await tester.pumpWidget(const example.RadioListTileApp());

    // Initially the third radio button is not selected.
    Radio<int> radio = tester.widget(find.byType(Radio<int>).at(2));
    expect(radio.value, 2);
    expect(radio.groupValue, null);

    // Tap the third radio button.
    await tester.tap(find.text('Philip Schuyler'));
    await tester.pumpAndSettle();

    // The third radio button is now selected.
    radio = tester.widget(find.byType(Radio<int>).at(2));
    expect(radio.value, 2);
    expect(radio.groupValue, 2);

    // Tap the third radio button again.
    await tester.tap(find.text('Philip Schuyler'));
    await tester.pumpAndSettle();

    // The third radio button is now unselected.
    radio = tester.widget(find.byType(Radio<int>).at(2));
    expect(radio.value, 2);
    expect(radio.groupValue, null);
  });
}
