// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/switch_list_tile/switch_list_tile.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('SwitchListTile can be toggled', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SwitchListTileApp());

    expect(find.byType(SwitchListTile), findsOneWidget);

    SwitchListTile switchListTile = tester.widget(find.byType(SwitchListTile));
    expect(switchListTile.value, isFalse);

    await tester.tap(find.byType(SwitchListTile));
    await tester.pumpAndSettle();

    switchListTile = tester.widget(find.byType(SwitchListTile));
    expect(switchListTile.value, isTrue);
  });
}
