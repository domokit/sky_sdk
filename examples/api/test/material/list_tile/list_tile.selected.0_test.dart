// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/list_tile/list_tile.selected.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('ListTile item can be selected', (WidgetTester tester) async {
    await tester.pumpWidget(const example.ListTileApp());

    expect(find.byType(ListTile), findsNWidgets(10));

    // The first item is selected by default.
    expect(tester.widget<ListTile>(find.byType(ListTile).at(0)).selected, true);

    // Tap a list item to select it.
    await tester.tap(find.byType(ListTile).at(5));
    await tester.pump();

    // The first item is no longer selected.
    expect(tester.widget<ListTile>(find.byType(ListTile).at(0)).selected, false);
    expect(tester.widget<ListTile>(find.byType(ListTile).at(5)).selected, true);
  });
}
