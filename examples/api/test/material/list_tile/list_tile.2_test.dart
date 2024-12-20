// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/list_tile/list_tile.2.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('ListTile leading and trailing widgets are aligned appropriately', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.ListTileApp());

    expect(find.byType(ListTile), findsNWidgets(3));

    Offset listTileTopLeft = tester.getTopLeft(find.byType(ListTile).at(0));
    Offset leadingTopLeft = tester.getTopLeft(find.byType(CircleAvatar).at(0));
    Offset trailingTopLeft = tester.getTopLeft(find.byType(Icon).at(0));

    // The leading and trailing widgets are centered vertically with the text.
    expect(leadingTopLeft - listTileTopLeft, const Offset(16.0, 16.0));
    expect(trailingTopLeft - listTileTopLeft, const Offset(752.0, 24.0));

    listTileTopLeft = tester.getTopLeft(find.byType(ListTile).at(1));
    leadingTopLeft = tester.getTopLeft(find.byType(CircleAvatar).at(1));
    trailingTopLeft = tester.getTopLeft(find.byType(Icon).at(1));

    // The leading and trailing widgets are centered vertically with the text.
    expect(leadingTopLeft - listTileTopLeft, const Offset(16.0, 30.0));
    expect(trailingTopLeft - listTileTopLeft, const Offset(752.0, 38.0));

    listTileTopLeft = tester.getTopLeft(find.byType(ListTile).at(2));
    leadingTopLeft = tester.getTopLeft(find.byType(CircleAvatar).at(2));
    trailingTopLeft = tester.getTopLeft(find.byType(Icon).at(2));

    // The leading and trailing widgets are aligned to the top vertically with the text.
    expect(leadingTopLeft - listTileTopLeft, const Offset(16.0, 8.0));
    expect(trailingTopLeft - listTileTopLeft, const Offset(752.0, 8.0));
  });
}
