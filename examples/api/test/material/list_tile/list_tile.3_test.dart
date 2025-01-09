// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_api_samples/material/list_tile/list_tile.3.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('ListTile color properties respect Material state color', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.ListTileApp());
    ListTile listTile = tester.widget(find.byType(ListTile));

    // Enabled list tile uses black color for icon and headline.
    expect(listTile.enabled, true);
    expect(listTile.selected, false);
    RenderParagraph headline = _getTextRenderObject(tester, 'Headline');
    expect(headline.text.style!.color, Colors.black);
    RichText icon = tester.widget(find.byType(RichText).at(0));
    expect(icon.text.style!.color, Colors.black);

    // Tap list tile to select it.
    await tester.tap(find.byType(ListTile));
    await tester.pumpAndSettle();

    // Selected list tile uses green color for icon and headline.
    listTile = tester.widget(find.byType(ListTile));
    expect(listTile.enabled, true);
    expect(listTile.selected, true);
    headline = _getTextRenderObject(tester, 'Headline');
    expect(headline.text.style!.color, Colors.green);
    icon = tester.widget(find.byType(RichText).at(0));
    expect(icon.text.style!.color, Colors.green);

    // Tap switch to disable list tile.
    await tester.tap(find.byType(Switch));
    await tester.pumpAndSettle();

    // Disabled list tile uses red color for icon and headline.
    listTile = tester.widget(find.byType(ListTile));
    expect(listTile.enabled, false);
    expect(listTile.selected, true);
    headline = _getTextRenderObject(tester, 'Headline');
    expect(headline.text.style!.color, Colors.red);
    icon = tester.widget(find.byType(RichText).at(0));
    expect(icon.text.style!.color, Colors.red);
  });
}

RenderParagraph _getTextRenderObject(WidgetTester tester, String text) {
  return tester.renderObject(find.descendant(of: find.byType(ListTile), matching: find.text(text)));
}
