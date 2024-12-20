// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/menu_anchor/menu_anchor.2.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('The menu should display three items', (WidgetTester tester) async {
    await tester.pumpWidget(const example.MenuAnchorApp());

    expect(find.widgetWithText(AppBar, 'MenuAnchorButton'), findsOne);
    expect(
      find.descendant(
        of: find.byType(MenuAnchor),
        matching: find.widgetWithIcon(IconButton, Icons.more_horiz),
      ),
      findsOne,
    );

    await tester.tap(find.byIcon(Icons.more_horiz));
    await tester.pumpAndSettle();

    for (int i = 1; i <= 3; i++) {
      expect(find.widgetWithText(MenuItemButton, 'Item $i'), findsOne);
    }
  });
}
