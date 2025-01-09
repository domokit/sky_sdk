// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/popup_menu/popup_menu.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Can open popup menu', (WidgetTester tester) async {
    const String menuItem = 'Item 1';

    await tester.pumpWidget(const example.PopupMenuApp());

    expect(find.text(menuItem), findsNothing);

    // Open popup menu.
    await tester.tap(find.byIcon(Icons.adaptive.more));
    await tester.pumpAndSettle();
    expect(find.text(menuItem), findsOneWidget);

    // Close popup menu.
    await tester.tapAt(const Offset(1, 1));
    await tester.pumpAndSettle();
    expect(find.text(menuItem), findsNothing);
  });
}
