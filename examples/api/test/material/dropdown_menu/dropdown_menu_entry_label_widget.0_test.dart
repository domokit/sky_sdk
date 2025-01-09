// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/dropdown_menu/dropdown_menu_entry_label_widget.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('DropdownEntryLabelWidget appears', (WidgetTester tester) async {
    await tester.pumpWidget(const example.DropdownMenuEntryLabelWidgetExampleApp());

    const String longText =
        'is a color that sings of hope, A hue that shines like gold. It is the color of dreams, A shade that never grows old.';
    Finder findMenuItemText(String label) {
      final String labelText = '$label $longText\n';
      return find
          .descendant(
            of: find.widgetWithText(MenuItemButton, labelText),
            matching: find.byType(Text),
          )
          .last;
    }

    // Open the menu
    await tester.tap(find.byType(TextField));
    expect(findMenuItemText('Blue'), findsOneWidget);
    expect(findMenuItemText('Pink'), findsOneWidget);
    expect(findMenuItemText('Green'), findsOneWidget);
    expect(findMenuItemText('Yellow'), findsOneWidget);
    expect(findMenuItemText('Grey'), findsOneWidget);

    // Close the menu
    await tester.tap(find.byType(TextField));
    await tester.pumpAndSettle();
  });
}
