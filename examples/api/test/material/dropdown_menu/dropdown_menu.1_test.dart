// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/dropdown_menu/dropdown_menu.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets(
    'The DropdownMenu should display a menu with the list of entries the user can select',
    (WidgetTester tester) async {
      await tester.pumpWidget(const example.DropdownMenuApp());

      expect(find.widgetWithText(TextField, 'One'), findsOne);
      final Finder menu = find.byType(DropdownMenu<String>);
      expect(menu, findsOne);

      Finder findMenuItem(String label) {
        return find.widgetWithText(MenuItemButton, label).last;
      }

      await tester.tap(menu);
      await tester.pumpAndSettle();
      expect(findMenuItem('One'), findsOne);
      expect(findMenuItem('Two'), findsOne);
      expect(findMenuItem('Three'), findsOne);
      expect(findMenuItem('Four'), findsOne);

      await tester.tap(findMenuItem('Two'));

      // The DropdownMenu's onSelected callback is delayed
      // with SchedulerBinding.instance.addPostFrameCallback
      // to give the focus a chance to return to where it was
      // before the menu appeared. The pumpAndSettle()
      // give the callback a chance to run.
      await tester.pumpAndSettle();

      expect(find.widgetWithText(TextField, 'Two'), findsOne);
    },
  );

  testWidgets('DropdownMenu has focus when tapping on the text field', (WidgetTester tester) async {
    await tester.pumpWidget(const example.DropdownMenuApp());

    // Make sure the dropdown menus are there.
    final Finder menu = find.byType(DropdownMenu<String>);
    expect(menu, findsOne);

    // Tap on the menu and make sure it is focused.
    await tester.tap(menu);
    await tester.pumpAndSettle();
    expect(FocusScope.of(tester.element(menu)).hasFocus, isTrue);
  });
}
