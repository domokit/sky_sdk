// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/dropdown_menu/dropdown_menu.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('DropdownMenu', (WidgetTester tester) async {
    await tester.pumpWidget(const example.DropdownMenuExample());

    expect(find.text('You selected a Blue Smile'), findsNothing);

    final Finder colorMenu = find.byType(DropdownMenu<example.ColorLabel>);
    final Finder iconMenu = find.byType(DropdownMenu<example.IconLabel>);
    expect(colorMenu, findsOneWidget);
    expect(iconMenu, findsOneWidget);

    Finder findMenuItem(String label) {
      return find.widgetWithText(MenuItemButton, label).last;
    }

    await tester.tap(colorMenu);
    await tester.pumpAndSettle();
    expect(findMenuItem('Blue'), findsOneWidget);
    expect(findMenuItem('Pink'), findsOneWidget);
    expect(findMenuItem('Green'), findsOneWidget);
    expect(findMenuItem('Orange'), findsOneWidget);
    expect(findMenuItem('Grey'), findsOneWidget);

    await tester.tap(findMenuItem('Blue'));

    // The DropdownMenu's onSelected callback is delayed
    // with SchedulerBinding.instance.addPostFrameCallback
    // to give the focus a chance to return to where it was
    // before the menu appeared. The pumpAndSettle()
    // give the callback a chance to run.
    await tester.pumpAndSettle();

    await tester.tap(iconMenu);
    await tester.pumpAndSettle();
    expect(findMenuItem('Smile'), findsOneWidget);
    expect(findMenuItem('Cloud'), findsOneWidget);
    expect(findMenuItem('Brush'), findsOneWidget);
    expect(findMenuItem('Heart'), findsOneWidget);

    await tester.tap(findMenuItem('Smile'));
    await tester.pumpAndSettle();

    expect(find.text('You selected a Blue Smile'), findsOneWidget);
  });

  testWidgets('DropdownMenu has focus when tapping on the text field', (WidgetTester tester) async {
    await tester.pumpWidget(const example.DropdownMenuExample());

    // Make sure the dropdown menus are there.
    final Finder colorMenu = find.byType(DropdownMenu<example.ColorLabel>);
    final Finder iconMenu = find.byType(DropdownMenu<example.IconLabel>);
    expect(colorMenu, findsOneWidget);
    expect(iconMenu, findsOneWidget);

    // Tap on the color menu and make sure it is focused.
    await tester.tap(colorMenu);
    await tester.pumpAndSettle();
    expect(FocusScope.of(tester.element(colorMenu)).hasFocus, isTrue);

    // Tap on the icon menu and make sure it is focused.
    await tester.tap(iconMenu);
    await tester.pumpAndSettle();
    expect(FocusScope.of(tester.element(iconMenu)).hasFocus, isTrue);
  });

  testWidgets('DropdownMenu on small screen', (WidgetTester tester) async {
    await tester.pumpWidget(const example.DropdownMenuExample());

    final Finder colorMenu = find.byType(DropdownMenu<example.ColorLabel>);
    final Finder iconMenu = find.byType(DropdownMenu<example.IconLabel>);
    expect(colorMenu, findsOneWidget);
    expect(iconMenu, findsOneWidget);

    Finder findMenuItem(String label) {
      return find.widgetWithText(MenuItemButton, label).last;
    }

    await tester.tap(colorMenu);
    await tester.pumpAndSettle();
    final Finder menuBlue = findMenuItem('Blue');
    await tester.ensureVisible(menuBlue);
    await tester.tap(menuBlue);
    await tester.pumpAndSettle();

    await tester.tap(iconMenu);
    await tester.pumpAndSettle();
    final Finder menuSmile = findMenuItem('Smile');
    await tester.ensureVisible(menuSmile);
    await tester.tap(menuSmile);
    await tester.pumpAndSettle();

    expect(find.text('You selected a Blue Smile'), findsOneWidget);

    // Resize the screen to small screen and make sure no overflowed error appears.
    tester.binding.window.physicalSizeTestValue = const Size(200, 160);
    tester.binding.window.devicePixelRatioTestValue = 1.0;
    addTearDown(tester.binding.window.clearPhysicalSizeTestValue);
    await tester.pump();

    expect(tester.takeException(), isNull);
  });
}
