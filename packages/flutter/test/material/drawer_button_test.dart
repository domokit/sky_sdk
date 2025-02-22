// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('DrawerButton control test', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(home: Scaffold(body: DrawerButton(), drawer: Drawer())),
    );

    await tester.pumpAndSettle();

    expect(find.byType(Drawer), findsNothing);

    await tester.tap(find.byType(DrawerButton));

    await tester.pumpAndSettle();

    expect(find.byType(Drawer), findsOneWidget);
  });

  testWidgets('DrawerButton onPressed overrides default end drawer open behaviour', (
    WidgetTester tester,
  ) async {
    bool customCallbackWasCalled = false;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Center(child: DrawerButton(onPressed: () => customCallbackWasCalled = true)),
          drawer: const Drawer(),
        ),
      ),
    );

    await tester.pumpAndSettle();
    expect(find.byType(Drawer), findsNothing); // Start off with a closed drawer
    expect(customCallbackWasCalled, false); // customCallbackWasCalled should still be false.
    await tester.tap(find.byType(DrawerButton));

    await tester.pumpAndSettle();

    // Drawer is still closed
    expect(find.byType(Drawer), findsNothing);
    // The custom callback is called, setting customCallbackWasCalled to true.
    expect(customCallbackWasCalled, true);
  });

  testWidgets('DrawerButton icon', (WidgetTester tester) async {
    final Key androidKey = UniqueKey();
    final Key iOSKey = UniqueKey();
    final Key linuxKey = UniqueKey();
    final Key macOSKey = UniqueKey();
    final Key windowsKey = UniqueKey();

    await tester.pumpWidget(
      MaterialApp(
        home: Column(
          children: <Widget>[
            Theme(
              data: ThemeData(platform: TargetPlatform.android),
              child: DrawerButtonIcon(key: androidKey),
            ),
            Theme(
              data: ThemeData(platform: TargetPlatform.iOS),
              child: DrawerButtonIcon(key: iOSKey),
            ),
            Theme(
              data: ThemeData(platform: TargetPlatform.linux),
              child: DrawerButtonIcon(key: linuxKey),
            ),
            Theme(
              data: ThemeData(platform: TargetPlatform.macOS),
              child: DrawerButtonIcon(key: macOSKey),
            ),
            Theme(
              data: ThemeData(platform: TargetPlatform.windows),
              child: DrawerButtonIcon(key: windowsKey),
            ),
          ],
        ),
      ),
    );

    final Icon androidIcon = tester.widget(
      find.descendant(of: find.byKey(androidKey), matching: find.byType(Icon)),
    );
    final Icon iOSIcon = tester.widget(
      find.descendant(of: find.byKey(iOSKey), matching: find.byType(Icon)),
    );
    final Icon linuxIcon = tester.widget(
      find.descendant(of: find.byKey(linuxKey), matching: find.byType(Icon)),
    );
    final Icon macOSIcon = tester.widget(
      find.descendant(of: find.byKey(macOSKey), matching: find.byType(Icon)),
    );
    final Icon windowsIcon = tester.widget(
      find.descendant(of: find.byKey(windowsKey), matching: find.byType(Icon)),
    );

    // All icons for drawer are the same
    expect(iOSIcon.icon == androidIcon.icon, isTrue);
    expect(linuxIcon.icon == androidIcon.icon, isTrue);
    expect(macOSIcon.icon == androidIcon.icon, isTrue);
    expect(windowsIcon.icon == androidIcon.icon, isTrue);
  });

  testWidgets('DrawerButton color', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(home: Material(child: DrawerButton(color: Colors.red))),
    );

    final RichText iconText = tester.firstWidget(
      find.descendant(of: find.byType(DrawerButton), matching: find.byType(RichText)),
    );
    expect(iconText.text.style!.color, Colors.red);
  });

  testWidgets('DrawerButton color with ButtonStyle', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Material(
          child: DrawerButton(
            style: ButtonStyle(iconColor: MaterialStatePropertyAll<Color>(Colors.red)),
          ),
        ),
      ),
    );

    final RichText iconText = tester.firstWidget(
      find.descendant(of: find.byType(DrawerButton), matching: find.byType(RichText)),
    );
    expect(iconText.text.style!.color, Colors.red);
  });

  testWidgets('DrawerButton semantics', (WidgetTester tester) async {
    final SemanticsHandle handle = tester.ensureSemantics();
    await tester.pumpWidget(
      const MaterialApp(home: Material(child: Center(child: DrawerButton()))),
    );

    await tester.pumpAndSettle();

    final String? expectedLabel;
    switch (defaultTargetPlatform) {
      case TargetPlatform.android:
        expectedLabel = 'Open navigation menu';
      case TargetPlatform.fuchsia:
      case TargetPlatform.iOS:
      case TargetPlatform.linux:
      case TargetPlatform.macOS:
      case TargetPlatform.windows:
        expectedLabel = null;
    }
    expect(
      tester.getSemantics(find.byType(DrawerButton)),
      matchesSemantics(
        tooltip: 'Open navigation menu',
        label: expectedLabel,
        isButton: true,
        hasEnabledState: true,
        isEnabled: true,
        hasTapAction: true,
        hasFocusAction: defaultTargetPlatform != TargetPlatform.iOS,
        isFocusable: true,
      ),
    );
    handle.dispose();
  }, variant: TargetPlatformVariant.all());

  testWidgets('EndDrawerButton control test', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(home: Scaffold(body: EndDrawerButton(), endDrawer: Drawer())),
    );

    await tester.pumpAndSettle();

    expect(find.byType(Drawer), findsNothing);

    await tester.tap(find.byType(EndDrawerButton));

    await tester.pumpAndSettle();

    expect(find.byType(Drawer), findsOneWidget);
  });

  testWidgets('EndDrawerButton semantics', (WidgetTester tester) async {
    final SemanticsHandle handle = tester.ensureSemantics();
    await tester.pumpWidget(
      const MaterialApp(home: Material(child: Center(child: EndDrawerButton()))),
    );

    await tester.pumpAndSettle();
    final String? expectedLabel;
    switch (defaultTargetPlatform) {
      case TargetPlatform.android:
        expectedLabel = 'Open navigation menu';
      case TargetPlatform.fuchsia:
      case TargetPlatform.iOS:
      case TargetPlatform.linux:
      case TargetPlatform.macOS:
      case TargetPlatform.windows:
        expectedLabel = null;
    }
    expect(
      tester.getSemantics(find.byType(EndDrawerButton)),
      matchesSemantics(
        tooltip: 'Open navigation menu',
        label: expectedLabel,
        isButton: true,
        hasEnabledState: true,
        isEnabled: true,
        hasTapAction: true,
        hasFocusAction: defaultTargetPlatform != TargetPlatform.iOS,
        isFocusable: true,
      ),
    );
    handle.dispose();
  }, variant: TargetPlatformVariant.all());

  testWidgets('EndDrawerButton color', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(home: Material(child: EndDrawerButton(color: Colors.red))),
    );

    final RichText iconText = tester.firstWidget(
      find.descendant(of: find.byType(EndDrawerButton), matching: find.byType(RichText)),
    );
    expect(iconText.text.style!.color, Colors.red);
  });

  testWidgets('EndDrawerButton color with ButtonStyle', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Material(
          child: EndDrawerButton(
            style: ButtonStyle(iconColor: MaterialStatePropertyAll<Color>(Colors.red)),
          ),
        ),
      ),
    );

    final RichText iconText = tester.firstWidget(
      find.descendant(of: find.byType(EndDrawerButton), matching: find.byType(RichText)),
    );
    expect(iconText.text.style!.color, Colors.red);
  });

  testWidgets('EndDrawerButton onPressed overrides default end drawer open behaviour', (
    WidgetTester tester,
  ) async {
    bool customCallbackWasCalled = false;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Center(child: EndDrawerButton(onPressed: () => customCallbackWasCalled = true)),
          endDrawer: const Drawer(),
        ),
      ),
    );

    await tester.pumpAndSettle();
    expect(find.byType(Drawer), findsNothing); // Start off with a closed drawer
    expect(customCallbackWasCalled, false); // customCallbackWasCalled should still be false.
    await tester.tap(find.byType(EndDrawerButton));

    await tester.pumpAndSettle();

    // Drawer is still closed
    expect(find.byType(Drawer), findsNothing);
    // The custom callback is called, setting customCallbackWasCalled to true.
    expect(customCallbackWasCalled, true);
  });
}
