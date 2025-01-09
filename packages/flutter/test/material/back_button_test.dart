// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('BackButton control test', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: const Material(child: Text('Home')),
        routes: <String, WidgetBuilder>{
          '/next': (BuildContext context) {
            return const Material(child: Center(child: BackButton()));
          },
        },
      ),
    );

    tester.state<NavigatorState>(find.byType(Navigator)).pushNamed('/next');

    await tester.pumpAndSettle();

    await tester.tap(find.byType(BackButton));

    await tester.pumpAndSettle();

    expect(find.text('Home'), findsOneWidget);
  });

  testWidgets('BackButton onPressed overrides default pop behavior', (WidgetTester tester) async {
    bool customCallbackWasCalled = false;
    await tester.pumpWidget(
      MaterialApp(
        home: const Material(child: Text('Home')),
        routes: <String, WidgetBuilder>{
          '/next': (BuildContext context) {
            return Material(
              child: Center(child: BackButton(onPressed: () => customCallbackWasCalled = true)),
            );
          },
        },
      ),
    );

    tester.state<NavigatorState>(find.byType(Navigator)).pushNamed('/next');

    await tester.pumpAndSettle();

    expect(find.text('Home'), findsNothing); // Start off on the second page.
    expect(customCallbackWasCalled, false); // customCallbackWasCalled should still be false.
    await tester.tap(find.byType(BackButton));

    await tester.pumpAndSettle();

    // We're still on the second page.
    expect(find.text('Home'), findsNothing);
    // But the custom callback is called.
    expect(customCallbackWasCalled, true);
  });

  testWidgets('BackButton icon', (WidgetTester tester) async {
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
              child: BackButtonIcon(key: androidKey),
            ),
            Theme(
              data: ThemeData(platform: TargetPlatform.iOS),
              child: BackButtonIcon(key: iOSKey),
            ),
            Theme(
              data: ThemeData(platform: TargetPlatform.linux),
              child: BackButtonIcon(key: linuxKey),
            ),
            Theme(
              data: ThemeData(platform: TargetPlatform.macOS),
              child: BackButtonIcon(key: macOSKey),
            ),
            Theme(
              data: ThemeData(platform: TargetPlatform.windows),
              child: BackButtonIcon(key: windowsKey),
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
    expect(iOSIcon.icon == androidIcon.icon, kIsWeb ? isTrue : isFalse);
    expect(linuxIcon.icon == androidIcon.icon, isTrue);
    expect(macOSIcon.icon == androidIcon.icon, kIsWeb ? isTrue : isFalse);
    expect(macOSIcon.icon == iOSIcon.icon, isTrue);
    expect(windowsIcon.icon == androidIcon.icon, isTrue);
  });

  testWidgets('BackButton color', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(home: Material(child: BackButton(color: Colors.red))),
    );

    final RichText iconText = tester.firstWidget(
      find.descendant(of: find.byType(BackButton), matching: find.byType(RichText)),
    );
    expect(iconText.text.style!.color, Colors.red);
  });

  testWidgets('BackButton color with ButtonStyle', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: true),
        home: const Material(
          child: BackButton(
            style: ButtonStyle(iconColor: MaterialStatePropertyAll<Color>(Colors.red)),
          ),
        ),
      ),
    );

    final RichText iconText = tester.firstWidget(
      find.descendant(of: find.byType(BackButton), matching: find.byType(RichText)),
    );
    expect(iconText.text.style!.color, Colors.red);
  });

  testWidgets('BackButton.style.iconColor parameter overrides BackButton.color', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: true),
        home: const Material(
          child: BackButton(
            color: Colors.green,
            style: ButtonStyle(iconColor: MaterialStatePropertyAll<Color>(Colors.red)),
          ),
        ),
      ),
    );

    final RichText iconText = tester.firstWidget(
      find.descendant(of: find.byType(BackButton), matching: find.byType(RichText)),
    );

    expect(iconText.text.style!.color, Colors.red);
  });

  testWidgets('BackButton semantics', (WidgetTester tester) async {
    final SemanticsHandle handle = tester.ensureSemantics();
    await tester.pumpWidget(
      MaterialApp(
        home: const Material(child: Text('Home')),
        routes: <String, WidgetBuilder>{
          '/next': (BuildContext context) {
            return const Material(child: Center(child: BackButton()));
          },
        },
      ),
    );

    tester.state<NavigatorState>(find.byType(Navigator)).pushNamed('/next');

    await tester.pumpAndSettle();
    final String? expectedLabel;
    switch (defaultTargetPlatform) {
      case TargetPlatform.android:
        expectedLabel = 'Back';
      case TargetPlatform.fuchsia:
      case TargetPlatform.iOS:
      case TargetPlatform.linux:
      case TargetPlatform.macOS:
      case TargetPlatform.windows:
        expectedLabel = null;
    }
    expect(
      tester.getSemantics(find.byType(BackButton)),
      matchesSemantics(
        tooltip: 'Back',
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

  testWidgets('CloseButton semantics', (WidgetTester tester) async {
    final SemanticsHandle handle = tester.ensureSemantics();
    await tester.pumpWidget(
      MaterialApp(
        home: const Material(child: Text('Home')),
        routes: <String, WidgetBuilder>{
          '/next': (BuildContext context) {
            return const Material(child: Center(child: CloseButton()));
          },
        },
      ),
    );

    tester.state<NavigatorState>(find.byType(Navigator)).pushNamed('/next');

    await tester.pumpAndSettle();
    final String? expectedLabel;
    switch (defaultTargetPlatform) {
      case TargetPlatform.android:
        expectedLabel = 'Close';
      case TargetPlatform.fuchsia:
      case TargetPlatform.iOS:
      case TargetPlatform.linux:
      case TargetPlatform.macOS:
      case TargetPlatform.windows:
        expectedLabel = null;
    }
    expect(
      tester.getSemantics(find.byType(CloseButton)),
      matchesSemantics(
        tooltip: 'Close',
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

  testWidgets('CloseButton color', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(home: Material(child: CloseButton(color: Colors.red))),
    );

    final RichText iconText = tester.firstWidget(
      find.descendant(of: find.byType(CloseButton), matching: find.byType(RichText)),
    );
    expect(iconText.text.style!.color, Colors.red);
  });

  testWidgets('CloseButton color with ButtonStyle', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: true),
        home: const Material(
          child: CloseButton(
            style: ButtonStyle(iconColor: MaterialStatePropertyAll<Color>(Colors.red)),
          ),
        ),
      ),
    );

    final RichText iconText = tester.firstWidget(
      find.descendant(of: find.byType(CloseButton), matching: find.byType(RichText)),
    );
    expect(iconText.text.style!.color, Colors.red);
  });

  testWidgets('CloseButton.style.iconColor parameter overrides CloseButton.color', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: true),
        home: const Material(
          child: CloseButton(
            color: Colors.green,
            style: ButtonStyle(iconColor: MaterialStatePropertyAll<Color>(Colors.red)),
          ),
        ),
      ),
    );

    final RichText iconText = tester.firstWidget(
      find.descendant(of: find.byType(CloseButton), matching: find.byType(RichText)),
    );

    expect(iconText.text.style!.color, Colors.red);
  });

  testWidgets('CloseButton onPressed overrides default pop behavior', (WidgetTester tester) async {
    bool customCallbackWasCalled = false;
    await tester.pumpWidget(
      MaterialApp(
        home: const Material(child: Text('Home')),
        routes: <String, WidgetBuilder>{
          '/next': (BuildContext context) {
            return Material(
              child: Center(child: CloseButton(onPressed: () => customCallbackWasCalled = true)),
            );
          },
        },
      ),
    );

    tester.state<NavigatorState>(find.byType(Navigator)).pushNamed('/next');

    await tester.pumpAndSettle();
    expect(find.text('Home'), findsNothing); // Start off on the second page.
    expect(customCallbackWasCalled, false); // customCallbackWasCalled should still be false.
    await tester.tap(find.byType(CloseButton));

    await tester.pumpAndSettle();

    // We're still on the second page.
    expect(find.text('Home'), findsNothing);
    // The custom callback is called, setting customCallbackWasCalled to true.
    expect(customCallbackWasCalled, true);
  });
}
