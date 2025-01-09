// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/navigation_bar/navigation_bar.2.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('RootPage: only selected destination is on stage', (WidgetTester tester) async {
    await tester.pumpWidget(const MaterialApp(home: example.Home()));

    const String tealTitle = 'Teal RootPage - /';
    const String cyanTitle = 'Cyan RootPage - /';
    const String orangeTitle = 'Orange RootPage - /';
    const String blueTitle = 'Blue RootPage - /';

    await tester.tap(find.widgetWithText(NavigationDestination, 'Teal'));
    await tester.pumpAndSettle();
    expect(find.text(tealTitle), findsOneWidget);
    expect(find.text(cyanTitle), findsNothing);
    expect(find.text(orangeTitle), findsNothing);
    expect(find.text(blueTitle), findsNothing);

    await tester.tap(find.widgetWithText(NavigationDestination, 'Cyan'));
    await tester.pumpAndSettle();
    expect(find.text(tealTitle), findsNothing);
    expect(find.text(cyanTitle), findsOneWidget);
    expect(find.text(orangeTitle), findsNothing);
    expect(find.text(blueTitle), findsNothing);

    await tester.tap(find.widgetWithText(NavigationDestination, 'Orange'));
    await tester.pumpAndSettle();
    expect(find.text(tealTitle), findsNothing);
    expect(find.text(cyanTitle), findsNothing);
    expect(find.text(orangeTitle), findsOneWidget);
    expect(find.text(blueTitle), findsNothing);

    await tester.tap(find.widgetWithText(NavigationDestination, 'Blue'));
    await tester.pumpAndSettle();
    expect(find.text(tealTitle), findsNothing);
    expect(find.text(cyanTitle), findsNothing);
    expect(find.text(orangeTitle), findsNothing);
    expect(find.text(blueTitle), findsOneWidget);
  });

  testWidgets('RootPage', (WidgetTester tester) async {
    await tester.pumpWidget(const MaterialApp(home: example.Home()));

    await tester.tap(find.widgetWithText(NavigationDestination, 'Teal'));
    await tester.pumpAndSettle();
    await tester.tap(find.text('Local Dialog'));
    await tester.pumpAndSettle();
    expect(find.text('Teal AlertDialog'), findsOneWidget);
    await tester.tap(find.text('OK'));
    await tester.pumpAndSettle();
    expect(find.text('Teal AlertDialog'), findsNothing);

    await tester.pumpAndSettle();
    await tester.tap(find.text('Root Dialog'));
    await tester.pumpAndSettle();
    expect(find.text('Teal AlertDialog'), findsOneWidget);
    await tester.tapAt(const Offset(5, 5));
    await tester.pumpAndSettle();
    expect(find.text('Teal AlertDialog'), findsNothing);

    await tester.tap(find.text('Local BottomSheet'));
    await tester.pumpAndSettle();
    expect(find.byType(BottomSheet), findsOneWidget);
    await tester.tap(find.byType(BackButton));
    await tester.pumpAndSettle();
    expect(find.byType(BottomSheet), findsNothing);

    await tester.tap(find.text('Push /list'));
    await tester.pumpAndSettle();
    expect(find.text('Teal ListPage - /list'), findsOneWidget);
  });

  testWidgets('ListPage', (WidgetTester tester) async {
    await tester.pumpWidget(const MaterialApp(home: example.Home()));
    expect(find.text('Teal RootPage - /'), findsOneWidget);

    await tester.tap(find.widgetWithText(ElevatedButton, 'Push /list'));
    await tester.pumpAndSettle();
    expect(find.text('Teal ListPage - /list'), findsOneWidget);
    expect(find.text('Push /text [0]'), findsOneWidget);

    await tester.tap(find.widgetWithText(NavigationDestination, 'Orange'));
    await tester.pumpAndSettle();
    await tester.tap(find.widgetWithText(ElevatedButton, 'Push /list'));
    await tester.pumpAndSettle();
    expect(find.text('Orange ListPage - /list'), findsOneWidget);
    expect(find.text('Push /text [0]'), findsOneWidget);

    await tester.tap(find.byType(BackButton));
    await tester.pumpAndSettle();
    expect(find.text('Orange RootPage - /'), findsOneWidget);

    await tester.tap(find.widgetWithText(NavigationDestination, 'Teal'));
    await tester.pumpAndSettle();
    expect(find.text('Teal ListPage - /list'), findsOneWidget);

    await tester.tap(find.byType(BackButton));
    await tester.pumpAndSettle();
    expect(find.text('Teal RootPage - /'), findsOneWidget);
  });
}
