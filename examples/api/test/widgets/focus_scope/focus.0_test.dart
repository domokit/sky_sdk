// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_api_samples/widgets/focus_scope/focus.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('shows initial content', (WidgetTester tester) async {
    await tester.pumpWidget(const example.FocusExampleApp());
    expect(find.byType(Scaffold), findsOneWidget);
    expect(find.byType(AppBar), findsOneWidget);
    expect(find.byType(FocusScope), findsAtLeastNWidgets(1));
    expect(find.text('Press to focus'), findsOneWidget);
    expect(find.text('Focus Sample'), findsOneWidget);
    expect(find.byType(Container), findsOneWidget);

    final Container container = tester.widget<Container>(find.byType(Container));
    expect(container.color, Colors.white);
  });

  testWidgets('switches to focus mode', (WidgetTester tester) async {
    await tester.pumpWidget(const example.FocusExampleApp());
    expect(find.text('Press to focus'), findsOneWidget);
    expect(find.text("I'm in color! Press R,G,B!"), findsNothing);
    await tester.tap(find.text('Press to focus'));
    await tester.pumpAndSettle();
    expect(find.text('Press to focus'), findsNothing);
    expect(find.text("I'm in color! Press R,G,B!"), findsOneWidget);

    expect(find.byType(Container), findsOneWidget);
    final Container container = tester.widget<Container>(find.byType(Container));
    expect(container.color, Colors.white);

    await tester.tap(find.text("I'm in color! Press R,G,B!"));
    await tester.pumpAndSettle();
    expect(find.text('Press to focus'), findsOneWidget);
    expect(find.text("I'm in color! Press R,G,B!"), findsNothing);
  });

  testWidgets('changes color according to key presses', (WidgetTester tester) async {
    await tester.pumpWidget(const example.FocusExampleApp());
    expect(find.byType(Container), findsOneWidget);
    expect(tester.widget<Container>(find.byType(Container)).color, Colors.white);
    await tester.sendKeyEvent(LogicalKeyboardKey.keyR);
    await tester.pumpAndSettle();
    expect(tester.widget<Container>(find.byType(Container)).color, Colors.white);
    await tester.sendKeyEvent(LogicalKeyboardKey.keyG);
    await tester.pumpAndSettle();
    expect(tester.widget<Container>(find.byType(Container)).color, Colors.white);
    await tester.sendKeyEvent(LogicalKeyboardKey.keyB);
    await tester.pumpAndSettle();
    expect(tester.widget<Container>(find.byType(Container)).color, Colors.white);

    expect(find.text('Press to focus'), findsOneWidget);
    await tester.tap(find.text('Press to focus'));
    await tester.pumpAndSettle();
    expect(find.byType(Container), findsOneWidget);
    expect(tester.widget<Container>(find.byType(Container)).color, Colors.white);
    await tester.sendKeyEvent(LogicalKeyboardKey.keyR);
    await tester.pumpAndSettle();
    expect(tester.widget<Container>(find.byType(Container)).color, Colors.red);
    await tester.sendKeyEvent(LogicalKeyboardKey.keyG);
    await tester.pumpAndSettle();
    expect(tester.widget<Container>(find.byType(Container)).color, Colors.green);
    await tester.sendKeyEvent(LogicalKeyboardKey.keyB);
    await tester.pumpAndSettle();
    expect(tester.widget<Container>(find.byType(Container)).color, Colors.blue);

    expect(find.text("I'm in color! Press R,G,B!"), findsOneWidget);
    await tester.tap(find.text("I'm in color! Press R,G,B!"));
    await tester.pumpAndSettle();
    expect(tester.widget<Container>(find.byType(Container)).color, Colors.white);
  });
}
