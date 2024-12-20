// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_api_samples/widgets/focus_manager/focus_node.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('FocusNode gets focused and unfocused on ColorfulButton tap', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.FocusNodeExampleApp());

    final Element button = tester.element(find.byType(example.ColorfulButton));

    expect(tester.binding.focusManager.primaryFocus?.context, isNot(equals(button)));

    // Tapping on ColorfulButton to focus on FocusNode.
    await tester.tap(find.byType(example.ColorfulButton));
    await tester.pump();

    expect(tester.binding.focusManager.primaryFocus?.context, equals(button));

    // Tapping on ColorfulButton to unfocus from FocusNode.
    await tester.tap(find.byType(example.ColorfulButton));
    await tester.pump();

    expect(tester.binding.focusManager.primaryFocus?.context, isNot(equals(button)));
  });

  testWidgets('FocusNode updates the text label when focused or unfocused', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.FocusNodeExampleApp());

    expect(find.text('Press to focus'), findsOneWidget);
    expect(find.text("I'm in color! Press R,G,B!"), findsNothing);

    // Tapping on ColorfulButton to focus on FocusNode.
    await tester.tap(find.byType(example.ColorfulButton));
    await tester.pump();

    expect(find.text('Press to focus'), findsNothing);
    expect(find.text("I'm in color! Press R,G,B!"), findsOneWidget);

    // Tapping on ColorfulButton to unfocus from FocusNode.
    await tester.tap(find.byType(example.ColorfulButton));
    await tester.pump();

    expect(find.text('Press to focus'), findsOneWidget);
    expect(find.text("I'm in color! Press R,G,B!"), findsNothing);
  });

  testWidgets('FocusNode updates color of the Container according to the key events when focused', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.FocusNodeExampleApp());

    // Tapping on ColorfulButton to focus on FocusNode.
    await tester.tap(find.byType(example.ColorfulButton));
    await tester.pump();

    final Finder containerFinder = find.descendant(
      of: find.byType(example.ColorfulButton),
      matching: find.byType(Container),
    );

    Container container = tester.widget<Container>(containerFinder);
    expect(container.color, equals(Colors.white));

    await tester.sendKeyEvent(LogicalKeyboardKey.keyR);
    await tester.pump();

    container = tester.widget<Container>(containerFinder);
    expect(container.color, equals(Colors.red));

    await tester.sendKeyEvent(LogicalKeyboardKey.keyG);
    await tester.pump();

    container = tester.widget<Container>(containerFinder);
    expect(container.color, equals(Colors.green));

    await tester.sendKeyEvent(LogicalKeyboardKey.keyB);
    await tester.pump();

    container = tester.widget<Container>(containerFinder);
    expect(container.color, equals(Colors.blue));
  });

  testWidgets('FocusNode does not listen to the key events when unfocused', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.FocusNodeExampleApp());

    final Finder containerFinder = find.descendant(
      of: find.byType(example.ColorfulButton),
      matching: find.byType(Container),
    );

    Container container = tester.widget<Container>(containerFinder);
    expect(container.color, equals(Colors.white));

    await tester.sendKeyEvent(LogicalKeyboardKey.keyR);
    await tester.pump();

    container = tester.widget<Container>(containerFinder);
    expect(container.color, equals(Colors.white));

    await tester.sendKeyEvent(LogicalKeyboardKey.keyG);
    await tester.pump();

    container = tester.widget<Container>(containerFinder);
    expect(container.color, equals(Colors.white));

    await tester.sendKeyEvent(LogicalKeyboardKey.keyB);
    await tester.pump();

    container = tester.widget<Container>(containerFinder);
    expect(container.color, equals(Colors.white));
  });

  testWidgets(
    'FocusNode sets color to the white when unfocused and sets it back to the selected one when focused',
    (WidgetTester tester) async {
      await tester.pumpWidget(const example.FocusNodeExampleApp());

      final Finder containerFinder = find.descendant(
        of: find.byType(example.ColorfulButton),
        matching: find.byType(Container),
      );

      Container container = tester.widget<Container>(containerFinder);
      expect(container.color, equals(Colors.white));

      // Tapping on ColorfulButton to focus on FocusNode.
      await tester.tap(find.byType(example.ColorfulButton));
      await tester.pump();

      container = tester.widget<Container>(containerFinder);
      expect(container.color, equals(Colors.white));

      await tester.sendKeyEvent(LogicalKeyboardKey.keyR);
      await tester.pump();

      container = tester.widget<Container>(containerFinder);
      expect(container.color, equals(Colors.red));

      // Tapping on ColorfulButton to unfocus from FocusNode.
      await tester.tap(find.byType(example.ColorfulButton));
      await tester.pump();

      container = tester.widget<Container>(containerFinder);
      expect(container.color, equals(Colors.white));

      // Tapping on ColorfulButton to focus on FocusNode.
      await tester.tap(find.byType(example.ColorfulButton));
      await tester.pump();

      container = tester.widget<Container>(containerFinder);
      expect(container.color, equals(Colors.red));
    },
  );
}
