// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_api_samples/widgets/focus_scope/focus.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('FocusableText shows content and color depending on focus', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      const MaterialApp(home: Scaffold(body: example.FocusableText('Item 0', autofocus: false))),
    );
    // Autofocus needs to check that no other node in the [FocusScope] is
    // focused and can only request focus for the second frame.
    await tester.pumpAndSettle();
    expect(
      find.descendant(of: find.byType(example.FocusableText), matching: find.byType(Focus)),
      findsOneWidget,
    );
    expect(find.text('Item 0'), findsOneWidget);

    expect(find.byType(Container), findsOneWidget);
    final Container container1 = tester.widget<Container>(find.byType(Container));
    expect(container1.color, Colors.white);

    await tester.pumpWidget(
      const MaterialApp(home: Scaffold(body: example.FocusableText('Item 1', autofocus: true))),
    );
    await tester.pumpAndSettle();

    final Finder focusableTextFinder2 = find.ancestor(
      of: find.text('Item 1'),
      matching: find.byType(example.FocusableText),
    );
    expect(
      tester
          .widget<Focus>(find.descendant(of: focusableTextFinder2, matching: find.byType(Focus)))
          .autofocus,
      isTrue,
    );
    final Container container2 = tester.widget<Container>(
      find.descendant(of: focusableTextFinder2, matching: find.byType(Container)),
    );
    expect(container2.color, Colors.red);
  });

  testWidgets('builds list showcasing focus traversal', (WidgetTester tester) async {
    await tester.pumpWidget(const example.FocusExampleApp());
    await tester.pumpAndSettle();

    expect(find.byType(ListView), findsOneWidget);

    final Finder childFinder = find.descendant(
      of: find.byType(ListView),
      matching: find.byType(example.FocusableText),
    );
    expect(childFinder, findsAtLeastNWidgets(2));

    Container container0 = tester.widget<Container>(
      find.descendant(of: childFinder.first, matching: find.byType(Container)),
    );
    Container container1 = tester.widget<Container>(
      find.descendant(of: childFinder.at(1), matching: find.byType(Container)),
    );
    expect(container0.color, Colors.red);
    expect(container1.color, Colors.white);

    await tester.sendKeyDownEvent(LogicalKeyboardKey.arrowDown);
    await tester.pumpAndSettle();

    container0 = tester.widget<Container>(
      find.descendant(of: childFinder.first, matching: find.byType(Container)),
    );
    container1 = tester.widget<Container>(
      find.descendant(of: childFinder.at(1), matching: find.byType(Container)),
    );
    expect(container0.color, Colors.white);
    expect(container1.color, Colors.red);
  });
}
