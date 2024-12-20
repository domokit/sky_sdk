// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/basic/indexed_stack.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('has correct forward rendering mechanism', (WidgetTester tester) async {
    await tester.pumpWidget(const example.IndexedStackApp());

    final Finder gesture2 = find.byKey(const Key('gesture2'));
    final Element containerFinder =
        find.byKey(const Key('Dash'), skipOffstage: false).evaluate().first;
    expect(containerFinder.renderObject!.debugNeedsPaint, false);
    final Element containerFinder1 =
        find.byKey(const Key('John'), skipOffstage: false).evaluate().first;
    expect(containerFinder1.renderObject!.debugNeedsPaint, true);
    final Element containerFinder2 =
        find.byKey(const Key('Mary'), skipOffstage: false).evaluate().first;
    expect(containerFinder2.renderObject!.debugNeedsPaint, true);

    await tester.tap(gesture2);
    await tester.pump();
    expect(containerFinder.renderObject!.debugNeedsPaint, false);
    expect(containerFinder1.renderObject!.debugNeedsPaint, false);
    expect(containerFinder2.renderObject!.debugNeedsPaint, true);

    await tester.tap(gesture2);
    await tester.pump();
    expect(containerFinder.renderObject!.debugNeedsPaint, false);
    expect(containerFinder1.renderObject!.debugNeedsPaint, false);
    expect(containerFinder2.renderObject!.debugNeedsPaint, false);
  });
  testWidgets('has correct backward rendering mechanism', (WidgetTester tester) async {
    await tester.pumpWidget(const example.IndexedStackApp());

    final Finder gesture1 = find.byKey(const Key('gesture1'));
    final Element containerFinder =
        find.byKey(const Key('Dash'), skipOffstage: false).evaluate().first;
    final Element containerFinder1 =
        find.byKey(const Key('John'), skipOffstage: false).evaluate().first;
    final Element containerFinder2 =
        find.byKey(const Key('Mary'), skipOffstage: false).evaluate().first;

    await tester.tap(gesture1);
    await tester.pump();
    expect(containerFinder.renderObject!.debugNeedsPaint, false);
    expect(containerFinder1.renderObject!.debugNeedsPaint, true);
    expect(containerFinder2.renderObject!.debugNeedsPaint, false);

    await tester.tap(gesture1);
    await tester.pump();
    expect(containerFinder.renderObject!.debugNeedsPaint, false);
    expect(containerFinder1.renderObject!.debugNeedsPaint, false);
    expect(containerFinder2.renderObject!.debugNeedsPaint, false);
  });
  testWidgets('has correct element addition handling', (WidgetTester tester) async {
    await tester.pumpWidget(const example.IndexedStackApp());

    expect(find.byType(example.PersonTracker), findsOneWidget);
    expect(find.byType(example.PersonTracker, skipOffstage: false), findsNWidgets(3));
    final Finder textField = find.byType(TextField);
    await tester.enterText(textField, 'hello');
    await tester.testTextInput.receiveAction(TextInputAction.done);
    await tester.pump();
    expect(find.byType(example.PersonTracker, skipOffstage: false), findsNWidgets(4));

    await tester.enterText(textField, 'hello1');
    await tester.testTextInput.receiveAction(TextInputAction.done);
    await tester.pump();
    expect(find.byType(example.PersonTracker, skipOffstage: false), findsNWidgets(5));
  });
  testWidgets('has state preservation', (WidgetTester tester) async {
    await tester.pumpWidget(const example.IndexedStackApp());

    final Finder gesture1 = find.byKey(const Key('gesture1'));
    final Finder gesture2 = find.byKey(const Key('gesture2'));
    final Finder containerFinder = find.byKey(const Key('Dash'));
    final Finder incrementFinder = find.byKey(const Key('incrementDash'));
    Finder counterFinder(int score) {
      return find.descendant(of: containerFinder, matching: find.text('Score: $score'));
    }

    expect(counterFinder(0), findsOneWidget);
    await tester.tap(incrementFinder);
    await tester.pump();

    expect(counterFinder(1), findsOneWidget);

    await tester.tap(gesture2);
    await tester.pump();
    await tester.tap(gesture1);
    await tester.pump();

    expect(counterFinder(1), findsOneWidget);
    expect(counterFinder(0), findsNothing);
  });
}
