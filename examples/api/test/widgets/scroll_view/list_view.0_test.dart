// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/scroll_view/list_view.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('long press ListTile should enable edit mode', (WidgetTester tester) async {
    await tester.pumpWidget(const example.ListViewExampleApp());

    final Finder listView = find.byType(ListView);
    final Finder selectAllFinder = find.text('select all');
    final Finder checkBoxFinder = find.byType(Checkbox);
    expect(listView, findsWidgets);
    expect(selectAllFinder, findsNothing);
    expect(checkBoxFinder, findsNothing);
    await tester.longPress(listView.first);
    await tester.pump();
    expect(selectAllFinder, findsOneWidget);
    expect(checkBoxFinder, findsWidgets);
  });

  testWidgets('Pressing cross button should disable edit mode', (WidgetTester tester) async {
    await tester.pumpWidget(const example.ListViewExampleApp());

    final Finder listView = find.byType(ListView);
    final Finder crossIconFinder = find.byIcon(Icons.close);
    expect(listView, findsWidgets);
    expect(crossIconFinder, findsNothing);

    /// enable edit mode
    await tester.longPress(listView.first);
    await tester.pump();

    expect(crossIconFinder, findsOneWidget);
    await tester.tap(crossIconFinder);
    await tester.pump();
    final Finder selectAllFinder = find.text('select all');
    expect(selectAllFinder, findsNothing);
    expect(crossIconFinder, findsNothing);
  });

  testWidgets('tapping ListTile or checkBox should toggle ListTile state', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.ListViewExampleApp());

    final Finder listView = find.byType(ListView);
    final Finder selectAllFinder = find.text('select all');
    expect(listView, findsWidgets);

    /// enable edit mode
    await tester.longPress(listView.first);
    await tester.pump();

    final Finder checkBoxFinder = find.byType(Checkbox).first;
    expect(selectAllFinder, findsOneWidget);
    expect(checkBoxFinder, findsOneWidget);
    expect(tester.widget<Checkbox>(checkBoxFinder).value, false);
    await tester.tap(checkBoxFinder);
    await tester.pump();
    expect(tester.widget<Checkbox>(checkBoxFinder).value, true);
  });
}
