// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/navigator/restorable_route_future.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('It pushes a restorable route and pops it', (WidgetTester tester) async {
    await tester.pumpWidget(const example.RestorableRouteFutureExampleApp());

    expect(find.widgetWithText(AppBar, 'RestorableRouteFuture Example'), findsOne);
    expect(find.byType(BackButton), findsNothing);

    expect(find.text('Last count: 0'), findsOne);

    await tester.tap(find.widgetWithText(ElevatedButton, 'Open Counter'));
    await tester.pumpAndSettle();

    expect(find.widgetWithText(AppBar, 'Awesome Counter'), findsOne);
    expect(find.text('Count: 0'), findsOne);

    await tester.tap(find.widgetWithIcon(FloatingActionButton, Icons.add));
    await tester.pump();

    expect(find.text('Count: 1'), findsOne);

    await tester.tap(find.byType(BackButton));
    await tester.pumpAndSettle();

    expect(find.widgetWithText(AppBar, 'RestorableRouteFuture Example'), findsOne);
    expect(find.text('Last count: 1'), findsOne);
  });

  testWidgets('It pushes a restorable route and restores it', (WidgetTester tester) async {
    await tester.pumpWidget(const example.RestorableRouteFutureExampleApp());

    expect(find.widgetWithText(AppBar, 'RestorableRouteFuture Example'), findsOne);
    expect(find.byType(BackButton), findsNothing);

    expect(find.text('Last count: 0'), findsOne);

    await tester.tap(find.widgetWithText(ElevatedButton, 'Open Counter'));
    await tester.pumpAndSettle();

    expect(find.widgetWithText(AppBar, 'Awesome Counter'), findsOne);
    expect(find.text('Count: 0'), findsOne);

    await tester.tap(find.widgetWithIcon(FloatingActionButton, Icons.add));
    await tester.pump();

    expect(find.text('Count: 1'), findsOne);

    await tester.restartAndRestore();
    expect(find.byType(BackButton), findsOne);

    expect(find.text('Count: 1'), findsOne);

    final TestRestorationData data = await tester.getRestorationData();

    await tester.tap(find.byType(BackButton));
    await tester.pumpAndSettle();

    expect(find.byType(BackButton), findsNothing);

    expect(find.widgetWithText(AppBar, 'RestorableRouteFuture Example'), findsOne);
    expect(find.text('Last count: 1'), findsOne);

    await tester.restoreFrom(data);

    expect(find.widgetWithText(AppBar, 'Awesome Counter'), findsOne);
    expect(find.byType(BackButton), findsOne);
    expect(find.text('Count: 1'), findsOne);
  });
}
