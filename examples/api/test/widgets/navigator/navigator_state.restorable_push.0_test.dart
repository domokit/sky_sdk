// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/navigator/navigator_state.restorable_push.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('It pushes a restorable route and pops it', (WidgetTester tester) async {
    await tester.pumpWidget(const example.RestorablePushExampleApp());

    expect(find.widgetWithText(AppBar, 'Sample Code'), findsOne);
    expect(find.byType(BackButton), findsNothing);

    await tester.tap(find.widgetWithIcon(FloatingActionButton, Icons.add));
    await tester.pumpAndSettle();

    await tester.tap(find.byType(BackButton));
    await tester.pumpAndSettle();

    expect(find.byType(BackButton), findsNothing);
  });

  testWidgets('It pushes a restorable route and restores it', (WidgetTester tester) async {
    await tester.pumpWidget(const example.RestorablePushExampleApp());

    expect(find.widgetWithText(AppBar, 'Sample Code'), findsOne);
    expect(find.byType(BackButton), findsNothing);

    await tester.tap(find.widgetWithIcon(FloatingActionButton, Icons.add));
    await tester.pumpAndSettle();

    await tester.restartAndRestore();

    expect(find.byType(BackButton), findsOne);

    final TestRestorationData data = await tester.getRestorationData();

    await tester.tap(find.byType(BackButton));
    await tester.pumpAndSettle();

    expect(find.byType(BackButton), findsNothing);

    await tester.restoreFrom(data);
    expect(find.byType(BackButton), findsOne);
  });
}
