// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter_api_samples/cupertino/sheet/cupertino_sheet.2.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Tap on button displays cupertino sheet', (WidgetTester tester) async {
    await tester.pumpWidget(const example.RestorableSheetExampleApp());

    final Finder dialogTitle = find.text('Current Count: 0');
    expect(dialogTitle, findsNothing);

    await tester.tap(find.byType(CupertinoButton));
    await tester.pumpAndSettle();
    expect(dialogTitle, findsOneWidget);

    await tester.tap(find.text('Pop Sheet'));
    await tester.pumpAndSettle();
    expect(dialogTitle, findsNothing);
  });

  testWidgets('State restoration keeps the counter at the right value', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.RestorableSheetExampleApp());

    await tester.tap(find.byType(CupertinoButton));
    await tester.pumpAndSettle();

    expect(find.text('Current Count: 0'), findsOneWidget);

    await tester.tap(find.text('Increase'));
    await tester.pumpAndSettle();

    await tester.tap(find.text('Increase'));
    await tester.pumpAndSettle();

    expect(find.text('Current Count: 2'), findsOneWidget);

    await tester.restartAndRestore();

    expect(find.text('Current Count: 2'), findsOneWidget);
  });
}
