// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter_api_samples/cupertino/sheet/cupertino_sheet.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Tap on button displays cupertino sheet', (WidgetTester tester) async {
    await tester.pumpWidget(const example.CupertinoSheetApp());

    final Finder dialogTitle = find.text('CupertinoSheetRoute');
    final Finder nextPageTitle = find.text('Next Page');
    expect(dialogTitle, findsNothing);
    expect(nextPageTitle, findsNothing);

    await tester.tap(find.byType(CupertinoButton));
    await tester.pumpAndSettle();
    expect(dialogTitle, findsOneWidget);
    expect(nextPageTitle, findsNothing);

    await tester.tap(find.text('Push Nested Page'));
    await tester.pumpAndSettle();
    expect(dialogTitle, findsNothing);
    expect(nextPageTitle, findsOneWidget);

    await tester.tap(find.text('Push Another Sheet'));
    await tester.pumpAndSettle();
    // Both titles are on the screen, though one is covered by the second sheet.
    expect(dialogTitle, findsOneWidget);
    expect(nextPageTitle, findsOneWidget);

    await tester.tap(find.text('Pop Whole Sheet').last);
    await tester.pumpAndSettle();
    expect(dialogTitle, findsNothing);
    expect(nextPageTitle, findsOneWidget);

    await tester.tap(find.text('Pop Whole Sheet'));
    await tester.pumpAndSettle();
    expect(dialogTitle, findsNothing);
    expect(nextPageTitle, findsNothing);
  });
}
