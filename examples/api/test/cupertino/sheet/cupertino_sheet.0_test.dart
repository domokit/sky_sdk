// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter_api_samples/cupertino/sheet/cupertino_sheet.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Tap on button displays cupertino sheet', (WidgetTester tester) async {
    await tester.pumpWidget(const example.CupertinoSheetApp());

    final Finder dialogTitle = find.text('CupertinoSheetRoute');
    expect(dialogTitle, findsNothing);

    await tester.tap(find.byType(CupertinoButton));
    await tester.pumpAndSettle();
    expect(dialogTitle, findsOneWidget);

    await tester.tap(find.text('Go Back'));
    await tester.pumpAndSettle();
    expect(dialogTitle, findsNothing);
  });
}
