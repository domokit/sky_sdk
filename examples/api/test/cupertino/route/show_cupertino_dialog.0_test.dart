// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter_api_samples/cupertino/route/show_cupertino_dialog.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Tap on button displays cupertino dialog', (WidgetTester tester) async {
    await tester.pumpWidget(const example.CupertinoDialogApp());

    final Finder dialogTitle = find.text('Title');
    expect(dialogTitle, findsNothing);

    await tester.tap(find.byType(CupertinoButton));
    await tester.pumpAndSettle();
    expect(dialogTitle, findsOneWidget);

    await tester.tap(find.text('Yes'));
    await tester.pumpAndSettle();
    expect(dialogTitle, findsNothing);
  });
}
