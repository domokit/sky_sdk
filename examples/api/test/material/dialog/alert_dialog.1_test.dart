// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/dialog/alert_dialog.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Show Alert dialog', (WidgetTester tester) async {
    const String dialogTitle = 'AlertDialog Title';
    await tester.pumpWidget(const example.AlertDialogExampleApp());

    expect(find.text(dialogTitle), findsNothing);

    await tester.tap(find.widgetWithText(TextButton, 'Show Dialog'));
    await tester.pumpAndSettle();
    expect(find.text(dialogTitle), findsOneWidget);

    await tester.tap(find.text('OK'));
    await tester.pumpAndSettle();
    expect(find.text(dialogTitle), findsNothing);
  });
}
