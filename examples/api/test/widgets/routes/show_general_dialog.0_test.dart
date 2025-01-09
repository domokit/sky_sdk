// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/routes/show_general_dialog.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Open and dismiss general dialog', (WidgetTester tester) async {
    const String dialogText = 'Alert!';

    await tester.pumpWidget(const example.GeneralDialogApp());

    expect(find.text(dialogText), findsNothing);

    // Tap on the button to show the dialog.
    await tester.tap(find.byType(OutlinedButton));
    await tester.pumpAndSettle();
    expect(find.text(dialogText), findsOneWidget);

    // Try to dismiss the dialog with a tap on the scrim.
    await tester.tapAt(const Offset(10.0, 10.0));
    await tester.pumpAndSettle();
    expect(find.text(dialogText), findsNothing);
  });
}
