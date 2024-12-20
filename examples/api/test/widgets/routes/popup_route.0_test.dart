// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_api_samples/widgets/routes/popup_route.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Dismiss dialog with tap on the scrim and escape key', (WidgetTester tester) async {
    const String dialogText = 'Tap in the scrim or press escape key to dismiss.';

    await tester.pumpWidget(const example.PopupRouteApp());

    expect(find.text(dialogText), findsNothing);

    // Tap on the button to show the dialog.
    await tester.tap(find.byType(OutlinedButton));
    await tester.pumpAndSettle();
    expect(find.text(dialogText), findsOneWidget);

    // Try to dismiss the dialog with a tap on the scrim.
    await tester.tapAt(const Offset(10.0, 10.0));
    await tester.pumpAndSettle();
    expect(find.text(dialogText), findsNothing);

    // Open the dialog again.
    await tester.tap(find.byType(OutlinedButton));
    await tester.pumpAndSettle();
    expect(find.text(dialogText), findsOneWidget);

    // Try to dismiss the dialog with the escape key.
    await tester.sendKeyEvent(LogicalKeyboardKey.escape);
    await tester.pumpAndSettle();
    expect(find.text(dialogText), findsNothing);
  });
}
