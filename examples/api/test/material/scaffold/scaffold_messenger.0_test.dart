// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/scaffold/scaffold_messenger.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('The snack bar should be visible after tapping the button', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.ScaffoldMessengerExampleApp());

    expect(find.widgetWithText(AppBar, 'ScaffoldMessenger Sample'), findsOne);

    await tester.tap(find.widgetWithText(OutlinedButton, 'Show SnackBar'));
    await tester.pumpAndSettle();

    expect(find.widgetWithText(SnackBar, 'A SnackBar has been shown.'), findsOne);
  });
}
