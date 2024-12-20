// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/snack_bar/snack_bar.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Clicking on Button shows a SnackBar', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SnackBarExampleApp());

    expect(find.widgetWithText(AppBar, 'SnackBar Sample'), findsOneWidget);
    expect(find.widgetWithText(ElevatedButton, 'Show Snackbar'), findsOneWidget);
    await tester.tap(find.widgetWithText(ElevatedButton, 'Show Snackbar'));
    await tester.pump();
    expect(find.text('Awesome Snackbar!'), findsOneWidget);
    expect(find.text('Action'), findsOneWidget);
  });
}
