// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/scaffold/scaffold_messenger_state.show_snack_bar.1.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Floating SnackBar is visible', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SnackBarApp());

    final Finder buttonFinder = find.byType(ElevatedButton);
    await tester.tap(buttonFinder.first);
    // Have the SnackBar fully animate out.
    await tester.pumpAndSettle();

    final Finder snackBarFinder = find.byType(SnackBar);
    expect(snackBarFinder, findsOneWidget);

    // Grow logo to send SnackBar off screen.
    await tester.tap(buttonFinder.last);
    await tester.pumpAndSettle();

    final AssertionError exception = tester.takeException() as AssertionError;
    const String message =
        'Floating SnackBar presented off screen.\n'
        'A SnackBar with behavior property set to SnackBarBehavior.floating is fully '
        'or partially off screen because some or all the widgets provided to '
        'Scaffold.floatingActionButton, Scaffold.persistentFooterButtons and '
        'Scaffold.bottomNavigationBar take up too much vertical space.\n'
        'Consider constraining the size of these widgets to allow room for the SnackBar to be visible.';
    expect(exception.message, message);
  });
}
