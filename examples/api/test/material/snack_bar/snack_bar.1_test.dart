// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/snack_bar/snack_bar.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Tapping on button shows snackbar', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SnackBarExampleApp());

    expect(find.byType(SnackBar), findsNothing);
    expect(find.widgetWithText(AppBar, 'SnackBar Sample'), findsOneWidget);

    await tester.tap(find.widgetWithText(ElevatedButton, 'Show Snackbar'));
    await tester.pump();

    expect(find.text('Awesome SnackBar!'), findsOneWidget);
    expect(find.widgetWithText(SnackBarAction, 'Action'), findsOneWidget);

    final SnackBar bar = tester.widget<SnackBar>(
      find.ancestor(of: find.text('Awesome SnackBar!'), matching: find.byType(SnackBar)),
    );
    expect(bar.behavior, SnackBarBehavior.floating);
  });

  testWidgets('Snackbar is styled correctly', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SnackBarExampleApp());
    await tester.tap(find.byType(ElevatedButton));
    await tester.pump();

    expect(find.byType(SnackBar), findsOneWidget);

    final SnackBar bar = tester.widget<SnackBar>(find.byType(SnackBar));
    expect(bar.behavior, SnackBarBehavior.floating);
    expect(bar.width, 280.0);
    expect(
      bar.shape,
      isA<RoundedRectangleBorder>().having(
        (RoundedRectangleBorder b) => b.borderRadius,
        'radius',
        BorderRadius.circular(10.0),
      ),
    );
  });

  testWidgets('Snackbar should disappear after timeout', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SnackBarExampleApp());
    expect(find.byType(SnackBar), findsNothing);

    await tester.tap(find.byType(ElevatedButton));
    await tester.pumpAndSettle();
    expect(find.byType(SnackBar), findsOneWidget);

    await tester.pump(const Duration(milliseconds: 1500));
    await tester.pumpAndSettle();
    expect(find.byType(SnackBar), findsNothing);
  });
}
