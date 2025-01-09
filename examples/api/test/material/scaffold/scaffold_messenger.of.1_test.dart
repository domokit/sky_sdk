// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/scaffold/scaffold_messenger.of.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('A snack bar is displayed after 10 taps', (WidgetTester tester) async {
    await tester.pumpWidget(const example.OfExampleApp());

    expect(find.widgetWithText(AppBar, 'ScaffoldMessenger Demo'), findsOne);
    expect(find.text('You have pushed the button this many times:'), findsOne);

    for (int i = 0; i < 9; i++) {
      await tester.tap(find.byType(FloatingActionButton));
      await tester.pump();
      expect(find.text('${i + 1}'), findsOne);
    }
    expect(find.byType(SnackBar), findsNothing);

    await tester.tap(find.byType(FloatingActionButton));
    await tester.pumpAndSettle();
    expect(find.text('10'), findsOne);

    expect(find.widgetWithText(SnackBar, 'A multiple of ten!'), findsOne);
  });
}
