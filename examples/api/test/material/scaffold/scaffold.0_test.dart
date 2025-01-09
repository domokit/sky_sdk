// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/scaffold/scaffold.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('The count should be incremented when the floating action button is tapped', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.ScaffoldExampleApp());

    expect(find.widgetWithText(AppBar, 'Sample Code'), findsOne);
    expect(find.widgetWithIcon(FloatingActionButton, Icons.add), findsOne);
    expect(find.text('You have pressed the button 0 times.'), findsOne);

    for (int i = 1; i <= 5; i++) {
      await tester.tap(find.byType(FloatingActionButton));
      await tester.pump();
      expect(find.text('You have pressed the button $i times.'), findsOne);
    }
  });
}
