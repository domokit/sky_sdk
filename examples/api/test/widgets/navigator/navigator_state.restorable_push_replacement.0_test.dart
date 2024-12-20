// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/navigator/navigator_state.restorable_push_replacement.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('It pushes a restorable route and restores it', (WidgetTester tester) async {
    await tester.pumpWidget(const example.RestorablePushReplacementExampleApp());

    expect(find.widgetWithText(AppBar, 'Sample Code'), findsOne);
    expect(find.text('This is the initial route.'), findsOne);

    final TestRestorationData initialData = await tester.getRestorationData();

    await tester.tap(find.widgetWithIcon(FloatingActionButton, Icons.add));
    await tester.pumpAndSettle();

    expect(find.text('This is a new route.'), findsOne);

    await tester.restartAndRestore();

    expect(find.text('This is a new route.'), findsOne);

    final TestRestorationData pushedData = await tester.getRestorationData();

    await tester.restoreFrom(initialData);

    await tester.pumpAndSettle();
    expect(find.text('This is the initial route.'), findsOne);

    await tester.restoreFrom(pushedData);
    expect(find.text('This is a new route.'), findsOne);
  });
}
