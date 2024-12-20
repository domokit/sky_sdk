// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/restoration/restoration_mixin.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('The state of the counter can be restored', (WidgetTester tester) async {
    await tester.pumpWidget(const example.RestorationExampleApp());

    expect(find.widgetWithText(AppBar, 'Restorable Counter'), findsOne);
    expect(find.text('You have pushed the button this many times:'), findsOne);
    expect(find.text('0'), findsOne);

    await tester.tap(find.widgetWithIcon(FloatingActionButton, Icons.add));
    await tester.pump();
    expect(find.text('1'), findsOne);

    await tester.restartAndRestore();

    expect(find.text('1'), findsOne);

    final TestRestorationData data = await tester.getRestorationData();

    await tester.tap(find.widgetWithIcon(FloatingActionButton, Icons.add));
    await tester.pump();
    expect(find.text('2'), findsOne);

    await tester.restoreFrom(data);

    expect(find.text('1'), findsOne);
  });
}
