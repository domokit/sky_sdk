// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:integration_test/integration_test.dart';
import 'package:web_e2e_tests/scroll_wheel_main.dart' as app;

void main() {
  IntegrationTestWidgetsFlutterBinding.ensureInitialized();

  testWidgets('Test mousewheel scroll by line', (WidgetTester tester) async {
    app.main();
    await tester.pumpAndSettle();

    final Finder finder = find.byKey(const Key('scroll-button'));
    expect(finder, findsOneWidget);
    await tester.tap(finder);
    await tester.pumpAndSettle();
    await tester.tap(finder);
    await tester.pumpAndSettle();

    await expectLater(find.byType(app.MyApp), matchesGoldenFile('scroll_wheel_by_line'));
  });
}
