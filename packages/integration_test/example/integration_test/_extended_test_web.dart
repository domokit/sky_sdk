// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This is a basic Flutter widget test.
//
// To perform an interaction with a widget in your test, use the WidgetTester
// utility that Flutter provides. For example, you can send tap and scroll
// gestures. You can also use WidgetTester to find child widgets in the widget
// tree, read text, and verify that the values of widget properties are correct.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:integration_test/integration_test.dart';

import 'package:integration_test_example/main.dart' as app;
import 'package:web/web.dart' as web;

void main() {
  final IntegrationTestWidgetsFlutterBinding binding =
      IntegrationTestWidgetsFlutterBinding.ensureInitialized();

  testWidgets('verify text', (WidgetTester tester) async {
    // Build our app and trigger a frame.
    app.main();

    // Trigger a frame.
    await tester.pumpAndSettle();

    // Take a screenshot.
    await binding.takeScreenshot(
      'platform_name',
      // The optional parameter 'args' can be used to pass values to the
      // [integrationDriver.onScreenshot] handler
      // (see test_driver/extended_integration_test.dart). For example, you
      // could look up environment variables in this test that were passed to
      // the run command via `--dart-define=`, and then pass the values to the
      // [integrationDriver.onScreenshot] handler through this 'args' map.
      <String, Object?>{'someArgumentKey': 'someArgumentValue'},
    );

    // Verify that platform is retrieved.
    expect(
      find.byWidgetPredicate(
        (Widget widget) =>
            widget is Text &&
            widget.data!.startsWith('Platform: ${web.window.navigator.platform}\n'),
      ),
      findsOneWidget,
    );
  });

  testWidgets('verify screenshot', (WidgetTester tester) async {
    // Build our app and trigger a frame.
    app.main();

    // Trigger a frame.
    await tester.pumpAndSettle();

    // Multiple methods can take screenshots. Screenshots are taken with the
    // same order the methods run.  We pass an argument that can be looked up
    // from the [onScreenshot] handler in
    // [test_driver/extended_integration_test.dart].
    await binding.takeScreenshot(
      'platform_name_2',
      // The optional parameter 'args' can be used to pass values to the
      // [integrationDriver.onScreenshot] handler
      // (see test_driver/extended_integration_test.dart). For example, you
      // could look up environment variables in this test that were passed to
      // the run command via `--dart-define=`, and then pass the values to the
      // [integrationDriver.onScreenshot] handler through this 'args' map.
      <String, Object?>{'someArgumentKey': 'someArgumentValue'},
    );
  });
}
