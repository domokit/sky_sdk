// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:integration_test/integration_test.dart';

import 'package:integration_test_example/main.dart' as app;

/// This file is placed in `test_driver/` instead of `integration_test/`, so
/// that the CI tooling of flutter/plugins only uses this together with
/// `failure_test.dart` as the driver. It is only used for testing of
/// `package:integration_test` – do not follow the conventions here if you are a
/// user of `package:integration_test`.

// Tests the failure behavior of the IntegrationTestWidgetsFlutterBinding
//
// This test fails intentionally! It should be run using a test runner that
// expects failure.
void main() {
  IntegrationTestWidgetsFlutterBinding.ensureInitialized();

  testWidgets('success', (WidgetTester tester) async {
    expect(1 + 1, 2); // This should pass
  });

  testWidgets('failure 1', (WidgetTester tester) async {
    // Build our app and trigger a frame.
    app.main();

    // Verify that platform version is retrieved.
    await expectLater(
      find.byWidgetPredicate(
        (Widget widget) => widget is Text && widget.data!.startsWith('This should fail'),
      ),
      findsOneWidget,
    );
  });

  testWidgets('failure 2', (WidgetTester tester) async {
    expect(1 + 1, 3); // This should fail
  });
}
