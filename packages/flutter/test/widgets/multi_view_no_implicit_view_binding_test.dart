// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

import 'multi_view_testing.dart';

void main() {
  // Overrides the default test bindings so we have the ability to hide the
  // implicitView inside the body of a test.
  final NoImplicitViewWidgetsBinding binding = NoImplicitViewWidgetsBinding();

  testWidgets('NoImplicitViewWidgetsBinding self-test', (WidgetTester tester) async {
    expect(tester.platformDispatcher.implicitView, isNotNull);

    // Hide the implicitView from the test harness.
    binding.hideImplicitView();

    expect(tester.platformDispatcher.implicitView, isNull);

    // Ensure the test harness finds the implicitView.
    binding.showImplicitView();
  });

  testWidgets('null implicitView - runApp throws assertion, suggests to use `runWidget`.', (
    WidgetTester tester,
  ) async {
    // Hide the implicitView from the test harness.
    binding.hideImplicitView();

    expect(
      () {
        runApp(Container());
      },
      throwsA(
        isA<StateError>().having(
          (StateError error) => error.message,
          'description',
          contains('Try using `runWidget` instead of `runApp`'),
        ),
      ),
    );

    // Ensure the test harness finds the implicitView.
    binding.showImplicitView();
  });
}
