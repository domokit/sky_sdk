// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('initialLifecycleState is used to init state paused', (WidgetTester tester) async {
    final TestWidgetsFlutterBinding binding = tester.binding;
    binding.resetInternalState();
    // Use paused as the initial state.
    binding.platformDispatcher.initialLifecycleStateTestValue = 'AppLifecycleState.paused';
    binding.readTestInitialLifecycleStateFromNativeWindow(); // Re-attempt the initialization.

    // The lifecycleState should now be the state we passed above,
    // even though no lifecycle event was fired from the platform.
    expect(binding.lifecycleState.toString(), equals('AppLifecycleState.paused'));
  });
  testWidgets('Handles all of the allowed states of AppLifecycleState', (
    WidgetTester tester,
  ) async {
    final TestWidgetsFlutterBinding binding = tester.binding;
    for (final AppLifecycleState state in AppLifecycleState.values) {
      binding.resetInternalState();
      binding.platformDispatcher.initialLifecycleStateTestValue = state.toString();
      binding.readTestInitialLifecycleStateFromNativeWindow();
      expect(ServicesBinding.instance.lifecycleState.toString(), equals(state.toString()));
    }
  });
  test('AppLifecycleState values are in the right order for the state machine to be correct', () {
    expect(
      AppLifecycleState.values,
      equals(<AppLifecycleState>[
        AppLifecycleState.detached,
        AppLifecycleState.resumed,
        AppLifecycleState.inactive,
        AppLifecycleState.hidden,
        AppLifecycleState.paused,
      ]),
    );
  });
}
