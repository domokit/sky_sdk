// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:flutter/services.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:leak_tracker_flutter_testing/leak_tracker_flutter_testing.dart';

void main() {
  AppLifecycleListener? listener;

  Future<void> setAppLifeCycleState(AppLifecycleState state) async {
    final ByteData? message = const StringCodec().encodeMessage(state.toString());
    await TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.handlePlatformMessage(
      'flutter/lifecycle',
      message,
      (_) {},
    );
  }

  Future<void> sendAppExitRequest() async {
    final ByteData message = const JSONMethodCodec().encodeMethodCall(
      const MethodCall('System.requestAppExit'),
    );
    await TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.handlePlatformMessage(
      'flutter/platform',
      message,
      (_) {},
    );
  }

  setUp(() async {
    WidgetsFlutterBinding.ensureInitialized();
    WidgetsBinding.instance
      ..resetEpoch()
      ..platformDispatcher.onBeginFrame = null
      ..platformDispatcher.onDrawFrame = null;
    final TestWidgetsFlutterBinding binding = TestWidgetsFlutterBinding.instance;
    binding.readTestInitialLifecycleStateFromNativeWindow();
    // Reset the state to detached. Going to paused first makes it a valid
    // transition from any state, since the intermediate transitions will be
    // generated.
    await setAppLifeCycleState(AppLifecycleState.paused);
    await setAppLifeCycleState(AppLifecycleState.detached);
  });

  tearDown(() {
    listener?.dispose();
    listener = null;
    final TestWidgetsFlutterBinding binding = TestWidgetsFlutterBinding.instance;
    binding.resetInternalState();
    binding.platformDispatcher.resetInitialLifecycleState();
    assert(
      TestAppLifecycleListener.registerCount == 0,
      'There were ${TestAppLifecycleListener.registerCount} listeners that were not disposed of in tests.',
    );
  });

  testWidgets('Default Diagnostics', (WidgetTester tester) async {
    listener = TestAppLifecycleListener(binding: tester.binding);
    expect(
      listener.toString(),
      equalsIgnoringHashCodes(
        'TestAppLifecycleListener#00000(binding: <AutomatedTestWidgetsFlutterBinding>)',
      ),
    );
  });

  testWidgets('Diagnostics', (WidgetTester tester) async {
    Future<AppExitResponse> handleExitRequested() async {
      return AppExitResponse.cancel;
    }

    listener = TestAppLifecycleListener(
      binding: WidgetsBinding.instance,
      onExitRequested: handleExitRequested,
      onStateChange: (AppLifecycleState _) {},
    );
    expect(
      listener.toString(),
      equalsIgnoringHashCodes(
        'TestAppLifecycleListener#00000(binding: <AutomatedTestWidgetsFlutterBinding>, onStateChange, onExitRequested)',
      ),
    );
  });

  testWidgets('listens to AppLifecycleState', (WidgetTester tester) async {
    final List<AppLifecycleState> states = <AppLifecycleState>[tester.binding.lifecycleState!];
    void stateChange(AppLifecycleState state) {
      states.add(state);
    }

    listener = TestAppLifecycleListener(
      binding: WidgetsBinding.instance,
      onStateChange: stateChange,
    );
    expect(states, equals(<AppLifecycleState>[AppLifecycleState.detached]));
    await setAppLifeCycleState(AppLifecycleState.inactive);
    // "resumed" is generated.
    expect(
      states,
      equals(<AppLifecycleState>[
        AppLifecycleState.detached,
        AppLifecycleState.resumed,
        AppLifecycleState.inactive,
      ]),
    );
    await setAppLifeCycleState(AppLifecycleState.resumed);
    expect(
      states,
      equals(<AppLifecycleState>[
        AppLifecycleState.detached,
        AppLifecycleState.resumed,
        AppLifecycleState.inactive,
        AppLifecycleState.resumed,
      ]),
    );
  });

  testWidgets('Triggers correct state transition callbacks', (WidgetTester tester) async {
    final List<String> transitions = <String>[];
    listener = TestAppLifecycleListener(
      binding: WidgetsBinding.instance,
      onDetach: () => transitions.add('detach'),
      onHide: () => transitions.add('hide'),
      onInactive: () => transitions.add('inactive'),
      onPause: () => transitions.add('pause'),
      onRestart: () => transitions.add('restart'),
      onResume: () => transitions.add('resume'),
      onShow: () => transitions.add('show'),
    );

    // Try "standard" sequence
    await setAppLifeCycleState(AppLifecycleState.resumed);
    expect(transitions, equals(<String>['resume']));
    await setAppLifeCycleState(AppLifecycleState.inactive);
    expect(transitions, equals(<String>['resume', 'inactive']));
    await setAppLifeCycleState(AppLifecycleState.hidden);
    expect(transitions, equals(<String>['resume', 'inactive', 'hide']));
    await setAppLifeCycleState(AppLifecycleState.paused);
    expect(transitions, equals(<String>['resume', 'inactive', 'hide', 'pause']));

    // Go back to resume
    transitions.clear();
    await setAppLifeCycleState(AppLifecycleState.hidden);
    expect(transitions, equals(<String>['restart']));
    await setAppLifeCycleState(AppLifecycleState.inactive);
    expect(transitions, equals(<String>['restart', 'show']));
    await setAppLifeCycleState(AppLifecycleState.resumed);
    expect(transitions, equals(<String>['restart', 'show', 'resume']));

    // Generates intermediate states from lower to higher lifecycle states.
    transitions.clear();
    await setAppLifeCycleState(AppLifecycleState.paused);
    expect(transitions, equals(<String>['inactive', 'hide', 'pause']));

    // Wraps around from pause to detach.
    await setAppLifeCycleState(AppLifecycleState.detached);
    expect(transitions, equals(<String>['inactive', 'hide', 'pause', 'detach']));
    await setAppLifeCycleState(AppLifecycleState.resumed);
    expect(transitions, equals(<String>['inactive', 'hide', 'pause', 'detach', 'resume']));
    await setAppLifeCycleState(AppLifecycleState.paused);
    expect(
      transitions,
      equals(<String>[
        'inactive',
        'hide',
        'pause',
        'detach',
        'resume',
        'inactive',
        'hide',
        'pause',
      ]),
    );

    // Generates intermediate states from higher to lower lifecycle states.
    transitions.clear();
    await setAppLifeCycleState(AppLifecycleState.resumed);
    expect(transitions, equals(<String>['restart', 'show', 'resume']));

    // Go to detached
    transitions.clear();
    await setAppLifeCycleState(AppLifecycleState.detached);
    expect(transitions, equals(<String>['inactive', 'hide', 'pause', 'detach']));
  });

  testWidgets('Receives exit requests', (WidgetTester tester) async {
    bool exitRequested = false;
    Future<AppExitResponse> handleExitRequested() async {
      exitRequested = true;
      return AppExitResponse.cancel;
    }

    listener = TestAppLifecycleListener(
      binding: WidgetsBinding.instance,
      onExitRequested: handleExitRequested,
    );
    await sendAppExitRequest();
    expect(exitRequested, isTrue);
  });

  test('AppLifecycleListener dispatches memory events', () async {
    await expectLater(
      await memoryEvents(
        () => AppLifecycleListener(binding: WidgetsBinding.instance).dispose(),
        AppLifecycleListener,
      ),
      areCreateAndDispose,
    );
  });
}

class TestAppLifecycleListener extends AppLifecycleListener {
  TestAppLifecycleListener({
    super.binding,
    super.onResume,
    super.onInactive,
    super.onHide,
    super.onShow,
    super.onPause,
    super.onRestart,
    super.onDetach,
    super.onExitRequested,
    super.onStateChange,
  }) {
    registerCount += 1;
  }

  static int registerCount = 0;

  @override
  void dispose() {
    super.dispose();
    registerCount -= 1;
  }
}
