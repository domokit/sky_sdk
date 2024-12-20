// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' as ui show AppLifecycleState;

import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  TestWidgetsFlutterBinding.ensureInitialized();

  testWidgets('SystemChrome overlay style test', (WidgetTester tester) async {
    final List<MethodCall> log = <MethodCall>[];

    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
      SystemChannels.platform,
      (MethodCall methodCall) async {
        log.add(methodCall);
        return null;
      },
    );

    // The first call is a cache miss and will queue a microtask
    SystemChrome.setSystemUIOverlayStyle(SystemUiOverlayStyle.light);
    expect(tester.binding.microtaskCount, equals(1));

    // Flush all microtasks
    await tester.idle();
    expect(log, hasLength(1));
    expect(
      log.single,
      isMethodCall(
        'SystemChrome.setSystemUIOverlayStyle',
        arguments: <String, dynamic>{
          'systemNavigationBarColor': 4278190080,
          'systemNavigationBarDividerColor': null,
          'systemStatusBarContrastEnforced': null,
          'statusBarColor': null,
          'statusBarBrightness': 'Brightness.dark',
          'statusBarIconBrightness': 'Brightness.light',
          'systemNavigationBarIconBrightness': 'Brightness.light',
          'systemNavigationBarContrastEnforced': null,
        },
      ),
    );
    log.clear();
    expect(tester.binding.microtaskCount, equals(0));
    expect(log.isEmpty, isTrue);

    // The second call with the same value should be a no-op
    SystemChrome.setSystemUIOverlayStyle(SystemUiOverlayStyle.light);
    expect(tester.binding.microtaskCount, equals(0));

    SystemChrome.setSystemUIOverlayStyle(
      const SystemUiOverlayStyle(
        systemStatusBarContrastEnforced: false,
        systemNavigationBarContrastEnforced: true,
      ),
    );
    expect(tester.binding.microtaskCount, equals(1));
    await tester.idle();
    expect(log, hasLength(1));
    expect(
      log.single,
      isMethodCall(
        'SystemChrome.setSystemUIOverlayStyle',
        arguments: <String, dynamic>{
          'systemNavigationBarColor': null,
          'systemNavigationBarDividerColor': null,
          'systemStatusBarContrastEnforced': false,
          'statusBarColor': null,
          'statusBarBrightness': null,
          'statusBarIconBrightness': null,
          'systemNavigationBarIconBrightness': null,
          'systemNavigationBarContrastEnforced': true,
        },
      ),
    );
  });

  test('setPreferredOrientations control test', () async {
    final List<MethodCall> log = <MethodCall>[];

    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
      SystemChannels.platform,
      (MethodCall methodCall) async {
        log.add(methodCall);
        return null;
      },
    );

    await SystemChrome.setPreferredOrientations(<DeviceOrientation>[DeviceOrientation.portraitUp]);

    expect(log, hasLength(1));
    expect(
      log.single,
      isMethodCall(
        'SystemChrome.setPreferredOrientations',
        arguments: <String>['DeviceOrientation.portraitUp'],
      ),
    );
  });

  test('setApplicationSwitcherDescription control test', () async {
    final List<MethodCall> log = <MethodCall>[];

    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
      SystemChannels.platform,
      (MethodCall methodCall) async {
        log.add(methodCall);
        return null;
      },
    );

    await SystemChrome.setApplicationSwitcherDescription(
      const ApplicationSwitcherDescription(label: 'Example label', primaryColor: 0xFF00FF00),
    );

    expect(log, hasLength(1));
    expect(
      log.single,
      isMethodCall(
        'SystemChrome.setApplicationSwitcherDescription',
        arguments: <String, dynamic>{'label': 'Example label', 'primaryColor': 4278255360},
      ),
    );
  });

  test('setApplicationSwitcherDescription missing plugin', () async {
    final List<ByteData?> log = <ByteData>[];

    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMessageHandler(
      'flutter/platform',
      (ByteData? message) async {
        log.add(message);
        return null;
      },
    );

    await SystemChrome.setApplicationSwitcherDescription(
      const ApplicationSwitcherDescription(label: 'Example label', primaryColor: 0xFF00FF00),
    );

    expect(log, isNotEmpty);
  });

  test('setEnabledSystemUIMode control test', () async {
    final List<MethodCall> log = <MethodCall>[];

    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
      SystemChannels.platform,
      (MethodCall methodCall) async {
        log.add(methodCall);
        return null;
      },
    );

    await SystemChrome.setEnabledSystemUIMode(SystemUiMode.leanBack);

    expect(log, hasLength(1));
    expect(
      log.single,
      isMethodCall('SystemChrome.setEnabledSystemUIMode', arguments: 'SystemUiMode.leanBack'),
    );
  });

  test('setEnabledSystemUIMode asserts for overlays in manual configuration', () async {
    expect(
      () async {
        await SystemChrome.setEnabledSystemUIMode(SystemUiMode.manual);
      },
      throwsA(
        isA<AssertionError>().having(
          (AssertionError error) => error.toString(),
          'description',
          contains('mode == SystemUiMode.manual && overlays != null'),
        ),
      ),
    );
  });

  test('setEnabledSystemUIMode passes correct overlays for manual configuration', () async {
    final List<MethodCall> log = <MethodCall>[];

    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
      SystemChannels.platform,
      (MethodCall methodCall) async {
        log.add(methodCall);
        return null;
      },
    );

    await SystemChrome.setEnabledSystemUIMode(
      SystemUiMode.manual,
      overlays: <SystemUiOverlay>[SystemUiOverlay.top],
    );

    expect(log, hasLength(1));
    expect(
      log.single,
      isMethodCall(
        'SystemChrome.setEnabledSystemUIOverlays',
        arguments: <String>['SystemUiOverlay.top'],
      ),
    );
  });

  test('setSystemUIChangeCallback control test', () async {
    final List<MethodCall> log = <MethodCall>[];

    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
      SystemChannels.platform,
      (MethodCall methodCall) async {
        log.add(methodCall);
        return null;
      },
    );

    await SystemChrome.setSystemUIChangeCallback(null);
    expect(log, hasLength(0));

    await SystemChrome.setSystemUIChangeCallback((bool overlaysAreVisible) async {});
    expect(log, hasLength(1));
    expect(log.single, isMethodCall('SystemChrome.setSystemUIChangeListener', arguments: null));
  });

  test('toString works as intended', () async {
    const SystemUiOverlayStyle systemUiOverlayStyle = SystemUiOverlayStyle();

    expect(
      systemUiOverlayStyle.toString(),
      'SystemUiOverlayStyle({'
      'systemNavigationBarColor: null, '
      'systemNavigationBarDividerColor: null, '
      'systemStatusBarContrastEnforced: null, '
      'statusBarColor: null, '
      'statusBarBrightness: null, '
      'statusBarIconBrightness: null, '
      'systemNavigationBarIconBrightness: null, '
      'systemNavigationBarContrastEnforced: null})',
    );
  });

  testWidgets('SystemChrome handles detached lifecycle state', (WidgetTester tester) async {
    final List<MethodCall> log = <MethodCall>[];

    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
      SystemChannels.platform,
      (MethodCall methodCall) async {
        log.add(methodCall);
        return null;
      },
    );

    const SystemUiOverlayStyle systemUiOverlayStyle = SystemUiOverlayStyle();
    SystemChrome.setSystemUIOverlayStyle(systemUiOverlayStyle);
    await tester.idle();
    expect(log.length, equals(1));

    // Setting the same state should not send another message to the host.
    SystemChrome.setSystemUIOverlayStyle(systemUiOverlayStyle);
    await tester.idle();
    expect(log.length, equals(1));

    // The state should be sent again if the app was detached.
    SystemChrome.handleAppLifecycleStateChanged(ui.AppLifecycleState.detached);
    await tester.idle();
    SystemChrome.handleAppLifecycleStateChanged(ui.AppLifecycleState.resumed);
    SystemChrome.setSystemUIOverlayStyle(systemUiOverlayStyle);
    await tester.idle();
    expect(log.length, equals(2));
  });
}
