// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@TestOn('browser')
library;

import 'dart:js_util';

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  int callOrder = 1;
  int initCalled = 0;
  int runCalled = 0;

  setUp(() {
    callOrder = 1;
    initCalled = 0;
    runCalled = 0;
  });

  Future<void> mockInit([JsFlutterConfiguration? configuration]) async {
    debugOverrideJsConfiguration(configuration);
    addTearDown(() => debugOverrideJsConfiguration(null));
    initCalled = callOrder++;
    await Future<void>.delayed(const Duration(milliseconds: 1));
  }

  Future<void> mockRunApp() async {
    runCalled = callOrder++;
  }

  test('autoStart() immediately calls init and run', () async {
    final AppBootstrap bootstrap = AppBootstrap(initializeEngine: mockInit, runApp: mockRunApp);

    await bootstrap.autoStart();

    expect(initCalled, 1, reason: 'initEngine should be called first.');
    expect(runCalled, 2, reason: 'runApp should be called after init.');
  });

  test('engineInitializer autoStart() does the same as Dart autoStart()', () async {
    final AppBootstrap bootstrap = AppBootstrap(initializeEngine: mockInit, runApp: mockRunApp);

    final FlutterEngineInitializer engineInitializer = bootstrap.prepareEngineInitializer();

    expect(engineInitializer, isNotNull);

    final Object maybeApp = await promiseToFuture<Object>(
      callMethod<Object>(engineInitializer, 'autoStart', <Object?>[]),
    );

    expect(maybeApp, isA<FlutterApp>());
    expect(initCalled, 1, reason: 'initEngine should be called first.');
    expect(runCalled, 2, reason: 'runApp should be called after init.');
  });

  test('engineInitializer initEngine() calls init and returns an appRunner', () async {
    final AppBootstrap bootstrap = AppBootstrap(initializeEngine: mockInit, runApp: mockRunApp);

    final FlutterEngineInitializer engineInitializer = bootstrap.prepareEngineInitializer();

    final Object maybeAppInitializer = await promiseToFuture<Object>(
      callMethod<Object>(engineInitializer, 'initializeEngine', <Object?>[]),
    );

    expect(maybeAppInitializer, isA<FlutterAppRunner>());
    expect(initCalled, 1, reason: 'initEngine should have been called.');
    expect(runCalled, 0, reason: 'runApp should not have been called.');
  });

  test('appRunner runApp() calls run and returns a FlutterApp', () async {
    final AppBootstrap bootstrap = AppBootstrap(initializeEngine: mockInit, runApp: mockRunApp);

    final FlutterEngineInitializer engineInitializer = bootstrap.prepareEngineInitializer();

    final Object appInitializer = await promiseToFuture<Object>(
      callMethod<Object>(engineInitializer, 'initializeEngine', <Object?>[]),
    );
    expect(appInitializer, isA<FlutterAppRunner>());
    final Object maybeApp = await promiseToFuture<Object>(
      callMethod<Object>(appInitializer, 'runApp', <Object?>[]),
    );
    expect(maybeApp, isA<FlutterApp>());
    expect(initCalled, 1, reason: 'initEngine should have been called.');
    expect(runCalled, 2, reason: 'runApp should have been called.');
  });

  group('FlutterApp', () {
    test('has addView/removeView methods', () async {
      final AppBootstrap bootstrap = AppBootstrap(initializeEngine: mockInit, runApp: mockRunApp);

      final FlutterEngineInitializer engineInitializer = bootstrap.prepareEngineInitializer();

      final Object appInitializer = await promiseToFuture<Object>(
        callMethod<Object>(engineInitializer, 'initializeEngine', <Object?>[]),
      );
      final Object maybeApp = await promiseToFuture<Object>(
        callMethod<Object>(appInitializer, 'runApp', <Object?>[]),
      );

      expect(maybeApp, isA<FlutterApp>());
      expect(getJsProperty<dynamic>(maybeApp, 'addView'), isA<Function>());
      expect(getJsProperty<dynamic>(maybeApp, 'removeView'), isA<Function>());
    });
    test('addView/removeView respectively adds/removes view', () async {
      final AppBootstrap bootstrap = AppBootstrap(initializeEngine: mockInit, runApp: mockRunApp);

      final FlutterEngineInitializer engineInitializer = bootstrap.prepareEngineInitializer();

      final Object appInitializer = await promiseToFuture<Object>(
        callMethod<Object>(engineInitializer, 'initializeEngine', <dynamic>[
          jsify(<String, Object?>{'multiViewEnabled': true}),
        ]),
      );
      final Object maybeApp = await promiseToFuture<Object>(
        callMethod<Object>(appInitializer, 'runApp', <Object?>[]),
      );
      final int viewId =
          callMethod<num>(maybeApp, 'addView', <dynamic>[
            jsify(<String, Object?>{'hostElement': createDomElement('div')}),
          ]).toInt();
      expect(bootstrap.viewManager[viewId], isNotNull);

      callMethod<Object>(maybeApp, 'removeView', <Object>[viewId]);
      expect(bootstrap.viewManager[viewId], isNull);
    });
  });
}
