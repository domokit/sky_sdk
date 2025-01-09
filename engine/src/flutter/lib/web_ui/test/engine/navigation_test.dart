// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:typed_data';

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;

import '../common/test_initialization.dart';

const MethodCodec codec = JSONMethodCodec();

EngineFlutterWindow get implicitView => EnginePlatformDispatcher.instance.implicitView!;

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  group('without implicit view', () {
    test('Handles navigation gracefully when no implicit view exists', () async {
      expect(EnginePlatformDispatcher.instance.implicitView, isNull);

      final Completer<ByteData?> completer = Completer<ByteData?>();
      ui.PlatformDispatcher.instance.sendPlatformMessage(
        'flutter/navigation',
        codec.encodeMethodCall(
          const MethodCall('routeUpdated', <String, dynamic>{'routeName': '/foo'}),
        ),
        (ByteData? response) => completer.complete(response),
      );
      final ByteData? response = await completer.future;
      expect(response, isNull);
    });
  });

  group('with implicit view', () {
    late TestUrlStrategy strategy;

    setUpImplicitView();

    setUp(() async {
      strategy = TestUrlStrategy();
      await implicitView.debugInitializeHistory(strategy, useSingle: true);
    });

    test('Tracks pushed, replaced and popped routes', () async {
      final Completer<void> completer = Completer<void>();
      ui.PlatformDispatcher.instance.sendPlatformMessage(
        'flutter/navigation',
        codec.encodeMethodCall(
          const MethodCall('routeUpdated', <String, dynamic>{'routeName': '/foo'}),
        ),
        (_) => completer.complete(),
      );
      await completer.future;
      expect(strategy.getPath(), '/foo');
    });
  });
}
