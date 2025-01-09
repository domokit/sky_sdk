// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@TestOn('chrome') // Uses web-only Flutter SDK
library;

import 'dart:async';
import 'dart:ui_web' as ui_web;

import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:flutter_web_plugins/flutter_web_plugins.dart';

void main() {
  // Disabling tester emulation because this test relies on real message channel communication.
  ui_web.debugEmulateFlutterTesterEnvironment = false;

  group('Plugin Event Channel', () {
    setUp(() {
      TestWidgetsFlutterBinding.ensureInitialized();
      webPluginRegistry.registerMessageHandler();
    });

    test('can send events to an $EventChannel (deprecated API)', () async {
      const EventChannel listeningChannel = EventChannel('test');
      const PluginEventChannel<String> sendingChannel = PluginEventChannel<String>('test');

      final StreamController<String> controller = StreamController<String>();
      sendingChannel.setController(controller);

      expect(listeningChannel.receiveBroadcastStream(), emitsInOrder(<String>['hello', 'world']));

      controller.add('hello');
      controller.add('world');
      await controller.close();
    });

    test('can send events to an $EventChannel', () async {
      const EventChannel listeningChannel = EventChannel('test');
      const PluginEventChannel<String> sendingChannel = PluginEventChannel<String>('test');

      final StreamController<String> controller = StreamController<String>();
      sendingChannel.setController(controller);

      expect(listeningChannel.receiveBroadcastStream(), emitsInOrder(<String>['hello', 'world']));

      controller.add('hello');
      controller.add('world');
      await controller.close();
    });

    test('can send errors to an $EventChannel (deprecated API)', () async {
      const EventChannel listeningChannel = EventChannel('test2');
      const PluginEventChannel<String> sendingChannel = PluginEventChannel<String>('test2');

      final StreamController<String> controller = StreamController<String>();
      sendingChannel.setController(controller);

      expect(
        listeningChannel.receiveBroadcastStream(),
        emitsError(
          predicate<dynamic>((dynamic e) => e is PlatformException && e.message == 'Test error'),
        ),
      );

      controller.addError('Test error');
      await controller.close();
    });

    test('can send errors to an $EventChannel', () async {
      const EventChannel listeningChannel = EventChannel('test2');
      const PluginEventChannel<String> sendingChannel = PluginEventChannel<String>('test2');

      final StreamController<String> controller = StreamController<String>();
      sendingChannel.setController(controller);

      expect(
        listeningChannel.receiveBroadcastStream(),
        emitsError(
          predicate<dynamic>((dynamic e) => e is PlatformException && e.message == 'Test error'),
        ),
      );

      controller.addError('Test error');
      await controller.close();
    });

    test('receives a listen event (deprecated API)', () async {
      const EventChannel listeningChannel = EventChannel('test3');
      const PluginEventChannel<String> sendingChannel = PluginEventChannel<String>('test3');

      final StreamController<String> controller = StreamController<String>(
        onListen: expectAsync0<void>(() {}),
      );
      sendingChannel.setController(controller);

      expect(listeningChannel.receiveBroadcastStream(), emitsInOrder(<String>['hello']));

      controller.add('hello');
      await controller.close();
    });

    test('receives a listen event', () async {
      const EventChannel listeningChannel = EventChannel('test3');
      const PluginEventChannel<String> sendingChannel = PluginEventChannel<String>('test3');

      final StreamController<String> controller = StreamController<String>(
        onListen: expectAsync0<void>(() {}),
      );
      sendingChannel.setController(controller);

      expect(listeningChannel.receiveBroadcastStream(), emitsInOrder(<String>['hello']));

      controller.add('hello');
      await controller.close();
    });

    test('receives a cancel event (deprecated API)', () async {
      const EventChannel listeningChannel = EventChannel('test4');
      const PluginEventChannel<String> sendingChannel = PluginEventChannel<String>('test4');

      final StreamController<String> controller = StreamController<String>(
        onCancel: expectAsync0<void>(() {}),
      );
      sendingChannel.setController(controller);

      final Stream<dynamic> eventStream = listeningChannel.receiveBroadcastStream();
      late StreamSubscription<dynamic> subscription;
      subscription = eventStream.listen(
        expectAsync1<void, dynamic>((dynamic x) {
          expect(x, equals('hello'));
          subscription.cancel();
        }),
      );

      controller.add('hello');
    });

    test('receives a cancel event', () async {
      const EventChannel listeningChannel = EventChannel('test4');
      const PluginEventChannel<String> sendingChannel = PluginEventChannel<String>('test4');

      final StreamController<String> controller = StreamController<String>(
        onCancel: expectAsync0<void>(() {}),
      );
      sendingChannel.setController(controller);

      final Stream<dynamic> eventStream = listeningChannel.receiveBroadcastStream();
      late StreamSubscription<dynamic> subscription;
      subscription = eventStream.listen(
        expectAsync1<void, dynamic>((dynamic x) {
          expect(x, equals('hello'));
          subscription.cancel();
        }),
      );

      controller.add('hello');
    });
  });
}
