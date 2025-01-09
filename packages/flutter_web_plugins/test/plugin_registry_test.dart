// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@TestOn('chrome') // Uses web-only Flutter SDK
library;

import 'dart:ui_web' as ui_web;

import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:flutter_web_plugins/flutter_web_plugins.dart';

class TestPlugin {
  static void registerWith(Registrar registrar) {
    final MethodChannel channel = MethodChannel(
      'test_plugin',
      const StandardMethodCodec(),
      registrar.messenger,
    );
    final TestPlugin testPlugin = TestPlugin();
    channel.setMethodCallHandler(testPlugin.handleMethodCall);
  }

  static final List<String> calledMethods = <String>[];

  Future<void> handleMethodCall(MethodCall call) async {
    calledMethods.add(call.method);
  }
}

void main() {
  // Disabling tester emulation because this test relies on real message channel communication.
  ui_web.debugEmulateFlutterTesterEnvironment = false;

  group('Plugin Registry', () {
    setUp(() {
      TestWidgetsFlutterBinding.ensureInitialized();
      webPluginRegistry.registerMessageHandler();
      final Registrar registrar = webPluginRegistry.registrarFor(TestPlugin);
      TestPlugin.registerWith(registrar);
    });

    test('can register a plugin', () {
      TestPlugin.calledMethods.clear();

      const MethodChannel frameworkChannel = MethodChannel('test_plugin');
      frameworkChannel.invokeMethod<void>('test1');

      expect(TestPlugin.calledMethods, equals(<String>['test1']));
    });

    test('can send a message from the plugin to the framework', () async {
      const StandardMessageCodec codec = StandardMessageCodec();

      final List<String> loggedMessages = <String>[];
      ServicesBinding.instance.defaultBinaryMessenger.setMessageHandler('test_send', (
        ByteData? data,
      ) {
        loggedMessages.add(codec.decodeMessage(data)! as String);
        return Future<ByteData?>.value();
      });

      await pluginBinaryMessenger.send('test_send', codec.encodeMessage('hello'));
      expect(loggedMessages, equals(<String>['hello']));

      await pluginBinaryMessenger.send('test_send', codec.encodeMessage('world'));
      expect(loggedMessages, equals(<String>['hello', 'world']));

      ServicesBinding.instance.defaultBinaryMessenger.setMessageHandler('test_send', null);
    });
  });
}
