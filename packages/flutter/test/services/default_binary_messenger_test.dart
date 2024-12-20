// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';

import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  TestWidgetsFlutterBinding.ensureInitialized();

  ByteData makeByteData(String str) {
    return ByteData.sublistView(utf8.encode(str));
  }

  test('default binary messenger calls callback once', () async {
    int countInbound = 0;
    int countOutbound = 0;
    const String channel = 'foo';
    final ByteData bar = makeByteData('bar');
    final Completer<void> done = Completer<void>();
    ServicesBinding.instance.channelBuffers.push(channel, bar, (ByteData? message) async {
      expect(message, isNull);
      countOutbound += 1;
      done.complete();
    });
    expect(countInbound, equals(0));
    expect(countOutbound, equals(0));
    ServicesBinding.instance.defaultBinaryMessenger.setMessageHandler(channel, (
      ByteData? message,
    ) async {
      expect(message, bar);
      countInbound += 1;
      return null;
    });
    expect(countInbound, equals(0));
    expect(countOutbound, equals(0));
    await done.future;
    expect(countInbound, equals(1));
    expect(countOutbound, equals(1));
  });

  test('can check the mock handler', () {
    Future<ByteData?> handler(ByteData? call) => Future<ByteData?>.value();
    final TestDefaultBinaryMessenger messenger =
        TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger;

    expect(messenger.checkMockMessageHandler('test_channel', null), true);
    expect(messenger.checkMockMessageHandler('test_channel', handler), false);
    messenger.setMockMessageHandler('test_channel', handler);
    expect(messenger.checkMockMessageHandler('test_channel', handler), true);
    messenger.setMockMessageHandler('test_channel', null);
  });
}
