// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:ui' as ui;
import 'package:flutter/foundation.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

class TestChannelBuffersFlutterBinding extends BindingBase with SchedulerBinding, ServicesBinding {}

void main() {
  ByteData makeByteData(String str) {
    return ByteData.sublistView(utf8.encode(str));
  }

  String getString(ByteData data) {
    return utf8.decode(Uint8List.sublistView(data));
  }

  test('does drain channel buffers', () async {
    const String channel = 'foo';
    final TestChannelBuffersFlutterBinding binding = TestChannelBuffersFlutterBinding();
    expect(binding.defaultBinaryMessenger, isNotNull);
    bool didCallCallback = false;
    void callback(ByteData? responseData) {
      didCallCallback = true;
    }

    const String payload = 'bar';
    final ByteData data = makeByteData(payload);
    ui.channelBuffers.push(channel, data, callback);
    bool didDrainData = false;
    binding.defaultBinaryMessenger.setMessageHandler(channel, (ByteData? message) async {
      expect(getString(message!), payload);
      didDrainData = true;
      return null;
    });
    // Flush the event queue.
    await Future<void>(() {});
    expect(didDrainData, isTrue);
    expect(didCallCallback, isTrue);
  });
}
