// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

class _TestBinding extends BindingBase with SchedulerBinding, ServicesBinding {
  @override
  Future<void> initializationComplete() async {
    return super.initializationComplete();
  }

  @override
  TestDefaultBinaryMessenger get defaultBinaryMessenger =>
      super.defaultBinaryMessenger as TestDefaultBinaryMessenger;

  @override
  TestDefaultBinaryMessenger createBinaryMessenger() {
    Future<ByteData?> keyboardHandler(ByteData? message) async {
      return const StandardMethodCodec().encodeSuccessEnvelope(<int, int>{1: 1});
    }

    return TestDefaultBinaryMessenger(
      super.createBinaryMessenger(),
      outboundHandlers: <String, MessageHandler>{'flutter/keyboard': keyboardHandler},
    );
  }
}

void main() {
  final _TestBinding binding = _TestBinding();

  test('can send message on completion of binding initialization', () async {
    bool called = false;
    binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.platform, (
      MethodCall method,
    ) async {
      if (method.method == 'System.initializationComplete') {
        called = true;
      }
      return null;
    });
    await binding.initializationComplete();
    expect(called, isTrue);
  });
}
