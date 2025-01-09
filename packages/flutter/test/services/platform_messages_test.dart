// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@TestOn('!chrome')
library;

import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  TestWidgetsFlutterBinding.ensureInitialized();

  test('Mock binary message handler control test', () async {
    final List<ByteData?> log = <ByteData>[];

    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMessageHandler(
      'test1',
      (ByteData? message) async {
        log.add(message);
        return null;
      },
    );

    final ByteData message = ByteData(2)..setUint16(0, 0xABCD);
    await TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.send('test1', message);
    expect(log, equals(<ByteData>[message]));
    log.clear();

    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMessageHandler(
      'test1',
      null,
    );
    await TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.send('test1', message);
    expect(log, isEmpty);
  });
}
