// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  // We need a separate test file for this test case (instead of including it
  // in platform_channel_test.dart) since we rely on the WidgetsFlutterBinding
  // not being initialized and we call ensureInitialized() in the other test
  // file.
  test('throws assertion error iff WidgetsFlutterBinding is not yet initialized', () {
    const MethodChannel methodChannel = MethodChannel('mock');

    // Ensure that accessing the binary messenger before initialization reports
    // a helpful error message.
    expect(
      () => methodChannel.binaryMessenger,
      throwsA(
        isA<AssertionError>().having(
          (AssertionError e) => e.message,
          'message',
          contains('WidgetsFlutterBinding.ensureInitialized()'),
        ),
      ),
    );

    TestWidgetsFlutterBinding.ensureInitialized();
    expect(() => methodChannel.binaryMessenger, returnsNormally);
  });
}
