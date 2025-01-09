// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

/// Simulates a system back, like a back gesture on Android.
///
/// Sends the same platform channel message that the engine sends when it
/// receives a system back.
Future<void> simulateSystemBack() {
  return TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.handlePlatformMessage(
    'flutter/navigation',
    const JSONMessageCodec().encodeMessage(<String, dynamic>{'method': 'popRoute'}),
    (ByteData? _) {},
  );
}
