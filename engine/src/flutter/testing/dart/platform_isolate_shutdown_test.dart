// Copyright 2024 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:ffi';
import 'dart:isolate';
import 'dart:ui';

import 'package:test/test.dart';

void main() {
  test('PlatformIsolate runOnPlatformThread, cancels pending jobs if shutdown', () async {
    final Future<int> slowTask = runOnPlatformThread(() async {
      await Future<void>.delayed(const Duration(seconds: 10));
      return 123;
    });

    await runOnPlatformThread(() {
      _forceShutdownIsolate();
      Future<void>(() => Isolate.exit());
    });

    bool throws = false;
    try {
      await slowTask;
    } catch (error) {
      expect(error.toString(), contains('PlatformIsolate shutdown unexpectedly'));
      throws = true;
    }
    expect(throws, true);

    // Platform isolate automatically restarts.
    final int result = await runOnPlatformThread(() => 123);
    expect(result, 123);
  });
}

@Native<Void Function()>(symbol: 'ForceShutdownIsolate')
external void _forceShutdownIsolate();
