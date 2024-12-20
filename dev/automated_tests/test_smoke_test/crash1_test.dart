// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' as system;

import 'package:flutter_test/flutter_test.dart';

// this is a test to make sure our tests consider engine crashes to be failures
// see //flutter/dev/bots/test.dart

void main() {
  test('test smoke test -- this test should fail', () async {
    if (system.Process.killPid(system.pid, system.ProcessSignal.sigsegv)) {
      print('system.Process.killPid returned before the process ended!');
      print(
        'Sleeping for a few seconds just in case signal delivery is delayed or our signal handler is being slow...',
      );
      system.sleep(const Duration(seconds: 10)); // don't sleep too much, we must not time out
    } else {
      print('system.Process.killPid reports that the SIGSEGV signal was not delivered!');
    }
    print('crash1_test.dart will now probably not crash, which will ruin the test.');
  });
}
