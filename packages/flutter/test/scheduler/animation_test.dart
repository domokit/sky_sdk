// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

import 'scheduler_tester.dart';

class TestSchedulerBinding extends BindingBase with SchedulerBinding, ServicesBinding {}

void main() {
  final SchedulerBinding scheduler = TestSchedulerBinding();

  test('Check for a time dilation being in effect', () {
    expect(timeDilation, equals(1.0));
  });

  test('Can cancel queued callback', () {
    late int secondId;

    bool firstCallbackRan = false;
    bool secondCallbackRan = false;

    void firstCallback(Duration timeStamp) {
      expect(firstCallbackRan, isFalse);
      expect(secondCallbackRan, isFalse);
      expect(timeStamp.inMilliseconds, equals(0));
      firstCallbackRan = true;
      scheduler.cancelFrameCallbackWithId(secondId);
    }

    void secondCallback(Duration timeStamp) {
      expect(firstCallbackRan, isTrue);
      expect(secondCallbackRan, isFalse);
      expect(timeStamp.inMilliseconds, equals(0));
      secondCallbackRan = true;
    }

    scheduler.scheduleFrameCallback(firstCallback);
    secondId = scheduler.scheduleFrameCallback(secondCallback);

    tick(const Duration(milliseconds: 16));

    expect(firstCallbackRan, isTrue);
    expect(secondCallbackRan, isFalse);

    firstCallbackRan = false;
    secondCallbackRan = false;

    tick(const Duration(milliseconds: 32));

    expect(firstCallbackRan, isFalse);
    expect(secondCallbackRan, isFalse);
  });
}
