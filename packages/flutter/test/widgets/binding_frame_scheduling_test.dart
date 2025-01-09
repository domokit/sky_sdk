// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:flutter/scheduler.dart';
import 'package:flutter/services.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test(
    'Instantiating WidgetsFlutterBinding does neither schedule a frame nor register frame callbacks',
    () async {
      // Regression test for https://github.com/flutter/flutter/issues/39494.

      // Preconditions.
      expect(PlatformDispatcher.instance.onBeginFrame, isNull);
      expect(PlatformDispatcher.instance.onDrawFrame, isNull);

      // Instantiation does nothing with regards to frame scheduling.
      final WidgetsFlutterBindingWithTestBinaryMessenger binding =
          WidgetsFlutterBindingWithTestBinaryMessenger();
      expect(binding, isA<WidgetsFlutterBinding>());
      expect(SchedulerBinding.instance.hasScheduledFrame, isFalse);
      expect(PlatformDispatcher.instance.onBeginFrame, isNull);
      expect(PlatformDispatcher.instance.onDrawFrame, isNull);

      // Framework starts with detached statue. Sends resumed signal to enable frame.
      final ByteData message = const StringCodec().encodeMessage('AppLifecycleState.resumed')!;
      await binding.defaultBinaryMessenger.handlePlatformMessage(
        'flutter/lifecycle',
        message,
        (_) {},
      );
      expect(PlatformDispatcher.instance.onBeginFrame, isNull);
      expect(PlatformDispatcher.instance.onDrawFrame, isNull);
      expect(SchedulerBinding.instance.hasScheduledFrame, isFalse);

      // Frame callbacks are registered lazily (and a frame scheduled) when the root widget is attached.
      WidgetsBinding.instance.attachRootWidget(const Placeholder());
      expect(PlatformDispatcher.instance.onBeginFrame, isNotNull);
      expect(PlatformDispatcher.instance.onDrawFrame, isNotNull);
      expect(SchedulerBinding.instance.hasScheduledFrame, isTrue);
    },
  );
}

class WidgetsFlutterBindingWithTestBinaryMessenger extends WidgetsFlutterBinding
    with TestDefaultBinaryMessengerBinding {}
