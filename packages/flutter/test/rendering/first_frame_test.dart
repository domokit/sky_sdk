// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  final TestRenderBinding binding = TestRenderBinding();
  test('Flutter dispatches first frame event on the web only', () async {
    final Completer<void> completer = Completer<void>();
    const MethodChannel firstFrameChannel = MethodChannel('flutter/service_worker');
    binding.defaultBinaryMessenger.setMockMethodCallHandler(firstFrameChannel, (
      MethodCall methodCall,
    ) async {
      completer.complete();
      return null;
    });

    binding.handleBeginFrame(Duration.zero);
    binding.handleDrawFrame();

    await expectLater(completer.future, completes);
  }, skip: !kIsWeb); // [intended] the test is only makes sense on the web.
}

class TestRenderBinding extends BindingBase
    with
        SchedulerBinding,
        ServicesBinding,
        GestureBinding,
        SemanticsBinding,
        RendererBinding,
        TestDefaultBinaryMessengerBinding {}
