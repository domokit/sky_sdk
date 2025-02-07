// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  group('FrameService', () {
    setUp(() {
      FrameService.debugOverrideFrameService(null);
      expect(FrameService.instance.runtimeType, FrameService);
      EnginePlatformDispatcher.instance.onBeginFrame = null;
      EnginePlatformDispatcher.instance.onDrawFrame = null;
    });

    test('instance is valid and can be overridden', () {
      final defaultInstance = FrameService.instance;
      expect(defaultInstance.runtimeType, FrameService);

      FrameService.debugOverrideFrameService(DummyFrameService());
      expect(FrameService.instance.runtimeType, DummyFrameService);

      FrameService.debugOverrideFrameService(null);
      expect(FrameService.instance.runtimeType, FrameService);
    });

    test('counts frames', () async {
      final instance = FrameService.instance;
      instance.debugResetFrameNumber();

      final frameCompleter = Completer<void>();
      instance.onFinishedRenderingFrame = () {
        frameCompleter.complete();
      };

      expect(instance.debugFrameNumber, 0);
      instance.scheduleFrame();
      await frameCompleter.future;
      expect(instance.debugFrameNumber, 1);
    });

    test('isFrameScheduled is true iff the frame is scheduled', () async {
      final instance = FrameService.instance;
      instance.debugResetFrameNumber();

      var frameCompleter = Completer<void>();
      instance.onFinishedRenderingFrame = () {
        frameCompleter.complete();
      };

      // Normal case: pump one frame
      expect(instance.isFrameScheduled, isFalse);
      instance.scheduleFrame();
      expect(instance.isFrameScheduled, isTrue);
      await frameCompleter.future;
      expect(instance.isFrameScheduled, isFalse);
      expect(instance.debugFrameNumber, 1);

      // Test idempotency
      instance.debugResetFrameNumber();
      frameCompleter = Completer<void>();
      instance.scheduleFrame();
      instance.scheduleFrame();
      instance.scheduleFrame();
      instance.scheduleFrame();

      expect(instance.isFrameScheduled, isTrue);
      await frameCompleter.future;
      expect(instance.isFrameScheduled, isFalse);
      expect(instance.debugFrameNumber, 1);
    });

    test('onBeginFrame and onDrawFrame are called with isRenderingFrame set to true', () async {
      final instance = FrameService.instance;

      bool? isRenderingInOnBeginFrame;
      EnginePlatformDispatcher.instance.onBeginFrame = (_) {
        isRenderingInOnBeginFrame = instance.isRenderingFrame;
      };

      bool? isRenderingInOnDrawFrame;
      EnginePlatformDispatcher.instance.onDrawFrame = () {
        isRenderingInOnDrawFrame = instance.isRenderingFrame;
      };

      final frameCompleter = Completer<void>();
      bool? valueInOnFinishedRenderingFrame;
      instance.onFinishedRenderingFrame = () {
        valueInOnFinishedRenderingFrame = instance.isRenderingFrame;
        frameCompleter.complete();
      };

      expect(instance.isRenderingFrame, isFalse);
      instance.scheduleFrame();

      // IMPORTANT: scheduled, but not yet rendering
      expect(instance.isRenderingFrame, isFalse);
      await frameCompleter.future;
      expect(instance.isFrameScheduled, isFalse);

      expect(isRenderingInOnBeginFrame, isTrue);
      expect(isRenderingInOnDrawFrame, isTrue);
      expect(valueInOnFinishedRenderingFrame, isFalse);
    });

    test('scheduleWarmUpFrame', () async {
      final instance = FrameService.instance;

      final frameCompleter = Completer<void>();
      bool? valueInOnFinishedRenderingFrame;
      instance.onFinishedRenderingFrame = () {
        valueInOnFinishedRenderingFrame = instance.isRenderingFrame;
        frameCompleter.complete();
      };

      bool? isRenderingInOnBeginFrame;
      bool? isRenderingInOnDrawFrame;

      expect(instance.isRenderingFrame, isFalse);
      expect(instance.isFrameScheduled, isFalse);

      instance.scheduleWarmUpFrame(
        beginFrame: () {
          isRenderingInOnBeginFrame = instance.isRenderingFrame;
        },
        drawFrame: () {
          isRenderingInOnDrawFrame = instance.isRenderingFrame;
        },
      );

      // Even though the warm-up frame is scheduled the value of
      // isFrameScheduled remains false. This is because, for reasons to be yet
      // addressed, the warm-up frame can be (and indeed is) interleaved with
      // a normal scheduleFrame request. See the TODOs inside the
      // scheduleWarmUpFrame code, and this discussion in particular:
      // https://github.com/flutter/engine/pull/50570#discussion_r1496671676
      expect(instance.isFrameScheduled, isFalse);
      expect(instance.isRenderingFrame, isFalse);
      await frameCompleter.future;
      expect(instance.isFrameScheduled, isFalse);

      expect(isRenderingInOnBeginFrame, isTrue);
      expect(isRenderingInOnDrawFrame, isTrue);
      expect(valueInOnFinishedRenderingFrame, isFalse);
    });
  });
}

class DummyFrameService extends FrameService {}
