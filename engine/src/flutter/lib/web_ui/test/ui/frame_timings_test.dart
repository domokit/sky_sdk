// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;

import '../common/test_initialization.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  setUp(() async {
    await bootstrapAndRunApp(withImplicitView: true);
  });

  test('collects frame timings', () async {
    final EnginePlatformDispatcher dispatcher =
        ui.PlatformDispatcher.instance as EnginePlatformDispatcher;
    List<ui.FrameTiming>? timings;
    dispatcher.onReportTimings = (List<ui.FrameTiming> data) {
      timings = data;
    };
    Completer<void> frameDone = Completer<void>();
    dispatcher.onDrawFrame = () {
      final ui.SceneBuilder sceneBuilder = ui.SceneBuilder();
      sceneBuilder
        ..pushOffset(0, 0)
        ..pop();
      dispatcher.render(sceneBuilder.build()).then((_) {
        frameDone.complete();
      });
    };

    // Frame 1.
    dispatcher.scheduleFrame();
    await frameDone.future;
    expect(timings, isNull, reason: "100 ms hasn't passed yet");
    await Future<void>.delayed(const Duration(milliseconds: 150));

    // Frame 2.
    frameDone = Completer<void>();
    dispatcher.scheduleFrame();
    await frameDone.future;
    expect(timings, hasLength(2), reason: '100 ms passed. 2 frames pumped.');
    for (final ui.FrameTiming timing in timings!) {
      expect(timing.vsyncOverhead, greaterThanOrEqualTo(Duration.zero));
      expect(timing.buildDuration, greaterThanOrEqualTo(Duration.zero));
      expect(timing.rasterDuration, greaterThanOrEqualTo(Duration.zero));
      expect(timing.totalSpan, greaterThanOrEqualTo(Duration.zero));
      expect(timing.layerCacheCount, equals(0));
      expect(timing.layerCacheBytes, equals(0));
      expect(timing.pictureCacheCount, equals(0));
      expect(timing.pictureCacheBytes, equals(0));
    }
  });
}
