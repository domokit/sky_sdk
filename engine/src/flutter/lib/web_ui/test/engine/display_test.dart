// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  group('EngineFlutterDisplay', () {
    test('overrides and restores devicePixelRatio', () {
      final EngineFlutterDisplay display = EngineFlutterDisplay(
        id: 0,
        size: const ui.Size(100.0, 100.0),
        refreshRate: 60.0,
      );

      final double originalDevicePixelRatio = display.devicePixelRatio;
      display.debugOverrideDevicePixelRatio(99.3);
      expect(display.devicePixelRatio, 99.3);

      display.debugOverrideDevicePixelRatio(null);
      expect(display.devicePixelRatio, originalDevicePixelRatio);
    });

    test('computes device pixel ratio using window.devicePixelRatio and visualViewport.scale', () {
      final EngineFlutterDisplay display = EngineFlutterDisplay(
        id: 0,
        size: const ui.Size(100.0, 100.0),
        refreshRate: 60.0,
      );
      final double windowDpr = domWindow.devicePixelRatio;
      final double visualViewportScale = domWindow.visualViewport!.scale!;
      expect(display.browserDevicePixelRatio, windowDpr * visualViewportScale);
    });
  });
}
