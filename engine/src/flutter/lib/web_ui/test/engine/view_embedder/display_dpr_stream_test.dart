// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';

void main() {
  internalBootstrapBrowserTest(() => doTests);
}

void doTests() {
  final DomEventTarget eventTarget = createDomElement('div');

  group('dprChanged Stream', () {
    late DisplayDprStream dprStream;

    setUp(() async {
      dprStream = DisplayDprStream(
        EngineFlutterDisplay.instance,
        overrides: DebugDisplayDprStreamOverrides(getMediaQuery: (_) => eventTarget),
      );
    });

    test('funnels display DPR on every mediaQuery "change" event.', () async {
      final Future<List<double>> dprs =
          dprStream.dprChanged.take(3).timeout(const Duration(seconds: 1)).toList();

      // Simulate the events
      EngineFlutterDisplay.instance.debugOverrideDevicePixelRatio(6.9);
      eventTarget.dispatchEvent(createDomEvent('Event', 'change'));
      EngineFlutterDisplay.instance.debugOverrideDevicePixelRatio(4.2);
      eventTarget.dispatchEvent(createDomEvent('Event', 'change'));
      EngineFlutterDisplay.instance.debugOverrideDevicePixelRatio(0.71);
      eventTarget.dispatchEvent(createDomEvent('Event', 'change'));

      expect(await dprs, <double>[6.9, 4.2, 0.71]);
    });
  });
}
