// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui show Size;

void main() {
  internalBootstrapBrowserTest(() => doTests);
}

void doTests() {
  final DomElement sizeSource = createDomElement('div')..style.display = 'block';

  group('computePhysicalSize', () {
    late CustomElementDimensionsProvider provider;

    setUp(() {
      sizeSource
        ..style.width = '10px'
        ..style.height = '10px';
      domDocument.body!.append(sizeSource);
      provider = CustomElementDimensionsProvider(sizeSource);
    });

    tearDown(() {
      provider.close(); // cleanup
      sizeSource.remove();
    });

    test('returns physical size of element (width * dpr)', () {
      const double dpr = 2.5;
      const double logicalWidth = 50;
      const double logicalHeight = 75;
      EngineFlutterDisplay.instance.debugOverrideDevicePixelRatio(dpr);

      sizeSource
        ..style.width = '${logicalWidth}px'
        ..style.height = '${logicalHeight}px';

      const ui.Size expected = ui.Size(logicalWidth * dpr, logicalHeight * dpr);

      final ui.Size computed = provider.computePhysicalSize();

      expect(computed, expected);
    });
  });

  group('computeKeyboardInsets', () {
    late CustomElementDimensionsProvider provider;

    setUp(() {
      sizeSource
        ..style.width = '10px'
        ..style.height = '10px';
      domDocument.body!.append(sizeSource);
      provider = CustomElementDimensionsProvider(sizeSource);
    });

    tearDown(() {
      provider.close(); // cleanup
      sizeSource.remove();
    });

    test('from viewport physical size (simulated keyboard) - always zero', () {
      // Simulate a 100px tall keyboard showing...
      const double dpr = 2.5;
      EngineFlutterDisplay.instance.debugOverrideDevicePixelRatio(dpr);
      const double keyboardGap = 100;
      final double physicalHeight = (domWindow.visualViewport!.height! + keyboardGap) * dpr;

      final ViewPadding computed = provider.computeKeyboardInsets(physicalHeight, false);

      expect(computed.top, 0);
      expect(computed.right, 0);
      expect(computed.bottom, 0);
      expect(computed.left, 0);
    });
  });

  group('onResize Stream', () {
    late CustomElementDimensionsProvider provider;

    setUp(() async {
      sizeSource
        ..style.width = '10px'
        ..style.height = '10px';
      domDocument.body!.append(sizeSource);
      provider = CustomElementDimensionsProvider(sizeSource);
      // Let the DOM settle before starting the test, so we don't get the first
      // 10,10 Size in the test. Otherwise, the ResizeObserver may trigger
      // unexpectedly after the test has started, and break our "first" result.
      await Future<void>.delayed(const Duration(milliseconds: 250));
    });

    tearDown(() {
      provider.close(); // cleanup
      sizeSource.remove();
    });

    test('funnels resize events on sizeSource', () async {
      EngineFlutterDisplay.instance.debugOverrideDevicePixelRatio(2.7);

      sizeSource
        ..style.width = '100px'
        ..style.height = '100px';

      expect(provider.onResize.first, completes);
      expect(provider.computePhysicalSize(), const ui.Size(270, 270));

      sizeSource
        ..style.width = '200px'
        ..style.height = '200px';

      expect(provider.onResize.first, completes);
      expect(provider.computePhysicalSize(), const ui.Size(540, 540));

      sizeSource
        ..style.width = '300px'
        ..style.height = '300px';

      expect(provider.onResize.first, completes);
      expect(provider.computePhysicalSize(), const ui.Size(810, 810));
    });

    test('funnels DPR change events too', () async {
      // Override the source of DPR events...
      final StreamController<double> dprController = StreamController<double>.broadcast();

      // Inject the dprController stream into the CustomElementDimensionsProvider.
      final CustomElementDimensionsProvider provider = CustomElementDimensionsProvider(
        sizeSource,
        onDprChange: dprController.stream,
      );

      // Set and broadcast the mock DPR value
      EngineFlutterDisplay.instance.debugOverrideDevicePixelRatio(3.2);
      dprController.add(3.2);

      expect(provider.onResize.first, completes);
      expect(provider.computePhysicalSize(), const ui.Size(32, 32));
    });

    test('closed by onHotRestart', () async {
      // Register an onDone listener for the stream
      final Completer<bool> completer = Completer<bool>();
      provider.onResize.listen(
        null,
        onDone: () {
          completer.complete(true);
        },
      );

      // Should close the stream
      provider.close();

      sizeSource
        ..style.width = '100px'
        ..style.height = '100px';
      // Give time to the mutationObserver to fire (if needed, it won't)
      await Future<void>.delayed(const Duration(milliseconds: 100));

      expect(provider.onResize.isEmpty, completion(isTrue));
      expect(completer.future, completion(isTrue));
    });
  });
}
