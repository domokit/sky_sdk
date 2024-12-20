// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:js_interop';
import 'dart:js_util' as js_util;

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;

import 'common.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  group('CanvasKit', () {
    setUpCanvasKitTest();
    setUp(() {
      EngineFlutterDisplay.instance.debugOverrideDevicePixelRatio(1.0);
    });

    test('Surface allocates canvases efficiently', () {
      final Surface surface = Surface();
      final CkSurface originalSurface = surface.acquireFrame(const ui.Size(9, 19)).skiaSurface;
      final DomOffscreenCanvas original = surface.debugOffscreenCanvas!;

      // Expect exact requested dimensions.
      expect(original.width, 9);
      expect(original.height, 19);
      expect(originalSurface.width(), 9);
      expect(originalSurface.height(), 19);

      // Shrinking reuses the existing canvas but translates it so
      // Skia renders into the visible area.
      final CkSurface shrunkSurface = surface.acquireFrame(const ui.Size(5, 15)).skiaSurface;
      final DomOffscreenCanvas shrunk = surface.debugOffscreenCanvas!;
      expect(shrunk, same(original));
      expect(shrunkSurface, isNot(same(originalSurface)));
      expect(shrunkSurface.width(), 5);
      expect(shrunkSurface.height(), 15);

      // The first increase will allocate a new surface, but will overallocate
      // by 40% to accommodate future increases.
      final CkSurface firstIncreaseSurface =
          surface.acquireFrame(const ui.Size(10, 20)).skiaSurface;
      final DomOffscreenCanvas firstIncrease = surface.debugOffscreenCanvas!;
      expect(firstIncrease, same(original));
      expect(firstIncreaseSurface, isNot(same(shrunkSurface)));

      // Expect overallocated dimensions
      expect(firstIncrease.width, 14);
      expect(firstIncrease.height, 28);
      expect(firstIncreaseSurface.width(), 10);
      expect(firstIncreaseSurface.height(), 20);

      // Subsequent increases within 40% reuse the old canvas.
      final CkSurface secondIncreaseSurface =
          surface.acquireFrame(const ui.Size(11, 22)).skiaSurface;
      final DomOffscreenCanvas secondIncrease = surface.debugOffscreenCanvas!;
      expect(secondIncrease, same(firstIncrease));
      expect(secondIncreaseSurface, isNot(same(firstIncreaseSurface)));
      expect(secondIncreaseSurface.width(), 11);
      expect(secondIncreaseSurface.height(), 22);

      // Increases beyond the 40% limit will cause a new allocation.
      final CkSurface hugeSurface = surface.acquireFrame(const ui.Size(20, 40)).skiaSurface;
      final DomOffscreenCanvas huge = surface.debugOffscreenCanvas!;
      expect(huge, same(secondIncrease));
      expect(hugeSurface, isNot(same(secondIncreaseSurface)));

      // Also over-allocated
      expect(huge.width, 28);
      expect(huge.height, 56);
      expect(hugeSurface.width(), 20);
      expect(hugeSurface.height(), 40);

      // Shrink again. Reuse the last allocated surface.
      final CkSurface shrunkSurface2 = surface.acquireFrame(const ui.Size(5, 15)).skiaSurface;
      final DomOffscreenCanvas shrunk2 = surface.debugOffscreenCanvas!;
      expect(shrunk2, same(huge));
      expect(shrunkSurface2, isNot(same(hugeSurface)));
      expect(shrunkSurface2.width(), 5);
      expect(shrunkSurface2.height(), 15);

      // Doubling the DPR should halve the CSS width, height, and translation of the canvas.
      // This tests https://github.com/flutter/flutter/issues/77084
      EngineFlutterDisplay.instance.debugOverrideDevicePixelRatio(2.0);
      final CkSurface dpr2Surface2 = surface.acquireFrame(const ui.Size(5, 15)).skiaSurface;
      final DomOffscreenCanvas dpr2Canvas = surface.debugOffscreenCanvas!;
      expect(dpr2Canvas, same(huge));
      expect(dpr2Surface2, isNot(same(hugeSurface)));
      expect(dpr2Surface2.width(), 5);
      expect(dpr2Surface2.height(), 15);

      // Skipping on Firefox for now since Firefox headless doesn't support WebGL
      // This causes issues in the test since we create a Canvas-backed surface,
      // which cannot be a different size from the canvas.
      // TODO(hterkelsen): See if we can give a custom size for software
      //     surfaces.
    }, skip: isFirefox || !Surface.offscreenCanvasSupported);

    test('Surface used as DisplayCanvas resizes correctly', () {
      final Surface surface = Surface(isDisplayCanvas: true);

      surface.createOrUpdateSurface(const BitmapSize(9, 19));
      final DomCanvasElement original = getDisplayCanvas(surface);
      ui.Size canvasSize = getCssSize(surface);

      // Expect exact requested dimensions.
      expect(original.width, 9);
      expect(original.height, 19);
      expect(canvasSize.width, 9);
      expect(canvasSize.height, 19);

      // Shrinking reuses the existing canvas but translates it so
      // Skia renders into the visible area.
      surface.createOrUpdateSurface(const BitmapSize(5, 15));
      final DomCanvasElement shrunk = getDisplayCanvas(surface);
      canvasSize = getCssSize(surface);
      expect(shrunk.width, 9);
      expect(shrunk.height, 19);
      expect(canvasSize.width, 9);
      expect(canvasSize.height, 19);

      // The first increase will allocate a new surface, but will overallocate
      // by 40% to accommodate future increases.
      surface.createOrUpdateSurface(const BitmapSize(10, 20));
      final DomCanvasElement firstIncrease = getDisplayCanvas(surface);
      canvasSize = getCssSize(surface);

      expect(firstIncrease, same(original));

      // Expect overallocated dimensions
      expect(firstIncrease.width, 14);
      expect(firstIncrease.height, 28);
      expect(canvasSize.width, 14);
      expect(canvasSize.height, 28);

      // Subsequent increases within 40% reuse the old canvas.
      surface.createOrUpdateSurface(const BitmapSize(11, 22));
      final DomCanvasElement secondIncrease = getDisplayCanvas(surface);
      canvasSize = getCssSize(surface);

      expect(secondIncrease, same(firstIncrease));
      expect(secondIncrease.width, 14);
      expect(secondIncrease.height, 28);
      expect(canvasSize.width, 14);
      expect(canvasSize.height, 28);

      // Increases beyond the 40% limit will cause a new allocation.
      surface.createOrUpdateSurface(const BitmapSize(20, 40));
      final DomCanvasElement huge = getDisplayCanvas(surface);
      canvasSize = getCssSize(surface);

      expect(huge, same(secondIncrease));

      // Also over-allocated
      expect(huge.width, 28);
      expect(huge.height, 56);
      expect(canvasSize.width, 28);
      expect(canvasSize.height, 56);

      // Shrink again. Reuse the last allocated surface.
      surface.createOrUpdateSurface(const BitmapSize(5, 15));
      final DomCanvasElement shrunk2 = getDisplayCanvas(surface);
      canvasSize = getCssSize(surface);

      expect(shrunk2, same(huge));
      expect(shrunk2.width, 28);
      expect(shrunk2.height, 56);
      expect(canvasSize.width, 28);
      expect(canvasSize.height, 56);

      // Doubling the DPR should halve the CSS width, height, and translation of the canvas.
      // This tests https://github.com/flutter/flutter/issues/77084
      EngineFlutterDisplay.instance.debugOverrideDevicePixelRatio(2.0);
      surface.createOrUpdateSurface(const BitmapSize(5, 15));
      final DomCanvasElement dpr2Canvas = getDisplayCanvas(surface);
      canvasSize = getCssSize(surface);

      expect(dpr2Canvas, same(huge));
      expect(dpr2Canvas.width, 28);
      expect(dpr2Canvas.height, 56);
      // Canvas is half the size in logical pixels because device pixel ratio is
      // 2.0.
      expect(canvasSize.width, 14);
      expect(canvasSize.height, 28);
      // Skip on wasm since same() doesn't work for JSValues.
    }, skip: isWasm);

    test(
      'Surface creates new context when WebGL context is restored',
      () async {
        final Surface surface = Surface();
        expect(surface.debugForceNewContext, isTrue);
        final CkSurface before = surface.acquireFrame(const ui.Size(9, 19)).skiaSurface;
        expect(surface.debugForceNewContext, isFalse);

        // Pump a timer to flush any microtasks.
        await Future<void>.delayed(Duration.zero);
        final CkSurface afterAcquireFrame = surface.acquireFrame(const ui.Size(9, 19)).skiaSurface;
        // Existing context is reused.
        expect(afterAcquireFrame, same(before));

        // Emulate WebGL context loss.
        final DomOffscreenCanvas canvas = surface.debugOffscreenCanvas!;
        final Object ctx = canvas.getContext('webgl2')!;
        final Object loseContextExtension = js_util.callMethod(ctx, 'getExtension', <String>[
          'WEBGL_lose_context',
        ]);
        js_util.callMethod<void>(loseContextExtension, 'loseContext', const <void>[]);

        // Pump a timer to allow the "lose context" event to propagate.
        await Future<void>.delayed(Duration.zero);
        // We don't create a new GL context until the context is restored.
        expect(surface.debugContextLost, isTrue);
        final bool isContextLost = js_util.callMethod<bool>(ctx, 'isContextLost', const <void>[]);
        expect(isContextLost, isTrue);

        // Emulate WebGL context restoration.
        js_util.callMethod<void>(loseContextExtension, 'restoreContext', const <void>[]);

        // Pump a timer to allow the "restore context" event to propagate.
        await Future<void>.delayed(Duration.zero);
        expect(surface.debugForceNewContext, isTrue);

        final CkSurface afterContextLost = surface.acquireFrame(const ui.Size(9, 19)).skiaSurface;
        // A new context is created.
        expect(afterContextLost, isNot(same(before)));
      },
      // Firefox can't create a WebGL2 context in headless mode.
      skip: isFirefox || !Surface.offscreenCanvasSupported,
    );

    // Regression test for https://github.com/flutter/flutter/issues/75286
    test(
      'updates canvas logical size when device-pixel ratio changes',
      () {
        final Surface surface = Surface();
        final CkSurface original = surface.acquireFrame(const ui.Size(10, 16)).skiaSurface;

        expect(original.width(), 10);
        expect(original.height(), 16);
        expect(surface.debugOffscreenCanvas!.width, 10);
        expect(surface.debugOffscreenCanvas!.height, 16);

        // Increase device-pixel ratio: this makes CSS pixels bigger, so we need
        // fewer of them to cover the browser window.
        EngineFlutterDisplay.instance.debugOverrideDevicePixelRatio(2.0);
        final CkSurface highDpr = surface.acquireFrame(const ui.Size(10, 16)).skiaSurface;
        expect(highDpr.width(), 10);
        expect(highDpr.height(), 16);
        expect(surface.debugOffscreenCanvas!.width, 10);
        expect(surface.debugOffscreenCanvas!.height, 16);

        // Decrease device-pixel ratio: this makes CSS pixels smaller, so we need
        // more of them to cover the browser window.
        EngineFlutterDisplay.instance.debugOverrideDevicePixelRatio(0.5);
        final CkSurface lowDpr = surface.acquireFrame(const ui.Size(10, 16)).skiaSurface;
        expect(lowDpr.width(), 10);
        expect(lowDpr.height(), 16);
        expect(surface.debugOffscreenCanvas!.width, 10);
        expect(surface.debugOffscreenCanvas!.height, 16);

        // See https://github.com/flutter/flutter/issues/77084#issuecomment-1120151172
        EngineFlutterDisplay.instance.debugOverrideDevicePixelRatio(2.0);
        final CkSurface changeRatioAndSize =
            surface.acquireFrame(const ui.Size(9.9, 15.9)).skiaSurface;
        expect(changeRatioAndSize.width(), 10);
        expect(changeRatioAndSize.height(), 16);
        expect(surface.debugOffscreenCanvas!.width, 10);
        expect(surface.debugOffscreenCanvas!.height, 16);
      },
      skip: !Surface.offscreenCanvasSupported,
    );

    test('can recover from MakeSWCanvasSurface failure', () async {
      debugOverrideJsConfiguration(
        <String, Object?>{'canvasKitForceCpuOnly': true}.jsify() as JsFlutterConfiguration?,
      );
      addTearDown(() => debugOverrideJsConfiguration(null));

      final Surface surface = Surface();
      surface.debugThrowOnSoftwareSurfaceCreation = true;
      expect(
        () => surface.createOrUpdateSurface(const BitmapSize(12, 34)),
        throwsA(isA<CanvasKitError>()),
      );
      await Future<void>.delayed(Duration.zero);

      expect(surface.debugForceNewContext, isFalse);

      surface.debugThrowOnSoftwareSurfaceCreation = false;
      final ckSurface = surface.createOrUpdateSurface(const BitmapSize(12, 34));

      expect(ckSurface.surface.width(), 12);
      expect(ckSurface.surface.height(), 34);
    });
  });
}

DomCanvasElement getDisplayCanvas(Surface surface) {
  assert(surface.isDisplayCanvas);
  return surface.hostElement.children.first as DomCanvasElement;
}

/// Extracts the CSS style values of 'width' and 'height' and returns them
/// as a [ui.Size].
ui.Size getCssSize(Surface surface) {
  final DomCanvasElement canvas = getDisplayCanvas(surface);
  final String cssWidth = canvas.style.width;
  final String cssHeight = canvas.style.height;
  // CSS width and height should be in the form 'NNNpx'. So cut off the 'px' and
  // convert to a number.
  final double width = double.parse(cssWidth.substring(0, cssWidth.length - 2).trim());
  final double height = double.parse(cssHeight.substring(0, cssHeight.length - 2).trim());
  return ui.Size(width, height);
}
