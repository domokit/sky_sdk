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
      surface.createOrUpdateSurface(const BitmapSize(9, 19));
      final CkSurface originalSurface = surface.debugGetCkSurface()!;
      final DomOffscreenCanvas original = surface.debugGetOffscreenCanvas()!;

      // Expect exact requested dimensions.
      expect(original.width, 9);
      expect(original.height, 19);
      expect(originalSurface.width(), 9);
      expect(originalSurface.height(), 19);

      // Shrinking causes the surface to create a new canvas with the exact
      // size requested.
      surface.createOrUpdateSurface(const BitmapSize(5, 15));
      final CkSurface shrunkSurface = surface.debugGetCkSurface()!;
      final DomOffscreenCanvas shrunk = surface.debugGetOffscreenCanvas()!;
      expect(shrunk, same(original));
      expect(shrunkSurface, isNot(same(originalSurface)));
      expect(shrunkSurface.width(), 5);
      expect(shrunkSurface.height(), 15);

      // The first increase will allocate a new surface to exactly the
      // requested size.
      surface.createOrUpdateSurface(const BitmapSize(10, 20));
      final CkSurface firstIncreaseSurface = surface.debugGetCkSurface()!;
      final DomOffscreenCanvas firstIncrease = surface.debugGetOffscreenCanvas()!;
      expect(firstIncrease, same(original));
      expect(firstIncreaseSurface, isNot(same(shrunkSurface)));

      // Expect exact dimensions
      expect(firstIncrease.width, 10);
      expect(firstIncrease.height, 20);
      expect(firstIncreaseSurface.width(), 10);
      expect(firstIncreaseSurface.height(), 20);

      // Subsequent increases within 40% will still allocate a new canvas.
      surface.createOrUpdateSurface(const BitmapSize(11, 22));
      final CkSurface secondIncreaseSurface = surface.debugGetCkSurface()!;
      final DomOffscreenCanvas secondIncrease = surface.debugGetOffscreenCanvas()!;
      expect(secondIncrease, same(firstIncrease));
      expect(secondIncreaseSurface, isNot(same(firstIncreaseSurface)));
      expect(secondIncreaseSurface.width(), 11);
      expect(secondIncreaseSurface.height(), 22);

      // Increases beyond the 40% limit will cause a new allocation.
      surface.createOrUpdateSurface(const BitmapSize(20, 40));
      final CkSurface hugeSurface = surface.debugGetCkSurface()!;
      final DomOffscreenCanvas huge = surface.debugGetOffscreenCanvas()!;
      expect(huge, same(secondIncrease));
      expect(hugeSurface, isNot(same(secondIncreaseSurface)));

      // Also exactly-allocated
      expect(huge.width, 20);
      expect(huge.height, 40);
      expect(hugeSurface.width(), 20);
      expect(hugeSurface.height(), 40);

      // Shrink again. Create a new surface.
      surface.createOrUpdateSurface(const BitmapSize(5, 15));
      final CkSurface shrunkSurface2 = surface.debugGetCkSurface()!;
      final DomOffscreenCanvas shrunk2 = surface.debugGetOffscreenCanvas()!;
      expect(shrunk2, same(huge));
      expect(shrunkSurface2, isNot(same(hugeSurface)));
      expect(shrunkSurface2.width(), 5);
      expect(shrunkSurface2.height(), 15);

      // Doubling the DPR should halve the CSS width, height, and translation of the canvas.
      // This tests https://github.com/flutter/flutter/issues/77084
      EngineFlutterDisplay.instance.debugOverrideDevicePixelRatio(2.0);
      surface.createOrUpdateSurface(const BitmapSize(5, 15));
      final CkSurface dpr2Surface = surface.debugGetCkSurface()!;
      final DomOffscreenCanvas dpr2Canvas = surface.debugGetOffscreenCanvas()!;
      expect(dpr2Canvas, same(huge));
      expect(dpr2Surface, isNot(same(hugeSurface)));
      expect(dpr2Surface.width(), 5);
      expect(dpr2Surface.height(), 15);

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

      // Shrinking causes us to resize the canvas.
      surface.createOrUpdateSurface(const BitmapSize(5, 15));
      final DomCanvasElement shrunk = getDisplayCanvas(surface);
      canvasSize = getCssSize(surface);
      expect(shrunk.width, 5);
      expect(shrunk.height, 15);
      expect(canvasSize.width, 5);
      expect(canvasSize.height, 15);

      // Increasing the size causes us to resize the canvas.
      surface.createOrUpdateSurface(const BitmapSize(10, 20));
      final DomCanvasElement firstIncrease = getDisplayCanvas(surface);
      canvasSize = getCssSize(surface);

      expect(firstIncrease, same(original));

      // Expect exact dimensions
      expect(firstIncrease.width, 10);
      expect(firstIncrease.height, 20);
      expect(canvasSize.width, 10);
      expect(canvasSize.height, 20);

      // Subsequent increases also cause canvas resizing.
      surface.createOrUpdateSurface(const BitmapSize(11, 22));
      final DomCanvasElement secondIncrease = getDisplayCanvas(surface);
      canvasSize = getCssSize(surface);

      expect(secondIncrease, same(firstIncrease));
      expect(secondIncrease.width, 11);
      expect(secondIncrease.height, 22);
      expect(canvasSize.width, 11);
      expect(canvasSize.height, 22);

      // Increases beyond the 40% limit will cause a canvas resize.
      surface.createOrUpdateSurface(const BitmapSize(20, 40));
      final DomCanvasElement huge = getDisplayCanvas(surface);
      canvasSize = getCssSize(surface);

      expect(huge, same(secondIncrease));

      // Also exact
      expect(huge.width, 20);
      expect(huge.height, 40);
      expect(canvasSize.width, 20);
      expect(canvasSize.height, 40);

      // Shrink again. Resize the canvas.
      surface.createOrUpdateSurface(const BitmapSize(5, 15));
      final DomCanvasElement shrunk2 = getDisplayCanvas(surface);
      canvasSize = getCssSize(surface);

      expect(shrunk2, same(huge));
      expect(shrunk2.width, 5);
      expect(shrunk2.height, 15);
      expect(canvasSize.width, 5);
      expect(canvasSize.height, 15);

      // Doubling the DPR should halve the CSS width, height, and translation of the canvas.
      // This tests https://github.com/flutter/flutter/issues/77084
      EngineFlutterDisplay.instance.debugOverrideDevicePixelRatio(2.0);
      surface.createOrUpdateSurface(const BitmapSize(5, 15));
      final DomCanvasElement dpr2Canvas = getDisplayCanvas(surface);
      canvasSize = getCssSize(surface);

      expect(dpr2Canvas, same(huge));
      expect(dpr2Canvas.width, 5);
      expect(dpr2Canvas.height, 15);
      // Canvas is half the size in logical pixels because device pixel ratio is
      // 2.0.
      expect(canvasSize.width, 2.5);
      expect(canvasSize.height, 7.5);
      // Skip on wasm since same() doesn't work for JSValues.
    }, skip: isWasm);

    test(
      'Surface creates new context when WebGL context is restored',
      () async {
        final Surface surface = Surface();
        expect(surface.debugForceNewContext, isTrue);
        surface.createOrUpdateSurface(const BitmapSize(9, 19));
        final CkSurface before = surface.debugGetCkSurface()!;
        expect(surface.debugForceNewContext, isFalse);

        // Pump a timer to flush any microtasks.
        await Future<void>.delayed(Duration.zero);
        surface.createOrUpdateSurface(const BitmapSize(9, 19));
        final CkSurface afterAcquireFrame = surface.debugGetCkSurface()!;
        // Existing context is reused.
        expect(afterAcquireFrame, same(before));

        // Emulate WebGL context loss.
        final DomOffscreenCanvas canvas = surface.debugGetOffscreenCanvas()!;
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

        surface.createOrUpdateSurface(const BitmapSize(9, 19));
        final CkSurface afterContextLost = surface.debugGetCkSurface()!;
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
        surface.createOrUpdateSurface(const BitmapSize(10, 16));
        final CkSurface original = surface.debugGetCkSurface()!;

        expect(original.width(), 10);
        expect(original.height(), 16);
        expect(surface.debugGetOffscreenCanvas()!.width, 10);
        expect(surface.debugGetOffscreenCanvas()!.height, 16);

        // Increase device-pixel ratio: this makes CSS pixels bigger, so we need
        // fewer of them to cover the browser window.
        EngineFlutterDisplay.instance.debugOverrideDevicePixelRatio(2.0);
        surface.createOrUpdateSurface(const BitmapSize(10, 16));
        final CkSurface highDpr = surface.debugGetCkSurface()!;
        expect(highDpr.width(), 10);
        expect(highDpr.height(), 16);
        expect(surface.debugGetOffscreenCanvas()!.width, 10);
        expect(surface.debugGetOffscreenCanvas()!.height, 16);

        // Decrease device-pixel ratio: this makes CSS pixels smaller, so we need
        // more of them to cover the browser window.
        EngineFlutterDisplay.instance.debugOverrideDevicePixelRatio(0.5);
        surface.createOrUpdateSurface(const BitmapSize(10, 16));
        final CkSurface lowDpr = surface.debugGetCkSurface()!;
        expect(lowDpr.width(), 10);
        expect(lowDpr.height(), 16);
        expect(surface.debugGetOffscreenCanvas()!.width, 10);
        expect(surface.debugGetOffscreenCanvas()!.height, 16);

        // See https://github.com/flutter/flutter/issues/77084#issuecomment-1120151172
        EngineFlutterDisplay.instance.debugOverrideDevicePixelRatio(2.0);
        surface.createOrUpdateSurface(BitmapSize.fromSize(const ui.Size(9.9, 15.9)));
        final CkSurface changeRatioAndSize = surface.debugGetCkSurface()!;
        expect(changeRatioAndSize.width(), 10);
        expect(changeRatioAndSize.height(), 16);
        expect(surface.debugGetOffscreenCanvas()!.width, 10);
        expect(surface.debugGetOffscreenCanvas()!.height, 16);
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
