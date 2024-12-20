// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
import 'dart:js_interop';

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
    setUpCanvasKitTest(withImplicitView: true);
    setUp(() async {
      EngineFlutterDisplay.instance.debugOverrideDevicePixelRatio(1.0);
    });

    Future<DomImageBitmap> newBitmap(int width, int height) async {
      return createImageBitmap(createBlankDomImageData(width, height) as JSAny, (
        x: 0,
        y: 0,
        width: width,
        height: height,
      ));
    }

    // Regression test for https://github.com/flutter/flutter/issues/75286
    test('updates canvas logical size when device-pixel ratio changes', () async {
      final RenderCanvas canvas = RenderCanvas();
      canvas.render(await newBitmap(10, 16));

      expect(canvas.canvasElement.width, 10);
      expect(canvas.canvasElement.height, 16);
      expect(canvas.canvasElement.style.width, '10px');
      expect(canvas.canvasElement.style.height, '16px');

      // Increase device-pixel ratio: this makes CSS pixels bigger, so we need
      // fewer of them to cover the browser window.
      EngineFlutterDisplay.instance.debugOverrideDevicePixelRatio(2.0);
      canvas.render(await newBitmap(10, 16));
      expect(canvas.canvasElement.width, 10);
      expect(canvas.canvasElement.height, 16);
      expect(canvas.canvasElement.style.width, '5px');
      expect(canvas.canvasElement.style.height, '8px');

      // Decrease device-pixel ratio: this makes CSS pixels smaller, so we need
      // more of them to cover the browser window.
      EngineFlutterDisplay.instance.debugOverrideDevicePixelRatio(0.5);
      canvas.render(await newBitmap(10, 16));
      expect(canvas.canvasElement.width, 10);
      expect(canvas.canvasElement.height, 16);
      expect(canvas.canvasElement.style.width, '20px');
      expect(canvas.canvasElement.style.height, '32px');
    });

    test('rounds physical size to nearest integer size', () async {
      final EngineFlutterWindow implicitView = EnginePlatformDispatcher.instance.implicitView!;
      implicitView.debugPhysicalSizeOverride = const ui.Size(199.999999, 200.000001);

      final ui.SceneBuilder sceneBuilder = LayerSceneBuilder();
      final CkPictureRecorder recorder = CkPictureRecorder();
      final CkCanvas canvas = recorder.beginRecording(ui.Rect.largest);
      canvas.drawPaint(CkPaint()..color = const ui.Color(0xff00ff00));
      final CkPicture picture = recorder.endRecording();
      sceneBuilder.addPicture(ui.Offset.zero, picture);
      final ui.Scene scene = sceneBuilder.build();

      await renderScene(scene);

      expect(
        CanvasKitRenderer.instance.debugGetRasterizerForView(implicitView)!.currentFrameSize,
        const BitmapSize(200, 200),
      );

      implicitView.debugPhysicalSizeOverride = null;
      implicitView.debugForceResize();
    });
  });
}
