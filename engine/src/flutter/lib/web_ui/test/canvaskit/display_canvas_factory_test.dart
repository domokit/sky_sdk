// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';

import 'common.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

class DummyDisplayCanvas extends DisplayCanvas {
  @override
  void dispose() {}

  final DomElement _element = createDomElement('div');

  @override
  DomElement get hostElement => _element;

  @override
  void initialize() {}

  @override
  bool get isConnected => throw UnimplementedError();
}

void testMain() {
  group('$DisplayCanvasFactory', () {
    setUpCanvasKitTest(withImplicitView: true);

    test('getCanvas', () {
      final DisplayCanvasFactory<DisplayCanvas> factory = DisplayCanvasFactory<DisplayCanvas>(
        createCanvas: () => DummyDisplayCanvas(),
      );
      expect(factory.baseCanvas, isNotNull);

      expect(factory.debugSurfaceCount, equals(1));

      // Get a canvas from the factory, it should be unique.
      final DisplayCanvas newCanvas = factory.getCanvas();
      expect(newCanvas, isNot(equals(factory.baseCanvas)));

      expect(factory.debugSurfaceCount, equals(2));

      // Get another canvas from the factory. Now we are at maximum capacity.
      final DisplayCanvas anotherCanvas = factory.getCanvas();
      expect(anotherCanvas, isNot(equals(factory.baseCanvas)));

      expect(factory.debugSurfaceCount, equals(3));
    });

    test('releaseCanvas', () {
      final DisplayCanvasFactory<DisplayCanvas> factory = DisplayCanvasFactory<DisplayCanvas>(
        createCanvas: () => DummyDisplayCanvas(),
      );

      // Create a new canvas and immediately release it.
      final DisplayCanvas canvas = factory.getCanvas();
      factory.releaseCanvas(canvas);

      // If we create a new canvas, it should be the same as the one we
      // just created.
      final DisplayCanvas newCanvas = factory.getCanvas();
      expect(newCanvas, equals(canvas));
    });

    test('isLive', () {
      final DisplayCanvasFactory<DisplayCanvas> factory = DisplayCanvasFactory<DisplayCanvas>(
        createCanvas: () => DummyDisplayCanvas(),
      );

      expect(factory.isLive(factory.baseCanvas), isTrue);

      final DisplayCanvas canvas = factory.getCanvas();
      expect(factory.isLive(canvas), isTrue);

      factory.releaseCanvas(canvas);
      expect(factory.isLive(canvas), isFalse);
    });

    test('hot restart', () {
      void expectDisposed(DisplayCanvas canvas) {
        expect(canvas.isConnected, isFalse);
      }

      final EngineFlutterView implicitView = EnginePlatformDispatcher.instance.implicitView!;

      final DisplayCanvasFactory<DisplayCanvas> originalFactory =
          CanvasKitRenderer.instance.debugGetRasterizerForView(implicitView)!.displayFactory;

      // Cause the surface and its canvas to be attached to the page
      implicitView.dom.sceneHost.prepend(originalFactory.baseCanvas.hostElement);
      expect(originalFactory.baseCanvas.isConnected, isTrue);

      // Create a few overlay canvases
      final List<DisplayCanvas> overlays = <DisplayCanvas>[];
      for (int i = 0; i < 3; i++) {
        final DisplayCanvas canvas = originalFactory.getCanvas();
        implicitView.dom.sceneHost.prepend(canvas.hostElement);
        overlays.add(canvas);
      }
      expect(originalFactory.debugSurfaceCount, 4);

      // Trigger hot restart clean-up logic and check that we indeed clean up.
      debugEmulateHotRestart();
      expectDisposed(originalFactory.baseCanvas);
      overlays.forEach(expectDisposed);
      expect(originalFactory.debugSurfaceCount, 1);
    });
  });
}
