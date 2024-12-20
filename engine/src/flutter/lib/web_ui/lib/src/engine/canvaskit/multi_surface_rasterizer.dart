// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;

/// A Rasterizer which uses one or many on-screen WebGL contexts to display the
/// scene. This way of rendering is prone to bugs because there is a limit to
/// how many WebGL contexts can be live at one time as well as bugs in sharing
/// GL resources between the contexts. However, using [createImageBitmap] is
/// currently very slow on Firefox and Safari browsers, so directly rendering
/// to several [Surface]s is how we can achieve 60 fps on these browsers.
class MultiSurfaceRasterizer extends Rasterizer {
  @override
  MultiSurfaceViewRasterizer createViewRasterizer(EngineFlutterView view) {
    return _viewRasterizers.putIfAbsent(view, () => MultiSurfaceViewRasterizer(view, this));
  }

  final Map<EngineFlutterView, MultiSurfaceViewRasterizer> _viewRasterizers =
      <EngineFlutterView, MultiSurfaceViewRasterizer>{};

  @override
  void dispose() {
    for (final MultiSurfaceViewRasterizer viewRasterizer in _viewRasterizers.values) {
      viewRasterizer.dispose();
    }
    _viewRasterizers.clear();
  }

  @override
  void setResourceCacheMaxBytes(int bytes) {
    for (final MultiSurfaceViewRasterizer viewRasterizer in _viewRasterizers.values) {
      viewRasterizer.displayFactory.forEachCanvas((Surface surface) {
        surface.setSkiaResourceCacheMaxBytes(bytes);
      });
    }
  }
}

class MultiSurfaceViewRasterizer extends ViewRasterizer {
  MultiSurfaceViewRasterizer(super.view, this.rasterizer);

  final MultiSurfaceRasterizer rasterizer;

  @override
  final DisplayCanvasFactory<Surface> displayFactory = DisplayCanvasFactory<Surface>(
    createCanvas: () => Surface(isDisplayCanvas: true),
  );

  @override
  void prepareToDraw() {
    displayFactory.baseCanvas.createOrUpdateSurface(currentFrameSize);
  }

  @override
  Future<void> rasterizeToCanvas(DisplayCanvas canvas, List<CkPicture> pictures) {
    final Surface surface = canvas as Surface;
    surface.createOrUpdateSurface(currentFrameSize);
    surface.positionToShowFrame(currentFrameSize);
    final CkCanvas skCanvas = surface.getCanvas();
    skCanvas.clear(const ui.Color(0x00000000));
    pictures.forEach(skCanvas.drawPicture);
    surface.flush();
    return Future<void>.value();
  }
}
