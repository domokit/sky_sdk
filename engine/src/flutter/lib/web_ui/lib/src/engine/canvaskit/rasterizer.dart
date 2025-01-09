// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
import 'dart:async';

import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;

abstract class Rasterizer {
  /// Creates a [ViewRasterizer] for a given [view].
  ViewRasterizer createViewRasterizer(EngineFlutterView view);

  /// Sets the maximum size of the resource cache to [bytes].
  void setResourceCacheMaxBytes(int bytes);

  /// Disposes this rasterizer and all [ViewRasterizer]s that it created.
  void dispose();
}

abstract class ViewRasterizer {
  ViewRasterizer(this.view);

  /// The view this rasterizer renders into.
  final EngineFlutterView view;

  /// The queue of render requests for this view.
  final RenderQueue queue = RenderQueue();

  /// The size of the current frame being rasterized.
  BitmapSize currentFrameSize = BitmapSize.zero;

  /// The context which is persisted between frames.
  final CompositorContext context = CompositorContext();

  /// The platform view embedder.
  late final HtmlViewEmbedder viewEmbedder = HtmlViewEmbedder(sceneHost, this);

  /// A factory for creating overlays.
  DisplayCanvasFactory<DisplayCanvas> get displayFactory;

  /// The scene host which this rasterizer should raster into.
  DomElement get sceneHost => view.dom.sceneHost;

  /// Draws the [layerTree] to the screen for the view associated with this
  /// rasterizer.
  Future<void> draw(LayerTree layerTree) async {
    final ui.Size frameSize = view.physicalSize;
    if (frameSize.isEmpty) {
      // Available drawing area is empty. Skip drawing.
      return;
    }

    // The [frameSize] may be slightly imprecise if the `devicePixelRatio` isn't
    // an integer. For example, is you zoom to 110% in Chrome on a Macbook, the
    // `devicePixelRatio` is `2.200000047683716`, so when the physical size is
    // computed by multiplying the logical size by the device pixel ratio, the
    // result is slightly imprecise as well. Nevertheless, the number should
    // be close to an integer, so round the frame size to be more precice.
    final BitmapSize bitmapSize = BitmapSize.fromSize(frameSize);

    currentFrameSize = bitmapSize;
    prepareToDraw();
    viewEmbedder.frameSize = currentFrameSize;
    final Frame compositorFrame = context.acquireFrame(viewEmbedder);

    compositorFrame.raster(layerTree, currentFrameSize, ignoreRasterCache: true);

    await viewEmbedder.submitFrame();
  }

  /// Do some initialization to prepare to draw a frame.
  ///
  /// For example, in the [OffscreenCanvasRasterizer], this ensures the backing
  /// [OffscreenCanvas] is the correct size to draw the frame.
  void prepareToDraw();

  /// Rasterize the [pictures] to the given [canvas].
  Future<void> rasterizeToCanvas(DisplayCanvas canvas, List<CkPicture> pictures);

  /// Get a [DisplayCanvas] to use as an overlay.
  DisplayCanvas getOverlay() {
    return displayFactory.getCanvas();
  }

  /// Release the given [overlay] so it may be reused.
  void releaseOverlay(DisplayCanvas overlay) {
    displayFactory.releaseCanvas(overlay);
  }

  /// Release all overlays.
  void releaseOverlays() {
    displayFactory.releaseCanvases();
  }

  /// Remove all overlays that have been created from the DOM.
  void removeOverlaysFromDom() {
    displayFactory.removeCanvasesFromDom();
  }

  /// Disposes this rasterizer.
  void dispose() {
    viewEmbedder.dispose();
    displayFactory.dispose();
  }

  /// Clears the state. Used in tests.
  void debugClear() {
    viewEmbedder.debugClear();
  }
}

/// A [DisplayCanvas] is an abstraction for a canvas element which displays
/// Skia-drawn pictures to the screen. They are also sometimes called "overlays"
/// because they can be overlaid on top of platform views, which are HTML
/// content that isn't rendered by Skia.
///
/// [DisplayCanvas]es are drawn into with [ViewRasterizer.rasterizeToCanvas].
abstract class DisplayCanvas {
  /// The DOM element which, when appended to the scene host, will display the
  /// Skia-rendered content to the screen.
  DomElement get hostElement;

  /// Whether or not this overlay canvas is attached to the DOM.
  bool get isConnected;

  /// Initialize the overlay.
  void initialize();

  /// Disposes this overlay.
  void dispose();
}

/// Encapsulates a request to render a [ui.Scene]. Contains the scene to render
/// and a [Completer] which completes when the scene has been rendered.
typedef RenderRequest =
    ({ui.Scene scene, Completer<void> completer, FrameTimingRecorder? recorder});

/// A per-view queue of render requests. Only contains the current render
/// request and the next render request. If a new render request is made before
/// the current request is complete, then the next render request is replaced
/// with the most recently requested render and the other one is dropped.
class RenderQueue {
  RenderQueue();

  RenderRequest? current;
  RenderRequest? next;
}
