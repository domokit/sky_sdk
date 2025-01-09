// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:ui/ui.dart' as ui;

import '../../engine.dart' show BitmapSize, kProfileApplyFrame, kProfilePrerollFrame;
import '../profiler.dart';
import 'canvas.dart';
import 'embedded_views.dart';
import 'layer.dart';
import 'layer_visitor.dart';
import 'n_way_canvas.dart';
import 'picture_recorder.dart';
import 'raster_cache.dart';

/// A tree of [Layer]s that, together with a [Size] compose a frame.
class LayerTree {
  LayerTree(this.rootLayer);

  /// The root of the layer tree.
  final RootLayer rootLayer;

  /// The devicePixelRatio of the frame to paint this layer tree into.
  double? devicePixelRatio;

  /// Performs a preroll phase before painting the layer tree.
  ///
  /// In this phase, the paint boundary for each layer is computed and
  /// pictures are registered with the raster cache as potential candidates
  /// to raster. If [ignoreRasterCache] is `true`, then there will be no
  /// attempt to register pictures to cache.
  void preroll(Frame frame, {bool ignoreRasterCache = false}) {
    final PrerollVisitor prerollVisitor = PrerollVisitor(frame.viewEmbedder);
    rootLayer.accept(prerollVisitor);
  }

  /// Performs a paint pass with a recording canvas for each picture in the
  /// tree. This paint pass is just used to measure the bounds for each picture
  /// so we can optimize the total number of canvases required.
  void measure(Frame frame, BitmapSize size, {bool ignoreRasterCache = false}) {
    final MeasureVisitor measureVisitor = MeasureVisitor(size, frame.viewEmbedder!);
    if (rootLayer.needsPainting) {
      rootLayer.accept(measureVisitor);
    }
    measureVisitor.dispose();
  }

  /// Paints the layer tree into the given [frame].
  ///
  /// If [ignoreRasterCache] is `true`, then the raster cache will
  /// not be used.
  void paint(Frame frame, {bool ignoreRasterCache = false}) {
    final CkNWayCanvas internalNodesCanvas = CkNWayCanvas();
    final Iterable<CkCanvas> overlayCanvases = frame.viewEmbedder!.getOptimizedCanvases();
    overlayCanvases.forEach(internalNodesCanvas.addCanvas);
    final PaintVisitor paintVisitor = PaintVisitor(internalNodesCanvas, frame.viewEmbedder!);
    if (rootLayer.needsPainting) {
      rootLayer.accept(paintVisitor);
    }
  }

  /// Flattens the tree into a single [ui.Picture].
  ///
  /// This picture does not contain any platform views.
  ui.Picture flatten(ui.Size size) {
    final CkPictureRecorder recorder = CkPictureRecorder();
    final CkCanvas canvas = recorder.beginRecording(ui.Offset.zero & size);
    final PrerollVisitor prerollVisitor = PrerollVisitor(null);
    rootLayer.accept(prerollVisitor);

    final CkNWayCanvas internalNodesCanvas = CkNWayCanvas();
    internalNodesCanvas.addCanvas(canvas);
    final PaintVisitor paintVisitor = PaintVisitor.forToImage(internalNodesCanvas, canvas);
    if (rootLayer.needsPainting) {
      rootLayer.accept(paintVisitor);
    }
    return recorder.endRecording();
  }
}

/// A single frame to be rendered.
class Frame {
  Frame(this.rasterCache, this.viewEmbedder);

  /// A cache of pre-rastered pictures.
  final RasterCache? rasterCache;

  /// The platform view embedder.
  final HtmlViewEmbedder? viewEmbedder;

  /// Rasterize the given layer tree into this frame.
  bool raster(LayerTree layerTree, BitmapSize size, {bool ignoreRasterCache = false}) {
    timeAction<void>(kProfilePrerollFrame, () {
      layerTree.preroll(this, ignoreRasterCache: ignoreRasterCache);
      layerTree.measure(this, size, ignoreRasterCache: ignoreRasterCache);
      viewEmbedder?.optimizeRendering();
    });
    timeAction<void>(kProfileApplyFrame, () {
      layerTree.paint(this, ignoreRasterCache: ignoreRasterCache);
    });
    return true;
  }
}

/// The state of the compositor, which is persisted between frames.
class CompositorContext {
  /// A cache of pictures, which is shared between successive frames.
  RasterCache? rasterCache;

  /// Acquire a frame using this compositor's settings.
  Frame acquireFrame(HtmlViewEmbedder? viewEmbedder) {
    return Frame(rasterCache, viewEmbedder);
  }
}
