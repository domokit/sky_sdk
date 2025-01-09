// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;

class SurfaceScene implements ui.Scene {
  /// This class is created by the engine, and should not be instantiated
  /// or extended directly.
  ///
  /// To create a Scene object, use a [SceneBuilder].
  SurfaceScene(this.webOnlyRootElement, {required this.timingRecorder});

  final DomElement? webOnlyRootElement;
  final FrameTimingRecorder? timingRecorder;

  /// Creates a raster image representation of the current state of the scene.
  /// This is a slow operation that is performed on a background thread.
  @override
  Future<ui.Image> toImage(int width, int height) {
    throw UnsupportedError('toImage is not supported on the Web');
  }

  @override
  ui.Image toImageSync(int width, int height) {
    throw UnsupportedError('toImageSync is not supported on the Web');
  }

  /// Releases the resources used by this scene.
  ///
  /// After calling this function, the scene is cannot be used further.
  @override
  void dispose() {}
}

/// A surface that creates a DOM element for whole app.
class PersistedScene extends PersistedContainerSurface {
  PersistedScene(PersistedScene? super.oldLayer) {
    transform = Matrix4.identity();
  }

  @override
  void recomputeTransformAndClip() {
    // Must be the true DPR from the browser, nothing overridable.
    // See: https://github.com/flutter/flutter/issues/143124
    final double browserDpr = EngineFlutterDisplay.instance.browserDevicePixelRatio;
    // The scene clip is the size of the entire window **in Logical pixels**.
    //
    // Even though the majority of the engine uses `physicalSize`, there are some
    // bits (like the HTML renderer, or dynamic view sizing) that are implemented
    // using CSS, and CSS operates in logical pixels.
    //
    // See also: [EngineFlutterView.resize].
    final ui.Size bounds = window.physicalSize / browserDpr;
    localClipBounds = ui.Rect.fromLTRB(0, 0, bounds.width, bounds.height);
    projectedClip = null;
  }

  /// Cached inverse of transform on this node. Unlike transform, this
  /// Matrix only contains local transform (not chain multiplied since root).
  Matrix4? _localTransformInverse;

  @override
  Matrix4? get localTransformInverse => _localTransformInverse ??= Matrix4.identity();

  @override
  DomElement createElement() {
    return defaultCreateElement('flt-scene');
  }

  @override
  void apply() {}
}
