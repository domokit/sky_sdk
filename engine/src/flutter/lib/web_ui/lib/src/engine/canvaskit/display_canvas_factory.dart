// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
import 'package:meta/meta.dart';

import '../../engine.dart';

/// Caches canvases used to display Skia-drawn content.
class DisplayCanvasFactory<T extends DisplayCanvas> {
  DisplayCanvasFactory({required this.createCanvas}) {
    assert(() {
      registerHotRestartListener(dispose);
      return true;
    }());
  }

  /// A function which is passed in as a constructor parameter which is used to
  /// create new display canvases.
  final T Function() createCanvas;

  /// The base canvas to paint on. This is the default canvas which will be
  /// painted to. If there are no platform views, then this canvas will render
  /// the entire scene.
  late final T baseCanvas = createCanvas()..initialize();

  /// Canvases created by this factory which are currently in use.
  final List<T> _liveCanvases = <T>[];

  /// Canvases created by this factory which are no longer in use. These can be
  /// reused.
  final List<T> _cache = <T>[];

  /// The number of canvases which have been created by this factory.
  int get _canvasCount => _liveCanvases.length + _cache.length + 1;

  /// The number of surfaces created by this factory. Used for testing.
  @visibleForTesting
  int get debugSurfaceCount => _canvasCount;

  /// Returns the number of cached surfaces.
  ///
  /// Useful in tests.
  int get debugCacheSize => _cache.length;

  /// Gets a display canvas from the cache or creates a new one if there are
  /// none in the cache.
  T getCanvas() {
    if (_cache.isNotEmpty) {
      final T canvas = _cache.removeLast();
      _liveCanvases.add(canvas);
      return canvas;
    } else {
      final T canvas = createCanvas();
      canvas.initialize();
      _liveCanvases.add(canvas);
      return canvas;
    }
  }

  /// Releases all surfaces so they can be reused in the next frame.
  ///
  /// If a released surface is in the DOM, it is not removed. This allows the
  /// engine to release the surfaces at the end of the frame so they are ready
  /// to be used in the next frame, but still used for painting in the current
  /// frame.
  void releaseCanvases() {
    _cache.addAll(_liveCanvases);
    _liveCanvases.clear();
  }

  /// Removes all canvases except the base canvas from the DOM.
  ///
  /// This is called at the beginning of the frame to prepare for painting into
  /// the new canvases.
  void removeCanvasesFromDom() {
    _cache.forEach(_removeFromDom);
    _liveCanvases.forEach(_removeFromDom);
  }

  /// Calls [callback] on each canvas created by this factory.
  void forEachCanvas(void Function(T canvas) callback) {
    callback(baseCanvas);
    _cache.forEach(callback);
    _liveCanvases.forEach(callback);
  }

  // Removes [canvas] from the DOM.
  void _removeFromDom(T canvas) {
    canvas.hostElement.remove();
  }

  /// Signals that a canvas is no longer being used. It can be reused.
  void releaseCanvas(T canvas) {
    assert(canvas != baseCanvas, 'Attempting to release the base canvas');
    assert(
      _liveCanvases.contains(canvas),
      'Attempting to release a Canvas which '
      'was not created by this factory',
    );
    canvas.hostElement.remove();
    _liveCanvases.remove(canvas);
    _cache.add(canvas);
  }

  /// Returns [true] if [canvas] is currently being used to paint content.
  ///
  /// The base canvas always counts as live.
  ///
  /// If a canvas is not live, then it must be in the cache and ready to be
  /// reused.
  bool isLive(T canvas) {
    if (canvas == baseCanvas || _liveCanvases.contains(canvas)) {
      return true;
    }
    assert(_cache.contains(canvas));
    return false;
  }

  /// Dispose all canvases created by this factory.
  void dispose() {
    for (final T canvas in _cache) {
      canvas.dispose();
    }
    for (final T canvas in _liveCanvases) {
      canvas.dispose();
    }
    baseCanvas.dispose();
    _liveCanvases.clear();
    _cache.clear();
  }
}
