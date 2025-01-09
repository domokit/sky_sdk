// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:math' as math;
import 'dart:typed_data';

import 'package:ui/ui.dart' as ui;

import '../dom.dart';
import '../engine_canvas.dart';
import '../frame_reference.dart';
import '../util.dart';
import '../vector_math.dart';
import 'bitmap_canvas.dart';
import 'debug_canvas_reuse_overlay.dart';
import 'dom_canvas.dart';
import 'image.dart';
import 'path/path_metrics.dart';
import 'recording_canvas.dart';
import 'surface.dart';
import 'surface_stats.dart';

class EnginePictureRecorder implements ui.PictureRecorder {
  EnginePictureRecorder();

  RecordingCanvas? _canvas;
  late ui.Rect cullRect;
  bool _isRecording = false;

  RecordingCanvas beginRecording(ui.Rect bounds) {
    assert(!_isRecording);
    cullRect = bounds;
    _isRecording = true;
    return _canvas = RecordingCanvas(cullRect);
  }

  @override
  bool get isRecording => _isRecording;

  @override
  EnginePicture endRecording() {
    if (!_isRecording) {
      // The mobile version returns an empty picture in this case. To match the
      // behavior we produce a blank picture too.
      beginRecording(ui.Rect.largest);
    }
    _isRecording = false;
    _canvas!.endRecording();
    final EnginePicture result = EnginePicture(_canvas, cullRect);
    // We invoke the handler here, not in the Picture constructor, because we want
    // [result.approximateBytesUsed] to be available for the handler.
    ui.Picture.onCreate?.call(result);
    return result;
  }
}

/// An implementation of [ui.Picture] which is backed by a [RecordingCanvas].
class EnginePicture implements ui.Picture {
  /// This class is created by the engine, and should not be instantiated
  /// or extended directly.
  ///
  /// To create a [Picture], use a [PictureRecorder].
  EnginePicture(this.recordingCanvas, this.cullRect);

  @override
  Future<ui.Image> toImage(int width, int height) async {
    final ui.Rect imageRect = ui.Rect.fromLTRB(0, 0, width.toDouble(), height.toDouble());
    final BitmapCanvas canvas = BitmapCanvas.imageData(imageRect);
    recordingCanvas!.apply(canvas, imageRect);
    final String imageDataUrl = canvas.toDataUrl();
    final DomHTMLImageElement imageElement =
        createDomHTMLImageElement()
          ..src = imageDataUrl
          ..width = width.toDouble()
          ..height = height.toDouble();

    // The image loads asynchronously. We need to wait before returning,
    // otherwise the returned HtmlImage will be temporarily unusable.
    final Completer<ui.Image> onImageLoaded = Completer<ui.Image>.sync();

    // Ignoring the returned futures from onError and onLoad because we're
    // communicating through the `onImageLoaded` completer.
    late final DomEventListener errorListener;
    errorListener = createDomEventListener((DomEvent event) {
      onImageLoaded.completeError(event);
      imageElement.removeEventListener('error', errorListener);
    });
    imageElement.addEventListener('error', errorListener);
    late final DomEventListener loadListener;
    loadListener = createDomEventListener((DomEvent event) {
      onImageLoaded.complete(HtmlImage(imageElement, width, height));
      imageElement.removeEventListener('load', loadListener);
    });
    imageElement.addEventListener('load', loadListener);
    return onImageLoaded.future;
  }

  @override
  ui.Image toImageSync(int width, int height) {
    throw UnsupportedError(
      'toImageSync is not supported on the HTML backend. Use drawPicture instead, or toImage.',
    );
  }

  bool _disposed = false;

  @override
  void dispose() {
    ui.Picture.onDispose?.call(this);
    _disposed = true;
  }

  @override
  bool get debugDisposed {
    bool? result;
    assert(() {
      result = _disposed;
      return true;
    }());

    if (result != null) {
      return result!;
    }

    throw StateError('Picture.debugDisposed is only available when asserts are enabled.');
  }

  @override
  int get approximateBytesUsed => 0;

  final RecordingCanvas? recordingCanvas;
  final ui.Rect? cullRect;
}

// TODO(yjbanov): this is currently very naive. We probably want to cache
//                fewer large canvases than small canvases. We could also
//                improve cache hit count if we did not require exact canvas
//                size match, but instead could choose a canvas that's big
//                enough. The optimal heuristic will need to be figured out.
//                For example, we probably don't want to pick a full-screen
//                canvas to draw a 10x10 picture. Let's revisit this after
//                Harry's layer merging refactor.
/// The maximum number canvases cached.
const int _kCanvasCacheSize = 30;

/// Canvases available for reuse, capped at [_kCanvasCacheSize].
List<BitmapCanvas> get recycledCanvases => _recycledCanvases;
final List<BitmapCanvas> _recycledCanvases = <BitmapCanvas>[];

/// Reduces recycled canvas list by 50% to reduce bitmap canvas memory use.
void reduceCanvasMemoryUsage() {
  final int canvasCount = _recycledCanvases.length;
  for (int i = 0; i < canvasCount; i++) {
    _recycledCanvases[i].dispose();
  }
  _recycledCanvases.clear();
}

/// A request to repaint a canvas.
///
/// Paint requests are prioritized such that the larger pictures go first. This
/// makes canvas allocation more efficient by letting large pictures claim
/// larger recycled canvases. Otherwise, small pictures would claim the large
/// canvases forcing us to allocate new large canvases.
class PaintRequest {
  PaintRequest({required this.canvasSize, required this.paintCallback});

  final ui.Size canvasSize;
  final ui.VoidCallback paintCallback;
}

/// Repaint requests produced by [PersistedPicture]s that actually paint on the
/// canvas. Painting is delayed until the layer tree is updated to maximize
/// the number of reusable canvases.
List<PaintRequest> paintQueue = <PaintRequest>[];

void _recycleCanvas(EngineCanvas? canvas) {
  // If a canvas is in the paint queue it maybe be recycled. To
  // prevent subsequent dispose recycling again check.
  if (canvas != null && _recycledCanvases.contains(canvas)) {
    return;
  }
  if (canvas is BitmapCanvas) {
    canvas.setElementCache(null);
    if (canvas.isReusable()) {
      _recycledCanvases.add(canvas);
      if (_recycledCanvases.length > _kCanvasCacheSize) {
        final BitmapCanvas removedCanvas = _recycledCanvases.removeAt(0);
        removedCanvas.dispose();
        if (debugShowCanvasReuseStats) {
          DebugCanvasReuseOverlay.instance.disposedCount++;
        }
      }
      if (debugShowCanvasReuseStats) {
        DebugCanvasReuseOverlay.instance.inRecycleCount = _recycledCanvases.length;
      }
    } else {
      canvas.dispose();
    }
  }
}

/// A surface that uses a combination of `<canvas>`, `<div>` and `<p>` elements
/// to draw shapes and text.
class PersistedPicture extends PersistedLeafSurface {
  PersistedPicture(this.dx, this.dy, this.picture, this.hints)
    : localPaintBounds = picture.recordingCanvas!.pictureBounds;

  EngineCanvas? _canvas;

  /// Returns the canvas used by this picture layer.
  EngineCanvas? get canvas => _canvas;

  final double dx;
  final double dy;
  final EnginePicture picture;
  final ui.Rect? localPaintBounds;
  final int hints;
  double _density = 1.0;

  /// Cull rect changes and density changes due to transforms should
  /// call applyPaint for picture when retain() or update() is called after
  /// preroll is complete.
  bool _requiresRepaint = false;

  /// Cache for reusing elements such as images across picture updates.
  CrossFrameCache<DomHTMLElement>? _elementCache = CrossFrameCache<DomHTMLElement>();

  @override
  DomElement createElement() {
    final DomElement element = defaultCreateElement('flt-picture');

    // The DOM elements used to render pictures are used purely to put pixels on
    // the screen. They have no semantic information. If an assistive technology
    // attempts to scan picture content it will look like garbage and confuse
    // users. UI semantics are exported as a separate DOM tree rendered parallel
    // to pictures.
    //
    // Why are layer and scene elements not hidden from ARIA? Because those
    // elements may contain platform views, and platform views must be
    // accessible.
    element.setAttribute('aria-hidden', 'true');

    return element;
  }

  @override
  void preroll(PrerollSurfaceContext prerollContext) {
    if (prerollContext.activeShaderMaskCount != 0 || prerollContext.activeColorFilterCount != 0) {
      picture.recordingCanvas?.renderStrategy.isInsideSvgFilterTree = true;
    }
    super.preroll(prerollContext);
  }

  @override
  void recomputeTransformAndClip() {
    transform = parent!.transform;
    if (dx != 0.0 || dy != 0.0) {
      transform = transform!.clone();
      transform!.translate(dx, dy);
    }
    final double paintWidth = localPaintBounds!.width;
    final double paintHeight = localPaintBounds!.height;
    final double newDensity =
        localPaintBounds == null || paintWidth == 0 || paintHeight == 0
            ? 1.0
            : _computePixelDensity(transform, paintWidth, paintHeight);
    if (newDensity != _density) {
      _density = newDensity;
      _requiresRepaint = true;
    }
    _computeExactCullRects();
  }

  /// The rectangle that contains all visible pixels drawn by [picture] inside
  /// the current layer hierarchy in local coordinates.
  ///
  /// This value is a conservative estimate, i.e. it must be big enough to
  /// contain everything that's visible, but it may be bigger than necessary.
  /// Therefore it should not be used for clipping. It is meant to be used for
  /// optimizing canvas allocation.
  ui.Rect? get optimalLocalCullRect => _optimalLocalCullRect;
  ui.Rect? _optimalLocalCullRect;

  /// Same as [optimalLocalCullRect] but in screen coordinate system.
  ui.Rect? get debugExactGlobalCullRect => _exactGlobalCullRect;
  ui.Rect? _exactGlobalCullRect;

  ui.Rect? _exactLocalCullRect;

  /// Computes the canvas paint bounds based on the estimated paint bounds and
  /// the scaling produced by transformations.
  ///
  /// Return `true` if the local cull rect changed, indicating that a repaint
  /// may be required. Returns `false` otherwise. Global cull rect changes do
  /// not necessarily incur repaints. For example, if the layer sub-tree was
  /// translated from one frame to another we may not need to repaint, just
  /// translate the canvas.
  void _computeExactCullRects() {
    assert(transform != null);
    assert(localPaintBounds != null);

    if (parent!.projectedClip == null) {
      // Compute and cache chain of clipping bounds on parent of picture since
      // parent may include multiple pictures so it can be reused by all
      // child pictures.
      ui.Rect? bounds;
      PersistedSurface? parentSurface = parent;
      final Matrix4 clipTransform = Matrix4.identity();
      while (parentSurface != null) {
        final ui.Rect? localClipBounds = parentSurface.localClipBounds;
        if (localClipBounds != null) {
          if (bounds == null) {
            bounds = clipTransform.transformRect(localClipBounds);
          } else {
            bounds = bounds.intersect(clipTransform.transformRect(localClipBounds));
          }
        }
        final Matrix4? localInverse = parentSurface.localTransformInverse;
        if (localInverse != null && !localInverse.isIdentity()) {
          clipTransform.multiply(localInverse);
        }
        parentSurface = parentSurface.parent;
      }
      if (bounds != null && (bounds.width <= 0 || bounds.height <= 0)) {
        bounds = ui.Rect.zero;
      }
      // Cache projected clip on parent.
      parent!.projectedClip = bounds;
    }
    // Intersect localPaintBounds with parent projected clip to calculate
    // and cache [_exactLocalCullRect].
    if (parent!.projectedClip == null) {
      _exactLocalCullRect = localPaintBounds;
    } else {
      _exactLocalCullRect = localPaintBounds!.intersect(parent!.projectedClip!);
    }
    if (_exactLocalCullRect!.width <= 0 || _exactLocalCullRect!.height <= 0) {
      _exactLocalCullRect = ui.Rect.zero;
      _exactGlobalCullRect = ui.Rect.zero;
    } else {
      assert(() {
        _exactGlobalCullRect = transform!.transformRect(_exactLocalCullRect!);
        return true;
      }());
    }
  }

  void _computeOptimalCullRect(PersistedPicture? oldSurface) {
    assert(_exactLocalCullRect != null);

    if (oldSurface == null || !oldSurface.picture.recordingCanvas!.didDraw) {
      // First useful paint.
      _optimalLocalCullRect = _exactLocalCullRect;
      _requiresRepaint = true;
      return;
    }

    assert(oldSurface._optimalLocalCullRect != null);

    final bool surfaceBeingRetained = identical(oldSurface, this);
    final ui.Rect? oldOptimalLocalCullRect =
        surfaceBeingRetained ? _optimalLocalCullRect : oldSurface._optimalLocalCullRect;

    if (_exactLocalCullRect == ui.Rect.zero) {
      // The clip collapsed into a zero-sized rectangle. If it was already zero,
      // no need to signal cull rect change.
      _optimalLocalCullRect = ui.Rect.zero;
      if (oldOptimalLocalCullRect != ui.Rect.zero) {
        _requiresRepaint = true;
      }
      return;
    }

    if (rectContainsOther(oldOptimalLocalCullRect!, _exactLocalCullRect!)) {
      // The cull rect we computed in the past contains the newly computed cull
      // rect. This can happen, for example, when the picture is being shrunk by
      // a clip when it is scrolled out of the screen. In this case we do not
      // repaint the picture. We just let it be shrunk by the outer clip.
      _optimalLocalCullRect = oldOptimalLocalCullRect;
      return;
    }

    // The new cull rect contains area not covered by a previous rect. Perhaps
    // the clip is growing, moving around the picture, or both. In this case
    // a part of the picture may not have been painted. We will need to
    // request a new canvas and paint the picture on it. However, this is also
    // a strong signal that the clip will continue growing as typically
    // Flutter uses animated transitions. So instead of allocating the canvas
    // the size of the currently visible area, we try to allocate a canvas of
    // a bigger size. This will prevent any further repaints as future frames
    // will hit the above case where the new cull rect is fully contained
    // within the cull rect we compute now.

    // Compute the delta, by which each of the side of the clip rect has "moved"
    // since the last time we updated the cull rect.
    final double leftwardDelta = oldOptimalLocalCullRect.left - _exactLocalCullRect!.left;
    final double upwardDelta = oldOptimalLocalCullRect.top - _exactLocalCullRect!.top;
    final double rightwardDelta = _exactLocalCullRect!.right - oldOptimalLocalCullRect.right;
    final double bottomwardDelta = _exactLocalCullRect!.bottom - oldOptimalLocalCullRect.bottom;

    // Compute the new optimal rect to paint into.
    final ui.Rect newLocalCullRect = ui.Rect.fromLTRB(
      _exactLocalCullRect!.left - _predictTrend(leftwardDelta, _exactLocalCullRect!.width),
      _exactLocalCullRect!.top - _predictTrend(upwardDelta, _exactLocalCullRect!.height),
      _exactLocalCullRect!.right + _predictTrend(rightwardDelta, _exactLocalCullRect!.width),
      _exactLocalCullRect!.bottom + _predictTrend(bottomwardDelta, _exactLocalCullRect!.height),
    ).intersect(localPaintBounds!);

    _requiresRepaint = _optimalLocalCullRect != newLocalCullRect;
    _optimalLocalCullRect = newLocalCullRect;
  }

  /// Predicts the delta a particular side of a clip rect will move given the
  /// [delta] it moved by last, and the respective [extent] (width or height)
  /// of the clip.
  static double _predictTrend(double delta, double extent) {
    if (delta <= 0.0) {
      // Shrinking. Give it 10% of the extent in case the trend is reversed.
      return extent * 0.1;
    } else {
      // Growing. Predict 10 more frames of similar deltas. Give it at least
      // 50% of the extent (protect from extremely slow growth trend such as
      // slow scrolling). Give no more than the full extent (protects from
      // fast scrolling that could lead to overallocation).
      return math.min(math.max(extent * 0.5, delta * 10.0), extent);
    }
  }

  /// Number of bitmap pixel painted by this picture.
  ///
  /// If the implementation does not paint onto a bitmap canvas, it should
  /// return zero.
  int get bitmapPixelCount {
    if (_canvas is! BitmapCanvas) {
      return 0;
    }

    final BitmapCanvas bitmapCanvas = _canvas! as BitmapCanvas;
    return bitmapCanvas.bitmapPixelCount;
  }

  void _applyPaint(PersistedPicture? oldSurface) {
    final EngineCanvas? oldCanvas = oldSurface?._canvas;
    _requiresRepaint = false;
    if (!picture.recordingCanvas!.didDraw || _optimalLocalCullRect!.isEmpty) {
      // The picture is empty, or it has been completely clipped out. Skip
      // painting. This removes all the setup work and scaffolding objects
      // that won't be useful for anything anyway.
      _recycleCanvas(oldCanvas);
      if (oldSurface != null) {
        // Make sure it doesn't get reused/recycled again.
        oldSurface._canvas = null;
      }
      if (rootElement != null) {
        removeAllChildren(rootElement!);
      }
      if (_canvas != null && _canvas != oldCanvas) {
        _recycleCanvas(_canvas);
      }
      _canvas = null;
      return;
    }

    if (debugExplainSurfaceStats) {
      surfaceStatsFor(this).paintCount++;
    }

    assert(_optimalLocalCullRect != null);
    applyPaint(oldCanvas);
  }

  @override
  double matchForUpdate(PersistedPicture existingSurface) {
    if (existingSurface.picture == picture) {
      // Picture is the same, return perfect score.
      return 0.0;
    }

    if (!existingSurface.picture.recordingCanvas!.didDraw) {
      // The previous surface didn't draw anything and therefore has no
      // resources to reuse.
      return 1.0;
    }

    final bool didRequireBitmap =
        existingSurface.picture.recordingCanvas!.renderStrategy.hasArbitraryPaint;
    final bool requiresBitmap = picture.recordingCanvas!.renderStrategy.hasArbitraryPaint;
    if (didRequireBitmap != requiresBitmap) {
      // Switching canvas types is always expensive.
      return 1.0;
    } else if (!requiresBitmap) {
      // Currently DomCanvas is always expensive to repaint, as we always throw
      // out all the DOM we rendered before. This may change in the future, at
      // which point we may return other values here.
      return 1.0;
    } else {
      final BitmapCanvas? oldCanvas = existingSurface._canvas as BitmapCanvas?;
      if (oldCanvas == null) {
        // We did not allocate a canvas last time. This can happen when the
        // picture is completely clipped out of the view.
        return 1.0;
      } else if (!oldCanvas.doesFitBounds(_exactLocalCullRect!, _density)) {
        // The canvas needs to be resized before painting.
        return 1.0;
      } else {
        final int newPixelCount =
            BitmapCanvas.widthToPhysical(_exactLocalCullRect!.width) *
            BitmapCanvas.heightToPhysical(_exactLocalCullRect!.height);
        final int oldPixelCount = oldCanvas.widthInBitmapPixels * oldCanvas.heightInBitmapPixels;

        if (oldPixelCount == 0) {
          return 1.0;
        }

        final double pixelCountRatio = newPixelCount / oldPixelCount;
        assert(
          0 <= pixelCountRatio && pixelCountRatio <= 1.0,
          'Invalid pixel count ratio $pixelCountRatio',
        );
        return 1.0 - pixelCountRatio;
      }
    }
  }

  void applyPaint(EngineCanvas? oldCanvas) {
    if (picture.recordingCanvas!.renderStrategy.hasArbitraryPaint) {
      _applyBitmapPaint(oldCanvas);
    } else {
      _applyDomPaint(oldCanvas);
    }
  }

  void _applyDomPaint(EngineCanvas? oldCanvas) {
    _recycleCanvas(_canvas);
    final DomCanvas domCanvas = DomCanvas(rootElement!);
    _canvas = domCanvas;
    removeAllChildren(rootElement!);
    picture.recordingCanvas!.apply(domCanvas, _optimalLocalCullRect!);
  }

  void _applyBitmapPaint(EngineCanvas? oldCanvas) {
    if (oldCanvas is BitmapCanvas &&
        oldCanvas.doesFitBounds(_optimalLocalCullRect!, _density) &&
        oldCanvas.isReusable()) {
      final BitmapCanvas reusedCanvas = oldCanvas;
      if (debugShowCanvasReuseStats) {
        DebugCanvasReuseOverlay.instance.keptCount++;
      }
      // Re-use old bitmap canvas.
      reusedCanvas.bounds = _optimalLocalCullRect!;
      _canvas = reusedCanvas;
      reusedCanvas.setElementCache(_elementCache);
      reusedCanvas.clear();
      picture.recordingCanvas!.apply(reusedCanvas, _optimalLocalCullRect!);
    } else {
      // We can't use the old canvas because the size has changed, so we put
      // it in a cache for later reuse.
      _recycleCanvas(oldCanvas);
      if (_canvas is BitmapCanvas) {
        (_canvas! as BitmapCanvas).setElementCache(null);
      }
      _canvas = null;
      // We cannot paint immediately because not all canvases that we may be
      // able to reuse have been released yet. So instead we enqueue this
      // picture to be painted after the update cycle is done syncing the layer
      // tree then reuse canvases that were freed up.
      paintQueue.add(
        PaintRequest(
          canvasSize: _optimalLocalCullRect!.size,
          paintCallback: () {
            final BitmapCanvas bitmapCanvas = _findOrCreateCanvas(_optimalLocalCullRect!);
            _canvas = bitmapCanvas;
            bitmapCanvas.setElementCache(_elementCache);
            if (debugExplainSurfaceStats) {
              surfaceStatsFor(this).paintPixelCount += bitmapCanvas.bitmapPixelCount;
            }
            removeAllChildren(rootElement!);
            rootElement!.append(bitmapCanvas.rootElement);
            bitmapCanvas.clear();
            picture.recordingCanvas!.apply(bitmapCanvas, _optimalLocalCullRect!);
          },
        ),
      );
    }
  }

  /// Attempts to reuse a canvas from the [_recycledCanvases]. Allocates a new
  /// one if unable to reuse.
  ///
  /// The best recycled canvas is one that:
  ///
  /// - Fits the requested [canvasSize]. This is a hard requirement. Otherwise
  ///   we risk clipping the picture.
  /// - Is the smallest among all possible reusable canvases. This makes canvas
  ///   reuse more efficient.
  /// - Contains no more than twice the number of requested pixels. This makes
  ///   sure we do not use too much memory for small canvases.
  BitmapCanvas _findOrCreateCanvas(ui.Rect bounds) {
    final ui.Size canvasSize = bounds.size;
    BitmapCanvas? bestRecycledCanvas;
    double lastPixelCount = double.infinity;
    for (int i = 0; i < _recycledCanvases.length; i++) {
      final BitmapCanvas candidate = _recycledCanvases[i];
      if (!candidate.isReusable()) {
        continue;
      }

      final ui.Size candidateSize = candidate.size;
      final double candidatePixelCount = candidateSize.width * candidateSize.height;

      final bool fits = candidate.doesFitBounds(bounds, _density);
      final bool isSmaller = candidatePixelCount < lastPixelCount;
      if (fits && isSmaller) {
        // [isTooSmall] is used to make sure that a small picture doesn't
        // reuse and hold onto memory of a large canvas.
        final double requestedPixelCount = bounds.width * bounds.height;
        final bool isTooSmall =
            isSmaller && requestedPixelCount > 1 && (candidatePixelCount / requestedPixelCount) > 4;
        if (!isTooSmall) {
          bestRecycledCanvas = candidate;
          lastPixelCount = candidatePixelCount;
          final bool fitsExactly =
              candidateSize.width == canvasSize.width && candidateSize.height == canvasSize.height;
          if (fitsExactly) {
            // No need to keep looking any more.
            break;
          }
        }
      }
    }

    if (bestRecycledCanvas != null) {
      if (debugExplainSurfaceStats) {
        surfaceStatsFor(this).reuseCanvasCount++;
      }
      _recycledCanvases.remove(bestRecycledCanvas);
      if (debugShowCanvasReuseStats) {
        DebugCanvasReuseOverlay.instance.inRecycleCount = _recycledCanvases.length;
      }
      if (debugShowCanvasReuseStats) {
        DebugCanvasReuseOverlay.instance.reusedCount++;
      }
      bestRecycledCanvas.bounds = bounds;
      bestRecycledCanvas.setElementCache(_elementCache);
      return bestRecycledCanvas;
    }

    if (debugShowCanvasReuseStats) {
      DebugCanvasReuseOverlay.instance.createdCount++;
    }
    final BitmapCanvas canvas = BitmapCanvas(
      bounds,
      picture.recordingCanvas!.renderStrategy,
      density: _density,
    );
    canvas.setElementCache(_elementCache);
    if (debugExplainSurfaceStats) {
      surfaceStatsFor(this)
        ..allocateBitmapCanvasCount += 1
        ..allocatedBitmapSizeInPixels = canvas.widthInBitmapPixels * canvas.heightInBitmapPixels;
    }
    return canvas;
  }

  void _applyTranslate() {
    rootElement!.style.transform = 'translate(${dx}px, ${dy}px)';
  }

  @override
  void apply() {
    _applyTranslate();
    _applyPaint(null);
  }

  @override
  void build() {
    _computeOptimalCullRect(null);
    _requiresRepaint = true;
    super.build();
  }

  @override
  void update(PersistedPicture oldSurface) {
    super.update(oldSurface);
    // Transfer element cache over.
    _elementCache = oldSurface._elementCache;
    if (oldSurface != this) {
      oldSurface._elementCache = null;
    }

    if (dx != oldSurface.dx || dy != oldSurface.dy) {
      _applyTranslate();
    }

    _computeOptimalCullRect(oldSurface);
    if (identical(picture, oldSurface.picture)) {
      final bool densityChanged =
          _canvas is BitmapCanvas && _density != (_canvas! as BitmapCanvas).density;

      // The picture is the same. Attempt to avoid repaint.
      if (_requiresRepaint || densityChanged) {
        // Cull rect changed such that a repaint is still necessary.
        _applyPaint(oldSurface);
      } else {
        // Cull rect did not change, or changed such in a way that does not
        // require a repaint (e.g. it shrunk).
        _canvas = oldSurface._canvas;
      }
    } else {
      // We have a new picture. Repaint.
      _applyPaint(oldSurface);
    }
  }

  @override
  void retain() {
    super.retain();
    _computeOptimalCullRect(this);
    if (_requiresRepaint) {
      _applyPaint(this);
    }
  }

  @override
  void discard() {
    _recycleCanvas(_canvas);
    _canvas = null;
    super.discard();
  }

  @override
  void debugPrintChildren(StringBuffer buffer, int indent) {
    super.debugPrintChildren(buffer, indent);
    if (rootElement != null && rootElement!.firstChild != null) {
      final DomElement firstChild = rootElement!.firstChild! as DomElement;
      final String canvasTag = firstChild.tagName.toLowerCase();
      final int canvasHash = firstChild.hashCode;
      buffer.writeln('${'  ' * (indent + 1)}<$canvasTag @$canvasHash />');
    } else if (rootElement != null) {
      buffer.writeln('${'  ' * (indent + 1)}<${rootElement!.tagName.toLowerCase()} @$hashCode />');
    } else {
      buffer.writeln('${'  ' * (indent + 1)}<recycled-canvas />');
    }
  }

  @override
  void debugValidate(List<String> validationErrors) {
    super.debugValidate(validationErrors);

    if (picture.recordingCanvas!.didDraw) {
      if (!_optimalLocalCullRect!.isEmpty && canvas == null) {
        validationErrors.add('$runtimeType has non-trivial picture but it has null canvas');
      }
      if (_optimalLocalCullRect == null) {
        validationErrors.add('$runtimeType has null _optimalLocalCullRect');
      }
      if (_exactGlobalCullRect == null) {
        validationErrors.add('$runtimeType has null _exactGlobalCullRect');
      }
      if (_exactLocalCullRect == null) {
        validationErrors.add('$runtimeType has null _exactLocalCullRect');
      }
    }
  }
}

/// Given size of a rectangle and transform, computes pixel density
/// (scale factor).
double _computePixelDensity(Matrix4? transform, double width, double height) {
  if (transform == null || transform.isIdentityOrTranslation()) {
    return 1.0;
  }
  final Float32List m = transform.storage;
  // Apply perspective transform to all 4 corners. Can't use left,top, bottom,
  // right since for example rotating 45 degrees would yield inaccurate size.
  double minX = m[12] * m[15];
  double minY = m[13] * m[15];
  double maxX = minX;
  double maxY = minY;
  double x = width;
  double y = height;
  double wp = 1.0 / ((m[3] * x) + (m[7] * y) + m[15]);
  double xp = ((m[0] * x) + (m[4] * y) + m[12]) * wp;
  double yp = ((m[1] * x) + (m[5] * y) + m[13]) * wp;
  minX = math.min(minX, xp);
  maxX = math.max(maxX, xp);
  minY = math.min(minY, yp);
  maxY = math.max(maxY, yp);
  x = 0;
  wp = 1.0 / ((m[3] * x) + (m[7] * y) + m[15]);
  xp = ((m[0] * x) + (m[4] * y) + m[12]) * wp;
  yp = ((m[1] * x) + (m[5] * y) + m[13]) * wp;
  minX = math.min(minX, xp);
  maxX = math.max(maxX, xp);
  minY = math.min(minY, yp);
  maxY = math.max(maxY, yp);
  x = width;
  y = 0;
  wp = 1.0 / ((m[3] * x) + (m[7] * y) + m[15]);
  xp = ((m[0] * x) + (m[4] * y) + m[12]) * wp;
  yp = ((m[1] * x) + (m[5] * y) + m[13]) * wp;
  minX = math.min(minX, xp);
  maxX = math.max(maxX, xp);
  minY = math.min(minY, yp);
  maxY = math.max(maxY, yp);
  final double scaleX = (maxX - minX) / width;
  final double scaleY = (maxY - minY) / height;
  double scale = math.min(scaleX, scaleY);
  // kEpsilon guards against divide by zero below.
  if (scale < kEpsilon || scale == 1) {
    // Handle local paint bounds scaled to 0, typical when using
    // transform animations and nothing is drawn.
    return 1.0;
  }
  if (scale > 1) {
    // Normalize scale to multiples of 2: 1x, 2x, 4x, 6x, 8x.
    // This is to prevent frequent rescaling of canvas during animations.
    //
    // On a fullscreen high dpi device dpi*density*resolution will demand
    // too much memory, so clamp at 4.
    scale = math.min(4.0, (scale / 2.0).ceil() * 2.0);
    // Guard against webkit absolute limit.
    const double kPixelLimit = 1024 * 1024 * 4;
    if ((width * height * scale * scale) > kPixelLimit && scale > 2) {
      scale = (kPixelLimit * 0.8) / (width * height);
    }
  } else {
    scale = math.max(2.0 / (2.0 / scale).floor(), 0.0001);
  }
  return scale;
}
