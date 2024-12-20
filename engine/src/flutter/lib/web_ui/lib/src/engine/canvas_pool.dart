// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;
import 'dart:typed_data';

import 'package:meta/meta.dart';
import 'package:ui/ui.dart' as ui;
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

import 'display.dart';
import 'dom.dart';
import 'engine_canvas.dart';
import 'html/bitmap_canvas.dart';
import 'html/painting.dart';
import 'html/path/conic.dart';
import 'html/path/path.dart';
import 'html/path/path_ref.dart';
import 'html/path/path_utils.dart';
import 'html/picture.dart';
import 'html/shaders/image_shader.dart';
import 'html/shaders/shader.dart';
import 'rrect_renderer.dart';
import 'safe_browser_api.dart';
import 'shadow.dart';
import 'util.dart';
import 'vector_math.dart';

/// Renders picture to a CanvasElement by allocating and caching 0 or more
/// canvas(s) for [BitmapCanvas].
///
/// [BitmapCanvas] signals allocation of first canvas using allocateCanvas.
/// When a painting command such as drawImage or drawParagraph requires
/// multiple canvases for correct compositing, it calls [closeCurrentCanvas]
/// and adds the canvas(s) to [_activeCanvasList].
///
/// To make sure transformations and clips are preserved correctly when a new
/// canvas is allocated, [CanvasPool] replays the current stack on the newly
/// allocated canvas. It also maintains a [_saveContextCount] so that
/// the context stack can be reinitialized to default when reused in the future.
///
/// On a subsequent repaint, when a Picture determines that a [BitmapCanvas]
/// can be reused, [CanvasPool] will move canvas(s) from pool to reusablePool
/// to prevent reallocation.
class CanvasPool extends _SaveStackTracking {
  /// Initializes canvas pool for target size and dpi.
  CanvasPool(this._widthInBitmapPixels, this._heightInBitmapPixels, this._density);

  DomCanvasRenderingContext2D? _context;
  ContextStateHandle? _contextHandle;
  final int _widthInBitmapPixels, _heightInBitmapPixels;
  // List of canvases that have been allocated and used in this paint cycle.
  List<DomCanvasElement>? _activeCanvasList;
  // List of canvases available to reuse from prior paint cycle.
  List<DomCanvasElement>? _reusablePool;
  // Current canvas element or null if marked for lazy allocation.
  DomCanvasElement? _canvas;

  DomHTMLElement? _rootElement;
  int _saveContextCount = 0;
  final double _density;

  /// Initializes canvas pool to be hosted on a surface.
  void mount(DomHTMLElement rootElement) {
    _rootElement = rootElement;
  }

  /// Sets the translate transform to be applied to canvas to compensate for
  /// pixel padding applied to hosting [BitmapCanvas].
  ///
  /// Should be called during initialization after [CanvasPool] is mounted.
  set initialTransform(ui.Offset transform) {
    translate(transform.dx, transform.dy);
  }

  /// Returns [CanvasRenderingContext2D] api to draw into this canvas.
  DomCanvasRenderingContext2D get context {
    DomCanvasRenderingContext2D? ctx = _context;
    if (ctx == null) {
      _createCanvas();
      ctx = _context;
      assert(_context != null);
      assert(_canvas != null);
    }
    return ctx!;
  }

  /// Returns [ContextStateHandle] API to efficiently update state of
  /// drawing context.
  ContextStateHandle get contextHandle {
    if (_canvas == null) {
      _createCanvas();
      assert(_context != null);
      assert(_canvas != null);
    }
    return _contextHandle!;
  }

  /// Returns true if a canvas is currently available for drawing.
  ///
  /// Calling [contextHandle] or, transitively, any of the `draw*` methods while
  /// this returns true will reuse the existing canvas. Otherwise, a new canvas
  /// will be allocated.
  ///
  /// Previously allocated and closed canvases (see [closeCanvas]) are not
  /// considered by this getter.
  bool get hasCanvas => _canvas != null;

  /// Stops the currently available canvas from receiving any further drawing
  /// commands.
  ///
  /// After calling this method, a subsequent call to [contextHandle] or,
  /// transitively, any of the `draw*` methods will cause a new canvas to be
  /// allocated.
  ///
  /// The closed canvas becomes an "active" canvas, that is a canvas that's used
  /// to render picture content in the current frame. Active canvases may be
  /// reused in other pictures if their contents are no longer needed for this
  /// picture.
  void closeCanvas() {
    assert(_rootElement != null);
    // Place clean copy of current canvas with context stack restored and paint
    // reset into pool.
    if (_canvas != null) {
      _restoreContextSave();
      _contextHandle!.reset();
      _activeCanvasList ??= <DomCanvasElement>[];
      _activeCanvasList!.add(_canvas!);
      _canvas = null;
      _context = null;
      _contextHandle = null;
    }
  }

  void _createCanvas() {
    bool requiresClearRect = false;
    bool reused = false;
    DomCanvasElement? canvas;
    if (_canvas != null) {
      _canvas!.width = 0;
      _canvas!.height = 0;
      _canvas = null;
    }
    if (_reusablePool != null && _reusablePool!.isNotEmpty) {
      canvas = _canvas = _reusablePool!.removeAt(0);
      requiresClearRect = true;
      reused = true;
    } else {
      // Compute the final CSS canvas size given the actual pixel count we
      // allocated. This is done for the following reasons:
      //
      // * To satisfy the invariant: pixel size = css size * device pixel ratio.
      // * To make sure that when we scale the canvas by devicePixelRatio (see
      //   _initializeViewport below) the pixels line up.
      final double cssWidth =
          _widthInBitmapPixels / EngineFlutterDisplay.instance.browserDevicePixelRatio;
      final double cssHeight =
          _heightInBitmapPixels / EngineFlutterDisplay.instance.browserDevicePixelRatio;
      canvas = _allocCanvas(_widthInBitmapPixels, _heightInBitmapPixels);
      _canvas = canvas;

      // Why is this null check here, even though we just allocated a canvas element above?
      //
      // On iOS Safari, if you alloate too many canvases, the browser will stop allocating them
      // and return null instead. If that happens, we evict canvases from the cache, giving the
      // browser more memory to allocate a new canvas.
      if (_canvas == null) {
        // Evict BitmapCanvas(s) and retry.
        reduceCanvasMemoryUsage();
        canvas = _allocCanvas(_widthInBitmapPixels, _heightInBitmapPixels);
      }
      canvas!.style
        ..position = 'absolute'
        ..width = '${cssWidth}px'
        ..height = '${cssHeight}px';
    }

    // Before appending canvas, check if canvas is already on rootElement. This
    // optimization prevents DOM .append call when a PersistentSurface is
    // reused. Reading lastChild is faster than append call.
    if (_rootElement!.lastChild != canvas) {
      _rootElement!.append(canvas);
    }

    try {
      if (reused) {
        // If a canvas is the first element we set z-index = -1 in [BitmapCanvas]
        // endOfPaint to workaround blink compositing bug. To make sure this
        // does not leak when reused reset z-index.
        canvas.style.removeProperty('z-index');
      }
      _context = canvas.context2D;
    } catch (e) {
      // Handle OOM.
    }
    if (_context == null) {
      reduceCanvasMemoryUsage();
      _context = canvas.context2D;
    }
    if (_context == null) {
      /// Browser ran out of memory, try to recover current allocation
      /// and bail.
      _canvas?.width = 0;
      _canvas?.height = 0;
      _canvas = null;
      return;
    }
    _contextHandle = ContextStateHandle(this, _context!, _density);
    _initializeViewport(requiresClearRect);
    _replayClipStack();
  }

  DomCanvasElement? _allocCanvas(int width, int height) {
    // The dartdocs for `tryCreateCanvasElement` on why we don't use the
    // `DomCanvasElement` constructor.
    return tryCreateCanvasElement((width * _density).ceil(), (height * _density).ceil());
  }

  @override
  void clear() {
    super.clear();

    if (_canvas != null) {
      // Restore to the state where we have only applied the scaling.
      final DomCanvasRenderingContext2D? ctx = _context;
      if (ctx != null) {
        try {
          ctx.font = '';
        } catch (e) {
          // Firefox may explode here:
          // https://bugzilla.mozilla.org/show_bug.cgi?id=941146
          if (!isNsErrorFailureException(e)) {
            rethrow;
          }
        }
      }
    }
    reuse();
  }

  int _replaySingleSaveEntry(
    int clipDepth,
    Matrix4 prevTransform,
    Matrix4 transform,
    List<SaveClipEntry>? clipStack,
  ) {
    final DomCanvasRenderingContext2D ctx = context;
    if (clipStack != null) {
      for (final int clipCount = clipStack.length; clipDepth < clipCount; clipDepth++) {
        final SaveClipEntry clipEntry = clipStack[clipDepth];
        final Matrix4 clipTimeTransform = clipEntry.currentTransform;
        // If transform for entry recording change since last element, update.
        // Comparing only matrix3 elements since Canvas API restricted.
        if (clipTimeTransform[0] != prevTransform[0] ||
            clipTimeTransform[1] != prevTransform[1] ||
            clipTimeTransform[4] != prevTransform[4] ||
            clipTimeTransform[5] != prevTransform[5] ||
            clipTimeTransform[12] != prevTransform[12] ||
            clipTimeTransform[13] != prevTransform[13]) {
          final double ratio = dpi;
          ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
          ctx.transform(
            clipTimeTransform[0],
            clipTimeTransform[1],
            clipTimeTransform[4],
            clipTimeTransform[5],
            clipTimeTransform[12],
            clipTimeTransform[13],
          );
          prevTransform = clipTimeTransform;
        }
        if (clipEntry.rect != null) {
          _clipRect(ctx, clipEntry.rect!);
        } else if (clipEntry.rrect != null) {
          _clipRRect(ctx, clipEntry.rrect!);
        } else if (clipEntry.path != null) {
          final SurfacePath path = clipEntry.path! as SurfacePath;
          _runPath(ctx, path);
          if (path.fillType == ui.PathFillType.nonZero) {
            ctx.clip();
          } else {
            ctx.clip('evenodd');
          }
        }
      }
    }
    // If transform was changed between last clip operation and save call,
    // update.
    if (transform[0] != prevTransform[0] ||
        transform[1] != prevTransform[1] ||
        transform[4] != prevTransform[4] ||
        transform[5] != prevTransform[5] ||
        transform[12] != prevTransform[12] ||
        transform[13] != prevTransform[13]) {
      final double ratio = dpi;
      ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
      ctx.transform(
        transform[0],
        transform[1],
        transform[4],
        transform[5],
        transform[12],
        transform[13],
      );
    }
    return clipDepth;
  }

  void _replayClipStack() {
    // Replay save/clip stack on this canvas now.
    final DomCanvasRenderingContext2D ctx = context;
    int clipDepth = 0;
    Matrix4 prevTransform = Matrix4.identity();
    final int len = _saveStack.length;
    for (int saveStackIndex = 0; saveStackIndex < len; saveStackIndex++) {
      final SaveStackEntry saveEntry = _saveStack[saveStackIndex];
      clipDepth = _replaySingleSaveEntry(
        clipDepth,
        prevTransform,
        saveEntry.transform,
        saveEntry.clipStack,
      );
      prevTransform = saveEntry.transform;
      ctx.save();
      ++_saveContextCount;
    }
    _replaySingleSaveEntry(clipDepth, prevTransform, _currentTransform, clipStack);
  }

  /// Marks this pool for reuse.
  void reuse() {
    if (_canvas != null) {
      _restoreContextSave();
      _contextHandle!.reset();
      _activeCanvasList ??= <DomCanvasElement>[];
      _activeCanvasList!.add(_canvas!);
      _context = null;
      _contextHandle = null;
    }
    _reusablePool = _activeCanvasList;
    _activeCanvasList = null;
    _canvas = null;
    _context = null;
    _contextHandle = null;
    _resetTransform();
  }

  /// Signals to canvas pool the end of drawing commands so cached resources
  /// that are reused from last instance can be cleanup.
  void endOfPaint() {
    if (_reusablePool != null) {
      for (final DomCanvasElement e in _reusablePool!) {
        if (ui_web.browser.browserEngine == ui_web.BrowserEngine.webkit) {
          e.width = e.height = 0;
        }
        e.remove();
      }
      _reusablePool = null;
    }
    _restoreContextSave();
  }

  void _restoreContextSave() {
    while (_saveContextCount != 0) {
      _context!.restore();
      --_saveContextCount;
    }
  }

  /// Configures the canvas such that its coordinate system follows the scene's
  /// coordinate system, and the pixel ratio is applied such that CSS pixels are
  /// translated to bitmap pixels.
  void _initializeViewport(bool clearCanvas) {
    final DomCanvasRenderingContext2D ctx = context;
    // Save the canvas state with top-level transforms so we can undo
    // any clips later when we reuse the canvas.
    ctx.save();
    ++_saveContextCount;

    // We always start with identity transform because the surrounding transform
    // is applied on the DOM elements.
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    if (clearCanvas) {
      ctx.clearRect(0, 0, _widthInBitmapPixels * _density, _heightInBitmapPixels * _density);
    }

    // This scale makes sure that 1 CSS pixel is translated to the correct
    // number of bitmap pixels.
    ctx.scale(dpi, dpi);
  }

  /// Returns effective dpi (browser DPI and pixel density due to transform).
  double get dpi => EngineFlutterDisplay.instance.browserDevicePixelRatio * _density;

  void _resetTransform() {
    final DomCanvasElement? canvas = _canvas;
    if (canvas != null) {
      canvas.style.transformOrigin = '';
      canvas.style.transform = '';
    }
  }

  /// Returns a "data://" URI containing a representation of the image in this
  /// canvas in PNG format.
  String toDataUrl() {
    if (_canvas == null) {
      _createCanvas();
    }
    return _canvas!.toDataURL();
  }

  @override
  void save() {
    super.save();
    if (_canvas != null) {
      context.save();
      ++_saveContextCount;
    }
  }

  @override
  void restore() {
    super.restore();
    if (_canvas != null) {
      context.restore();
      contextHandle.reset();
      --_saveContextCount;
    }
  }

  @override
  void translate(double dx, double dy) {
    super.translate(dx, dy);
    if (_canvas != null) {
      context.translate(dx, dy);
    }
  }

  @override
  void scale(double sx, double sy) {
    super.scale(sx, sy);
    if (_canvas != null) {
      context.scale(sx, sy);
    }
  }

  @override
  void rotate(double radians) {
    super.rotate(radians);
    if (_canvas != null) {
      context.rotate(radians);
    }
  }

  @override
  void skew(double sx, double sy) {
    super.skew(sx, sy);
    if (_canvas != null) {
      context.transform(1, sy, sx, 1, 0, 0);
      //                |  |   |   |  |  |
      //                |  |   |   |  |  f - vertical translation
      //                |  |   |   |  e - horizontal translation
      //                |  |   |   d - vertical scaling
      //                |  |   c - horizontal skewing
      //                |  b - vertical skewing
      //                a - horizontal scaling
      //
      // Source: https://developer.mozilla.org/en-US/docs/Web/API/CanvasRenderingContext2D/transform
    }
  }

  @override
  void transform(Float32List matrix4) {
    super.transform(matrix4);
    // Canvas2D transform API:
    //
    // ctx.transform(a, b, c, d, e, f);
    //
    // In 3x3 matrix form assuming vector representation of (x, y, 1):
    //
    // a c e
    // b d f
    // 0 0 1
    //
    // This translates to 4x4 matrix with vector representation of (x, y, z, 1)
    // as:
    //
    // a c 0 e
    // b d 0 f
    // 0 0 1 0
    // 0 0 0 1
    //
    // This matrix is sufficient to represent 2D rotates, translates, scales,
    // and skews.
    if (_canvas != null) {
      context.transform(matrix4[0], matrix4[1], matrix4[4], matrix4[5], matrix4[12], matrix4[13]);
    }
  }

  @override
  void clipRect(ui.Rect rect) {
    super.clipRect(rect);
    if (_canvas != null) {
      _clipRect(context, rect);
    }
  }

  void _clipRect(DomCanvasRenderingContext2D ctx, ui.Rect rect) {
    ctx.beginPath();
    ctx.rect(rect.left, rect.top, rect.width, rect.height);
    ctx.clip();
  }

  @override
  void clipRRect(ui.RRect rrect) {
    super.clipRRect(rrect);
    if (_canvas != null) {
      _clipRRect(context, rrect);
    }
  }

  void _clipRRect(DomCanvasRenderingContext2D ctx, ui.RRect rrect) {
    final ui.Path path = ui.Path()..addRRect(rrect);
    _runPath(ctx, path as SurfacePath);
    ctx.clip();
  }

  @override
  void clipPath(ui.Path path) {
    super.clipPath(path);
    if (_canvas != null) {
      final DomCanvasRenderingContext2D ctx = context;
      _runPath(ctx, path as SurfacePath);
      if (path.fillType == ui.PathFillType.nonZero) {
        ctx.clip();
      } else {
        ctx.clip('evenodd');
      }
    }
  }

  /// Fill a virtually infinite rect with a color and optional blendMode.
  void drawColor(ui.Color color, ui.BlendMode blendMode) {
    final DomCanvasRenderingContext2D ctx = context;
    contextHandle.blendMode = blendMode;
    contextHandle.fillStyle = color.toCssString();
    contextHandle.strokeStyle = '';
    ctx.beginPath();
    // Fill a virtually infinite rect with the color.
    //
    // We can't use (0, 0, width, height) because the current transform can
    // cause it to not fill the entire clip.
    ctx.fillRect(-10000, -10000, 20000, 20000);
  }

  /// Fill a virtually infinite rect with the color.
  void fill() {
    final DomCanvasRenderingContext2D ctx = context;
    ctx.beginPath();
    // We can't use (0, 0, width, height) because the current transform can
    // cause it to not fill the entire clip.
    ctx.fillRect(-10000, -10000, 20000, 20000);
  }

  /// Draws a line from [p1] to [p2].
  void strokeLine(ui.Offset p1, ui.Offset p2) {
    final DomCanvasRenderingContext2D ctx = context;
    ctx.beginPath();
    final ui.Rect? shaderBounds = contextHandle._shaderBounds;
    if (shaderBounds == null) {
      ctx.moveTo(p1.dx, p1.dy);
      ctx.lineTo(p2.dx, p2.dy);
    } else {
      ctx.moveTo(p1.dx - shaderBounds.left, p1.dy - shaderBounds.top);
      ctx.lineTo(p2.dx - shaderBounds.left, p2.dy - shaderBounds.top);
    }
    ctx.stroke();
  }

  /// Draws a set of points with given radius, lines between points or
  /// a polygon.
  void drawPoints(ui.PointMode pointMode, Float32List points, double radius) {
    final DomCanvasRenderingContext2D ctx = context;
    final int len = points.length;
    final ui.Rect? shaderBounds = contextHandle._shaderBounds;
    final double offsetX = shaderBounds == null ? 0 : -shaderBounds.left;
    final double offsetY = shaderBounds == null ? 0 : -shaderBounds.top;
    switch (pointMode) {
      case ui.PointMode.points:
        for (int i = 0; i < len; i += 2) {
          final double x = points[i] + offsetX;
          final double y = points[i + 1] + offsetY;
          ctx.beginPath();
          ctx.arc(x, y, radius, 0, 2.0 * math.pi);
          ctx.fill();
        }
      case ui.PointMode.lines:
        ctx.beginPath();
        for (int i = 0; i < (len - 2); i += 4) {
          ctx.moveTo(points[i] + offsetX, points[i + 1] + offsetY);
          ctx.lineTo(points[i + 2] + offsetX, points[i + 3] + offsetY);
          ctx.stroke();
        }
      case ui.PointMode.polygon:
        ctx.beginPath();
        ctx.moveTo(points[0] + offsetX, points[1] + offsetY);
        for (int i = 2; i < len; i += 2) {
          ctx.lineTo(points[i] + offsetX, points[i + 1] + offsetY);
        }
        ctx.stroke();
    }
  }

  // Float buffer used for path iteration.
  static final Float32List _runBuffer = Float32List(PathRefIterator.kMaxBufferSize);

  /// 'Runs' the given [path] by applying all of its commands to the canvas.
  void _runPath(DomCanvasRenderingContext2D ctx, SurfacePath path) {
    ctx.beginPath();
    final Float32List p = _runBuffer;
    final PathRefIterator iter = PathRefIterator(path.pathRef);
    int verb = 0;
    while ((verb = iter.next(p)) != SPath.kDoneVerb) {
      switch (verb) {
        case SPath.kMoveVerb:
          ctx.moveTo(p[0], p[1]);
        case SPath.kLineVerb:
          ctx.lineTo(p[2], p[3]);
        case SPath.kCubicVerb:
          ctx.bezierCurveTo(p[2], p[3], p[4], p[5], p[6], p[7]);
        case SPath.kQuadVerb:
          ctx.quadraticCurveTo(p[2], p[3], p[4], p[5]);
        case SPath.kConicVerb:
          final double w = iter.conicWeight;
          final Conic conic = Conic(p[0], p[1], p[2], p[3], p[4], p[5], w);
          final List<ui.Offset> points = conic.toQuads();
          final int len = points.length;
          for (int i = 1; i < len; i += 2) {
            final double p1x = points[i].dx;
            final double p1y = points[i].dy;
            final double p2x = points[i + 1].dx;
            final double p2y = points[i + 1].dy;
            ctx.quadraticCurveTo(p1x, p1y, p2x, p2y);
          }
        case SPath.kCloseVerb:
          ctx.closePath();
        default:
          throw UnimplementedError('Unknown path verb $verb');
      }
    }
  }

  /// Draws a rectangle filled or stroked based on [style].
  void drawRect(ui.Rect rect, ui.PaintingStyle? style) {
    context.beginPath();
    final ui.Rect? shaderBounds = contextHandle._shaderBounds;
    if (shaderBounds == null) {
      context.rect(rect.left, rect.top, rect.width, rect.height);
    } else {
      context.rect(
        rect.left - shaderBounds.left,
        rect.top - shaderBounds.top,
        rect.width,
        rect.height,
      );
    }
    contextHandle.paint(style);
  }

  /// Applies path to drawing context, preparing for fill and other operations.
  ///
  /// WARNING: Don't refactor _runPath/_runPathWithOffset. Latency sensitive
  void _runPathWithOffset(
    DomCanvasRenderingContext2D ctx,
    SurfacePath path,
    double offsetX,
    double offsetY,
  ) {
    ctx.beginPath();
    final Float32List p = _runBuffer;
    final PathRefIterator iter = PathRefIterator(path.pathRef);
    int verb = 0;
    while ((verb = iter.next(p)) != SPath.kDoneVerb) {
      switch (verb) {
        case SPath.kMoveVerb:
          ctx.moveTo(p[0] + offsetX, p[1] + offsetY);
        case SPath.kLineVerb:
          ctx.lineTo(p[2] + offsetX, p[3] + offsetY);
        case SPath.kCubicVerb:
          ctx.bezierCurveTo(
            p[2] + offsetX,
            p[3] + offsetY,
            p[4] + offsetX,
            p[5] + offsetY,
            p[6] + offsetX,
            p[7] + offsetY,
          );
        case SPath.kQuadVerb:
          ctx.quadraticCurveTo(p[2] + offsetX, p[3] + offsetY, p[4] + offsetX, p[5] + offsetY);
        case SPath.kConicVerb:
          final double w = iter.conicWeight;
          final Conic conic = Conic(p[0], p[1], p[2], p[3], p[4], p[5], w);
          final List<ui.Offset> points = conic.toQuads();
          final int len = points.length;
          for (int i = 1; i < len; i += 2) {
            final double p1x = points[i].dx;
            final double p1y = points[i].dy;
            final double p2x = points[i + 1].dx;
            final double p2y = points[i + 1].dy;
            ctx.quadraticCurveTo(p1x + offsetX, p1y + offsetY, p2x + offsetX, p2y + offsetY);
          }
        case SPath.kCloseVerb:
          ctx.closePath();
        default:
          throw UnimplementedError('Unknown path verb $verb');
      }
    }
  }

  /// Draws a rounded rectangle filled or stroked based on [style].
  void drawRRect(ui.RRect roundRect, ui.PaintingStyle? style) {
    final ui.Rect? shaderBounds = contextHandle._shaderBounds;
    RRectToCanvasRenderer(context).render(
      shaderBounds == null
          ? roundRect
          : roundRect.shift(ui.Offset(-shaderBounds.left, -shaderBounds.top)),
    );
    contextHandle.paint(style);
  }

  /// Fills or strokes the area between [outer] and [inner] rounded rectangles.
  ///
  /// Typically used to draw a thick round border.
  void drawDRRect(ui.RRect outer, ui.RRect inner, ui.PaintingStyle? style) {
    final RRectRenderer renderer = RRectToCanvasRenderer(context);
    final ui.Rect? shaderBounds = contextHandle._shaderBounds;
    if (shaderBounds == null) {
      renderer.render(outer);
      renderer.render(inner, startNewPath: false, reverse: true);
    } else {
      final ui.Offset shift = ui.Offset(-shaderBounds.left, -shaderBounds.top);
      renderer.render(outer.shift(shift));
      renderer.render(inner.shift(shift), startNewPath: false, reverse: true);
    }
    contextHandle.paint(style);
  }

  /// Draws an axis-aligned oval that fills the given axis-aligned rectangle.
  void drawOval(ui.Rect rect, ui.PaintingStyle? style) {
    context.beginPath();
    final ui.Rect? shaderBounds = contextHandle._shaderBounds;
    final double cx = shaderBounds == null ? rect.center.dx : rect.center.dx - shaderBounds.left;
    final double cy = shaderBounds == null ? rect.center.dy : rect.center.dy - shaderBounds.top;
    drawEllipse(context, cx, cy, rect.width / 2, rect.height / 2, 0, 0, 2.0 * math.pi, false);
    contextHandle.paint(style);
  }

  /// Draws a circle centered at [c] with [radius].
  void drawCircle(ui.Offset c, double radius, ui.PaintingStyle? style) {
    context.beginPath();
    final ui.Rect? shaderBounds = contextHandle._shaderBounds;
    final double cx = shaderBounds == null ? c.dx : c.dx - shaderBounds.left;
    final double cy = shaderBounds == null ? c.dy : c.dy - shaderBounds.top;
    drawEllipse(context, cx, cy, radius, radius, 0, 0, 2.0 * math.pi, false);
    contextHandle.paint(style);
  }

  /// Draws or strokes a path based on [style] and current context state.
  void drawPath(ui.Path path, ui.PaintingStyle? style) {
    final ui.Rect? shaderBounds = contextHandle._shaderBounds;
    if (shaderBounds == null) {
      _runPath(context, path as SurfacePath);
    } else {
      _runPathWithOffset(context, path as SurfacePath, -shaderBounds.left, -shaderBounds.top);
    }
    contextHandle.paintPath(style, path.fillType);
  }

  void drawImage(DomHTMLImageElement element, ui.Offset p) {
    context.drawImage(element, p.dx, p.dy);
  }

  /// Draws a shadow for a Path representing the given material elevation.
  void drawShadow(ui.Path path, ui.Color color, double elevation, bool transparentOccluder) {
    final SurfaceShadowData? shadow = computeShadow(path.getBounds(), elevation);
    if (shadow != null) {
      // On April 2020 Web canvas 2D did not support shadow color alpha. So
      // instead we apply alpha separately using globalAlpha, then paint a
      // solid shadow.
      final ui.Color shadowColor = toShadowColor(color);
      final double opacity = shadowColor.alpha / 255;
      final String solidColor = colorComponentsToCssString(
        shadowColor.red,
        shadowColor.green,
        shadowColor.blue,
        255,
      );
      context.save();
      context.globalAlpha = opacity;

      // TODO(hterkelsen): Shadows with transparent occluders are not supported
      // on webkit since filter is unsupported.
      if (transparentOccluder && ui_web.browser.browserEngine != ui_web.BrowserEngine.webkit) {
        // We paint shadows using a path and a mask filter instead of the
        // built-in shadow* properties. This is because the color alpha of the
        // paint is added to the shadow. The effect we're looking for is to just
        // paint the shadow without the path itself, but if we use a non-zero
        // alpha for the paint the path is painted in addition to the shadow,
        // which is undesirable.
        context.translate(shadow.offset.dx, shadow.offset.dy);
        context.filter = maskFilterToCanvasFilter(
          ui.MaskFilter.blur(ui.BlurStyle.normal, shadow.blurWidth),
        );
        context.strokeStyle = '';
        context.fillStyle = solidColor;
      } else {
        // TODO(yjbanov): the following comment by hterkelsen makes sense, but
        //                somehow we lost the implementation described in it.
        //                Perhaps we should revisit this and actually do what
        //                the comment says.
        // TODO(hterkelsen): We fill the path with this paint, then later we clip
        // by the same path and fill it with a fully opaque color (we know
        // the color is fully opaque because `transparentOccluder` is false.
        // However, due to anti-aliasing of the clip, a few pixels of the
        // path we are about to paint may still be visible after we fill with
        // the opaque occluder. For that reason, we fill with the shadow color,
        // and set the shadow color to fully opaque. This way, the visible
        // pixels are less opaque and less noticeable.
        context.filter = 'none';
        context.strokeStyle = '';
        context.fillStyle = solidColor;
        context.shadowBlur = shadow.blurWidth;
        context.shadowColor = solidColor;
        context.shadowOffsetX = shadow.offset.dx;
        context.shadowOffsetY = shadow.offset.dy;
      }
      _runPath(context, path as SurfacePath);
      context.fill();

      // This also resets globalAlpha and shadow attributes. See:
      // https://developer.mozilla.org/en-US/docs/Web/API/CanvasRenderingContext2D/save#Drawing_state
      context.restore();
    }
  }

  /// Disposes html canvas element(s) used by this pool when persistent surface
  /// is disposed.
  ///
  /// When this pool is reused, [clear] is called instead to be able to
  /// draw using existing canvas elements.
  void dispose() {
    // Webkit has a threshold for the amount of canvas pixels an app can
    // allocate. Even though our canvases are being garbage-collected as
    // expected when we don't need them, Webkit keeps track of their sizes
    // towards the threshold. Setting width and height to zero tricks Webkit
    // into thinking that this canvas has a zero size so it doesn't count it
    // towards the threshold.
    if (ui_web.browser.browserEngine == ui_web.BrowserEngine.webkit && _canvas != null) {
      _canvas!.width = _canvas!.height = 0;
    }
    _clearActiveCanvasList();
  }

  void _clearActiveCanvasList() {
    if (_activeCanvasList != null) {
      for (final DomCanvasElement c in _activeCanvasList!) {
        if (ui_web.browser.browserEngine == ui_web.BrowserEngine.webkit) {
          c.width = c.height = 0;
        }
        c.remove();
      }
    }
    _activeCanvasList = null;
  }
}

/// Optimizes applying paint parameters to html canvas.
///
/// See https://www.w3.org/TR/2dcontext/ for defaults used in this class
/// to initialize current values.
class ContextStateHandle {
  /// Initializes context state for a [CanvasPool].
  ContextStateHandle(this._canvasPool, this.context, this.density);

  /// Associated canvas element context tracked by this context state.
  final DomCanvasRenderingContext2D context;
  final CanvasPool _canvasPool;

  /// Dpi of context.
  final double density;
  ui.BlendMode? _currentBlendMode = ui.BlendMode.srcOver;
  ui.StrokeCap? _currentStrokeCap = ui.StrokeCap.butt;
  ui.StrokeJoin? _currentStrokeJoin = ui.StrokeJoin.miter;
  // Fill style and stroke style are Object since they can have a String or
  // shader object such as a gradient.
  Object? _currentFillStyle;
  Object? _currentStrokeStyle;
  double _currentLineWidth = 1.0;

  /// See [DomCanvasRenderingContext2D].
  set blendMode(ui.BlendMode? blendMode) {
    if (blendMode != _currentBlendMode) {
      _currentBlendMode = blendMode;
      context.globalCompositeOperation = blendModeToCssMixBlendMode(blendMode) ?? 'source-over';
    }
  }

  /// See [DomCanvasRenderingContext2D].
  set strokeCap(ui.StrokeCap? strokeCap) {
    strokeCap ??= ui.StrokeCap.butt;
    if (strokeCap != _currentStrokeCap) {
      _currentStrokeCap = strokeCap;
      context.lineCap = stringForStrokeCap(strokeCap)!;
    }
  }

  /// See [DomCanvasRenderingContext2D].
  set lineWidth(double lineWidth) {
    if (lineWidth != _currentLineWidth) {
      _currentLineWidth = lineWidth;
      context.lineWidth = lineWidth;
    }
  }

  /// See [DomCanvasRenderingContext2D].
  set strokeJoin(ui.StrokeJoin? strokeJoin) {
    strokeJoin ??= ui.StrokeJoin.miter;
    if (strokeJoin != _currentStrokeJoin) {
      _currentStrokeJoin = strokeJoin;
      context.lineJoin = stringForStrokeJoin(strokeJoin);
    }
  }

  /// See [DomCanvasRenderingContext2D].
  set fillStyle(Object? colorOrGradient) {
    if (!identical(colorOrGradient, _currentFillStyle)) {
      _currentFillStyle = colorOrGradient;
      context.fillStyle = colorOrGradient;
    }
  }

  /// See [DomCanvasRenderingContext2D].
  set strokeStyle(Object? colorOrGradient) {
    if (!identical(colorOrGradient, _currentStrokeStyle)) {
      _currentStrokeStyle = colorOrGradient;
      context.strokeStyle = colorOrGradient;
    }
  }

  ui.MaskFilter? _currentFilter;
  SurfacePaintData? _lastUsedPaint;

  /// Currently active shader bounds.
  ///
  /// When a paint style uses a shader that produces a pattern, the pattern
  /// origin is relative to current transform. Therefore any painting operations
  /// will have to reverse the transform to correctly align pattern with
  /// drawing bounds.
  ui.Rect? _shaderBounds;

  /// The painting state.
  ///
  /// Used to validate that the [setUpPaint] and [tearDownPaint] are called in
  /// a correct sequence.
  bool _debugIsPaintSetUp = false;

  /// Whether to use WebKit's method of rendering [MaskFilter].
  ///
  /// This is used in screenshot tests to test Safari codepaths.
  static bool debugEmulateWebKitMaskFilter = false;

  bool get _renderMaskFilterForWebkit =>
      ui_web.browser.browserEngine == ui_web.BrowserEngine.webkit || debugEmulateWebKitMaskFilter;

  /// Sets paint properties on the current canvas.
  ///
  /// [tearDownPaint] must be called after calling this method.
  void setUpPaint(SurfacePaintData paint, ui.Rect? shaderBounds) {
    assert(() {
      final bool wasPaintSetUp = _debugIsPaintSetUp;
      _debugIsPaintSetUp = true;
      // When setting up paint, the previous paint must be torn down.
      return !wasPaintSetUp;
    }());

    _lastUsedPaint = paint;
    lineWidth = paint.strokeWidth ?? 1.0;
    blendMode = paint.blendMode;
    strokeCap = paint.strokeCap;
    strokeJoin = paint.strokeJoin;

    if (paint.shader != null) {
      if (paint.shader is EngineGradient) {
        final EngineGradient engineShader = paint.shader! as EngineGradient;
        final Object paintStyle = engineShader.createPaintStyle(
          _canvasPool.context,
          shaderBounds,
          density,
        );
        fillStyle = paintStyle;
        strokeStyle = paintStyle;
        _shaderBounds = shaderBounds;
        // Align pattern origin to destination.
        context.translate(shaderBounds!.left, shaderBounds.top);
      } else if (paint.shader is EngineImageShader) {
        final EngineImageShader imageShader = paint.shader! as EngineImageShader;
        final Object paintStyle = imageShader.createPaintStyle(
          _canvasPool.context,
          shaderBounds,
          density,
        );
        fillStyle = paintStyle;
        strokeStyle = paintStyle;
        if (imageShader.requiresTileOffset) {
          _shaderBounds = shaderBounds;
          // Align pattern origin to destination.
          context.translate(shaderBounds!.left, shaderBounds.top);
        }
      }
    } else {
      final String colorString = colorValueToCssString(paint.color);
      fillStyle = colorString;
      strokeStyle = colorString;
    }

    final ui.MaskFilter? maskFilter = paint.maskFilter;
    if (!_renderMaskFilterForWebkit) {
      if (_currentFilter != maskFilter) {
        _currentFilter = maskFilter;
        context.filter = maskFilterToCanvasFilter(maskFilter);
      }
    } else {
      // WebKit does not support the `filter` property. Instead we apply a
      // shadow to the shape of the same color as the paint and the same blur
      // as the mask filter.
      //
      // Note that on WebKit the cached value of _currentFilter is not useful.
      // Instead we destructure it into the shadow properties and cache those.
      if (maskFilter != null) {
        context.save();
        context.shadowBlur = convertSigmaToRadius(maskFilter.webOnlySigma);
        // Shadow color must be fully opaque.
        context.shadowColor = ui.Color(paint.color).withAlpha(255).toCssString();

        // On the web a shadow must always be painted together with the shape
        // that casts it. In order to paint just the shadow, we offset the shape
        // by a large enough value that it moved outside the canvas bounds, then
        // offset the shadow in the opposite direction such that it lands exactly
        // where the shape is.
        const double kOutsideTheBoundsOffset = 50000;

        context.translate(-kOutsideTheBoundsOffset, 0);

        // Shadow offset is not affected by the current canvas context transform.
        // We have to apply the transform ourselves. To do that we transform the
        // tip of the vector from the shape to the shadow, then we transform the
        // origin (0, 0). The desired shadow offset is the difference between the
        // two. In vector notation, this is:
        //
        // transformedShadowDelta = M*shadowDelta - M*origin.
        final Float32List tempVector = Float32List(2);
        tempVector[0] = kOutsideTheBoundsOffset * EngineFlutterDisplay.instance.devicePixelRatio;
        _canvasPool.currentTransform.transform2(tempVector);
        final double shadowOffsetX = tempVector[0];
        final double shadowOffsetY = tempVector[1];

        tempVector[0] = tempVector[1] = 0;
        _canvasPool.currentTransform.transform2(tempVector);
        context.shadowOffsetX = shadowOffsetX - tempVector[0];
        context.shadowOffsetY = shadowOffsetY - tempVector[1];
      }
    }
  }

  /// Removes paint properties on the current canvas used by the last draw
  /// command.
  ///
  /// Not all properties are cleared. Properties that are set by all paint
  /// commands prior to painting do not need to be cleared.
  ///
  /// Must be called after calling [setUpPaint].
  void tearDownPaint() {
    assert(() {
      final bool wasPaintSetUp = _debugIsPaintSetUp;
      _debugIsPaintSetUp = false;
      // When tearing down paint, we expect that it was set up before.
      return wasPaintSetUp;
    }());

    final ui.MaskFilter? maskFilter = _lastUsedPaint?.maskFilter;
    if (maskFilter != null && _renderMaskFilterForWebkit) {
      // On Safari (WebKit) we use a translated shadow to emulate
      // MaskFilter.blur. We use restore to undo the translation and
      // shadow attributes.
      context.restore();
    }
    if (_shaderBounds != null) {
      context.translate(-_shaderBounds!.left, -_shaderBounds!.top);
      _shaderBounds = null;
    }
  }

  /// Fills or strokes the currently active path.
  void paint(ui.PaintingStyle? style) {
    if (style == ui.PaintingStyle.stroke) {
      context.stroke();
    } else {
      context.fill();
    }
  }

  /// Fills or strokes the currently active path based on fill type.
  void paintPath(ui.PaintingStyle? style, ui.PathFillType pathFillType) {
    if (style == ui.PaintingStyle.stroke) {
      context.stroke();
    } else {
      if (pathFillType == ui.PathFillType.nonZero) {
        context.fill();
      } else {
        context.fill('evenodd');
      }
    }
  }

  /// Resets drawing context state to defaults for
  /// [DomCanvasRenderingContext2D].
  void reset() {
    context.fillStyle = '';
    // Read back fillStyle/strokeStyle values from context so that input such
    // as rgba(0, 0, 0, 0) is correctly compared and doesn't cause diff on
    // setter.
    _currentFillStyle = context.fillStyle;
    context.strokeStyle = '';
    _currentStrokeStyle = context.strokeStyle;
    context.shadowBlur = 0;
    context.shadowColor = 'none';
    context.shadowOffsetX = 0;
    context.shadowOffsetY = 0;
    context.globalCompositeOperation = 'source-over';
    _currentBlendMode = ui.BlendMode.srcOver;
    context.lineWidth = 1.0;
    _currentLineWidth = 1.0;
    context.lineCap = 'butt';
    _currentStrokeCap = ui.StrokeCap.butt;
    context.lineJoin = 'miter';
    _currentStrokeJoin = ui.StrokeJoin.miter;
    _shaderBounds = null;
  }
}

/// Provides save stack tracking functionality to implementations of
/// [EngineCanvas].
class _SaveStackTracking {
  final List<SaveStackEntry> _saveStack = <SaveStackEntry>[];

  /// The stack that maintains clipping operations used when text is painted
  /// onto bitmap canvas but is composited as separate element.
  List<SaveClipEntry>? clipStack;

  /// Returns whether there are active clipping regions on the canvas.
  bool get isClipped => clipStack != null;

  /// Empties the save stack and the element stack, and resets the transform
  /// and clip parameters.
  @mustCallSuper
  void clear() {
    _saveStack.clear();
    clipStack = null;
    _currentTransform = Matrix4.identity();
  }

  /// The current transformation matrix.
  Matrix4 get currentTransform => _currentTransform;
  Matrix4 _currentTransform = Matrix4.identity();

  /// Saves current clip and transform on the save stack.
  @mustCallSuper
  void save() {
    _saveStack.add(
      SaveStackEntry(
        transform: _currentTransform.clone(),
        clipStack: clipStack == null ? null : List<SaveClipEntry>.from(clipStack!),
      ),
    );
  }

  /// Restores current clip and transform from the save stack.
  @mustCallSuper
  void restore() {
    if (_saveStack.isEmpty) {
      return;
    }
    final SaveStackEntry entry = _saveStack.removeLast();
    _currentTransform = entry.transform;
    clipStack = entry.clipStack;
  }

  /// Multiplies the [currentTransform] matrix by a translation.
  @mustCallSuper
  void translate(double dx, double dy) {
    _currentTransform.translate(dx, dy);
  }

  /// Scales the [currentTransform] matrix.
  @mustCallSuper
  void scale(double sx, double sy) {
    _currentTransform.scale(sx, sy);
  }

  /// Rotates the [currentTransform] matrix.
  @mustCallSuper
  void rotate(double radians) {
    _currentTransform.rotate(kUnitZ, radians);
  }

  /// Skews the [currentTransform] matrix.
  @mustCallSuper
  void skew(double sx, double sy) {
    final Matrix4 skewMatrix = Matrix4.identity();
    final Float32List storage = skewMatrix.storage;
    storage[1] = sy;
    storage[4] = sx;
    _currentTransform.multiply(skewMatrix);
  }

  /// Multiplies the [currentTransform] matrix by another matrix.
  @mustCallSuper
  void transform(Float32List matrix4) {
    _currentTransform.multiply(Matrix4.fromFloat32List(matrix4));
  }

  /// Adds a rectangle to clipping stack.
  @mustCallSuper
  void clipRect(ui.Rect rect) {
    clipStack ??= <SaveClipEntry>[];
    clipStack!.add(SaveClipEntry.rect(rect, _currentTransform.clone()));
  }

  /// Adds a round rectangle to clipping stack.
  @mustCallSuper
  void clipRRect(ui.RRect rrect) {
    clipStack ??= <SaveClipEntry>[];
    clipStack!.add(SaveClipEntry.rrect(rrect, _currentTransform.clone()));
  }

  /// Adds a path to clipping stack.
  @mustCallSuper
  void clipPath(ui.Path path) {
    clipStack ??= <SaveClipEntry>[];
    clipStack!.add(SaveClipEntry.path(path, _currentTransform.clone()));
  }
}
