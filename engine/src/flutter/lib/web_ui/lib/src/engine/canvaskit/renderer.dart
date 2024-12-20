// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:js_interop';
import 'dart:math' as math;
import 'dart:typed_data';

import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

enum CanvasKitVariant {
  /// The appropriate variant is chosen based on the browser.
  ///
  /// This is the default variant.
  auto,

  /// The full variant that can be used in any browser.
  full,

  /// The variant that is optimized for Chromium browsers.
  ///
  /// WARNING: In most cases, you should use [auto] instead of this variant. Using
  /// this variant in a non-Chromium browser will result in a broken app.
  chromium,
}

class CanvasKitRenderer implements Renderer {
  static CanvasKitRenderer get instance => _instance;
  static late CanvasKitRenderer _instance;

  Future<void>? _initialized;

  @override
  String get rendererTag => 'canvaskit';

  late final SkiaFontCollection _fontCollection = SkiaFontCollection();

  @override
  SkiaFontCollection get fontCollection => _fontCollection;

  /// The scene host, where the root canvas and overlay canvases are added to.
  DomElement? _sceneHost;
  DomElement? get sceneHost => _sceneHost;

  Rasterizer _rasterizer = _createRasterizer();

  static Rasterizer _createRasterizer() {
    if (isSafari || isFirefox) {
      return MultiSurfaceRasterizer();
    }
    return OffscreenCanvasRasterizer();
  }

  /// Override the rasterizer with the given [_rasterizer]. Used in tests.
  void debugOverrideRasterizer(Rasterizer testRasterizer) {
    _rasterizer = testRasterizer;
  }

  set resourceCacheMaxBytes(int bytes) => _rasterizer.setResourceCacheMaxBytes(bytes);

  /// A surface used specifically for `Picture.toImage` when software rendering
  /// is supported.
  final Surface pictureToImageSurface = Surface();

  // Listens for view creation events from the view manager.
  StreamSubscription<int>? _onViewCreatedListener;
  // Listens for view disposal events from the view manager.
  StreamSubscription<int>? _onViewDisposedListener;

  @override
  Future<void> initialize() async {
    _initialized ??= () async {
      if (windowFlutterCanvasKit != null) {
        canvasKit = windowFlutterCanvasKit!;
      } else if (windowFlutterCanvasKitLoaded != null) {
        // CanvasKit is being preloaded by flutter.js. Wait for it to complete.
        canvasKit = await promiseToFuture<CanvasKit>(windowFlutterCanvasKitLoaded!);
      } else {
        canvasKit = await downloadCanvasKit();
        windowFlutterCanvasKit = canvasKit;
      }
      // Views may have been registered before this renderer was initialized.
      // Create rasterizers for them and then start listening for new view
      // creation/disposal events.
      final FlutterViewManager viewManager = EnginePlatformDispatcher.instance.viewManager;
      if (_onViewCreatedListener == null) {
        for (final EngineFlutterView view in viewManager.views) {
          _onViewCreated(view.viewId);
        }
      }
      _onViewCreatedListener ??= viewManager.onViewCreated.listen(_onViewCreated);
      _onViewDisposedListener ??= viewManager.onViewDisposed.listen(_onViewDisposed);
      _instance = this;
    }();
    return _initialized;
  }

  @override
  ui.Paint createPaint() => CkPaint();

  @override
  ui.Vertices createVertices(
    ui.VertexMode mode,
    List<ui.Offset> positions, {
    List<ui.Offset>? textureCoordinates,
    List<ui.Color>? colors,
    List<int>? indices,
  }) => CkVertices(
    mode,
    positions,
    textureCoordinates: textureCoordinates,
    colors: colors,
    indices: indices,
  );

  @override
  ui.Vertices createVerticesRaw(
    ui.VertexMode mode,
    Float32List positions, {
    Float32List? textureCoordinates,
    Int32List? colors,
    Uint16List? indices,
  }) => CkVertices.raw(
    mode,
    positions,
    textureCoordinates: textureCoordinates,
    colors: colors,
    indices: indices,
  );

  @override
  ui.Canvas createCanvas(ui.PictureRecorder recorder, [ui.Rect? cullRect]) =>
      CanvasKitCanvas(recorder, cullRect);

  @override
  ui.Gradient createLinearGradient(
    ui.Offset from,
    ui.Offset to,
    List<ui.Color> colors, [
    List<double>? colorStops,
    ui.TileMode tileMode = ui.TileMode.clamp,
    Float32List? matrix4,
  ]) => CkGradientLinear(from, to, colors, colorStops, tileMode, matrix4);

  @override
  ui.Gradient createRadialGradient(
    ui.Offset center,
    double radius,
    List<ui.Color> colors, [
    List<double>? colorStops,
    ui.TileMode tileMode = ui.TileMode.clamp,
    Float32List? matrix4,
  ]) => CkGradientRadial(center, radius, colors, colorStops, tileMode, matrix4);

  @override
  ui.Gradient createConicalGradient(
    ui.Offset focal,
    double focalRadius,
    ui.Offset center,
    double radius,
    List<ui.Color> colors, [
    List<double>? colorStops,
    ui.TileMode tileMode = ui.TileMode.clamp,
    Float32List? matrix,
  ]) => CkGradientConical(focal, focalRadius, center, radius, colors, colorStops, tileMode, matrix);

  @override
  ui.Gradient createSweepGradient(
    ui.Offset center,
    List<ui.Color> colors, [
    List<double>? colorStops,
    ui.TileMode tileMode = ui.TileMode.clamp,
    double startAngle = 0.0,
    double endAngle = math.pi * 2,
    Float32List? matrix4,
  ]) => CkGradientSweep(center, colors, colorStops, tileMode, startAngle, endAngle, matrix4);

  @override
  ui.PictureRecorder createPictureRecorder() => CkPictureRecorder();

  @override
  ui.SceneBuilder createSceneBuilder() => LayerSceneBuilder();

  @override
  ui.ImageFilter createBlurImageFilter({
    double sigmaX = 0.0,
    double sigmaY = 0.0,
    ui.TileMode? tileMode,
  }) => CkImageFilter.blur(sigmaX: sigmaX, sigmaY: sigmaY, tileMode: tileMode);

  @override
  ui.ImageFilter createDilateImageFilter({double radiusX = 0.0, double radiusY = 0.0}) =>
      CkImageFilter.dilate(radiusX: radiusX, radiusY: radiusY);

  @override
  ui.ImageFilter createErodeImageFilter({double radiusX = 0.0, double radiusY = 0.0}) =>
      CkImageFilter.erode(radiusX: radiusX, radiusY: radiusY);

  @override
  ui.ImageFilter createMatrixImageFilter(
    Float64List matrix4, {
    ui.FilterQuality filterQuality = ui.FilterQuality.low,
  }) => CkImageFilter.matrix(matrix: matrix4, filterQuality: filterQuality);

  @override
  ui.ImageFilter composeImageFilters({
    required ui.ImageFilter outer,
    required ui.ImageFilter inner,
  }) {
    if (outer is EngineColorFilter) {
      final CkColorFilter colorFilter = createCkColorFilter(outer)!;
      outer = CkColorFilterImageFilter(colorFilter: colorFilter);
    }
    if (inner is EngineColorFilter) {
      final CkColorFilter colorFilter = createCkColorFilter(inner)!;
      inner = CkColorFilterImageFilter(colorFilter: colorFilter);
    }
    return CkImageFilter.compose(outer: outer as CkImageFilter, inner: inner as CkImageFilter);
  }

  @override
  Future<ui.Codec> instantiateImageCodec(
    Uint8List list, {
    int? targetWidth,
    int? targetHeight,
    bool allowUpscaling = true,
  }) async => skiaInstantiateImageCodec(list, targetWidth, targetHeight, allowUpscaling);

  @override
  Future<ui.Codec> instantiateImageCodecFromUrl(
    Uri uri, {
    ui_web.ImageCodecChunkCallback? chunkCallback,
  }) => skiaInstantiateWebImageCodec(uri.toString(), chunkCallback);

  @override
  ui.Image createImageFromImageBitmap(DomImageBitmap imageBitmap) {
    final SkImage? skImage = canvasKit.MakeLazyImageFromImageBitmap(imageBitmap, true);
    if (skImage == null) {
      throw Exception('Failed to convert image bitmap to an SkImage.');
    }
    return CkImage(skImage, imageSource: ImageBitmapImageSource(imageBitmap));
  }

  @override
  FutureOr<ui.Image> createImageFromTextureSource(
    JSAny object, {
    required int width,
    required int height,
    required bool transferOwnership,
  }) async {
    if (!transferOwnership) {
      final DomImageBitmap bitmap = await createImageBitmap(object, (
        x: 0,
        y: 0,
        width: width,
        height: height,
      ));
      return createImageFromImageBitmap(bitmap);
    }
    final SkImage? skImage = canvasKit.MakeLazyImageFromTextureSourceWithInfo(
      object,
      SkPartialImageInfo(
        width: width.toDouble(),
        height: height.toDouble(),
        alphaType: canvasKit.AlphaType.Premul,
        colorType: canvasKit.ColorType.RGBA_8888,
        colorSpace: SkColorSpaceSRGB,
      ),
    );

    if (skImage == null) {
      throw Exception('Failed to convert image bitmap to an SkImage.');
    }
    return CkImage(skImage);
  }

  @override
  void decodeImageFromPixels(
    Uint8List pixels,
    int width,
    int height,
    ui.PixelFormat format,
    ui.ImageDecoderCallback callback, {
    int? rowBytes,
    int? targetWidth,
    int? targetHeight,
    bool allowUpscaling = true,
  }) => skiaDecodeImageFromPixels(
    pixels,
    width,
    height,
    format,
    callback,
    rowBytes: rowBytes,
    targetWidth: targetWidth,
    targetHeight: targetHeight,
    allowUpscaling: allowUpscaling,
  );

  @override
  ui.ImageShader createImageShader(
    ui.Image image,
    ui.TileMode tmx,
    ui.TileMode tmy,
    Float64List matrix4,
    ui.FilterQuality? filterQuality,
  ) => CkImageShader(image, tmx, tmy, matrix4, filterQuality);

  @override
  ui.Path createPath() => CkPath();

  @override
  ui.Path copyPath(ui.Path src) => CkPath.from(src as CkPath);

  @override
  ui.Path combinePaths(ui.PathOperation op, ui.Path path1, ui.Path path2) =>
      CkPath.combine(op, path1, path2);

  @override
  ui.TextStyle createTextStyle({
    ui.Color? color,
    ui.TextDecoration? decoration,
    ui.Color? decorationColor,
    ui.TextDecorationStyle? decorationStyle,
    double? decorationThickness,
    ui.FontWeight? fontWeight,
    ui.FontStyle? fontStyle,
    ui.TextBaseline? textBaseline,
    String? fontFamily,
    List<String>? fontFamilyFallback,
    double? fontSize,
    double? letterSpacing,
    double? wordSpacing,
    double? height,
    ui.TextLeadingDistribution? leadingDistribution,
    ui.Locale? locale,
    ui.Paint? background,
    ui.Paint? foreground,
    List<ui.Shadow>? shadows,
    List<ui.FontFeature>? fontFeatures,
    List<ui.FontVariation>? fontVariations,
  }) => CkTextStyle(
    color: color,
    decoration: decoration,
    decorationColor: decorationColor,
    decorationStyle: decorationStyle,
    decorationThickness: decorationThickness,
    fontWeight: fontWeight,
    fontStyle: fontStyle,
    textBaseline: textBaseline,
    fontFamily: fontFamily,
    fontFamilyFallback: fontFamilyFallback,
    fontSize: fontSize,
    letterSpacing: letterSpacing,
    wordSpacing: wordSpacing,
    height: height,
    leadingDistribution: leadingDistribution,
    locale: locale,
    background: background as CkPaint?,
    foreground: foreground as CkPaint?,
    shadows: shadows,
    fontFeatures: fontFeatures,
    fontVariations: fontVariations,
  );

  @override
  ui.ParagraphStyle createParagraphStyle({
    ui.TextAlign? textAlign,
    ui.TextDirection? textDirection,
    int? maxLines,
    String? fontFamily,
    double? fontSize,
    double? height,
    ui.TextHeightBehavior? textHeightBehavior,
    ui.FontWeight? fontWeight,
    ui.FontStyle? fontStyle,
    ui.StrutStyle? strutStyle,
    String? ellipsis,
    ui.Locale? locale,
  }) => CkParagraphStyle(
    textAlign: textAlign,
    textDirection: textDirection,
    maxLines: maxLines,
    fontFamily: fontFamily,
    fontSize: fontSize,
    height: height,
    textHeightBehavior: textHeightBehavior,
    fontWeight: fontWeight,
    fontStyle: fontStyle,
    strutStyle: strutStyle,
    ellipsis: ellipsis,
    locale: locale,
  );

  @override
  ui.StrutStyle createStrutStyle({
    String? fontFamily,
    List<String>? fontFamilyFallback,
    double? fontSize,
    double? height,
    ui.TextLeadingDistribution? leadingDistribution,
    double? leading,
    ui.FontWeight? fontWeight,
    ui.FontStyle? fontStyle,
    bool? forceStrutHeight,
  }) => CkStrutStyle(
    fontFamily: fontFamily,
    fontFamilyFallback: fontFamilyFallback,
    fontSize: fontSize,
    height: height,
    leadingDistribution: leadingDistribution,
    leading: leading,
    fontWeight: fontWeight,
    fontStyle: fontStyle,
    forceStrutHeight: forceStrutHeight,
  );

  @override
  ui.ParagraphBuilder createParagraphBuilder(ui.ParagraphStyle style) => CkParagraphBuilder(style);

  // TODO(harryterkelsen): Merge this logic with the async logic in
  // [EngineScene], https://github.com/flutter/flutter/issues/142072.
  @override
  Future<void> renderScene(ui.Scene scene, EngineFlutterView view) async {
    assert(
      _rasterizers.containsKey(view.viewId),
      "Unable to render to a view which hasn't been registered",
    );
    final ViewRasterizer rasterizer = _rasterizers[view.viewId]!;
    final RenderQueue renderQueue = rasterizer.queue;
    final FrameTimingRecorder? recorder =
        FrameTimingRecorder.frameTimingsEnabled ? FrameTimingRecorder() : null;
    if (renderQueue.current != null) {
      // If a scene is already queued up, drop it and queue this one up instead
      // so that the scene view always displays the most recently requested scene.
      renderQueue.next?.completer.complete();
      final Completer<void> completer = Completer<void>();
      renderQueue.next = (scene: scene, completer: completer, recorder: recorder);
      return completer.future;
    }
    final Completer<void> completer = Completer<void>();
    renderQueue.current = (scene: scene, completer: completer, recorder: recorder);
    unawaited(_kickRenderLoop(rasterizer));
    return completer.future;
  }

  Future<void> _kickRenderLoop(ViewRasterizer rasterizer) async {
    final RenderQueue renderQueue = rasterizer.queue;
    final RenderRequest current = renderQueue.current!;
    try {
      await _renderScene(current.scene, rasterizer, current.recorder);
      current.completer.complete();
    } catch (error, stackTrace) {
      current.completer.completeError(error, stackTrace);
    }
    renderQueue.current = renderQueue.next;
    renderQueue.next = null;
    if (renderQueue.current == null) {
      return;
    } else {
      return _kickRenderLoop(rasterizer);
    }
  }

  Future<void> _renderScene(
    ui.Scene scene,
    ViewRasterizer rasterizer,
    FrameTimingRecorder? recorder,
  ) async {
    // "Build finish" and "raster start" happen back-to-back because we
    // render on the same thread, so there's no overhead from hopping to
    // another thread.
    //
    // CanvasKit works differently from the HTML renderer in that in HTML
    // we update the DOM in SceneBuilder.build, which is these function calls
    // here are CanvasKit-only.
    recorder?.recordBuildFinish();
    recorder?.recordRasterStart();

    await rasterizer.draw((scene as LayerScene).layerTree);
    recorder?.recordRasterFinish();
    recorder?.submitTimings();
  }

  // Map from view id to the associated Rasterizer for that view.
  final Map<int, ViewRasterizer> _rasterizers = <int, ViewRasterizer>{};

  void _onViewCreated(int viewId) {
    final EngineFlutterView view = EnginePlatformDispatcher.instance.viewManager[viewId]!;
    _rasterizers[view.viewId] = _rasterizer.createViewRasterizer(view);
  }

  void _onViewDisposed(int viewId) {
    // The view has already been disposed.
    if (!_rasterizers.containsKey(viewId)) {
      return;
    }
    final ViewRasterizer rasterizer = _rasterizers.remove(viewId)!;
    rasterizer.dispose();
  }

  ViewRasterizer? debugGetRasterizerForView(EngineFlutterView view) {
    return _rasterizers[view.viewId];
  }

  /// Disposes this renderer.
  void dispose() {
    _onViewCreatedListener?.cancel();
    _onViewDisposedListener?.cancel();
    for (final ViewRasterizer rasterizer in _rasterizers.values) {
      rasterizer.dispose();
    }
    _rasterizers.clear();
  }

  /// Clears the state of this renderer. Used in tests.
  void debugClear() {
    for (final ViewRasterizer rasterizer in _rasterizers.values) {
      rasterizer.debugClear();
    }
  }

  @override
  void clearFragmentProgramCache() {
    _programs.clear();
  }

  static final Map<String, Future<ui.FragmentProgram>> _programs =
      <String, Future<ui.FragmentProgram>>{};

  @override
  Future<ui.FragmentProgram> createFragmentProgram(String assetKey) {
    if (_programs.containsKey(assetKey)) {
      return _programs[assetKey]!;
    }
    return _programs[assetKey] = ui_web.assetManager.load(assetKey).then((ByteData data) {
      return CkFragmentProgram.fromBytes(assetKey, data.buffer.asUint8List());
    });
  }

  @override
  ui.LineMetrics createLineMetrics({
    required bool hardBreak,
    required double ascent,
    required double descent,
    required double unscaledAscent,
    required double height,
    required double width,
    required double left,
    required double baseline,
    required int lineNumber,
  }) => EngineLineMetrics(
    hardBreak: hardBreak,
    ascent: ascent,
    descent: descent,
    unscaledAscent: unscaledAscent,
    height: height,
    width: width,
    left: left,
    baseline: baseline,
    lineNumber: lineNumber,
  );
}
