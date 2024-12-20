// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:js_interop';
import 'dart:math' as math;
import 'dart:typed_data';

import 'package:ui/src/engine.dart';
import 'package:ui/src/engine/skwasm/skwasm_impl.dart'
    if (dart.library.html) 'package:ui/src/engine/skwasm/skwasm_stub.dart';
import 'package:ui/ui.dart' as ui;
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

final Renderer _renderer = Renderer._internal();
Renderer get renderer => _renderer;

/// This class is an abstraction over the rendering backend for the web engine.
/// Which backend is selected is based off of the `--web-renderer` command-line
/// argument passed to the flutter tool. It provides many of the rendering
/// primitives of the dart:ui library, as well as other backend-specific pieces
/// of functionality needed by the rest of the generic web engine code.
abstract class Renderer {
  factory Renderer._internal() {
    if (FlutterConfiguration.flutterWebUseSkwasm) {
      return SkwasmRenderer();
    } else {
      bool useCanvasKit;
      if (FlutterConfiguration.flutterWebAutoDetect) {
        if (configuration.requestedRendererType != null) {
          useCanvasKit = configuration.requestedRendererType == 'canvaskit';
        } else {
          // If requestedRendererType is not specified, use CanvasKit for desktop and
          // html for mobile.
          useCanvasKit = isDesktop;
        }
      } else {
        useCanvasKit = FlutterConfiguration.useSkia;
      }

      // Warn users in development that anything other than canvaskit is deprecated.
      assert(() {
        if (!useCanvasKit) {
          // The user requested 'html' or 'auto' either in the command-line or JS.
          final String requested =
              configuration.requestedRendererType ??
              (FlutterConfiguration.flutterWebAutoDetect ? 'auto' : 'html');
          printWarning(
            'The HTML Renderer is being deprecated. Stop using the "$requested" renderer mode.'
            '\nSee: https://docs.flutter.dev/to/web-html-renderer-deprecation',
          );
        }
        return true;
      }());
      return useCanvasKit ? CanvasKitRenderer() : HtmlRenderer();
    }
  }

  String get rendererTag;
  FlutterFontCollection get fontCollection;

  FutureOr<void> initialize();

  ui.Paint createPaint();

  ui.Vertices createVertices(
    ui.VertexMode mode,
    List<ui.Offset> positions, {
    List<ui.Offset>? textureCoordinates,
    List<ui.Color>? colors,
    List<int>? indices,
  });
  ui.Vertices createVerticesRaw(
    ui.VertexMode mode,
    Float32List positions, {
    Float32List? textureCoordinates,
    Int32List? colors,
    Uint16List? indices,
  });

  ui.PictureRecorder createPictureRecorder();
  ui.Canvas createCanvas(ui.PictureRecorder recorder, [ui.Rect? cullRect]);
  ui.SceneBuilder createSceneBuilder();

  ui.Gradient createLinearGradient(
    ui.Offset from,
    ui.Offset to,
    List<ui.Color> colors, [
    List<double>? colorStops,
    ui.TileMode tileMode = ui.TileMode.clamp,
    Float32List? matrix4,
  ]);
  ui.Gradient createRadialGradient(
    ui.Offset center,
    double radius,
    List<ui.Color> colors, [
    List<double>? colorStops,
    ui.TileMode tileMode = ui.TileMode.clamp,
    Float32List? matrix4,
  ]);
  ui.Gradient createConicalGradient(
    ui.Offset focal,
    double focalRadius,
    ui.Offset center,
    double radius,
    List<ui.Color> colors, [
    List<double>? colorStops,
    ui.TileMode tileMode = ui.TileMode.clamp,
    Float32List? matrix,
  ]);
  ui.Gradient createSweepGradient(
    ui.Offset center,
    List<ui.Color> colors, [
    List<double>? colorStops,
    ui.TileMode tileMode = ui.TileMode.clamp,
    double startAngle = 0.0,
    double endAngle = math.pi * 2,
    Float32List? matrix4,
  ]);

  ui.ImageFilter createBlurImageFilter({
    double sigmaX = 0.0,
    double sigmaY = 0.0,
    ui.TileMode? tileMode,
  });
  ui.ImageFilter createDilateImageFilter({double radiusX = 0.0, double radiusY = 0.0});
  ui.ImageFilter createErodeImageFilter({double radiusX = 0.0, double radiusY = 0.0});
  ui.ImageFilter createMatrixImageFilter(
    Float64List matrix4, {
    ui.FilterQuality filterQuality = ui.FilterQuality.low,
  });
  ui.ImageFilter composeImageFilters({
    required ui.ImageFilter outer,
    required ui.ImageFilter inner,
  });

  Future<ui.Codec> instantiateImageCodec(
    Uint8List list, {
    int? targetWidth,
    int? targetHeight,
    bool allowUpscaling = true,
  });

  Future<ui.Codec> instantiateImageCodecFromUrl(
    Uri uri, {
    ui_web.ImageCodecChunkCallback? chunkCallback,
  });

  FutureOr<ui.Image> createImageFromImageBitmap(DomImageBitmap imageSource);

  FutureOr<ui.Image> createImageFromTextureSource(
    JSAny object, {
    required int width,
    required int height,
    required bool transferOwnership,
  });

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
  });

  ui.ImageShader createImageShader(
    ui.Image image,
    ui.TileMode tmx,
    ui.TileMode tmy,
    Float64List matrix4,
    ui.FilterQuality? filterQuality,
  );

  void clearFragmentProgramCache();
  Future<ui.FragmentProgram> createFragmentProgram(String assetKey);

  ui.Path createPath();
  ui.Path copyPath(ui.Path src);
  ui.Path combinePaths(ui.PathOperation op, ui.Path path1, ui.Path path2);

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
  });

  ui.TextStyle createTextStyle({
    required ui.Color? color,
    required ui.TextDecoration? decoration,
    required ui.Color? decorationColor,
    required ui.TextDecorationStyle? decorationStyle,
    required double? decorationThickness,
    required ui.FontWeight? fontWeight,
    required ui.FontStyle? fontStyle,
    required ui.TextBaseline? textBaseline,
    required String? fontFamily,
    required List<String>? fontFamilyFallback,
    required double? fontSize,
    required double? letterSpacing,
    required double? wordSpacing,
    required double? height,
    required ui.TextLeadingDistribution? leadingDistribution,
    required ui.Locale? locale,
    required ui.Paint? background,
    required ui.Paint? foreground,
    required List<ui.Shadow>? shadows,
    required List<ui.FontFeature>? fontFeatures,
    required List<ui.FontVariation>? fontVariations,
  });

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
  });

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
  });

  ui.ParagraphBuilder createParagraphBuilder(ui.ParagraphStyle style);

  Future<void> renderScene(ui.Scene scene, EngineFlutterView view);
}
