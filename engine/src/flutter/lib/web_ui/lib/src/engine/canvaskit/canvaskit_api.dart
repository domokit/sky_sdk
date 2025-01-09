// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// Bindings for CanvasKit JavaScript API.
///
/// Prefer keeping the original CanvasKit names so it is easier to locate
/// the API behind these bindings in the Skia source code.
// ignore_for_file: non_constant_identifier_names
@JS()
library canvaskit_api;

import 'dart:async';
import 'dart:convert';
import 'dart:js_interop';
import 'dart:js_util' as js_util;
import 'dart:typed_data';

import 'package:meta/meta.dart';
import 'package:ui/ui.dart' as ui;

import '../browser_detection.dart';
import '../configuration.dart';
import '../dom.dart';
import 'renderer.dart';

/// Entrypoint into the CanvasKit API.
late CanvasKit canvasKit;

bool get _enableCanvasKitChromiumInAutoMode => browserSupportsCanvaskitChromium;

/// Sets the [CanvasKit] object on `window` so we can use `@JS()` to bind to
/// static APIs.
///
/// See, e.g. [SkPaint].
///
/// This also acts as a cache of an initialized CanvasKit instance. We can use
/// this, for example, to perform a hot restart without needing to redownload
/// and reinitialize CanvasKit.
@JS('window.flutterCanvasKit')
external set windowFlutterCanvasKit(CanvasKit? value);

@JS('window.flutterCanvasKit')
external CanvasKit? get windowFlutterCanvasKit;

@JS('window.flutterCanvasKitLoaded')
external JSPromise<JSAny>? get windowFlutterCanvasKitLoaded;

@JS()
@anonymous
@staticInterop
class CanvasKit {}

extension CanvasKitExtension on CanvasKit {
  external SkBlendModeEnum get BlendMode;
  external SkPaintStyleEnum get PaintStyle;
  external SkStrokeCapEnum get StrokeCap;
  external SkStrokeJoinEnum get StrokeJoin;
  external SkBlurStyleEnum get BlurStyle;
  external SkTileModeEnum get TileMode;
  external SkFilterModeEnum get FilterMode;
  external SkMipmapModeEnum get MipmapMode;
  external SkFillTypeEnum get FillType;
  external SkAlphaTypeEnum get AlphaType;
  external SkColorTypeEnum get ColorType;
  external SkPathOpEnum get PathOp;
  external SkClipOpEnum get ClipOp;
  external SkPointModeEnum get PointMode;
  external SkVertexModeEnum get VertexMode;
  external SkRectHeightStyleEnum get RectHeightStyle;
  external SkRectWidthStyleEnum get RectWidthStyle;
  external SkAffinityEnum get Affinity;
  external SkTextAlignEnum get TextAlign;
  external SkTextHeightBehaviorEnum get TextHeightBehavior;
  external SkTextDirectionEnum get TextDirection;
  external SkFontWeightEnum get FontWeight;
  external SkFontSlantEnum get FontSlant;

  @JS('MakeAnimatedImageFromEncoded')
  external SkAnimatedImage? _MakeAnimatedImageFromEncoded(JSUint8Array imageData);
  SkAnimatedImage? MakeAnimatedImageFromEncoded(Uint8List imageData) =>
      _MakeAnimatedImageFromEncoded(imageData.toJS);

  external SkShaderNamespace get Shader;
  external SkMaskFilterNamespace get MaskFilter;
  external SkColorFilterNamespace get ColorFilter;
  external SkImageFilterNamespace get ImageFilter;
  external SkPathNamespace get Path;
  external SkTonalColors computeTonalColors(SkTonalColors inTonalColors);

  @JS('MakeVertices')
  external SkVertices _MakeVertices(
    SkVertexMode mode,
    JSFloat32Array positions,
    JSFloat32Array? textureCoordinates,
    JSUint32Array? colors,
    JSUint16Array? indices,
  );
  SkVertices MakeVertices(
    SkVertexMode mode,
    Float32List positions,
    Float32List? textureCoordinates,
    Uint32List? colors,
    Uint16List? indices,
  ) => _MakeVertices(mode, positions.toJS, textureCoordinates?.toJS, colors?.toJS, indices?.toJS);

  external SkParagraphBuilderNamespace get ParagraphBuilder;
  external SkParagraphStyle ParagraphStyle(SkParagraphStyleProperties properties);
  external SkTextStyle TextStyle(SkTextStyleProperties properties);
  external SkSurface MakeWebGLCanvasSurface(DomCanvasElement canvas);

  @JS('MakeSurface')
  external SkSurface _MakeSurface(JSNumber width, JSNumber height);
  SkSurface MakeSurface(double width, double height) => _MakeSurface(width.toJS, height.toJS);

  @JS('getDataBytes')
  external JSUint8Array _getDataBytes(SkData skData);
  Uint8List getDataBytes(SkData skData) => _getDataBytes(skData).toDart;

  // Text decoration enum is embedded in the CanvasKit object itself.
  @JS('NoDecoration')
  external JSNumber get _NoDecoration;
  double get NoDecoration => _NoDecoration.toDartDouble;

  @JS('UnderlineDecoration')
  external JSNumber get _UnderlineDecoration;
  double get UnderlineDecoration => _UnderlineDecoration.toDartDouble;

  @JS('OverlineDecoration')
  external JSNumber get _OverlineDecoration;
  double get OverlineDecoration => _OverlineDecoration.toDartDouble;

  @JS('LineThroughDecoration')
  external JSNumber get _LineThroughDecoration;
  double get LineThroughDecoration => _LineThroughDecoration.toDartDouble;
  // End of text decoration enum.

  external SkTextDecorationStyleEnum get DecorationStyle;
  external SkTextBaselineEnum get TextBaseline;
  external SkPlaceholderAlignmentEnum get PlaceholderAlignment;

  external SkFontMgrNamespace get FontMgr;
  external TypefaceFontProviderNamespace get TypefaceFontProvider;
  external FontCollectionNamespace get FontCollection;
  external SkTypefaceFactory get Typeface;

  @JS('GetWebGLContext')
  external JSNumber _GetWebGLContext(DomCanvasElement canvas, SkWebGLContextOptions options);
  double GetWebGLContext(DomCanvasElement canvas, SkWebGLContextOptions options) =>
      _GetWebGLContext(canvas, options).toDartDouble;

  @JS('GetWebGLContext')
  external JSNumber _GetOffscreenWebGLContext(
    DomOffscreenCanvas canvas,
    SkWebGLContextOptions options,
  );
  double GetOffscreenWebGLContext(DomOffscreenCanvas canvas, SkWebGLContextOptions options) =>
      _GetOffscreenWebGLContext(canvas, options).toDartDouble;

  @JS('MakeGrContext')
  external SkGrContext _MakeGrContext(JSNumber glContext);
  SkGrContext MakeGrContext(double glContext) => _MakeGrContext(glContext.toJS);

  @JS('MakeOnScreenGLSurface')
  external SkSurface? _MakeOnScreenGLSurface(
    SkGrContext grContext,
    JSNumber width,
    JSNumber height,
    ColorSpace colorSpace,
    JSNumber sampleCount,
    JSNumber stencil,
  );
  SkSurface? MakeOnScreenGLSurface(
    SkGrContext grContext,
    double width,
    double height,
    ColorSpace colorSpace,
    int sampleCount,
    int stencil,
  ) => _MakeOnScreenGLSurface(
    grContext,
    width.toJS,
    height.toJS,
    colorSpace,
    sampleCount.toJS,
    stencil.toJS,
  );

  @JS('MakeRenderTarget')
  external SkSurface? _MakeRenderTarget(SkGrContext grContext, JSNumber width, JSNumber height);
  SkSurface? MakeRenderTarget(SkGrContext grContext, int width, int height) =>
      _MakeRenderTarget(grContext, width.toJS, height.toJS);

  external SkSurface MakeSWCanvasSurface(DomCanvasElement canvas);

  @JS('MakeSWCanvasSurface')
  external SkSurface MakeOffscreenSWCanvasSurface(DomOffscreenCanvas canvas);

  /// Creates an image from decoded pixels represented as a list of bytes.
  ///
  /// The pixel data must be encoded according to the image info in [info].
  ///
  /// Typically pixel data is obtained using [SkImage.readPixels]. The
  /// parameters specified in [SkImageInfo] passed [SkImage.readPixels] must
  /// match [info].
  @JS('MakeImage')
  external SkImage? _MakeImage(SkImageInfo info, JSUint8Array pixels, JSNumber bytesPerRow);
  SkImage? MakeImage(SkImageInfo info, Uint8List pixels, double bytesPerRow) =>
      _MakeImage(info, pixels.toJS, bytesPerRow.toJS);

  @JS('MakeLazyImageFromTextureSource')
  external SkImage? _MakeLazyImageFromTextureSource2(JSAny src, SkPartialImageInfo info);

  @JS('MakeLazyImageFromTextureSource')
  external SkImage? _MakeLazyImageFromTextureSource3(
    JSAny src,
    JSNumber zeroSecondArgument,
    JSBoolean srcIsPremultiplied,
  );

  SkImage? MakeLazyImageFromTextureSourceWithInfo(Object src, SkPartialImageInfo info) =>
      _MakeLazyImageFromTextureSource2(src.toJSAnyShallow, info);

  SkImage? MakeLazyImageFromImageBitmap(DomImageBitmap imageBitmap, bool hasPremultipliedAlpha) =>
      _MakeLazyImageFromTextureSource3(imageBitmap as JSObject, 0.toJS, hasPremultipliedAlpha.toJS);
}

@JS()
@staticInterop
class CanvasKitModule {}

extension CanvasKitModuleExtension on CanvasKitModule {
  @JS('default')
  external JSPromise<JSAny> defaultExport(CanvasKitInitOptions options);
}

typedef LocateFileCallback = String Function(String file, String unusedBase);

JSFunction createLocateFileCallback(LocateFileCallback callback) => callback.toJS;

@JS()
@anonymous
@staticInterop
class CanvasKitInitOptions {
  external factory CanvasKitInitOptions({required JSFunction locateFile});
}

@JS('window.flutterCanvasKit.ColorSpace.SRGB')
external ColorSpace get SkColorSpaceSRGB;

@JS()
@staticInterop
class ColorSpace {}

@JS()
@anonymous
@staticInterop
class SkWebGLContextOptions {
  factory SkWebGLContextOptions({
    required double antialias,
    // WebGL version: 1 or 2.
    required double majorVersion,
  }) => SkWebGLContextOptions._(antialias: antialias.toJS, majorVersion: majorVersion.toJS);
  external factory SkWebGLContextOptions._({
    required JSNumber antialias,
    // WebGL version: 1 or 2.
    required JSNumber majorVersion,
  });
}

@JS('window.flutterCanvasKit.Surface')
@staticInterop
class SkSurface {}

extension SkSurfaceExtension on SkSurface {
  external SkCanvas getCanvas();
  external JSVoid flush();

  @JS('width')
  external JSNumber _width();
  double width() => _width().toDartDouble;

  @JS('height')
  external JSNumber _height();
  double height() => _height().toDartDouble;

  external JSVoid dispose();
  external SkImage makeImageSnapshot();
}

@JS()
@staticInterop
class SkGrContext {}

extension SkGrContextExtension on SkGrContext {
  @JS('setResourceCacheLimitBytes')
  external JSVoid _setResourceCacheLimitBytes(JSNumber limit);
  void setResourceCacheLimitBytes(double limit) => _setResourceCacheLimitBytes(limit.toJS);

  external JSVoid releaseResourcesAndAbandonContext();
  external JSVoid delete();
}

@JS()
@anonymous
@staticInterop
class SkFontSlantEnum {}

extension SkFontSlantEnumExtension on SkFontSlantEnum {
  external SkFontSlant get Upright;
  external SkFontSlant get Italic;
}

@JS('window.flutterCanvasKit.FontSlant')
@staticInterop
class SkFontSlant {}

extension SkFontSlantExtension on SkFontSlant {
  @JS('value')
  external JSNumber get _value;
  double get value => _value.toDartDouble;
}

final List<SkFontSlant> _skFontSlants = <SkFontSlant>[
  canvasKit.FontSlant.Upright,
  canvasKit.FontSlant.Italic,
];

SkFontSlant toSkFontSlant(ui.FontStyle style) {
  return _skFontSlants[style.index];
}

@JS()
@anonymous
@staticInterop
class SkFontWeightEnum {}

extension SkFontWeightEnumExtension on SkFontWeightEnum {
  external SkFontWeight get Thin;
  external SkFontWeight get ExtraLight;
  external SkFontWeight get Light;
  external SkFontWeight get Normal;
  external SkFontWeight get Medium;
  external SkFontWeight get SemiBold;
  external SkFontWeight get Bold;
  external SkFontWeight get ExtraBold;
  external SkFontWeight get ExtraBlack;
}

@JS()
@staticInterop
class SkFontWeight {}

extension SkFontWeightExtension on SkFontWeight {
  @JS('value')
  external JSNumber get _value;
  double get value => _value.toDartDouble;
}

final List<SkFontWeight> _skFontWeights = <SkFontWeight>[
  canvasKit.FontWeight.Thin,
  canvasKit.FontWeight.ExtraLight,
  canvasKit.FontWeight.Light,
  canvasKit.FontWeight.Normal,
  canvasKit.FontWeight.Medium,
  canvasKit.FontWeight.SemiBold,
  canvasKit.FontWeight.Bold,
  canvasKit.FontWeight.ExtraBold,
  canvasKit.FontWeight.ExtraBlack,
];

SkFontWeight toSkFontWeight(ui.FontWeight weight) {
  return _skFontWeights[weight.index];
}

@JS()
@staticInterop
class SkAffinityEnum {}

extension SkAffinityEnumExtension on SkAffinityEnum {
  external SkAffinity get Upstream;
  external SkAffinity get Downstream;
}

@JS()
@staticInterop
class SkAffinity {}

extension SkAffinityExtension on SkAffinity {
  @JS('value')
  external JSNumber get _value;
  double get value => _value.toDartDouble;
}

final List<SkAffinity> _skAffinitys = <SkAffinity>[
  canvasKit.Affinity.Upstream,
  canvasKit.Affinity.Downstream,
];

SkAffinity toSkAffinity(ui.TextAffinity affinity) {
  return _skAffinitys[affinity.index];
}

@JS()
@staticInterop
class SkTextDirectionEnum {}

extension SkTextDirectionEnumExtension on SkTextDirectionEnum {
  external SkTextDirection get RTL;
  external SkTextDirection get LTR;
}

@JS()
@staticInterop
class SkTextDirection {}

extension SkTextDirectionExtension on SkTextDirection {
  @JS('value')
  external JSNumber get _value;
  double get value => _value.toDartDouble;
}

// Flutter enumerates text directions as RTL, LTR, while CanvasKit
// enumerates them LTR, RTL.
final List<SkTextDirection> _skTextDirections = <SkTextDirection>[
  canvasKit.TextDirection.RTL,
  canvasKit.TextDirection.LTR,
];

SkTextDirection toSkTextDirection(ui.TextDirection direction) {
  return _skTextDirections[direction.index];
}

@JS()
@staticInterop
class SkTextAlignEnum {}

extension SkTextAlignEnumExtension on SkTextAlignEnum {
  external SkTextAlign get Left;
  external SkTextAlign get Right;
  external SkTextAlign get Center;
  external SkTextAlign get Justify;
  external SkTextAlign get Start;
  external SkTextAlign get End;
}

@JS()
@staticInterop
class SkTextAlign {}

extension SkTextAlignExtension on SkTextAlign {
  @JS('value')
  external JSNumber get _value;
  double get value => _value.toDartDouble;
}

final List<SkTextAlign> _skTextAligns = <SkTextAlign>[
  canvasKit.TextAlign.Left,
  canvasKit.TextAlign.Right,
  canvasKit.TextAlign.Center,
  canvasKit.TextAlign.Justify,
  canvasKit.TextAlign.Start,
  canvasKit.TextAlign.End,
];

SkTextAlign toSkTextAlign(ui.TextAlign align) {
  return _skTextAligns[align.index];
}

@JS()
@staticInterop
class SkTextHeightBehaviorEnum {}

extension SkTextHeightBehaviorEnumExtension on SkTextHeightBehaviorEnum {
  external SkTextHeightBehavior get All;
  external SkTextHeightBehavior get DisableFirstAscent;
  external SkTextHeightBehavior get DisableLastDescent;
  external SkTextHeightBehavior get DisableAll;
}

@JS()
@staticInterop
class SkTextHeightBehavior {}

extension SkTextHeightBehaviorExtension on SkTextHeightBehavior {
  @JS('value')
  external JSNumber get _value;
  double get value => _value.toDartDouble;
}

final List<SkTextHeightBehavior> _skTextHeightBehaviors = <SkTextHeightBehavior>[
  canvasKit.TextHeightBehavior.All,
  canvasKit.TextHeightBehavior.DisableFirstAscent,
  canvasKit.TextHeightBehavior.DisableLastDescent,
  canvasKit.TextHeightBehavior.DisableAll,
];

SkTextHeightBehavior toSkTextHeightBehavior(ui.TextHeightBehavior behavior) {
  final int index =
      (behavior.applyHeightToFirstAscent ? 0 : 1 << 0) |
      (behavior.applyHeightToLastDescent ? 0 : 1 << 1);
  return _skTextHeightBehaviors[index];
}

@JS()
@staticInterop
class SkRectHeightStyleEnum {}

extension SkRectHeightStyleEnumExtension on SkRectHeightStyleEnum {
  external SkRectHeightStyle get Tight;
  external SkRectHeightStyle get Max;
  external SkRectHeightStyle get IncludeLineSpacingMiddle;
  external SkRectHeightStyle get IncludeLineSpacingTop;
  external SkRectHeightStyle get IncludeLineSpacingBottom;
  external SkRectHeightStyle get Strut;
}

@JS()
@staticInterop
class SkRectHeightStyle {}

extension SkRectHeightStyleExtension on SkRectHeightStyle {
  @JS('value')
  external JSNumber get _value;
  double get value => _value.toDartDouble;
}

final List<SkRectHeightStyle> _skRectHeightStyles = <SkRectHeightStyle>[
  canvasKit.RectHeightStyle.Tight,
  canvasKit.RectHeightStyle.Max,
  canvasKit.RectHeightStyle.IncludeLineSpacingMiddle,
  canvasKit.RectHeightStyle.IncludeLineSpacingTop,
  canvasKit.RectHeightStyle.IncludeLineSpacingBottom,
  canvasKit.RectHeightStyle.Strut,
];

SkRectHeightStyle toSkRectHeightStyle(ui.BoxHeightStyle style) {
  return _skRectHeightStyles[style.index];
}

@JS()
@staticInterop
class SkRectWidthStyleEnum {}

extension SkRectWidthStyleEnumExtension on SkRectWidthStyleEnum {
  external SkRectWidthStyle get Tight;
  external SkRectWidthStyle get Max;
}

@JS()
@staticInterop
class SkRectWidthStyle {}

extension SkRectWidthStyleExtension on SkRectWidthStyle {
  @JS('value')
  external JSNumber get _value;
  double get value => _value.toDartDouble;
}

final List<SkRectWidthStyle> _skRectWidthStyles = <SkRectWidthStyle>[
  canvasKit.RectWidthStyle.Tight,
  canvasKit.RectWidthStyle.Max,
];

SkRectWidthStyle toSkRectWidthStyle(ui.BoxWidthStyle style) {
  final int index = style.index;
  return _skRectWidthStyles[index < 2 ? index : 0];
}

@JS()
@staticInterop
class SkVertexModeEnum {}

extension SkVertexModeEnumExtension on SkVertexModeEnum {
  external SkVertexMode get Triangles;
  external SkVertexMode get TrianglesStrip;
  external SkVertexMode get TriangleFan;
}

@JS()
@staticInterop
class SkVertexMode {}

extension SkVertexModeExtension on SkVertexMode {
  @JS('value')
  external JSNumber get _value;
  double get value => _value.toDartDouble;
}

final List<SkVertexMode> _skVertexModes = <SkVertexMode>[
  canvasKit.VertexMode.Triangles,
  canvasKit.VertexMode.TrianglesStrip,
  canvasKit.VertexMode.TriangleFan,
];

SkVertexMode toSkVertexMode(ui.VertexMode mode) {
  return _skVertexModes[mode.index];
}

@JS()
@staticInterop
class SkPointModeEnum {}

extension SkPointModeEnumExtension on SkPointModeEnum {
  external SkPointMode get Points;
  external SkPointMode get Lines;
  external SkPointMode get Polygon;
}

@JS()
@staticInterop
class SkPointMode {}

extension SkPointModeExtension on SkPointMode {
  @JS('value')
  external JSNumber get _value;
  double get value => _value.toDartDouble;
}

final List<SkPointMode> _skPointModes = <SkPointMode>[
  canvasKit.PointMode.Points,
  canvasKit.PointMode.Lines,
  canvasKit.PointMode.Polygon,
];

SkPointMode toSkPointMode(ui.PointMode mode) {
  return _skPointModes[mode.index];
}

@JS()
@staticInterop
class SkClipOpEnum {}

extension SkClipOpEnumExtension on SkClipOpEnum {
  external SkClipOp get Difference;
  external SkClipOp get Intersect;
}

@JS()
@staticInterop
class SkClipOp {}

extension SkClipOpExtension on SkClipOp {
  @JS('value')
  external JSNumber get _value;
  double get value => _value.toDartDouble;
}

final List<SkClipOp> _skClipOps = <SkClipOp>[
  canvasKit.ClipOp.Difference,
  canvasKit.ClipOp.Intersect,
];

SkClipOp toSkClipOp(ui.ClipOp clipOp) {
  return _skClipOps[clipOp.index];
}

@JS()
@staticInterop
class SkFillTypeEnum {}

extension SkFillTypeEnumExtension on SkFillTypeEnum {
  external SkFillType get Winding;
  external SkFillType get EvenOdd;
}

@JS()
@staticInterop
class SkFillType {}

extension SkFillTypeExtension on SkFillType {
  @JS('value')
  external JSNumber get _value;
  double get value => _value.toDartDouble;
}

final List<SkFillType> _skFillTypes = <SkFillType>[
  canvasKit.FillType.Winding,
  canvasKit.FillType.EvenOdd,
];

SkFillType toSkFillType(ui.PathFillType fillType) {
  return _skFillTypes[fillType.index];
}

@JS()
@staticInterop
class SkPathOpEnum {}

extension SkPathOpEnumExtension on SkPathOpEnum {
  external SkPathOp get Difference;
  external SkPathOp get Intersect;
  external SkPathOp get Union;
  external SkPathOp get XOR;
  external SkPathOp get ReverseDifference;
}

@JS()
@staticInterop
class SkPathOp {}

extension SkPathOpExtension on SkPathOp {
  @JS('value')
  external JSNumber get _value;
  double get value => _value.toDartDouble;
}

final List<SkPathOp> _skPathOps = <SkPathOp>[
  canvasKit.PathOp.Difference,
  canvasKit.PathOp.Intersect,
  canvasKit.PathOp.Union,
  canvasKit.PathOp.XOR,
  canvasKit.PathOp.ReverseDifference,
];

SkPathOp toSkPathOp(ui.PathOperation pathOp) {
  return _skPathOps[pathOp.index];
}

@JS()
@staticInterop
class SkBlurStyleEnum {}

extension SkBlurStyleEnumExtension on SkBlurStyleEnum {
  external SkBlurStyle get Normal;
  external SkBlurStyle get Solid;
  external SkBlurStyle get Outer;
  external SkBlurStyle get Inner;
}

@JS()
@staticInterop
class SkBlurStyle {}

extension SkBlurStyleExtension on SkBlurStyle {
  @JS('value')
  external JSNumber get _value;
  double get value => _value.toDartDouble;
}

final List<SkBlurStyle> _skBlurStyles = <SkBlurStyle>[
  canvasKit.BlurStyle.Normal,
  canvasKit.BlurStyle.Solid,
  canvasKit.BlurStyle.Outer,
  canvasKit.BlurStyle.Inner,
];

SkBlurStyle toSkBlurStyle(ui.BlurStyle style) {
  return _skBlurStyles[style.index];
}

@JS()
@staticInterop
class SkStrokeCapEnum {}

extension SkStrokeCapEnumExtension on SkStrokeCapEnum {
  external SkStrokeCap get Butt;
  external SkStrokeCap get Round;
  external SkStrokeCap get Square;
}

@JS()
@staticInterop
class SkStrokeCap {}

extension SkStrokeCapExtension on SkStrokeCap {
  @JS('value')
  external JSNumber get _value;
  double get value => _value.toDartDouble;
}

final List<SkStrokeCap> _skStrokeCaps = <SkStrokeCap>[
  canvasKit.StrokeCap.Butt,
  canvasKit.StrokeCap.Round,
  canvasKit.StrokeCap.Square,
];

SkStrokeCap toSkStrokeCap(ui.StrokeCap strokeCap) {
  return _skStrokeCaps[strokeCap.index];
}

@JS()
@staticInterop
class SkPaintStyleEnum {}

extension SkPaintStyleEnumExtension on SkPaintStyleEnum {
  external SkPaintStyle get Stroke;
  external SkPaintStyle get Fill;
}

@JS()
@staticInterop
class SkPaintStyle {}

extension SkPaintStyleExtension on SkPaintStyle {
  @JS('value')
  external JSNumber get _value;
  double get value => _value.toDartDouble;
}

final List<SkPaintStyle> _skPaintStyles = <SkPaintStyle>[
  canvasKit.PaintStyle.Fill,
  canvasKit.PaintStyle.Stroke,
];

SkPaintStyle toSkPaintStyle(ui.PaintingStyle paintStyle) {
  return _skPaintStyles[paintStyle.index];
}

@JS()
@staticInterop
class SkBlendModeEnum {}

extension SkBlendModeEnumExtension on SkBlendModeEnum {
  external SkBlendMode get Clear;
  external SkBlendMode get Src;
  external SkBlendMode get Dst;
  external SkBlendMode get SrcOver;
  external SkBlendMode get DstOver;
  external SkBlendMode get SrcIn;
  external SkBlendMode get DstIn;
  external SkBlendMode get SrcOut;
  external SkBlendMode get DstOut;
  external SkBlendMode get SrcATop;
  external SkBlendMode get DstATop;
  external SkBlendMode get Xor;
  external SkBlendMode get Plus;
  external SkBlendMode get Modulate;
  external SkBlendMode get Screen;
  external SkBlendMode get Overlay;
  external SkBlendMode get Darken;
  external SkBlendMode get Lighten;
  external SkBlendMode get ColorDodge;
  external SkBlendMode get ColorBurn;
  external SkBlendMode get HardLight;
  external SkBlendMode get SoftLight;
  external SkBlendMode get Difference;
  external SkBlendMode get Exclusion;
  external SkBlendMode get Multiply;
  external SkBlendMode get Hue;
  external SkBlendMode get Saturation;
  external SkBlendMode get Color;
  external SkBlendMode get Luminosity;
}

@JS()
@staticInterop
class SkBlendMode {}

extension SkBlendModeExtension on SkBlendMode {
  @JS('value')
  external JSNumber get _value;
  double get value => _value.toDartDouble;
}

final List<SkBlendMode> _skBlendModes = <SkBlendMode>[
  canvasKit.BlendMode.Clear,
  canvasKit.BlendMode.Src,
  canvasKit.BlendMode.Dst,
  canvasKit.BlendMode.SrcOver,
  canvasKit.BlendMode.DstOver,
  canvasKit.BlendMode.SrcIn,
  canvasKit.BlendMode.DstIn,
  canvasKit.BlendMode.SrcOut,
  canvasKit.BlendMode.DstOut,
  canvasKit.BlendMode.SrcATop,
  canvasKit.BlendMode.DstATop,
  canvasKit.BlendMode.Xor,
  canvasKit.BlendMode.Plus,
  canvasKit.BlendMode.Modulate,
  canvasKit.BlendMode.Screen,
  canvasKit.BlendMode.Overlay,
  canvasKit.BlendMode.Darken,
  canvasKit.BlendMode.Lighten,
  canvasKit.BlendMode.ColorDodge,
  canvasKit.BlendMode.ColorBurn,
  canvasKit.BlendMode.HardLight,
  canvasKit.BlendMode.SoftLight,
  canvasKit.BlendMode.Difference,
  canvasKit.BlendMode.Exclusion,
  canvasKit.BlendMode.Multiply,
  canvasKit.BlendMode.Hue,
  canvasKit.BlendMode.Saturation,
  canvasKit.BlendMode.Color,
  canvasKit.BlendMode.Luminosity,
];

SkBlendMode toSkBlendMode(ui.BlendMode blendMode) {
  return _skBlendModes[blendMode.index];
}

@JS()
@staticInterop
class SkStrokeJoinEnum {}

extension SkStrokeJoinEnumExtension on SkStrokeJoinEnum {
  external SkStrokeJoin get Miter;
  external SkStrokeJoin get Round;
  external SkStrokeJoin get Bevel;
}

@JS()
@staticInterop
class SkStrokeJoin {}

extension SkStrokeJoinExtension on SkStrokeJoin {
  @JS('value')
  external JSNumber get _value;
  double get value => _value.toDartDouble;
}

final List<SkStrokeJoin> _skStrokeJoins = <SkStrokeJoin>[
  canvasKit.StrokeJoin.Miter,
  canvasKit.StrokeJoin.Round,
  canvasKit.StrokeJoin.Bevel,
];

SkStrokeJoin toSkStrokeJoin(ui.StrokeJoin strokeJoin) {
  return _skStrokeJoins[strokeJoin.index];
}

@JS()
@staticInterop
class SkTileModeEnum {}

extension SkTileModeEnumExtension on SkTileModeEnum {
  external SkTileMode get Clamp;
  external SkTileMode get Repeat;
  external SkTileMode get Mirror;
  external SkTileMode get Decal;
}

@JS()
@staticInterop
class SkTileMode {}

extension SkTileModeExtension on SkTileMode {
  @JS('value')
  external JSNumber get _value;
  double get value => _value.toDartDouble;
}

final List<SkTileMode> _skTileModes = <SkTileMode>[
  canvasKit.TileMode.Clamp,
  canvasKit.TileMode.Repeat,
  canvasKit.TileMode.Mirror,
  canvasKit.TileMode.Decal,
];

SkTileMode toSkTileMode(ui.TileMode? mode) {
  return mode == null ? canvasKit.TileMode.Clamp : _skTileModes[mode.index];
}

@JS()
@staticInterop
class SkFilterModeEnum {}

extension SkFilterModeEnumExtension on SkFilterModeEnum {
  external SkFilterMode get Nearest;
  external SkFilterMode get Linear;
}

@JS()
@staticInterop
class SkFilterMode {}

extension SkFilterModeExtension on SkFilterMode {
  @JS('value')
  external JSNumber get _value;
  double get value => _value.toDartDouble;
}

SkFilterMode toSkFilterMode(ui.FilterQuality filterQuality) {
  return filterQuality == ui.FilterQuality.none
      ? canvasKit.FilterMode.Nearest
      : canvasKit.FilterMode.Linear;
}

@JS()
@staticInterop
class SkMipmapModeEnum {}

extension SkMipmapModeEnumExtension on SkMipmapModeEnum {
  external SkMipmapMode get None;
  external SkMipmapMode get Nearest;
  external SkMipmapMode get Linear;
}

@JS()
@staticInterop
class SkMipmapMode {}

extension SkMipmapModeExtension on SkMipmapMode {
  @JS('value')
  external JSNumber get _value;
  double get value => _value.toDartDouble;
}

SkMipmapMode toSkMipmapMode(ui.FilterQuality filterQuality) {
  return filterQuality == ui.FilterQuality.medium
      ? canvasKit.MipmapMode.Linear
      : canvasKit.MipmapMode.None;
}

@JS()
@staticInterop
class SkAlphaTypeEnum {}

extension SkAlphaTypeEnumExtension on SkAlphaTypeEnum {
  external SkAlphaType get Opaque;
  external SkAlphaType get Premul;
  external SkAlphaType get Unpremul;
}

@JS()
@staticInterop
class SkAlphaType {}

extension SkAlphaTypeExtension on SkAlphaType {
  @JS('value')
  external JSNumber get _value;
  double get value => _value.toDartDouble;
}

@JS()
@staticInterop
class SkColorTypeEnum {}

extension SkColorTypeEnumExtension on SkColorTypeEnum {
  external SkColorType get Alpha_8;
  external SkColorType get RGB_565;
  external SkColorType get ARGB_4444;
  external SkColorType get RGBA_8888;
  external SkColorType get RGB_888x;
  external SkColorType get BGRA_8888;
  external SkColorType get RGBA_1010102;
  external SkColorType get RGB_101010x;
  external SkColorType get Gray_8;
  external SkColorType get RGBA_F16;
  external SkColorType get RGBA_F32;
}

@JS()
@staticInterop
class SkColorType {}

extension SkColorTypeExtension on SkColorType {
  @JS('value')
  external JSNumber get _value;
  double get value => _value.toDartDouble;
}

@JS()
@anonymous
@staticInterop
class SkAnimatedImage {}

extension SkAnimatedImageExtension on SkAnimatedImage {
  @JS('getFrameCount')
  external JSNumber _getFrameCount();
  double getFrameCount() => _getFrameCount().toDartDouble;

  @JS('getRepetitionCount')
  external JSNumber _getRepetitionCount();
  double getRepetitionCount() => _getRepetitionCount().toDartDouble;

  /// Returns duration in milliseconds.
  @JS('currentFrameDuration')
  external JSNumber _currentFrameDuration();
  double currentFrameDuration() => _currentFrameDuration().toDartDouble;

  /// Advances to the next frame and returns its duration in milliseconds.
  @JS('decodeNextFrame')
  external JSNumber _decodeNextFrame();
  double decodeNextFrame() => _decodeNextFrame().toDartDouble;

  external SkImage makeImageAtCurrentFrame();

  @JS('width')
  external JSNumber _width();
  double width() => _width().toDartDouble;

  @JS('height')
  external JSNumber _height();
  double height() => _height().toDartDouble;

  /// Deletes the C++ object.
  ///
  /// This object is no longer usable after calling this method.
  external JSVoid delete();

  @JS('isDeleted')
  external JSBoolean _isDeleted();
  bool isDeleted() => _isDeleted().toDart;
}

@JS()
@anonymous
@staticInterop
class SkImage {}

extension SkImageExtension on SkImage {
  external JSVoid delete();

  @JS('width')
  external JSNumber _width();
  double width() => _width().toDartDouble;

  @JS('height')
  external JSNumber _height();
  double height() => _height().toDartDouble;

  @JS('makeShaderCubic')
  external SkShader _makeShaderCubic(
    SkTileMode tileModeX,
    SkTileMode tileModeY,
    JSNumber B,
    JSNumber C,
    JSFloat32Array? matrix, // 3x3 matrix
  );
  SkShader makeShaderCubic(
    SkTileMode tileModeX,
    SkTileMode tileModeY,
    double B,
    double C,
    Float32List? matrix, // 3x3 matrix
  ) => _makeShaderCubic(tileModeX, tileModeY, B.toJS, C.toJS, matrix?.toJS);

  @JS('makeShaderOptions')
  external SkShader _makeShaderOptions(
    SkTileMode tileModeX,
    SkTileMode tileModeY,
    SkFilterMode filterMode,
    SkMipmapMode mipmapMode,
    JSFloat32Array? matrix, // 3x3 matrix
  );
  SkShader makeShaderOptions(
    SkTileMode tileModeX,
    SkTileMode tileModeY,
    SkFilterMode filterMode,
    SkMipmapMode mipmapMode,
    Float32List? matrix, // 3x3 matrix
  ) => _makeShaderOptions(tileModeX, tileModeY, filterMode, mipmapMode, matrix?.toJS);

  @JS('readPixels')
  external JSUint8Array? _readPixels(JSNumber srcX, JSNumber srcY, SkImageInfo imageInfo);
  Uint8List? readPixels(double srcX, double srcY, SkImageInfo imageInfo) =>
      _readPixels(srcX.toJS, srcY.toJS, imageInfo)?.toDart;

  @JS('encodeToBytes')
  external JSUint8Array? _encodeToBytes();
  Uint8List? encodeToBytes() => _encodeToBytes()?.toDart;

  @JS('isAliasOf')
  external JSBoolean _isAliasOf(SkImage other);
  bool isAliasOf(SkImage other) => _isAliasOf(other).toDart;

  @JS('isDeleted')
  external JSBoolean _isDeleted();
  bool isDeleted() => _isDeleted().toDart;
}

@JS()
@staticInterop
class SkShaderNamespace {}

extension SkShaderNamespaceExtension on SkShaderNamespace {
  @JS('MakeLinearGradient')
  external SkShader _MakeLinearGradient(
    JSFloat32Array from, // 2-element array
    JSFloat32Array to, // 2-element array
    JSUint32Array colors,
    JSFloat32Array colorStops,
    SkTileMode tileMode,
    JSFloat32Array? matrix,
  );
  SkShader MakeLinearGradient(
    Float32List from, // 2-element array
    Float32List to, // 2-element array
    Uint32List colors,
    Float32List colorStops,
    SkTileMode tileMode,
    Float32List? matrix,
  ) =>
      _MakeLinearGradient(from.toJS, to.toJS, colors.toJS, colorStops.toJS, tileMode, matrix?.toJS);

  @JS('MakeRadialGradient')
  external SkShader _MakeRadialGradient(
    JSFloat32Array center, // 2-element array
    JSNumber radius,
    JSUint32Array colors,
    JSFloat32Array colorStops,
    SkTileMode tileMode,
    JSFloat32Array? matrix, // 3x3 matrix
    JSNumber flags,
  );
  SkShader MakeRadialGradient(
    Float32List center, // 2-element array
    double radius,
    Uint32List colors,
    Float32List colorStops,
    SkTileMode tileMode,
    Float32List? matrix, // 3x3 matrix
    double flags,
  ) => _MakeRadialGradient(
    center.toJS,
    radius.toJS,
    colors.toJS,
    colorStops.toJS,
    tileMode,
    matrix?.toJS,
    flags.toJS,
  );

  @JS('MakeTwoPointConicalGradient')
  external SkShader _MakeTwoPointConicalGradient(
    JSFloat32Array focal,
    JSNumber focalRadius,
    JSFloat32Array center,
    JSNumber radius,
    JSUint32Array colors,
    JSFloat32Array colorStops,
    SkTileMode tileMode,
    JSFloat32Array? matrix, // 3x3 matrix
    JSNumber flags,
  );
  SkShader MakeTwoPointConicalGradient(
    Float32List focal,
    double focalRadius,
    Float32List center,
    double radius,
    Uint32List colors,
    Float32List colorStops,
    SkTileMode tileMode,
    Float32List? matrix, // 3x3 matrix
    double flags,
  ) => _MakeTwoPointConicalGradient(
    focal.toJS,
    focalRadius.toJS,
    center.toJS,
    radius.toJS,
    colors.toJS,
    colorStops.toJS,
    tileMode,
    matrix?.toJS,
    flags.toJS,
  );

  @JS('MakeSweepGradient')
  external SkShader _MakeSweepGradient(
    JSNumber cx,
    JSNumber cy,
    JSUint32Array colors,
    JSFloat32Array colorStops,
    SkTileMode tileMode,
    JSFloat32Array? matrix, // 3x3 matrix
    JSNumber flags,
    JSNumber startAngle,
    JSNumber endAngle,
  );
  SkShader MakeSweepGradient(
    double cx,
    double cy,
    Uint32List colors,
    Float32List colorStops,
    SkTileMode tileMode,
    Float32List? matrix, // 3x3 matrix
    double flags,
    double startAngle,
    double endAngle,
  ) => _MakeSweepGradient(
    cx.toJS,
    cy.toJS,
    colors.toJS,
    colorStops.toJS,
    tileMode,
    matrix?.toJS,
    flags.toJS,
    startAngle.toJS,
    endAngle.toJS,
  );
}

@JS()
@anonymous
@staticInterop
class SkShader {}

extension SkShaderExtension on SkShader {
  external JSVoid delete();
}

@JS()
@staticInterop
class SkMaskFilterNamespace {}

extension SkMaskFilterNamespaceExtension on SkMaskFilterNamespace {
  // Creates a blur MaskFilter.
  //
  // Returns `null` if [sigma] is 0 or infinite.
  @JS('MakeBlur')
  external SkMaskFilter? _MakeBlur(SkBlurStyle blurStyle, JSNumber sigma, JSBoolean respectCTM);
  SkMaskFilter? MakeBlur(SkBlurStyle blurStyle, double sigma, bool respectCTM) =>
      _MakeBlur(blurStyle, sigma.toJS, respectCTM.toJS);
}

// This needs to be bound to top-level because SkPaint is initialized
// with `new`. Also in Dart you can't write this:
//
//     external SkPaint SkPaint();
@JS('window.flutterCanvasKit.Paint')
@staticInterop
class SkPaint {
  external factory SkPaint();
}

extension SkPaintExtension on SkPaint {
  external JSVoid setBlendMode(SkBlendMode blendMode);
  external JSVoid setStyle(SkPaintStyle paintStyle);

  @JS('setStrokeWidth')
  external JSVoid _setStrokeWidth(JSNumber width);
  JSVoid setStrokeWidth(double width) => _setStrokeWidth(width.toJS);

  external JSVoid setStrokeCap(SkStrokeCap cap);
  external JSVoid setStrokeJoin(SkStrokeJoin join);

  @JS('setAntiAlias')
  external JSVoid _setAntiAlias(JSBoolean isAntiAlias);
  void setAntiAlias(bool isAntiAlias) => _setAntiAlias(isAntiAlias.toJS);

  @JS('setColorInt')
  external JSVoid _setColorInt(JSNumber color);
  void setColorInt(int color) => _setColorInt(color.toJS);

  external JSVoid setShader(SkShader? shader);
  external JSVoid setMaskFilter(SkMaskFilter? maskFilter);
  external JSVoid setColorFilter(SkColorFilter? colorFilter);

  @JS('setStrokeMiter')
  external JSVoid _setStrokeMiter(JSNumber miterLimit);
  void setStrokeMiter(double miterLimit) => _setStrokeMiter(miterLimit.toJS);

  external JSVoid setImageFilter(SkImageFilter? imageFilter);
  external JSVoid delete();
}

@JS()
@anonymous
@staticInterop
abstract class CkFilterOptions {}

@JS()
@anonymous
@staticInterop
class _CkCubicFilterOptions extends CkFilterOptions {
  external factory _CkCubicFilterOptions({required JSNumber B, required JSNumber C});
}

@JS()
@anonymous
@staticInterop
class _CkTransformFilterOptions extends CkFilterOptions {
  external factory _CkTransformFilterOptions({SkFilterMode filter, SkMipmapMode mipmap});
}

final Map<ui.FilterQuality, CkFilterOptions> _filterOptions = <ui.FilterQuality, CkFilterOptions>{
  ui.FilterQuality.none: _CkTransformFilterOptions(
    filter: canvasKit.FilterMode.Nearest,
    mipmap: canvasKit.MipmapMode.None,
  ),
  ui.FilterQuality.low: _CkTransformFilterOptions(
    filter: canvasKit.FilterMode.Linear,
    mipmap: canvasKit.MipmapMode.None,
  ),
  ui.FilterQuality.medium: _CkTransformFilterOptions(
    filter: canvasKit.FilterMode.Linear,
    mipmap: canvasKit.MipmapMode.Linear,
  ),
  ui.FilterQuality.high: _CkCubicFilterOptions(B: (1.0 / 3).toJS, C: (1.0 / 3).toJS),
};

CkFilterOptions toSkFilterOptions(ui.FilterQuality filterQuality) {
  return _filterOptions[filterQuality]!;
}

@JS()
@anonymous
@staticInterop
class SkMaskFilter {}

extension SkMaskFilterExtension on SkMaskFilter {
  external JSVoid delete();
}

@JS()
@staticInterop
class SkColorFilterNamespace {}

extension SkColorFilterNamespaceExtension on SkColorFilterNamespace {
  @JS('MakeBlend')
  external SkColorFilter? _MakeBlend(JSFloat32Array color, SkBlendMode blendMode);
  SkColorFilter? MakeBlend(Float32List color, SkBlendMode blendMode) =>
      _MakeBlend(color.toJS, blendMode);

  @JS('MakeMatrix')
  external SkColorFilter _MakeMatrix(
    JSFloat32Array matrix, // 20-element matrix
  );
  SkColorFilter MakeMatrix(
    Float32List matrix, // 20-element matrix
  ) => _MakeMatrix(matrix.toJS);

  external SkColorFilter MakeLinearToSRGBGamma();
  external SkColorFilter MakeSRGBToLinearGamma();
  external SkColorFilter MakeCompose(SkColorFilter? outer, SkColorFilter inner);
}

@JS()
@anonymous
@staticInterop
class SkColorFilter {}

extension SkColorFilterExtension on SkColorFilter {
  external JSVoid delete();
}

@JS()
@staticInterop
class SkImageFilterNamespace {}

extension SkImageFilterNamespaceExtension on SkImageFilterNamespace {
  @JS('MakeBlur')
  external SkImageFilter _MakeBlur(
    JSNumber sigmaX,
    JSNumber sigmaY,
    SkTileMode tileMode,
    JSVoid input, // we don't use this yet
  );
  SkImageFilter MakeBlur(
    double sigmaX,
    double sigmaY,
    SkTileMode tileMode,
    void input, // we don't use this yet
  ) => _MakeBlur(sigmaX.toJS, sigmaY.toJS, tileMode, input);

  @JS('MakeMatrixTransform')
  external SkImageFilter _MakeMatrixTransform(
    JSFloat32Array matrix, // 3x3 matrix
    CkFilterOptions filterOptions,
    JSVoid input, // we don't use this yet
  );
  SkImageFilter MakeMatrixTransform(
    Float32List matrix, // 3x3 matrix
    CkFilterOptions filterOptions,
    void input, // we don't use this yet
  ) => _MakeMatrixTransform(matrix.toJS, filterOptions, input);

  external SkImageFilter MakeColorFilter(
    SkColorFilter colorFilter,
    JSVoid input, // we don't use this yet
  );

  external SkImageFilter MakeCompose(SkImageFilter outer, SkImageFilter inner);

  external SkImageFilter MakeDilate(
    double radiusX,
    double radiusY,
    void input, // we don't use this yet
  );

  external SkImageFilter MakeErode(
    double radiusX,
    double radiusY,
    void input, // we don't use this yet
  );
}

@JS()
@anonymous
@staticInterop
class SkImageFilter {}

extension SkImageFilterExtension on SkImageFilter {
  external JSVoid delete();

  @JS('isDeleted')
  external JSBoolean _isDeleted();
  bool isDeleted() => _isDeleted().toDart;

  @JS('getOutputBounds')
  external JSInt32Array _getOutputBounds(JSFloat32Array bounds);
  Int32List getOutputBounds(Float32List bounds) => _getOutputBounds(bounds.toJS).toDart;
}

@JS()
@staticInterop
class SkPathNamespace {}

extension SkPathNamespaceExtension on SkPathNamespace {
  /// Creates an [SkPath] using commands obtained from [SkPath.toCmds].
  @JS('MakeFromCmds')
  external SkPath _MakeFromCmds(JSAny pathCommands);
  SkPath MakeFromCmds(List<dynamic> pathCommands) => _MakeFromCmds(pathCommands.toJSAnyShallow);

  /// Creates an [SkPath] by combining [path1] and [path2] using [pathOp].
  external SkPath MakeFromOp(SkPath path1, SkPath path2, SkPathOp pathOp);
}

/// Converts a 4x4 Flutter matrix (represented as a [Float32List] in
/// column major order) to an SkM44 which is a 4x4 matrix represented
/// as a [Float32List] in row major order.
Float32List toSkM44FromFloat32(Float32List matrix4) {
  final Float32List skM44 = Float32List(16);
  for (int r = 0; r < 4; r++) {
    for (int c = 0; c < 4; c++) {
      skM44[c * 4 + r] = matrix4[r * 4 + c];
    }
  }
  return skM44;
}

// Mappings from SkMatrix-index to input-index.
const List<int> _skMatrixIndexToMatrix4Index = <int>[
  0, 4, 12, // Row 1
  1, 5, 13, // Row 2
  3, 7, 15, // Row 3
];

/// Converts a 4x4 Flutter matrix (represented as a [Float32List]) to an
/// SkMatrix, which is a 3x3 transform matrix.
Float32List toSkMatrixFromFloat32(Float32List matrix4) {
  final Float32List skMatrix = Float32List(9);
  for (int i = 0; i < 9; ++i) {
    final int matrix4Index = _skMatrixIndexToMatrix4Index[i];
    if (matrix4Index < matrix4.length) {
      skMatrix[i] = matrix4[matrix4Index];
    } else {
      skMatrix[i] = 0.0;
    }
  }
  return skMatrix;
}

/// Converts a 4x4 Flutter matrix (represented as a [Float32List]) to an
/// SkMatrix, which is a 3x3 transform matrix.
Float32List toSkMatrixFromFloat64(Float64List matrix4) {
  final Float32List skMatrix = Float32List(9);
  for (int i = 0; i < 9; ++i) {
    final int matrix4Index = _skMatrixIndexToMatrix4Index[i];
    if (matrix4Index < matrix4.length) {
      skMatrix[i] = matrix4[matrix4Index];
    } else {
      skMatrix[i] = 0.0;
    }
  }
  return skMatrix;
}

/// Converts an [offset] into an `[x, y]` pair stored in a `Float32List`.
///
/// The returned list can be passed to CanvasKit API that take points.
Float32List toSkPoint(ui.Offset offset) {
  final Float32List point = Float32List(2);
  point[0] = offset.dx;
  point[1] = offset.dy;
  return point;
}

/// Color stops used when the framework specifies `null`.
final Float32List _kDefaultSkColorStops =
    Float32List(2)
      ..[0] = 0
      ..[1] = 1;

/// Converts a list of color stops into a Skia-compatible JS array or color stops.
///
/// In Flutter `null` means two color stops `[0, 1]` that in Skia must be specified explicitly.
Float32List toSkColorStops(List<double>? colorStops) {
  if (colorStops == null) {
    return _kDefaultSkColorStops;
  }

  final int len = colorStops.length;
  final Float32List skColorStops = Float32List(len);
  for (int i = 0; i < len; i++) {
    skColorStops[i] = colorStops[i];
  }
  return skColorStops;
}

@JS()
@staticInterop
abstract class _NativeType {}

@JS('Float32Array')
external _NativeType get _nativeFloat32ArrayType;

@JS('Uint32Array')
external _NativeType get _nativeUint32ArrayType;

@JS('window.flutterCanvasKit.Malloc')
external JSAny _malloc(_NativeType nativeType, JSNumber length);

/// Allocates a [Float32List] of [length] elements, backed by WASM memory,
/// managed by a [SkFloat32List].
///
/// To free the allocated array use [free].
SkFloat32List mallocFloat32List(int length) {
  return _malloc(_nativeFloat32ArrayType, length.toJS) as SkFloat32List;
}

/// Allocates a [Uint32List] of [length] elements, backed by WASM memory,
/// managed by a [SkUint32List].
///
/// To free the allocated array use [free].
SkUint32List mallocUint32List(int length) {
  return _malloc(_nativeUint32ArrayType, length.toJS) as SkUint32List;
}

/// Frees the WASM memory occupied by a [SkFloat32List] or [SkUint32List].
///
/// The [list] is no longer usable after calling this function.
///
/// Use this function to free lists owned by the engine.
@JS('window.flutterCanvasKit.Free')
external JSVoid free(MallocObj list);

@JS()
@staticInterop
abstract class MallocObj {}

/// Wraps a [Float32List] backed by WASM memory.
///
/// This wrapper is necessary because the raw [Float32List] will get detached
/// when WASM grows its memory. Call [toTypedArray] to get a new instance
/// that's attached to the current WASM memory block.
@JS()
@staticInterop
class SkFloat32List extends MallocObj {}

extension SkFloat32ListExtension on SkFloat32List {
  /// The number of objects this pointer refers to.
  @JS('length')
  external JSNumber get _length;
  double get length => _length.toDartDouble;

  @JS('length')
  external set _length(JSNumber length);
  set length(double l) => _length = l.toJS;

  /// Returns the [Float32List] object backed by WASM memory.
  ///
  /// Do not reuse the returned array across multiple WASM function/method
  /// invocations that may lead to WASM memory to grow. When WASM memory
  /// grows, the returned [Float32List] object becomes "detached" and is no
  /// longer usable. Instead, call this method every time you need to read from
  /// or write to the list.
  @JS('toTypedArray')
  external JSFloat32Array _toTypedArray();
  Float32List toTypedArray() => _toTypedArray().toDart;
}

/// Wraps a [Uint32List] backed by WASM memory.
///
/// This wrapper is necessary because the raw [Uint32List] will get detached
/// when WASM grows its memory. Call [toTypedArray] to get a new instance
/// that's attached to the current WASM memory block.
@JS()
@staticInterop
class SkUint32List extends MallocObj {}

extension SkUint32ListExtension on SkUint32List {
  /// The number of objects this pointer refers to.
  @JS('length')
  external JSNumber get _length;
  double get length => _length.toDartDouble;

  @JS('length')
  external set _length(JSNumber length);
  set length(double l) => _length = l.toJS;

  /// Returns the [Uint32List] object backed by WASM memory.
  ///
  /// Do not reuse the returned array across multiple WASM function/method
  /// invocations that may lead to WASM memory to grow. When WASM memory
  /// grows, the returned [Uint32List] object becomes "detached" and is no
  /// longer usable. Instead, call this method every time you need to read from
  /// or write to the list.
  @JS('toTypedArray')
  external JSUint32Array _toTypedArray();
  Uint32List toTypedArray() => _toTypedArray().toDart;
}

/// Writes [color] information into the given [skColor] buffer.
Float32List _populateSkColor(SkFloat32List skColor, ui.Color color) {
  final Float32List array = skColor.toTypedArray();
  array[0] = color.red / 255.0;
  array[1] = color.green / 255.0;
  array[2] = color.blue / 255.0;
  array[3] = color.alpha / 255.0;
  return array;
}

/// Unpacks the [color] into CanvasKit-compatible representation stored
/// in a shared memory location #1.
///
/// Use this only for passing transient data to CanvasKit. Because the
/// memory is shared the value will not persist.
Float32List toSharedSkColor1(ui.Color color) {
  return _populateSkColor(_sharedSkColor1, color);
}

final SkFloat32List _sharedSkColor1 = mallocFloat32List(4);

/// Unpacks the [color] into CanvasKit-compatible representation stored
/// in a shared memory location #2.
///
/// Use this only for passing transient data to CanvasKit. Because the
/// memory is shared the value will not persist.
Float32List toSharedSkColor2(ui.Color color) {
  return _populateSkColor(_sharedSkColor2, color);
}

final SkFloat32List _sharedSkColor2 = mallocFloat32List(4);

/// Unpacks the [color] into CanvasKit-compatible representation stored
/// in a shared memory location #3.
///
/// Use this only for passing transient data to CanvasKit. Because the
/// memory is shared the value will not persist.
Float32List toSharedSkColor3(ui.Color color) {
  return _populateSkColor(_sharedSkColor3, color);
}

final SkFloat32List _sharedSkColor3 = mallocFloat32List(4);

@JS('window.flutterCanvasKit.Path')
@staticInterop
class SkPath {
  external factory SkPath();
  external factory SkPath.from(SkPath other);
}

extension SkPathExtension on SkPath {
  external JSVoid setFillType(SkFillType fillType);

  @JS('addArc')
  external JSVoid _addArc(
    JSFloat32Array oval,
    JSNumber startAngleDegrees,
    JSNumber sweepAngleDegrees,
  );
  void addArc(Float32List oval, double startAngleDegrees, double sweepAngleDegrees) =>
      _addArc(oval.toJS, startAngleDegrees.toJS, sweepAngleDegrees.toJS);

  @JS('addOval')
  external JSVoid _addOval(JSFloat32Array oval, JSBoolean counterClockWise, JSNumber startIndex);
  void addOval(Float32List oval, bool counterClockWise, double startIndex) =>
      _addOval(oval.toJS, counterClockWise.toJS, startIndex.toJS);

  @JS('addPath')
  external JSVoid _addPath(
    SkPath other,
    JSNumber scaleX,
    JSNumber skewX,
    JSNumber transX,
    JSNumber skewY,
    JSNumber scaleY,
    JSNumber transY,
    JSNumber pers0,
    JSNumber pers1,
    JSNumber pers2,
    JSBoolean extendPath,
  );
  void addPath(
    SkPath other,
    double scaleX,
    double skewX,
    double transX,
    double skewY,
    double scaleY,
    double transY,
    double pers0,
    double pers1,
    double pers2,
    bool extendPath,
  ) => _addPath(
    other,
    scaleX.toJS,
    skewX.toJS,
    transX.toJS,
    skewY.toJS,
    scaleY.toJS,
    transY.toJS,
    pers0.toJS,
    pers1.toJS,
    pers2.toJS,
    extendPath.toJS,
  );

  @JS('addPoly')
  external JSVoid _addPoly(JSFloat32Array points, JSBoolean close);
  void addPoly(Float32List points, bool close) => _addPoly(points.toJS, close.toJS);

  @JS('addRRect')
  external JSVoid _addRRect(JSFloat32Array rrect, JSBoolean counterClockWise);
  void addRRect(Float32List rrect, bool counterClockWise) =>
      _addRRect(rrect.toJS, counterClockWise.toJS);

  @JS('addRect')
  external JSVoid _addRect(JSFloat32Array rect);
  void addRect(Float32List rect) => _addRect(rect.toJS);

  @JS('arcToOval')
  external JSVoid _arcToOval(
    JSFloat32Array oval,
    JSNumber startAngleDegrees,
    JSNumber sweepAngleDegrees,
    JSBoolean forceMoveTo,
  );
  void arcToOval(
    Float32List oval,
    double startAngleDegrees,
    double sweepAngleDegrees,
    bool forceMoveTo,
  ) => _arcToOval(oval.toJS, startAngleDegrees.toJS, sweepAngleDegrees.toJS, forceMoveTo.toJS);

  @JS('arcToRotated')
  external JSVoid _arcToRotated(
    JSNumber radiusX,
    JSNumber radiusY,
    JSNumber rotation,
    JSBoolean useSmallArc,
    JSBoolean counterClockWise,
    JSNumber x,
    JSNumber y,
  );
  void arcToRotated(
    double radiusX,
    double radiusY,
    double rotation,
    bool useSmallArc,
    bool counterClockWise,
    double x,
    double y,
  ) => _arcToRotated(
    radiusX.toJS,
    radiusY.toJS,
    rotation.toJS,
    useSmallArc.toJS,
    counterClockWise.toJS,
    x.toJS,
    y.toJS,
  );

  external JSVoid close();

  @JS('conicTo')
  external JSVoid _conicTo(JSNumber x1, JSNumber y1, JSNumber x2, JSNumber y2, JSNumber w);
  void conicTo(double x1, double y1, double x2, double y2, double w) =>
      _conicTo(x1.toJS, y1.toJS, x2.toJS, y2.toJS, w.toJS);

  @JS('contains')
  external JSBoolean _contains(JSNumber x, JSNumber y);
  bool contains(double x, double y) => _contains(x.toJS, y.toJS).toDart;

  @JS('cubicTo')
  external JSVoid _cubicTo(
    JSNumber x1,
    JSNumber y1,
    JSNumber x2,
    JSNumber y2,
    JSNumber x3,
    JSNumber y3,
  );
  void cubicTo(double x1, double y1, double x2, double y2, double x3, double y3) =>
      _cubicTo(x1.toJS, y1.toJS, x2.toJS, y2.toJS, x3.toJS, y3.toJS);

  @JS('getBounds')
  external JSFloat32Array _getBounds();
  Float32List getBounds() => _getBounds().toDart;

  @JS('lineTo')
  external JSVoid _lineTo(JSNumber x, JSNumber y);
  void lineTo(double x, double y) => _lineTo(x.toJS, y.toJS);

  @JS('moveTo')
  external JSVoid _moveTo(JSNumber x, JSNumber y);
  void moveTo(double x, double y) => _moveTo(x.toJS, y.toJS);

  @JS('quadTo')
  external JSVoid _quadTo(JSNumber x1, JSNumber y1, JSNumber x2, JSNumber y2);
  void quadTo(double x1, double y1, double x2, double y2) =>
      _quadTo(x1.toJS, y1.toJS, x2.toJS, y2.toJS);

  @JS('rArcTo')
  external JSVoid _rArcTo(
    JSNumber x,
    JSNumber y,
    JSNumber rotation,
    JSBoolean useSmallArc,
    JSBoolean counterClockWise,
    JSNumber deltaX,
    JSNumber deltaY,
  );
  void rArcTo(
    double x,
    double y,
    double rotation,
    bool useSmallArc,
    bool counterClockWise,
    double deltaX,
    double deltaY,
  ) => _rArcTo(
    x.toJS,
    y.toJS,
    rotation.toJS,
    useSmallArc.toJS,
    counterClockWise.toJS,
    deltaX.toJS,
    deltaY.toJS,
  );

  @JS('rConicTo')
  external JSVoid _rConicTo(JSNumber x1, JSNumber y1, JSNumber x2, JSNumber y2, JSNumber w);
  void rConicTo(double x1, double y1, double x2, double y2, double w) =>
      _rConicTo(x1.toJS, y1.toJS, x2.toJS, y2.toJS, w.toJS);

  @JS('rCubicTo')
  external JSVoid _rCubicTo(
    JSNumber x1,
    JSNumber y1,
    JSNumber x2,
    JSNumber y2,
    JSNumber x3,
    JSNumber y3,
  );
  void rCubicTo(double x1, double y1, double x2, double y2, double x3, double y3) =>
      _rCubicTo(x1.toJS, y1.toJS, x2.toJS, y2.toJS, x3.toJS, y3.toJS);

  @JS('rLineTo')
  external JSVoid _rLineTo(JSNumber x, JSNumber y);
  void rLineTo(double x, double y) => _rLineTo(x.toJS, y.toJS);

  @JS('rMoveTo')
  external JSVoid _rMoveTo(JSNumber x, JSNumber y);
  void rMoveTo(double x, double y) => _rMoveTo(x.toJS, y.toJS);

  @JS('rQuadTo')
  external JSVoid _rQuadTo(JSNumber x1, JSNumber y1, JSNumber x2, JSNumber y2);
  void rQuadTo(double x1, double y1, double x2, double y2) =>
      _rQuadTo(x1.toJS, y1.toJS, x2.toJS, y2.toJS);

  external JSVoid reset();

  @JS('toSVGString')
  external JSString _toSVGString();
  String toSVGString() => _toSVGString().toDart;

  @JS('isEmpty')
  external JSBoolean _isEmpty();
  bool isEmpty() => _isEmpty().toDart;

  external SkPath copy();

  @JS('transform')
  external JSVoid _transform(
    JSNumber scaleX,
    JSNumber skewX,
    JSNumber transX,
    JSNumber skewY,
    JSNumber scaleY,
    JSNumber transY,
    JSNumber pers0,
    JSNumber pers1,
    JSNumber pers2,
  );
  void transform(
    double scaleX,
    double skewX,
    double transX,
    double skewY,
    double scaleY,
    double transY,
    double pers0,
    double pers1,
    double pers2,
  ) => _transform(
    scaleX.toJS,
    skewX.toJS,
    transX.toJS,
    skewY.toJS,
    scaleY.toJS,
    transY.toJS,
    pers0.toJS,
    pers1.toJS,
    pers2.toJS,
  );

  /// Serializes the path into a list of commands.
  ///
  /// The list can be used to create a new [SkPath] using
  /// [CanvasKit.Path.MakeFromCmds].
  @JS('toCmds')
  external JSAny _toCmds();
  List<dynamic> toCmds() => _toCmds().toObjectShallow as List<dynamic>;

  external JSVoid delete();
}

@JS('window.flutterCanvasKit.ContourMeasureIter')
@staticInterop
class SkContourMeasureIter {
  factory SkContourMeasureIter(SkPath path, bool forceClosed, double resScale) =>
      SkContourMeasureIter._(path, forceClosed.toJS, resScale.toJS);
  external factory SkContourMeasureIter._(SkPath path, JSBoolean forceClosed, JSNumber resScale);
}

extension SkContourMeasureIterExtension on SkContourMeasureIter {
  external SkContourMeasure? next();
  external JSVoid delete();
}

@JS()
@staticInterop
class SkContourMeasure {}

extension SkContourMeasureExtension on SkContourMeasure {
  @JS('getSegment')
  external SkPath _getSegment(JSNumber start, JSNumber end, JSBoolean startWithMoveTo);
  SkPath getSegment(double start, double end, bool startWithMoveTo) =>
      _getSegment(start.toJS, end.toJS, startWithMoveTo.toJS);

  @JS('getPosTan')
  external JSFloat32Array _getPosTan(JSNumber distance);
  Float32List getPosTan(double distance) => _getPosTan(distance.toJS).toDart;

  @JS('isClosed')
  external JSBoolean _isClosed();
  bool isClosed() => _isClosed().toDart;

  @JS('length')
  external JSNumber _length();
  double length() => _length().toDartDouble;

  external JSVoid delete();
}

// TODO(hterkelsen): Use a shared malloc'ed array for performance.
Float32List toSkRect(ui.Rect rect) {
  final Float32List skRect = Float32List(4);
  skRect[0] = rect.left;
  skRect[1] = rect.top;
  skRect[2] = rect.right;
  skRect[3] = rect.bottom;
  return skRect;
}

ui.Rect fromSkRect(Float32List skRect) {
  return ui.Rect.fromLTRB(skRect[0], skRect[1], skRect[2], skRect[3]);
}

ui.Rect rectFromSkIRect(Int32List skIRect) {
  return ui.Rect.fromLTRB(
    skIRect[0].toDouble(),
    skIRect[1].toDouble(),
    skIRect[2].toDouble(),
    skIRect[3].toDouble(),
  );
}

// TODO(hterkelsen): Use a shared malloc'ed array for performance.
Float32List toSkRRect(ui.RRect rrect) {
  final Float32List skRRect = Float32List(12);
  skRRect[0] = rrect.left;
  skRRect[1] = rrect.top;
  skRRect[2] = rrect.right;
  skRRect[3] = rrect.bottom;
  skRRect[4] = rrect.tlRadiusX;
  skRRect[5] = rrect.tlRadiusY;
  skRRect[6] = rrect.trRadiusX;
  skRRect[7] = rrect.trRadiusY;
  skRRect[8] = rrect.brRadiusX;
  skRRect[9] = rrect.brRadiusY;
  skRRect[10] = rrect.blRadiusX;
  skRRect[11] = rrect.blRadiusY;
  return skRRect;
}

// TODO(hterkelsen): Use a shared malloc'ed array for performance.
Float32List toOuterSkRect(ui.RRect rrect) {
  final Float32List skRect = Float32List(4);
  skRect[0] = rrect.left;
  skRect[1] = rrect.top;
  skRect[2] = rrect.right;
  skRect[3] = rrect.bottom;
  return skRect;
}

/// Encodes a list of offsets to CanvasKit-compatible point array.
///
/// Uses `CanvasKit.Malloc` to allocate storage for the points in the WASM
/// memory to avoid unnecessary copying. Unless CanvasKit takes ownership of
/// the list the returned list must be explicitly freed using
/// [free].
SkFloat32List toMallocedSkPoints(List<ui.Offset> points) {
  final int len = points.length;
  final SkFloat32List skPoints = mallocFloat32List(len * 2);
  final Float32List list = skPoints.toTypedArray();
  for (int i = 0; i < len; i++) {
    list[2 * i] = points[i].dx;
    list[2 * i + 1] = points[i].dy;
  }
  return skPoints;
}

/// Converts a list of [ui.Offset] into a flat list of points.
Float32List toFlatSkPoints(List<ui.Offset> points) {
  final int len = points.length;
  final Float32List result = Float32List(len * 2);
  for (int i = 0; i < len; i++) {
    result[2 * i] = points[i].dx;
    result[2 * i + 1] = points[i].dy;
  }
  return result;
}

/// Converts a list of [ui.Color] into a flat list of ints.
Uint32List toFlatColors(List<ui.Color> colors) {
  final int len = colors.length;
  final Uint32List result = Uint32List(len);
  for (int i = 0; i < len; i++) {
    result[i] = colors[i].value;
  }
  return result;
}

Uint16List toUint16List(List<int> ints) {
  final int len = ints.length;
  final Uint16List result = Uint16List(len);
  for (int i = 0; i < len; i++) {
    result[i] = ints[i];
  }
  return result;
}

@JS('window.flutterCanvasKit.PictureRecorder')
@staticInterop
class SkPictureRecorder {
  external factory SkPictureRecorder();
}

extension SkPictureRecorderExtension on SkPictureRecorder {
  @JS('beginRecording')
  external SkCanvas _beginRecording(JSFloat32Array bounds, JSBoolean computeBounds);
  SkCanvas beginRecording(Float32List bounds) => _beginRecording(bounds.toJS, true.toJS);

  external SkPicture finishRecordingAsPicture();
  external JSVoid delete();
}

/// We do not use the `delete` method (which may be removed in the future anyway).
///
/// By Skia coding convention raw pointers should always be treated as
/// "borrowed", i.e. their memory is managed by other objects. In the case of
/// [SkCanvas] it is managed by [SkPictureRecorder].
@JS()
@anonymous
@staticInterop
class SkCanvas {}

extension SkCanvasExtension on SkCanvas {
  @JS('clear')
  external JSVoid _clear(JSFloat32Array color);
  void clear(Float32List color) => _clear(color.toJS);

  @JS('clipPath')
  external JSVoid _clipPath(SkPath path, SkClipOp clipOp, JSBoolean doAntiAlias);
  void clipPath(SkPath path, SkClipOp clipOp, bool doAntiAlias) =>
      _clipPath(path, clipOp, doAntiAlias.toJS);

  @JS('clipRRect')
  external JSVoid _clipRRect(JSFloat32Array rrect, SkClipOp clipOp, JSBoolean doAntiAlias);
  void clipRRect(Float32List rrect, SkClipOp clipOp, bool doAntiAlias) =>
      _clipRRect(rrect.toJS, clipOp, doAntiAlias.toJS);

  @JS('clipRect')
  external JSVoid _clipRect(JSFloat32Array rect, SkClipOp clipOp, JSBoolean doAntiAlias);
  void clipRect(Float32List rect, SkClipOp clipOp, bool doAntiAlias) =>
      _clipRect(rect.toJS, clipOp, doAntiAlias.toJS);

  @JS('getDeviceClipBounds')
  external JSInt32Array _getDeviceClipBounds();
  Int32List getDeviceClipBounds() => _getDeviceClipBounds().toDart;

  @JS('drawArc')
  external JSVoid _drawArc(
    JSFloat32Array oval,
    JSNumber startAngleDegrees,
    JSNumber sweepAngleDegrees,
    JSBoolean useCenter,
    SkPaint paint,
  );
  void drawArc(
    Float32List oval,
    double startAngleDegrees,
    double sweepAngleDegrees,
    bool useCenter,
    SkPaint paint,
  ) => _drawArc(oval.toJS, startAngleDegrees.toJS, sweepAngleDegrees.toJS, useCenter.toJS, paint);

  @JS('drawAtlas')
  external JSVoid _drawAtlas(
    SkImage image,
    JSFloat32Array rects,
    JSFloat32Array rstTransforms,
    SkPaint paint,
    SkBlendMode blendMode,
    JSUint32Array? colors,
  );
  void drawAtlas(
    SkImage image,
    Float32List rects,
    Float32List rstTransforms,
    SkPaint paint,
    SkBlendMode blendMode,
    Uint32List? colors,
  ) => _drawAtlas(image, rects.toJS, rstTransforms.toJS, paint, blendMode, colors?.toJS);

  @JS('drawCircle')
  external JSVoid _drawCircle(JSNumber x, JSNumber y, JSNumber radius, SkPaint paint);
  void drawCircle(double x, double y, double radius, SkPaint paint) =>
      _drawCircle(x.toJS, y.toJS, radius.toJS, paint);

  @JS('drawColorInt')
  external JSVoid _drawColorInt(JSNumber color, SkBlendMode blendMode);
  void drawColorInt(double color, SkBlendMode blendMode) => _drawColorInt(color.toJS, blendMode);

  @JS('drawDRRect')
  external JSVoid _drawDRRect(JSFloat32Array outer, JSFloat32Array inner, SkPaint paint);
  void drawDRRect(Float32List outer, Float32List inner, SkPaint paint) =>
      _drawDRRect(outer.toJS, inner.toJS, paint);

  @JS('drawImageCubic')
  external JSVoid _drawImageCubic(
    SkImage image,
    JSNumber x,
    JSNumber y,
    JSNumber B,
    JSNumber C,
    SkPaint paint,
  );
  void drawImageCubic(SkImage image, double x, double y, double B, double C, SkPaint paint) =>
      _drawImageCubic(image, x.toJS, y.toJS, B.toJS, C.toJS, paint);

  @JS('drawImageOptions')
  external JSVoid _drawImageOptions(
    SkImage image,
    JSNumber x,
    JSNumber y,
    SkFilterMode filterMode,
    SkMipmapMode mipmapMode,
    SkPaint paint,
  );
  void drawImageOptions(
    SkImage image,
    double x,
    double y,
    SkFilterMode filterMode,
    SkMipmapMode mipmapMode,
    SkPaint paint,
  ) => _drawImageOptions(image, x.toJS, y.toJS, filterMode, mipmapMode, paint);

  @JS('drawImageRectCubic')
  external JSVoid _drawImageRectCubic(
    SkImage image,
    JSFloat32Array src,
    JSFloat32Array dst,
    JSNumber B,
    JSNumber C,
    SkPaint paint,
  );
  void drawImageRectCubic(
    SkImage image,
    Float32List src,
    Float32List dst,
    double B,
    double C,
    SkPaint paint,
  ) => _drawImageRectCubic(image, src.toJS, dst.toJS, B.toJS, C.toJS, paint);

  @JS('drawImageRectOptions')
  external JSVoid _drawImageRectOptions(
    SkImage image,
    JSFloat32Array src,
    JSFloat32Array dst,
    SkFilterMode filterMode,
    SkMipmapMode mipmapMode,
    SkPaint paint,
  );
  void drawImageRectOptions(
    SkImage image,
    Float32List src,
    Float32List dst,
    SkFilterMode filterMode,
    SkMipmapMode mipmapMode,
    SkPaint paint,
  ) => _drawImageRectOptions(image, src.toJS, dst.toJS, filterMode, mipmapMode, paint);

  @JS('drawImageNine')
  external JSVoid _drawImageNine(
    SkImage image,
    JSFloat32Array center,
    JSFloat32Array dst,
    SkFilterMode filterMode,
    SkPaint paint,
  );
  void drawImageNine(
    SkImage image,
    Float32List center,
    Float32List dst,
    SkFilterMode filterMode,
    SkPaint paint,
  ) => _drawImageNine(image, center.toJS, dst.toJS, filterMode, paint);

  @JS('drawLine')
  external JSVoid _drawLine(JSNumber x1, JSNumber y1, JSNumber x2, JSNumber y2, SkPaint paint);
  void drawLine(double x1, double y1, double x2, double y2, SkPaint paint) =>
      _drawLine(x1.toJS, y1.toJS, x2.toJS, y2.toJS, paint);

  @JS('drawOval')
  external JSVoid _drawOval(JSFloat32Array rect, SkPaint paint);
  void drawOval(Float32List rect, SkPaint paint) => _drawOval(rect.toJS, paint);

  external JSVoid drawPaint(SkPaint paint);
  external JSVoid drawPath(SkPath path, SkPaint paint);

  @JS('drawPoints')
  external JSVoid _drawPoints(SkPointMode pointMode, JSFloat32Array points, SkPaint paint);
  void drawPoints(SkPointMode pointMode, Float32List points, SkPaint paint) =>
      _drawPoints(pointMode, points.toJS, paint);

  @JS('drawRRect')
  external JSVoid _drawRRect(JSFloat32Array rrect, SkPaint paint);
  void drawRRect(Float32List rrect, SkPaint paint) => _drawRRect(rrect.toJS, paint);

  @JS('drawRect')
  external JSVoid _drawRect(JSFloat32Array rect, SkPaint paint);
  void drawRect(Float32List rect, SkPaint paint) => _drawRect(rect.toJS, paint);

  @JS('drawShadow')
  external JSVoid _drawShadow(
    SkPath path,
    JSFloat32Array zPlaneParams,
    JSFloat32Array lightPos,
    JSNumber lightRadius,
    JSFloat32Array ambientColor,
    JSFloat32Array spotColor,
    JSNumber flags,
  );
  void drawShadow(
    SkPath path,
    Float32List zPlaneParams,
    Float32List lightPos,
    double lightRadius,
    Float32List ambientColor,
    Float32List spotColor,
    double flags,
  ) => _drawShadow(
    path,
    zPlaneParams.toJS,
    lightPos.toJS,
    lightRadius.toJS,
    ambientColor.toJS,
    spotColor.toJS,
    flags.toJS,
  );

  external JSVoid drawVertices(SkVertices vertices, SkBlendMode blendMode, SkPaint paint);

  @JS('save')
  external JSNumber _save();
  double save() => _save().toDartDouble;

  @JS('getSaveCount')
  external JSNumber _getSaveCount();
  double getSaveCount() => _getSaveCount().toDartDouble;

  @JS('saveLayer')
  external JSVoid _saveLayer(
    SkPaint? paint,
    JSFloat32Array? bounds,
    SkImageFilter? backdrop,
    JSNumber? flags,
    SkTileMode backdropTileMode,
  );
  void saveLayer(
    SkPaint? paint,
    Float32List? bounds,
    SkImageFilter? backdrop,
    int? flags,
    SkTileMode backdropTileMode,
  ) => _saveLayer(paint, bounds?.toJS, backdrop, flags?.toJS, backdropTileMode);

  external JSVoid restore();

  @JS('restoreToCount')
  external JSVoid _restoreToCount(JSNumber count);
  void restoreToCount(double count) => _restoreToCount(count.toJS);

  @JS('rotate')
  external JSVoid _rotate(JSNumber angleDegrees, JSNumber px, JSNumber py);
  void rotate(double angleDegrees, double px, double py) =>
      _rotate(angleDegrees.toJS, px.toJS, py.toJS);

  @JS('scale')
  external JSVoid _scale(JSNumber x, JSNumber y);
  void scale(double x, double y) => _scale(x.toJS, y.toJS);

  @JS('skew')
  external JSVoid _skew(JSNumber x, JSNumber y);
  void skew(double x, double y) => _skew(x.toJS, y.toJS);

  @JS('concat')
  external JSVoid _concat(JSFloat32Array matrix);
  void concat(Float32List matrix) => _concat(matrix.toJS);

  @JS('translate')
  external JSVoid _translate(JSNumber x, JSNumber y);
  void translate(double x, double y) => _translate(x.toJS, y.toJS);

  @JS('getLocalToDevice')
  external JSAny _getLocalToDevice();
  List<dynamic> getLocalToDevice() => _getLocalToDevice().toObjectShallow as List<dynamic>;

  @JS('quickReject')
  external JSBoolean _quickReject(JSFloat32Array rect);
  bool quickReject(Float32List rect) => _quickReject(rect.toJS).toDart;

  external JSVoid drawPicture(SkPicture picture);

  @JS('drawParagraph')
  external JSVoid _drawParagraph(SkParagraph paragraph, JSNumber x, JSNumber y);
  void drawParagraph(SkParagraph paragraph, double x, double y) =>
      _drawParagraph(paragraph, x.toJS, y.toJS);
}

@JS()
@anonymous
@staticInterop
class SkPicture {}

extension SkPictureExtension on SkPicture {
  external JSVoid delete();

  @JS('cullRect')
  external JSFloat32Array _cullRect();
  Float32List cullRect() => _cullRect().toDart;

  @JS('approximateBytesUsed')
  external JSNumber _approximateBytesUsed();
  int approximateBytesUsed() => _approximateBytesUsed().toDartInt;
}

@JS()
@anonymous
@staticInterop
class SkParagraphBuilderNamespace {}

extension SkParagraphBuilderNamespaceExtension on SkParagraphBuilderNamespace {
  external SkParagraphBuilder MakeFromFontCollection(
    SkParagraphStyle paragraphStyle,
    SkFontCollection? fontCollection,
  );

  bool RequiresClientICU() {
    if (!js_util.hasProperty(this, 'RequiresClientICU')) {
      return false;
    }
    return js_util.callMethod(this, 'RequiresClientICU', const <Object>[]) as bool;
  }
}

@JS()
@anonymous
@staticInterop
class SkParagraphBuilder {}

extension SkParagraphBuilderExtension on SkParagraphBuilder {
  @JS('addText')
  external JSVoid _addText(JSString text);
  void addText(String text) => _addText(text.toJS);

  external JSVoid pushStyle(SkTextStyle textStyle);
  external JSVoid pushPaintStyle(SkTextStyle textStyle, SkPaint foreground, SkPaint background);
  external JSVoid pop();

  @JS('addPlaceholder')
  external JSVoid _addPlaceholder(
    JSNumber width,
    JSNumber height,
    SkPlaceholderAlignment alignment,
    SkTextBaseline baseline,
    JSNumber offset,
  );
  void addPlaceholder(
    double width,
    double height,
    SkPlaceholderAlignment alignment,
    SkTextBaseline baseline,
    double offset,
  ) => _addPlaceholder(width.toJS, height.toJS, alignment, baseline, offset.toJS);

  @JS('getText')
  external JSString _getTextUtf8();
  String getTextUtf8() => _getTextUtf8().toDart;
  // SkParagraphBuilder.getText() returns a utf8 string, we need to decode it
  // into a utf16 string.
  String getText() => utf8.decode(getTextUtf8().codeUnits);

  @JS('setWordsUtf8')
  external JSVoid _setWordsUtf8(JSUint32Array words);
  void setWordsUtf8(Uint32List words) => _setWordsUtf8(words.toJS);

  @JS('setWordsUtf16')
  external JSVoid _setWordsUtf16(JSUint32Array words);
  void setWordsUtf16(Uint32List words) => _setWordsUtf16(words.toJS);

  @JS('setGraphemeBreaksUtf8')
  external JSVoid _setGraphemeBreaksUtf8(JSUint32Array graphemes);
  void setGraphemeBreaksUtf8(Uint32List graphemes) => _setGraphemeBreaksUtf8(graphemes.toJS);

  @JS('setGraphemeBreaksUtf16')
  external JSVoid _setGraphemeBreaksUtf16(JSUint32Array graphemes);
  void setGraphemeBreaksUtf16(Uint32List graphemes) => _setGraphemeBreaksUtf16(graphemes.toJS);

  @JS('setLineBreaksUtf8')
  external JSVoid _setLineBreaksUtf8(JSUint32Array lineBreaks);
  void setLineBreaksUtf8(Uint32List lineBreaks) => _setLineBreaksUtf8(lineBreaks.toJS);

  @JS('setLineBreaksUtf16')
  external JSVoid _setLineBreaksUtf16(JSUint32Array lineBreaks);
  void setLineBreaksUtf16(Uint32List lineBreaks) => _setLineBreaksUtf16(lineBreaks.toJS);

  external SkParagraph build();
  external JSVoid delete();
}

@JS()
@anonymous
@staticInterop
class SkParagraphStyle {}

@JS()
@anonymous
@staticInterop
class SkParagraphStyleProperties {
  external factory SkParagraphStyleProperties();
}

extension SkParagraphStylePropertiesExtension on SkParagraphStyleProperties {
  external set textAlign(SkTextAlign? value);
  external set textDirection(SkTextDirection? value);

  @JS('heightMultiplier')
  external set _heightMultiplier(JSNumber? value);
  set heightMultiplier(double? value) => _heightMultiplier = value?.toJS;

  external set textHeightBehavior(SkTextHeightBehavior? value);

  @JS('maxLines')
  external set _maxLines(JSNumber? value);
  set maxLines(int? value) => _maxLines = value?.toJS;

  @JS('ellipsis')
  external set _ellipsis(JSString? value);
  set ellipsis(String? value) => _ellipsis = value?.toJS;

  external set textStyle(SkTextStyleProperties? value);
  external set strutStyle(SkStrutStyleProperties? strutStyle);

  @JS('replaceTabCharacters')
  external set _replaceTabCharacters(JSBoolean? bool);
  set replaceTabCharacters(bool? bool) => _replaceTabCharacters = bool?.toJS;

  external set applyRoundingHack(bool applyRoundingHack);
}

@JS()
@staticInterop
class SkTextStyle {}

@JS()
@staticInterop
class SkTextDecorationStyleEnum {}

extension SkTextDecorationStyleEnumExtension on SkTextDecorationStyleEnum {
  external SkTextDecorationStyle get Solid;
  external SkTextDecorationStyle get Double;
  external SkTextDecorationStyle get Dotted;
  external SkTextDecorationStyle get Dashed;
  external SkTextDecorationStyle get Wavy;
}

@JS()
@staticInterop
class SkTextDecorationStyle {}

extension SkTextDecorationStyleExtension on SkTextDecorationStyle {
  @JS('value')
  external JSNumber get _value;
  double get value => _value.toDartDouble;
}

final List<SkTextDecorationStyle> _skTextDecorationStyles = <SkTextDecorationStyle>[
  canvasKit.DecorationStyle.Solid,
  canvasKit.DecorationStyle.Double,
  canvasKit.DecorationStyle.Dotted,
  canvasKit.DecorationStyle.Dashed,
  canvasKit.DecorationStyle.Wavy,
];

SkTextDecorationStyle toSkTextDecorationStyle(ui.TextDecorationStyle style) {
  return _skTextDecorationStyles[style.index];
}

@JS()
@staticInterop
class SkTextBaselineEnum {}

extension SkTextBaselineEnumExtension on SkTextBaselineEnum {
  external SkTextBaseline get Alphabetic;
  external SkTextBaseline get Ideographic;
}

@JS()
@staticInterop
class SkTextBaseline {}

extension SkTextBaselineExtension on SkTextBaseline {
  @JS('value')
  external JSNumber get _value;
  double get value => _value.toDartDouble;
}

final List<SkTextBaseline> _skTextBaselines = <SkTextBaseline>[
  canvasKit.TextBaseline.Alphabetic,
  canvasKit.TextBaseline.Ideographic,
];

SkTextBaseline toSkTextBaseline(ui.TextBaseline baseline) {
  return _skTextBaselines[baseline.index];
}

@JS()
@staticInterop
class SkPlaceholderAlignmentEnum {}

extension SkPlaceholderAlignmentEnumExtension on SkPlaceholderAlignmentEnum {
  external SkPlaceholderAlignment get Baseline;
  external SkPlaceholderAlignment get AboveBaseline;
  external SkPlaceholderAlignment get BelowBaseline;
  external SkPlaceholderAlignment get Top;
  external SkPlaceholderAlignment get Bottom;
  external SkPlaceholderAlignment get Middle;
}

@JS()
@staticInterop
class SkPlaceholderAlignment {}

extension SkPlaceholderAlignmentExtension on SkPlaceholderAlignment {
  @JS('value')
  external JSNumber get _value;
  double get value => _value.toDartDouble;
}

final List<SkPlaceholderAlignment> _skPlaceholderAlignments = <SkPlaceholderAlignment>[
  canvasKit.PlaceholderAlignment.Baseline,
  canvasKit.PlaceholderAlignment.AboveBaseline,
  canvasKit.PlaceholderAlignment.BelowBaseline,
  canvasKit.PlaceholderAlignment.Top,
  canvasKit.PlaceholderAlignment.Bottom,
  canvasKit.PlaceholderAlignment.Middle,
];

SkPlaceholderAlignment toSkPlaceholderAlignment(ui.PlaceholderAlignment alignment) {
  return _skPlaceholderAlignments[alignment.index];
}

@JS()
@anonymous
@staticInterop
class SkTextStyleProperties {
  external factory SkTextStyleProperties();
}

extension SkTextStylePropertiesExtension on SkTextStyleProperties {
  @JS('backgroundColor')
  external set _backgroundColor(JSFloat32Array? value);
  set backgroundColor(Float32List? value) => _backgroundColor = value?.toJS;

  @JS('color')
  external set _color(JSFloat32Array? value);
  set color(Float32List? value) => _color = value?.toJS;

  @JS('foregroundColor')
  external set _foregroundColor(JSFloat32Array? value);
  set foregroundColor(Float32List? value) => _foregroundColor = value?.toJS;

  @JS('decoration')
  external set _decoration(JSNumber? value);
  set decoration(int? value) => _decoration = value?.toJS;

  @JS('decorationThickness')
  external set _decorationThickness(JSNumber? value);
  set decorationThickness(double? value) => _decorationThickness = value?.toJS;

  @JS('decorationColor')
  external set _decorationColor(JSFloat32Array? value);
  set decorationColor(Float32List? value) => _decorationColor = value?.toJS;

  external set decorationStyle(SkTextDecorationStyle? value);
  external set textBaseline(SkTextBaseline? value);

  @JS('fontSize')
  external set _fontSize(JSNumber? value);
  set fontSize(double? value) => _fontSize = value?.toJS;

  @JS('letterSpacing')
  external set _letterSpacing(JSNumber? value);
  set letterSpacing(double? value) => _letterSpacing = value?.toJS;

  @JS('wordSpacing')
  external set _wordSpacing(JSNumber? value);
  set wordSpacing(double? value) => _wordSpacing = value?.toJS;

  @JS('heightMultiplier')
  external set _heightMultiplier(JSNumber? value);
  set heightMultiplier(double? value) => _heightMultiplier = value?.toJS;

  @JS('halfLeading')
  external set _halfLeading(JSBoolean? value);
  set halfLeading(bool? value) => _halfLeading = value?.toJS;

  @JS('locale')
  external set _locale(JSString? value);
  set locale(String? value) => _locale = value?.toJS;

  @JS('fontFamilies')
  external set _fontFamilies(JSAny? value);
  set fontFamilies(List<String>? value) => _fontFamilies = value?.toJSAnyShallow;

  external set fontStyle(SkFontStyle? value);

  @JS('shadows')
  external set _shadows(JSArray<JSAny?>? value);
  set shadows(List<SkTextShadow>? value) =>
          // TODO(joshualitt): remove this cast when we reify JS types on JS
          // backends.
          // ignore: unnecessary_cast
          _shadows =
          (value as List<JSAny>?)?.toJS;

  @JS('fontFeatures')
  external set _fontFeatures(JSArray<JSAny?>? value);
  set fontFeatures(List<SkFontFeature>? value) =>
          // TODO(joshualitt): remove this cast when we reify JS types on JS
          // backends.
          // ignore: unnecessary_cast
          _fontFeatures =
          (value as List<JSAny>?)?.toJS;

  @JS('fontVariations')
  external set _fontVariations(JSArray<JSAny?>? value);
  set fontVariations(List<SkFontVariation>? value) =>
          // TODO(joshualitt): remove this cast when we reify JS types on JS
          // backends.
          // ignore: unnecessary_cast
          _fontVariations =
          (value as List<JSAny>?)?.toJS;
}

@JS()
@anonymous
@staticInterop
class SkStrutStyleProperties {
  external factory SkStrutStyleProperties();
}

extension SkStrutStylePropertiesExtension on SkStrutStyleProperties {
  @JS('fontFamilies')
  external set _fontFamilies(JSAny? value);
  set fontFamilies(List<String>? value) => _fontFamilies = value?.toJSAnyShallow;

  external set fontStyle(SkFontStyle? value);

  @JS('fontSize')
  external set _fontSize(JSNumber? value);
  set fontSize(double? value) => _fontSize = value?.toJS;

  @JS('heightMultiplier')
  external set _heightMultiplier(JSNumber? value);
  set heightMultiplier(double? value) => _heightMultiplier = value?.toJS;

  @JS('halfLeading')
  external set _halfLeading(JSBoolean? value);
  set halfLeading(bool? value) => _halfLeading = value?.toJS;

  @JS('leading')
  external set _leading(JSNumber? value);
  set leading(double? value) => _leading = value?.toJS;

  @JS('strutEnabled')
  external set _strutEnabled(JSBoolean? value);
  set strutEnabled(bool? value) => _strutEnabled = value?.toJS;

  @JS('forceStrutHeight')
  external set _forceStrutHeight(JSBoolean? value);
  set forceStrutHeight(bool? value) => _forceStrutHeight = value?.toJS;
}

@JS()
@anonymous
@staticInterop
class SkFontStyle {
  external factory SkFontStyle();
}

extension SkFontStyleExtension on SkFontStyle {
  external set weight(SkFontWeight? value);
  external set slant(SkFontSlant? value);
}

@JS()
@anonymous
@staticInterop
class SkTextShadow {
  external factory SkTextShadow();
}

extension SkTextShadowExtension on SkTextShadow {
  @JS('color')
  external set _color(JSFloat32Array? value);
  set color(Float32List? value) => _color = value?.toJS;

  @JS('offset')
  external set _offset(JSFloat32Array? value);
  set offset(Float32List? value) => _offset = value?.toJS;

  @JS('blurRadius')
  external set _blurRadius(JSNumber? value);
  set blurRadius(double? value) => _blurRadius = value?.toJS;
}

@JS()
@anonymous
@staticInterop
class SkFontFeature {
  external factory SkFontFeature();
}

extension SkFontFeatureExtension on SkFontFeature {
  @JS('name')
  external set _name(JSString? value);
  set name(String? value) => _name = value?.toJS;

  @JS('value')
  external set _value(JSNumber? value);
  set value(int? v) => _value = v?.toJS;
}

@JS()
@anonymous
@staticInterop
class SkFontVariation {
  external factory SkFontVariation();
}

extension SkFontVariationExtension on SkFontVariation {
  @JS('axis')
  external set _axis(JSString? value);
  set axis(String? value) => _axis = value?.toJS;

  @JS('value')
  external set _value(JSNumber? value);
  set value(double? v) => _value = v?.toJS;
}

@JS()
@anonymous
@staticInterop
class SkTypeface {}

@JS('window.flutterCanvasKit.Font')
@staticInterop
class SkFont {
  external factory SkFont(SkTypeface typeface);
}

extension SkFontExtension on SkFont {
  @JS('getGlyphIDs')
  external JSUint16Array _getGlyphIDs(JSString text);
  Uint16List getGlyphIDs(String text) => _getGlyphIDs(text.toJS).toDart;

  @JS('getGlyphBounds')
  external JSVoid _getGlyphBounds(JSAny glyphs, SkPaint? paint, JSUint8Array? output);
  void getGlyphBounds(List<int> glyphs, SkPaint? paint, Uint8List? output) =>
      _getGlyphBounds(glyphs.toJSAnyShallow, paint, output?.toJS);
}

@JS()
@anonymous
@staticInterop
class SkFontMgr {}

extension SkFontMgrExtension on SkFontMgr {
  @JS('getFamilyName')
  external JSString? _getFamilyName(JSNumber fontId);
  String? getFamilyName(double fontId) => _getFamilyName(fontId.toJS)?.toDart;

  external JSVoid delete();

  @JS('MakeTypefaceFromData')
  external SkTypeface? _MakeTypefaceFromData(JSUint8Array font);
  SkTypeface? MakeTypefaceFromData(Uint8List font) => _MakeTypefaceFromData(font.toJS);
}

@JS('window.flutterCanvasKit.TypefaceFontProvider')
@staticInterop
class TypefaceFontProvider extends SkFontMgr {}

extension TypefaceFontProviderExtension on TypefaceFontProvider {
  @JS('registerFont')
  external JSVoid _registerFont(JSUint8Array font, JSString family);
  void registerFont(Uint8List font, String family) => _registerFont(font.toJS, family.toJS);
}

@JS()
@anonymous
@staticInterop
class SkFontCollection {}

extension SkFontCollectionExtension on SkFontCollection {
  external void enableFontFallback();
  external void setDefaultFontManager(TypefaceFontProvider? fontManager);
  external void delete();
}

@JS()
@anonymous
@staticInterop
class SkLineMetrics {}

extension SkLineMetricsExtension on SkLineMetrics {
  @JS('startIndex')
  external JSNumber get _startIndex;
  double get startIndex => _startIndex.toDartDouble;

  @JS('endIndex')
  external JSNumber get _endIndex;
  double get endIndex => _endIndex.toDartDouble;

  @JS('endExcludingWhitespaces')
  external JSNumber get _endExcludingWhitespaces;
  double get endExcludingWhitespaces => _endExcludingWhitespaces.toDartDouble;

  @JS('endIncludingNewline')
  external JSNumber get _endIncludingNewline;
  double get endIncludingNewline => _endIncludingNewline.toDartDouble;

  @JS('isHardBreak')
  external JSBoolean get _isHardBreak;
  bool get isHardBreak => _isHardBreak.toDart;

  @JS('ascent')
  external JSNumber get _ascent;
  double get ascent => _ascent.toDartDouble;

  @JS('descent')
  external JSNumber get _descent;
  double get descent => _descent.toDartDouble;

  @JS('height')
  external JSNumber get _height;
  double get height => _height.toDartDouble;

  @JS('width')
  external JSNumber get _width;
  double get width => _width.toDartDouble;

  @JS('left')
  external JSNumber get _left;
  double get left => _left.toDartDouble;

  @JS('baseline')
  external JSNumber get _baseline;
  double get baseline => _baseline.toDartDouble;

  @JS('lineNumber')
  external JSNumber get _lineNumber;
  double get lineNumber => _lineNumber.toDartDouble;
}

@JS()
@anonymous
@staticInterop
class SkGlyphClusterInfo {}

extension SkGlyphClusterInfoExtension on SkGlyphClusterInfo {
  @JS('graphemeLayoutBounds')
  external JSArray<JSAny?> get _bounds;

  @JS('dir')
  external SkTextDirection get _direction;

  @JS('graphemeClusterTextRange')
  external SkTextRange get _textRange;

  ui.GlyphInfo get _glyphInfo {
    final List<JSNumber> list = _bounds.toDart.cast<JSNumber>();
    final ui.Rect bounds = ui.Rect.fromLTRB(
      list[0].toDartDouble,
      list[1].toDartDouble,
      list[2].toDartDouble,
      list[3].toDartDouble,
    );
    final ui.TextRange textRange = ui.TextRange(
      start: _textRange.start.toInt(),
      end: _textRange.end.toInt(),
    );
    return ui.GlyphInfo(bounds, textRange, ui.TextDirection.values[_direction.value.toInt()]);
  }
}

@JS()
@anonymous
@staticInterop
class SkRectWithDirection {}

extension SkRectWithDirectionExtension on SkRectWithDirection {
  @JS('rect')
  external JSFloat32Array get _rect;
  Float32List get rect => _rect.toDart;

  @JS('rect')
  external set _rect(JSFloat32Array rect);
  set rect(Float32List r) => _rect = r.toJS;

  external SkTextDirection dir;
}

@JS()
@anonymous
@staticInterop
class SkParagraph {}

extension SkParagraphExtension on SkParagraph {
  @JS('getAlphabeticBaseline')
  external JSNumber _getAlphabeticBaseline();
  double getAlphabeticBaseline() => _getAlphabeticBaseline().toDartDouble;

  @JS('didExceedMaxLines')
  external JSBoolean _didExceedMaxLines();
  bool didExceedMaxLines() => _didExceedMaxLines().toDart;

  @JS('getHeight')
  external JSNumber _getHeight();
  double getHeight() => _getHeight().toDartDouble;

  @JS('getIdeographicBaseline')
  external JSNumber _getIdeographicBaseline();
  double getIdeographicBaseline() => _getIdeographicBaseline().toDartDouble;

  @JS('getLineMetrics')
  external JSArray<JSAny?> _getLineMetrics();
  List<SkLineMetrics> getLineMetrics() => _getLineMetrics().toDart.cast<SkLineMetrics>();

  @JS('getLineMetricsAt')
  external SkLineMetrics? _getLineMetricsAt(JSNumber index);
  SkLineMetrics? getLineMetricsAt(double index) => _getLineMetricsAt(index.toJS);

  @JS('getNumberOfLines')
  external JSNumber _getNumberOfLines();
  double getNumberOfLines() => _getNumberOfLines().toDartDouble;

  @JS('getLineNumberAt')
  external JSNumber _getLineNumberAt(JSNumber index);
  double getLineNumberAt(double index) => _getLineNumberAt(index.toJS).toDartDouble;

  @JS('getLongestLine')
  external JSNumber _getLongestLine();
  double getLongestLine() => _getLongestLine().toDartDouble;

  @JS('getMaxIntrinsicWidth')
  external JSNumber _getMaxIntrinsicWidth();
  double getMaxIntrinsicWidth() => _getMaxIntrinsicWidth().toDartDouble;

  @JS('getMinIntrinsicWidth')
  external JSNumber _getMinIntrinsicWidth();
  double getMinIntrinsicWidth() => _getMinIntrinsicWidth().toDartDouble;

  @JS('getMaxWidth')
  external JSNumber _getMaxWidth();
  double getMaxWidth() => _getMaxWidth().toDartDouble;

  @JS('getRectsForRange')
  external JSArray<JSAny?> _getRectsForRange(
    JSNumber start,
    JSNumber end,
    SkRectHeightStyle heightStyle,
    SkRectWidthStyle widthStyle,
  );
  List<SkRectWithDirection> getRectsForRange(
    double start,
    double end,
    SkRectHeightStyle heightStyle,
    SkRectWidthStyle widthStyle,
  ) =>
      _getRectsForRange(
        start.toJS,
        end.toJS,
        heightStyle,
        widthStyle,
      ).toDart.cast<SkRectWithDirection>();

  @JS('getRectsForPlaceholders')
  external JSArray<JSAny?> _getRectsForPlaceholders();
  List<SkRectWithDirection> getRectsForPlaceholders() =>
      _getRectsForPlaceholders().toDart.cast<SkRectWithDirection>();

  @JS('getGlyphPositionAtCoordinate')
  external SkTextPosition _getGlyphPositionAtCoordinate(JSNumber x, JSNumber y);
  SkTextPosition getGlyphPositionAtCoordinate(double x, double y) =>
      _getGlyphPositionAtCoordinate(x.toJS, y.toJS);

  @JS('getGlyphInfoAt')
  external SkGlyphClusterInfo? _getGlyphInfoAt(JSNumber position);
  ui.GlyphInfo? getGlyphInfoAt(double position) => _getGlyphInfoAt(position.toJS)?._glyphInfo;

  @JS('getClosestGlyphInfoAtCoordinate')
  external SkGlyphClusterInfo? _getClosestGlyphInfoAtCoordinate(JSNumber x, JSNumber y);
  ui.GlyphInfo? getClosestGlyphInfoAt(double x, double y) =>
      _getClosestGlyphInfoAtCoordinate(x.toJS, y.toJS)?._glyphInfo;

  @JS('getWordBoundary')
  external SkTextRange _getWordBoundary(JSNumber position);
  SkTextRange getWordBoundary(double position) => _getWordBoundary(position.toJS);

  @JS('layout')
  external JSVoid _layout(JSNumber width);
  void layout(double width) => _layout(width.toJS);

  external JSVoid delete();
}

@JS()
@staticInterop
class SkTextPosition {}

extension SkTextPositionExtnsion on SkTextPosition {
  external SkAffinity get affinity;

  @JS('pos')
  external JSNumber get _pos;
  double get pos => _pos.toDartDouble;
}

@JS()
@staticInterop
class SkTextRange {}

extension SkTextRangeExtension on SkTextRange {
  @JS('start')
  external JSNumber get _start;
  double get start => _start.toDartDouble;

  @JS('end')
  external JSNumber get _end;
  double get end => _end.toDartDouble;
}

@JS()
@anonymous
@staticInterop
class SkVertices {}

extension SkVerticesExtension on SkVertices {
  external JSVoid delete();
}

@JS()
@anonymous
@staticInterop
class SkTonalColors {
  factory SkTonalColors({required Float32List ambient, required Float32List spot}) =>
      SkTonalColors._(ambient: ambient.toJS, spot: spot.toJS);
  external factory SkTonalColors._({required JSFloat32Array ambient, required JSFloat32Array spot});
}

extension SkTonalColorsExtension on SkTonalColors {
  @JS('ambient')
  external JSFloat32Array get _ambient;
  Float32List get ambient => _ambient.toDart;

  @JS('spot')
  external JSFloat32Array get _spot;
  Float32List get spot => _spot.toDart;
}

@JS()
@staticInterop
class SkFontMgrNamespace {}

extension SkFontMgrNamespaceExtension on SkFontMgrNamespace {
  // TODO(yjbanov): can this be made non-null? It returns null in our unit-tests right now.
  @JS('FromData')
  external SkFontMgr? _FromData(JSAny fonts);
  SkFontMgr? FromData(List<Uint8List> fonts) => _FromData(fonts.toJSAnyShallow);
}

@JS()
@staticInterop
class TypefaceFontProviderNamespace {}

extension TypefaceFontProviderNamespaceExtension on TypefaceFontProviderNamespace {
  external TypefaceFontProvider Make();
}

@JS()
@staticInterop
class FontCollectionNamespace {}

extension FontCollectionNamespaceExtension on FontCollectionNamespace {
  external SkFontCollection Make();
}

@JS()
@anonymous
@staticInterop
class SkTypefaceFactory {}

extension SkTypefaceFactoryExtension on SkTypefaceFactory {
  @JS('MakeFreeTypeFaceFromData')
  external SkTypeface? _MakeFreeTypeFaceFromData(JSArrayBuffer fontData);
  SkTypeface? MakeFreeTypeFaceFromData(ByteBuffer fontData) =>
      _MakeFreeTypeFaceFromData(fontData.toJS);
}

/// Any Skia object that has a `delete` method.
@JS()
@anonymous
@staticInterop
class SkDeletable {}

extension SkDeletableExtension on SkDeletable {
  /// Deletes the C++ side object.
  external JSVoid delete();

  /// Returns whether the corresponding C++ object has been deleted.
  @JS('isDeleted')
  external JSBoolean _isDeleted();
  bool isDeleted() => _isDeleted().toDart;

  /// Returns the JavaScript constructor for this object.
  ///
  /// This is useful for debugging.
  external JsConstructor get constructor;
}

@JS()
@anonymous
@staticInterop
class JsConstructor {}

extension JsConstructorExtension on JsConstructor {
  /// The name of the "constructor", typically the function name called with
  /// the `new` keyword, or the ES6 class name.
  ///
  /// This is useful for debugging.
  @JS('name')
  external JSString get _name;
  String get name => _name.toDart;
}

@JS()
@staticInterop
class SkData {}

extension SkDataExtension on SkData {
  @JS('size')
  external JSNumber _size();
  double size() => _size().toDartDouble;

  @JS('isEmpty')
  external JSBoolean _isEmpty();
  bool isEmpty() => _isEmpty().toDart;

  @JS('bytes')
  external JSUint8Array _bytes();
  Uint8List bytes() => _bytes().toDart;

  external JSVoid delete();
}

@JS()
@anonymous
@staticInterop
class SkImageInfo {
  factory SkImageInfo({
    required double width,
    required double height,
    required SkColorType colorType,
    required SkAlphaType alphaType,
    required ColorSpace colorSpace,
  }) => SkImageInfo._(
    width: width.toJS,
    height: height.toJS,
    colorType: colorType,
    alphaType: alphaType,
    colorSpace: colorSpace,
  );
  external factory SkImageInfo._({
    required JSNumber width,
    required JSNumber height,
    required SkColorType colorType,
    required SkAlphaType alphaType,
    required ColorSpace colorSpace,
  });
}

extension SkImageInfoExtension on SkImageInfo {
  external SkAlphaType get alphaType;
  external ColorSpace get colorSpace;
  external SkColorType get colorType;

  @JS('height')
  external JSNumber get _height;
  double get height => _height.toDartDouble;

  @JS('isEmpty')
  external JSBoolean get _isEmpty;
  bool get isEmpty => _isEmpty.toDart;

  @JS('isOpaque')
  external JSBoolean get _isOpaque;
  bool get isOpaque => _isOpaque.toDart;

  @JS('bounds')
  external JSFloat32Array get _bounds;
  Float32List get bounds => _bounds.toDart;

  @JS('width')
  external JSNumber get _width;
  double get width => _width.toDartDouble;

  external SkImageInfo makeAlphaType(SkAlphaType alphaType);
  external SkImageInfo makeColorSpace(ColorSpace colorSpace);
  external SkImageInfo makeColorType(SkColorType colorType);

  @JS('makeWH')
  external SkImageInfo _makeWH(JSNumber width, JSNumber height);
  SkImageInfo makeWH(double width, double height) => _makeWH(width.toJS, height.toJS);
}

@JS()
@anonymous
@staticInterop
class SkPartialImageInfo {
  factory SkPartialImageInfo({
    required double width,
    required double height,
    required SkColorType colorType,
    required SkAlphaType alphaType,
    required ColorSpace colorSpace,
  }) => SkPartialImageInfo._(
    width: width.toJS,
    height: height.toJS,
    colorType: colorType,
    alphaType: alphaType,
    colorSpace: colorSpace,
  );
  external factory SkPartialImageInfo._({
    required JSNumber width,
    required JSNumber height,
    required SkColorType colorType,
    required SkAlphaType alphaType,
    required ColorSpace colorSpace,
  });
}

extension SkPartialImageInfoExtension on SkPartialImageInfo {
  external SkAlphaType get alphaType;
  external ColorSpace get colorSpace;
  external SkColorType get colorType;

  @JS('height')
  external JSNumber get _height;
  double get height => _height.toDartDouble;

  @JS('width')
  external JSNumber get _width;
  double get width => _width.toDartDouble;
}

@JS('window.flutterCanvasKit.RuntimeEffect')
@anonymous
@staticInterop
class SkRuntimeEffect {}

@JS('window.flutterCanvasKit.RuntimeEffect.Make')
external SkRuntimeEffect? _MakeRuntimeEffect(JSString program);
SkRuntimeEffect? MakeRuntimeEffect(String program) => _MakeRuntimeEffect(program.toJS);

extension SkSkRuntimeEffectExtension on SkRuntimeEffect {
  @JS('makeShader')
  external SkShader? _makeShader(JSAny uniforms);
  SkShader? makeShader(SkFloat32List uniforms) => _makeShader(uniforms.toJSAnyShallow);

  @JS('makeShaderWithChildren')
  external SkShader? _makeShaderWithChildren(JSAny uniforms, JSAny children);
  SkShader? makeShaderWithChildren(SkFloat32List uniforms, List<Object?> children) =>
      _makeShaderWithChildren(uniforms.toJSAnyShallow, children.toJSAnyShallow);
}

const String _kFullCanvasKitJsFileName = 'canvaskit.js';
const String _kChromiumCanvasKitJsFileName = 'chromium/canvaskit.js';

String get _canvasKitBaseUrl => configuration.canvasKitBaseUrl;

@visibleForTesting
List<String> getCanvasKitJsFileNames(CanvasKitVariant variant) {
  switch (variant) {
    case CanvasKitVariant.auto:
      return <String>[
        if (_enableCanvasKitChromiumInAutoMode) _kChromiumCanvasKitJsFileName,
        _kFullCanvasKitJsFileName,
      ];
    case CanvasKitVariant.full:
      return <String>[_kFullCanvasKitJsFileName];
    case CanvasKitVariant.chromium:
      return <String>[_kChromiumCanvasKitJsFileName];
  }
}

Iterable<String> get _canvasKitJsUrls {
  return getCanvasKitJsFileNames(
    configuration.canvasKitVariant,
  ).map((String filename) => '$_canvasKitBaseUrl$filename');
}

@visibleForTesting
String canvasKitWasmModuleUrl(String file, String canvasKitBase) => canvasKitBase + file;

/// Download and initialize the CanvasKit module.
///
/// Downloads the CanvasKit JavaScript, then calls `CanvasKitInit` to download
/// and intialize the CanvasKit wasm.
Future<CanvasKit> downloadCanvasKit() async {
  final CanvasKitModule canvasKitModule = await _downloadOneOf(_canvasKitJsUrls);

  final CanvasKit canvasKit =
      (await canvasKitModule
              .defaultExport(
                CanvasKitInitOptions(locateFile: createLocateFileCallback(canvasKitWasmModuleUrl)),
              )
              .toDart)
          as CanvasKit;

  if (canvasKit.ParagraphBuilder.RequiresClientICU() && !browserSupportsCanvaskitChromium) {
    throw Exception(
      'The CanvasKit variant you are using only works on Chromium browsers. '
      'Please use a different CanvasKit variant, or use a Chromium browser.',
    );
  }

  return canvasKit;
}

/// Finds the first URL in [urls] that can be downloaded successfully, and
/// downloads it.
///
/// If none of the URLs can be downloaded, throws an [Exception].
Future<CanvasKitModule> _downloadOneOf(Iterable<String> urls) async {
  for (final String url in urls) {
    try {
      return await _downloadCanvasKitJs(url);
    } catch (_) {
      continue;
    }
  }

  // Reaching this point means that all URLs failed to download.
  throw Exception('Failed to download any of the following CanvasKit URLs: $urls');
}

String _resolveUrl(String url) {
  return createDomURL(url, domWindow.document.baseUri).toJSString().toDart;
}

/// Downloads the CanvasKit JavaScript file at [url].
///
/// Returns a [Future] that completes with `true` if the CanvasKit JavaScript
/// file was successfully downloaded, or `false` if it failed.
Future<CanvasKitModule> _downloadCanvasKitJs(String url) async {
  final JSAny scriptUrl = createTrustedScriptUrl(_resolveUrl(url));
  return (await importModule(scriptUrl).toDart) as CanvasKitModule;
}
