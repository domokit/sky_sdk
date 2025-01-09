// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:js_interop';
import 'dart:typed_data';

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';

import 'package:ui/ui.dart' as ui;
import 'package:ui/ui_web/src/ui_web.dart';

import '../common/test_initialization.dart';
import 'scene_builder_utils.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

class StubPictureRenderer implements PictureRenderer {
  final DomCanvasElement scratchCanvasElement = createDomCanvasElement(width: 500, height: 500);

  @override
  Future<RenderResult> renderPictures(List<ScenePicture> pictures) async {
    renderedPictures.addAll(pictures);
    final List<DomImageBitmap> bitmaps = await Future.wait(
      pictures.map((ScenePicture picture) {
        final ui.Rect cullRect = picture.cullRect;
        final Future<DomImageBitmap> bitmap = createImageBitmap(scratchCanvasElement as JSObject, (
          x: 0,
          y: 0,
          width: cullRect.width.toInt(),
          height: cullRect.height.toInt(),
        ));
        return bitmap;
      }),
    );
    return (imageBitmaps: bitmaps, rasterStartMicros: 0, rasterEndMicros: 0);
  }

  @override
  ScenePicture clipPicture(ScenePicture picture, ui.Rect clip) {
    clipRequests[picture] = clip;
    return picture;
  }

  List<ScenePicture> renderedPictures = <ScenePicture>[];
  Map<ScenePicture, ui.Rect> clipRequests = <ScenePicture, ui.Rect>{};
}

class StubFlutterView implements EngineFlutterView {
  // Overridden in some tests
  @override
  DomManager dom = StubDomManager();

  @override
  double get devicePixelRatio => throw UnimplementedError();

  @override
  EngineFlutterDisplay get display => throw UnimplementedError();

  @override
  List<ui.DisplayFeature> get displayFeatures => throw UnimplementedError();

  @override
  ui.GestureSettings get gestureSettings => throw UnimplementedError();

  @override
  ViewPadding get padding => throw UnimplementedError();

  @override
  ViewConstraints get physicalConstraints => throw UnimplementedError();

  @override
  ui.Size get physicalSize => const ui.Size(1000, 1000);

  @override
  EnginePlatformDispatcher get platformDispatcher => throw UnimplementedError();

  @override
  void render(ui.Scene scene, {ui.Size? size}) {}

  @override
  ViewPadding get systemGestureInsets => throw UnimplementedError();

  @override
  void updateSemantics(ui.SemanticsUpdate update) {}

  @override
  int get viewId => throw UnimplementedError();

  @override
  ViewPadding get viewInsets => throw UnimplementedError();

  @override
  ViewPadding get viewPadding => throw UnimplementedError();

  @override
  ui.Size? debugPhysicalSizeOverride;

  @override
  bool isDisposed = false;

  @override
  PointerBinding get pointerBinding => throw UnimplementedError();

  @override
  set pointerBinding(_) {
    throw UnimplementedError();
  }

  @override
  ContextMenu get contextMenu => throw UnimplementedError();

  @override
  void debugForceResize() {
    throw UnimplementedError();
  }

  @override
  DimensionsProvider get dimensionsProvider => throw UnimplementedError();

  @override
  void dispose() {
    throw UnimplementedError();
  }

  @override
  EmbeddingStrategy get embeddingStrategy => throw UnimplementedError();

  @override
  MouseCursor get mouseCursor => throw UnimplementedError();

  @override
  Stream<ui.Size?> get onResize => throw UnimplementedError();

  @override
  void resize(ui.Size newPhysicalSize) {
    throw UnimplementedError();
  }

  @override
  EngineSemanticsOwner get semantics => throw UnimplementedError();
}

class StubPath implements ScenePath {
  StubPath(this.pathString, this.transformedPathString);

  final String pathString;
  final String transformedPathString;
  Float64List? appliedTransform;

  @override
  ui.PathFillType get fillType => throw UnimplementedError();

  @override
  set fillType(ui.PathFillType value) => throw UnimplementedError();

  @override
  void moveTo(double x, double y) => throw UnimplementedError();

  @override
  void relativeMoveTo(double dx, double dy) => throw UnimplementedError();

  @override
  void lineTo(double x, double y) => throw UnimplementedError();

  @override
  void relativeLineTo(double dx, double dy) => throw UnimplementedError();

  @override
  void quadraticBezierTo(double x1, double y1, double x2, double y2) => throw UnimplementedError();

  @override
  void relativeQuadraticBezierTo(double x1, double y1, double x2, double y2) =>
      throw UnimplementedError();

  @override
  void cubicTo(double x1, double y1, double x2, double y2, double x3, double y3) =>
      throw UnimplementedError();

  @override
  void relativeCubicTo(double x1, double y1, double x2, double y2, double x3, double y3) =>
      throw UnimplementedError();

  @override
  void conicTo(double x1, double y1, double x2, double y2, double w) => throw UnimplementedError();

  @override
  void relativeConicTo(double x1, double y1, double x2, double y2, double w) =>
      throw UnimplementedError();

  @override
  void arcTo(ui.Rect rect, double startAngle, double sweepAngle, bool forceMoveTo) =>
      throw UnimplementedError();

  @override
  void arcToPoint(
    ui.Offset arcEnd, {
    ui.Radius radius = ui.Radius.zero,
    double rotation = 0.0,
    bool largeArc = false,
    bool clockwise = true,
  }) => throw UnimplementedError();

  @override
  void relativeArcToPoint(
    ui.Offset arcEndDelta, {
    ui.Radius radius = ui.Radius.zero,
    double rotation = 0.0,
    bool largeArc = false,
    bool clockwise = true,
  }) => throw UnimplementedError();

  @override
  void addRect(ui.Rect rect) => throw UnimplementedError();

  @override
  void addOval(ui.Rect oval) => throw UnimplementedError();

  @override
  void addArc(ui.Rect oval, double startAngle, double sweepAngle) => throw UnimplementedError();

  @override
  void addPolygon(List<ui.Offset> points, bool close) => throw UnimplementedError();

  @override
  void addRRect(ui.RRect rrect) => throw UnimplementedError();

  @override
  void addPath(ui.Path path, ui.Offset offset, {Float64List? matrix4}) =>
      throw UnimplementedError();

  @override
  void extendWithPath(ui.Path path, ui.Offset offset, {Float64List? matrix4}) =>
      throw UnimplementedError();

  @override
  void close() => throw UnimplementedError();

  @override
  void reset() => throw UnimplementedError();

  @override
  bool contains(ui.Offset point) => throw UnimplementedError();

  @override
  ui.Path shift(ui.Offset offset) => throw UnimplementedError();

  @override
  StubPath transform(Float64List matrix4) {
    appliedTransform = matrix4;
    return StubPath(transformedPathString, '');
  }

  @override
  ui.Rect getBounds() => throw UnimplementedError();

  @override
  ui.PathMetrics computeMetrics({bool forceClosed = false}) => throw UnimplementedError();

  @override
  String toSvgString() => pathString;
}

void testMain() {
  late EngineSceneView sceneView;
  late StubPictureRenderer stubPictureRenderer;

  setUpImplicitView();

  setUp(() {
    stubPictureRenderer = StubPictureRenderer();
    sceneView = EngineSceneView(stubPictureRenderer, StubFlutterView());
  });

  test('SceneView places canvas according to device-pixel ratio', () async {
    debugOverrideDevicePixelRatio(2.0);

    final StubPicture picture = StubPicture(const ui.Rect.fromLTWH(50, 80, 100, 120));
    final EngineRootLayer rootLayer = EngineRootLayer();
    rootLayer.slices.add(LayerSlice(picture, <PlatformView>[]));
    final EngineScene scene = EngineScene(rootLayer);
    await sceneView.renderScene(scene, null);

    final DomElement sceneElement = sceneView.sceneElement;
    final List<DomElement> children = sceneElement.children.toList();

    expect(children.length, 1);
    final DomElement containerElement = children.first;
    expect(containerElement.tagName, equalsIgnoringCase('flt-canvas-container'));

    final List<DomElement> containerChildren = containerElement.children.toList();
    expect(containerChildren.length, 1);
    final DomElement canvasElement = containerChildren.first;
    final DomCSSStyleDeclaration style = canvasElement.style;
    expect(style.left, '25px');
    expect(style.top, '40px');
    expect(style.width, '50px');
    expect(style.height, '60px');

    debugOverrideDevicePixelRatio(null);
  });

  test('SceneView places platform view according to device-pixel ratio', () async {
    debugOverrideDevicePixelRatio(2.0);

    final StubPath clipPath = StubPath('M 2 2', 'M 1 1');

    final PlatformView platformView = PlatformView(
      1,
      const ui.Rect.fromLTWH(50, 80, 100, 120),
      PlatformViewStyling(clip: PlatformViewPathClip(clipPath)),
    );
    final EngineRootLayer rootLayer = EngineRootLayer();
    rootLayer.slices.add(LayerSlice(StubPicture(ui.Rect.zero), <PlatformView>[platformView]));
    final EngineScene scene = EngineScene(rootLayer);
    await sceneView.renderScene(scene, null);

    final DomElement sceneElement = sceneView.sceneElement;
    final List<DomElement> children = sceneElement.children.toList();

    expect(children.length, 1);
    final DomElement clipElement = children.first;
    expect(clipElement.tagName, equalsIgnoringCase('flt-clip'));

    expect(clipElement.style.clipPath, 'path("M 1 1")');

    // We expect the path to be scaled down by 1 / DPR
    final Matrix4 expectedTransform = Matrix4.identity()..scale(0.5);
    expect(clipPath.appliedTransform, expectedTransform.toFloat64());

    final List<DomElement> clipChildren = clipElement.children.toList();
    expect(clipChildren.length, 1);

    final DomElement containerElement = clipChildren.first;
    final DomCSSStyleDeclaration style = containerElement.style;
    expect(style.left, '');
    expect(style.top, '');
    expect(style.width, '100px');
    expect(style.height, '120px');

    // The heavy lifting of offsetting and sizing is done by the transform
    expect(style.transform, 'matrix(0.5, 0, 0, 0.5, 25, 40)');

    debugOverrideDevicePixelRatio(null);
  });

  test('SceneView always renders most recent picture and skips intermediate pictures', () async {
    final List<StubPicture> pictures = <StubPicture>[];
    final List<Future<void>> renderFutures = <Future<void>>[];
    for (int i = 1; i < 20; i++) {
      final StubPicture picture = StubPicture(const ui.Rect.fromLTWH(50, 80, 100, 120));
      pictures.add(picture);
      final EngineRootLayer rootLayer = EngineRootLayer();
      rootLayer.slices.add(LayerSlice(picture, <PlatformView>[]));
      final EngineScene scene = EngineScene(rootLayer);
      renderFutures.add(sceneView.renderScene(scene, null));
    }
    await Future.wait(renderFutures);

    // Should just render the first and last pictures and skip the one inbetween.
    expect(stubPictureRenderer.renderedPictures.length, 2);
    expect(stubPictureRenderer.renderedPictures.first, pictures.first);
    expect(stubPictureRenderer.renderedPictures.last, pictures.last);
  });

  test('SceneView clips pictures that are outside the window screen', () async {
    final StubPicture picture = StubPicture(const ui.Rect.fromLTWH(-50, -50, 100, 120));

    final EngineRootLayer rootLayer = EngineRootLayer();
    rootLayer.slices.add(LayerSlice(picture, <PlatformView>[]));
    final EngineScene scene = EngineScene(rootLayer);
    await sceneView.renderScene(scene, null);

    expect(stubPictureRenderer.renderedPictures.length, 1);
    expect(stubPictureRenderer.clipRequests.containsKey(picture), true);
  });

  test('SceneView places platform view contents in the DOM', () async {
    const int expectedPlatformViewId = 1234;

    int? injectedViewId;
    final DomManager stubDomManager =
        StubDomManager()
          ..injectPlatformViewOverride = (int viewId) {
            injectedViewId = viewId;
          };
    sceneView = EngineSceneView(stubPictureRenderer, StubFlutterView()..dom = stubDomManager);

    final PlatformView platformView = PlatformView(
      expectedPlatformViewId,
      const ui.Rect.fromLTWH(50, 80, 100, 120),
      const PlatformViewStyling(),
    );

    final EngineRootLayer rootLayer = EngineRootLayer();
    rootLayer.slices.add(LayerSlice(StubPicture(ui.Rect.zero), <PlatformView>[platformView]));
    final EngineScene scene = EngineScene(rootLayer);
    await sceneView.renderScene(scene, null);

    expect(
      injectedViewId,
      expectedPlatformViewId,
      reason: 'SceneView should call injectPlatformView on its flutterView.dom',
    );
  });
}

class StubDomManager implements DomManager {
  void Function(int platformViewId) injectPlatformViewOverride = (int id) {};
  @override
  void injectPlatformView(int platformViewId) {
    injectPlatformViewOverride(platformViewId);
  }

  @override
  DomElement get platformViewsHost => throw UnimplementedError();

  @override
  DomShadowRoot get renderingHost => throw UnimplementedError();

  @override
  DomElement get rootElement => throw UnimplementedError();

  @override
  DomElement get sceneHost => throw UnimplementedError();

  @override
  DomElement get semanticsHost => throw UnimplementedError();

  @override
  void setScene(DomElement sceneElement) {}

  @override
  DomElement get textEditingHost => throw UnimplementedError();
}
