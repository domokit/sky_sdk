// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;
import 'package:web_engine_tester/golden_tester.dart';

import '../common/rendering.dart';
import '../common/test_initialization.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  setUpUnitTests(
    withImplicitView: true,
    emulateTesterEnvironment: false,
    setUpTestViewDimensions: false,
  );

  const ui.Rect region = ui.Rect.fromLTWH(0, 0, 300, 300);
  const String platformViewType = 'test-platform-view';

  setUp(() {
    ui_web.platformViewRegistry.registerViewFactory(platformViewType, (int viewId) {
      final DomElement element = createDomHTMLDivElement();
      element.style.backgroundColor = 'blue';
      element.style.width = '100%';
      element.style.height = '100%';
      return element;
    });
  });

  tearDown(() {
    PlatformViewManager.instance.debugClear();
  });

  test('picture + overlapping platformView', () async {
    await _createPlatformView(1, platformViewType);

    final ui.PictureRecorder recorder = ui.PictureRecorder();
    final ui.Canvas canvas = ui.Canvas(recorder);
    canvas.drawCircle(
      const ui.Offset(50, 50),
      50,
      ui.Paint()
        ..style = ui.PaintingStyle.fill
        ..color = const ui.Color(0xFFFF0000),
    );

    final ui.SceneBuilder sb = ui.SceneBuilder();
    sb.pushOffset(0, 0);
    sb.addPicture(const ui.Offset(100, 100), recorder.endRecording());

    sb.addPlatformView(1, offset: const ui.Offset(125, 125), width: 50, height: 50);
    await renderScene(sb.build());

    await matchGoldenFile('picture_platformview_overlap.png', region: region);
  });

  test('platformView sandwich', () async {
    await _createPlatformView(1, platformViewType);

    final ui.PictureRecorder recorder = ui.PictureRecorder();
    final ui.Canvas canvas = ui.Canvas(recorder);
    canvas.drawCircle(
      const ui.Offset(50, 50),
      50,
      ui.Paint()
        ..style = ui.PaintingStyle.fill
        ..color = const ui.Color(0xFF00FF00),
    );

    final ui.Picture picture = recorder.endRecording();

    final ui.SceneBuilder sb = ui.SceneBuilder();
    sb.pushOffset(0, 0);
    sb.addPicture(const ui.Offset(75, 75), picture);

    sb.addPlatformView(1, offset: const ui.Offset(100, 100), width: 100, height: 100);

    sb.addPicture(const ui.Offset(125, 125), picture);
    await renderScene(sb.build());

    await matchGoldenFile('picture_platformview_sandwich.png', region: region);
  });

  test('transformed platformview', () async {
    await _createPlatformView(1, platformViewType);

    final ui.PictureRecorder recorder = ui.PictureRecorder();
    final ui.Canvas canvas = ui.Canvas(recorder);
    canvas.drawCircle(
      const ui.Offset(50, 50),
      50,
      ui.Paint()
        ..style = ui.PaintingStyle.fill
        ..color = const ui.Color(0xFFFF0000),
    );

    final ui.SceneBuilder sb = ui.SceneBuilder();
    sb.pushOffset(0, 0);
    sb.addPicture(const ui.Offset(100, 100), recorder.endRecording());

    sb.pushTransform(Matrix4.rotationZ(0.1).toFloat64());
    sb.addPlatformView(1, offset: const ui.Offset(125, 125), width: 50, height: 50);
    await renderScene(sb.build());

    await matchGoldenFile('platformview_transformed.png', region: region);
  });

  test('transformed and offset platformview', () async {
    await _createPlatformView(1, platformViewType);

    final ui.PictureRecorder recorder = ui.PictureRecorder();
    final ui.Canvas canvas = ui.Canvas(recorder);
    canvas.drawCircle(
      const ui.Offset(50, 50),
      50,
      ui.Paint()
        ..style = ui.PaintingStyle.fill
        ..color = const ui.Color(0xFFFF0000),
    );

    final ui.SceneBuilder sb = ui.SceneBuilder();
    sb.pushOffset(0, 0);
    sb.addPicture(const ui.Offset(100, 100), recorder.endRecording());

    // Nest offsets both before and after the transform to make sure that they
    // are applied properly.
    sb.pushOffset(50, 50);
    sb.pushTransform(Matrix4.rotationZ(0.1).toFloat64());
    sb.pushOffset(25, 25);
    sb.addPlatformView(1, offset: const ui.Offset(50, 50), width: 50, height: 50);
    await renderScene(sb.build());

    await matchGoldenFile('platformview_transformed_offset.png', region: region);
  });

  test('offset platformview', () async {
    await _createPlatformView(1, platformViewType);

    final ui.PictureRecorder recorder = ui.PictureRecorder();
    final ui.Canvas canvas = ui.Canvas(recorder);
    canvas.drawCircle(
      const ui.Offset(50, 50),
      50,
      ui.Paint()
        ..style = ui.PaintingStyle.fill
        ..color = const ui.Color(0xFFFF0000),
    );

    final ui.Picture picture = recorder.endRecording();

    final ui.SceneBuilder sb = ui.SceneBuilder();
    sb.pushOffset(50, 50);
    sb.addPicture(const ui.Offset(100, 100), picture);

    final ui.EngineLayer retainedPlatformView = sb.pushOffset(50, 50);
    sb.addPlatformView(1, offset: const ui.Offset(125, 125), width: 50, height: 50);
    await renderScene(sb.build());

    await matchGoldenFile('platformview_offset.png', region: region);

    final ui.SceneBuilder sb2 = ui.SceneBuilder();
    sb2.pushOffset(0, 0);
    sb2.addPicture(const ui.Offset(100, 100), picture);

    sb2.addRetained(retainedPlatformView);
    await renderScene(sb2.build());

    await matchGoldenFile('platformview_offset_moved.png', region: region);
  });

  test('platformview with opacity', () async {
    await _createPlatformView(1, platformViewType);

    final ui.PictureRecorder recorder = ui.PictureRecorder();
    final ui.Canvas canvas = ui.Canvas(recorder);
    canvas.drawCircle(
      const ui.Offset(50, 50),
      50,
      ui.Paint()
        ..style = ui.PaintingStyle.fill
        ..color = const ui.Color(0xFFFF0000),
    );

    final ui.SceneBuilder sb = ui.SceneBuilder();
    sb.pushOffset(0, 0);
    sb.addPicture(const ui.Offset(100, 100), recorder.endRecording());

    sb.pushOpacity(127, offset: const ui.Offset(50, 50));
    sb.addPlatformView(1, offset: const ui.Offset(125, 125), width: 50, height: 50);
    await renderScene(sb.build());

    await matchGoldenFile('platformview_opacity.png', region: region);
  });

  test('platformview cliprect', () async {
    await _createPlatformView(1, platformViewType);

    final ui.PictureRecorder recorder = ui.PictureRecorder();
    final ui.Canvas canvas = ui.Canvas(recorder);
    canvas.drawCircle(
      const ui.Offset(50, 50),
      50,
      ui.Paint()
        ..style = ui.PaintingStyle.fill
        ..color = const ui.Color(0xFFFF0000),
    );

    final ui.SceneBuilder sb = ui.SceneBuilder();
    sb.pushOffset(50, 50);
    sb.pushClipRect(const ui.Rect.fromLTRB(60, 60, 100, 100));

    sb.addPicture(const ui.Offset(50, 50), recorder.endRecording());
    sb.addPlatformView(1, offset: const ui.Offset(75, 75), width: 50, height: 50);
    await renderScene(sb.build());

    await matchGoldenFile('platformview_cliprect.png', region: region);
  });

  test('platformview cliprrect', () async {
    await _createPlatformView(1, platformViewType);

    final ui.PictureRecorder recorder = ui.PictureRecorder();
    final ui.Canvas canvas = ui.Canvas(recorder);
    canvas.drawCircle(
      const ui.Offset(50, 50),
      50,
      ui.Paint()
        ..style = ui.PaintingStyle.fill
        ..color = const ui.Color(0xFFFF0000),
    );

    final ui.SceneBuilder sb = ui.SceneBuilder();
    sb.pushOffset(50, 50);
    sb.pushClipRRect(
      const ui.RRect.fromLTRBXY(60, 60, 100, 100, 5, 10),
      clipBehavior: ui.Clip.antiAlias,
    );

    sb.addPicture(const ui.Offset(50, 50), recorder.endRecording());
    sb.addPlatformView(1, offset: const ui.Offset(75, 75), width: 50, height: 50);
    await renderScene(sb.build());

    await matchGoldenFile('platformview_cliprrect.png', region: region);
  });

  test('platformview covered clip', () async {
    await _createPlatformView(1, platformViewType);

    final ui.PictureRecorder recorder = ui.PictureRecorder();
    final ui.Canvas canvas = ui.Canvas(recorder);
    canvas.drawCircle(
      const ui.Offset(50, 50),
      50,
      ui.Paint()
        ..style = ui.PaintingStyle.fill
        ..color = const ui.Color(0xFFFF0000),
    );

    final ui.SceneBuilder sb = ui.SceneBuilder();
    sb.pushOffset(50, 50);

    // The rrect should completely cover the rect for this test case.
    sb.pushClipRRect(
      const ui.RRect.fromLTRBXY(50, 50, 110, 110, 5, 10),
      clipBehavior: ui.Clip.antiAlias,
    );
    sb.pushClipRect(const ui.Rect.fromLTRB(60, 60, 100, 100));

    sb.addPicture(const ui.Offset(50, 50), recorder.endRecording());
    sb.addPlatformView(1, offset: const ui.Offset(75, 75), width: 50, height: 50);
    await renderScene(sb.build());

    await matchGoldenFile('platformview_covered_clip.png', region: region);
  });

  test('platformview clippath', () async {
    await _createPlatformView(1, platformViewType);

    final ui.PictureRecorder recorder = ui.PictureRecorder();
    final ui.Canvas canvas = ui.Canvas(recorder);
    canvas.drawCircle(
      const ui.Offset(50, 50),
      50,
      ui.Paint()
        ..style = ui.PaintingStyle.fill
        ..color = const ui.Color(0xFFFF0000),
    );

    final ui.SceneBuilder sb = ui.SceneBuilder();
    sb.pushOffset(50, 50);

    final ui.Path path = ui.Path();
    path.moveTo(80, 100);
    path.lineTo(60, 75);
    path.arcToPoint(const ui.Offset(80, 75), radius: const ui.Radius.elliptical(10, 15));
    path.arcToPoint(const ui.Offset(100, 75), radius: const ui.Radius.elliptical(10, 15));
    path.close();
    sb.pushClipPath(path);

    sb.addPicture(const ui.Offset(50, 50), recorder.endRecording());
    sb.addPlatformView(1, offset: const ui.Offset(75, 75), width: 50, height: 50);
    await renderScene(sb.build());

    await matchGoldenFile('platformview_clippath.png', region: region);
  });
}

// Sends a platform message to create a Platform View with the given id and viewType.
Future<void> _createPlatformView(int id, String viewType) {
  final Completer<void> completer = Completer<void>();
  const MethodCodec codec = StandardMethodCodec();
  ui.PlatformDispatcher.instance.sendPlatformMessage(
    'flutter/platform_views',
    codec.encodeMethodCall(MethodCall('create', <String, dynamic>{'id': id, 'viewType': viewType})),
    (dynamic _) => completer.complete(),
  );
  return completer.future;
}
