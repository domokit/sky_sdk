// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart';

import '../../common/test_initialization.dart';
import '../screenshot.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  setUpUnitTests(emulateTesterEnvironment: false, setUpTestViewDimensions: false);

  setUp(() async {
    debugShowClipLayers = true;
    SurfaceSceneBuilder.debugForgetFrameScene();
  });

  tearDown(() {
    for (final DomNode scene in domDocument.querySelectorAll('flt-scene')) {
      scene.remove();
    }
  });

  test('drawColor should cover entire viewport', () async {
    const Rect region = Rect.fromLTWH(0, 0, 400, 400);

    final SurfaceSceneBuilder builder = SurfaceSceneBuilder();
    final Picture testPicture = _drawTestPicture(region, useColor: true);
    builder.addPicture(Offset.zero, testPicture);
    await sceneScreenshot(builder, 'canvas_draw_color', region: region);
  }, skip: true); // TODO(ferhat): matchGolden fails when a div covers viewport.

  test('drawPaint should cover entire viewport', () async {
    const Rect region = Rect.fromLTWH(0, 0, 400, 400);

    final SurfaceSceneBuilder builder = SurfaceSceneBuilder();
    final Picture testPicture = _drawTestPicture(region);
    builder.addPicture(Offset.zero, testPicture);
    await sceneScreenshot(builder, 'canvas_draw_paint', region: region);
  }, skip: true); // TODO(ferhat): matchGolden fails when a div covers viewport.);
}

Picture _drawTestPicture(Rect region, {bool useColor = false}) {
  final EnginePictureRecorder recorder = PictureRecorder() as EnginePictureRecorder;
  const Rect r = Rect.fromLTWH(0, 0, 200, 200);
  final RecordingCanvas canvas = recorder.beginRecording(r);

  canvas.drawRect(
    region.deflate(8.0),
    Paint() as SurfacePaint
      ..style = PaintingStyle.fill
      ..color = const Color(0xFFE0E0E0),
  );

  canvas.transform(Matrix4.translationValues(50, 50, 0).storage);

  if (useColor) {
    canvas.drawColor(const Color.fromRGBO(0, 255, 0, 1), BlendMode.srcOver);
  } else {
    canvas.drawPaint(
      Paint() as SurfacePaint
        ..style = PaintingStyle.fill
        ..color = const Color.fromRGBO(0, 0, 255, 1),
    );
  }

  canvas.drawCircle(
    Offset(r.width / 2, r.height / 2),
    r.width / 2,
    Paint() as SurfacePaint
      ..style = PaintingStyle.fill
      ..color = const Color.fromRGBO(255, 0, 0, 1),
  );

  return recorder.endRecording();
}
