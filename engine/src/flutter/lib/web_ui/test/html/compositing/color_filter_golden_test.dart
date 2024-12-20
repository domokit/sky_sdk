// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart';

import 'package:web_engine_tester/golden_tester.dart';

import '../../common/test_initialization.dart';

const Rect region = Rect.fromLTWH(0, 0, 500, 500);

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  setUpUnitTests(
    withImplicitView: true,
    emulateTesterEnvironment: false,
    setUpTestViewDimensions: false,
  );

  setUp(() async {
    debugShowClipLayers = true;
    SurfaceSceneBuilder.debugForgetFrameScene();
    for (final DomNode scene in domDocument.querySelectorAll('flt-scene')) {
      scene.remove();
    }
  });

  test('Should apply color filter to image', () async {
    final SurfaceSceneBuilder builder = SurfaceSceneBuilder();
    final Picture backgroundPicture = _drawBackground();
    builder.addPicture(Offset.zero, backgroundPicture);
    builder.pushColorFilter(const EngineColorFilter.mode(Color(0xF0000080), BlendMode.color));
    final Picture circles1 = _drawTestPictureWithCircles(30, 30);
    builder.addPicture(Offset.zero, circles1);
    builder.pop();
    domDocument.body!.append(builder.build().webOnlyRootElement!);

    // TODO(ferhat): update golden for this test after canvas sandwich detection is
    // added to RecordingCanvas.
    await matchGoldenFile('color_filter_blendMode_color.png', region: region);
  });

  test('Should apply matrix color filter to image', () async {
    final List<double> colorMatrix = <double>[
      0.2126, 0.7152, 0.0722, 0, 0, //
      0.2126, 0.7152, 0.0722, 0, 0, //
      0.2126, 0.7152, 0.0722, 0, 0, //
      0, 0, 0, 1, 0, //
    ];
    final SurfaceSceneBuilder builder = SurfaceSceneBuilder();
    final Picture backgroundPicture = _drawBackground();
    builder.addPicture(Offset.zero, backgroundPicture);
    builder.pushColorFilter(EngineColorFilter.matrix(colorMatrix));
    final Picture circles1 = _drawTestPictureWithCircles(30, 30);
    builder.addPicture(Offset.zero, circles1);
    builder.pop();
    domDocument.body!.append(builder.build().webOnlyRootElement!);
    await matchGoldenFile('color_filter_matrix.png', region: region);
  });

  /// Regression test for https://github.com/flutter/flutter/issues/85733
  test('Should apply mode color filter to circles', () async {
    final SurfaceSceneBuilder builder = SurfaceSceneBuilder();
    final Picture backgroundPicture = _drawBackground();
    builder.addPicture(Offset.zero, backgroundPicture);
    builder.pushColorFilter(const ColorFilter.mode(Color(0xFFFF0000), BlendMode.srcIn));
    final Picture circles1 = _drawTestPictureWithCircles(30, 30);
    builder.addPicture(Offset.zero, circles1);
    builder.pop();
    domDocument.body!.append(builder.build().webOnlyRootElement!);
    await matchGoldenFile('color_filter_mode.png', region: region);
  });
}

Picture _drawTestPictureWithCircles(double offsetX, double offsetY) {
  final EnginePictureRecorder recorder = PictureRecorder() as EnginePictureRecorder;
  final RecordingCanvas canvas = recorder.beginRecording(const Rect.fromLTRB(0, 0, 400, 400));
  canvas.drawCircle(
    Offset(offsetX + 10, offsetY + 10),
    10,
    (Paint()..style = PaintingStyle.fill) as SurfacePaint,
  );
  canvas.drawCircle(
    Offset(offsetX + 60, offsetY + 10),
    10,
    (Paint()
          ..style = PaintingStyle.fill
          ..color = const Color.fromRGBO(255, 0, 0, 1))
        as SurfacePaint,
  );
  canvas.drawCircle(
    Offset(offsetX + 10, offsetY + 60),
    10,
    (Paint()
          ..style = PaintingStyle.fill
          ..color = const Color.fromRGBO(0, 255, 0, 1))
        as SurfacePaint,
  );
  canvas.drawCircle(
    Offset(offsetX + 60, offsetY + 60),
    10,
    (Paint()
          ..style = PaintingStyle.fill
          ..color = const Color.fromRGBO(0, 0, 255, 1))
        as SurfacePaint,
  );
  return recorder.endRecording();
}

Picture _drawBackground() {
  final EnginePictureRecorder recorder = PictureRecorder() as EnginePictureRecorder;
  final RecordingCanvas canvas = recorder.beginRecording(const Rect.fromLTRB(0, 0, 400, 400));
  canvas.drawRect(
    const Rect.fromLTWH(8, 8, 400.0 - 16, 400.0 - 16),
    (Paint()
          ..style = PaintingStyle.fill
          ..color = const Color(0xFFE0FFE0))
        as SurfacePaint,
  );
  return recorder.endRecording();
}
