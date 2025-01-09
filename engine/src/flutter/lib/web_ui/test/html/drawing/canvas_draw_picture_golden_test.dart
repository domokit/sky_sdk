// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart';

import '../../common/test_initialization.dart';
import '../screenshot.dart';

const Rect region = Rect.fromLTWH(0, 0, 500, 100);

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

SurfacePaint makePaint() => Paint() as SurfacePaint;

Future<void> testMain() async {
  setUpUnitTests(emulateTesterEnvironment: false, setUpTestViewDimensions: false);

  setUpAll(() async {
    debugShowClipLayers = true;
  });

  setUp(() async {
    SurfaceSceneBuilder.debugForgetFrameScene();
  });

  group('Add picture to scene', () {
    test('draw growing picture across frames', () async {
      final SurfaceSceneBuilder builder = SurfaceSceneBuilder();
      builder.pushClipRect(const Rect.fromLTRB(0, 0, 100, 100));

      _drawTestPicture(builder, 100, false);
      builder.pop();

      final DomElement elm1 = builder.build().webOnlyRootElement!;
      domDocument.body!.append(elm1);

      // Now draw picture again but at larger size.
      final SurfaceSceneBuilder builder2 = SurfaceSceneBuilder();
      builder2.pushClipRect(const Rect.fromLTRB(0, 0, 100, 100));
      // Now draw the picture at original target size, which will use a
      // different code path that should normally not have width/height set
      // on image element.
      _drawTestPicture(builder2, 20, false);
      builder2.pop();

      elm1.remove();
      await sceneScreenshot(builder2, 'canvas_draw_picture_acrossframes', region: region);
    });

    test('draw growing picture across frames clipped', () async {
      final SurfaceSceneBuilder builder = SurfaceSceneBuilder();
      builder.pushClipRect(const Rect.fromLTRB(0, 0, 100, 100));

      _drawTestPicture(builder, 100, true);
      builder.pop();

      final DomElement elm1 = builder.build().webOnlyRootElement!;
      domDocument.body!.append(elm1);

      // Now draw picture again but at larger size.
      final SurfaceSceneBuilder builder2 = SurfaceSceneBuilder();
      builder2.pushClipRect(const Rect.fromLTRB(0, 0, 100, 100));
      _drawTestPicture(builder2, 20, true);
      builder2.pop();

      elm1.remove();
      await sceneScreenshot(builder2, 'canvas_draw_picture_acrossframes_clipped', region: region);
    });

    test('PictureInPicture', () async {
      final SurfaceSceneBuilder builder = SurfaceSceneBuilder();
      final Picture greenRectPicture = _drawGreenRectIntoPicture();

      final EnginePictureRecorder recorder = PictureRecorder() as EnginePictureRecorder;
      final RecordingCanvas canvas = recorder.beginRecording(const Rect.fromLTRB(0, 0, 100, 100));
      canvas.drawPicture(greenRectPicture);
      builder.addPicture(const Offset(10, 10), recorder.endRecording());

      await sceneScreenshot(builder, 'canvas_draw_picture_in_picture_rect', region: region);
    });
  });
}

HtmlImage? sharedImage;

void _drawTestPicture(SceneBuilder builder, double targetSize, bool clipped) {
  sharedImage ??= _createRealTestImage();
  final EnginePictureRecorder recorder = PictureRecorder() as EnginePictureRecorder;
  final RecordingCanvas canvas = recorder.beginRecording(const Rect.fromLTRB(0, 0, 100, 100));
  canvas.debugEnforceArbitraryPaint();
  if (clipped) {
    canvas.clipRRect(RRect.fromLTRBR(0, 0, targetSize, targetSize, const Radius.circular(4)));
  }
  canvas.drawImageRect(
    sharedImage!,
    const Rect.fromLTWH(0, 0, 20, 20),
    Rect.fromLTWH(0, 0, targetSize, targetSize),
    makePaint(),
  );
  final Picture picture = recorder.endRecording();
  builder.addPicture(Offset.zero, picture);
}

Picture _drawGreenRectIntoPicture() {
  final EnginePictureRecorder recorder = PictureRecorder() as EnginePictureRecorder;
  final RecordingCanvas canvas = recorder.beginRecording(const Rect.fromLTRB(0, 0, 100, 100));
  canvas.drawRect(
    const Rect.fromLTWH(20, 20, 50, 50),
    makePaint()..color = const Color(0xFF00FF00),
  );
  return recorder.endRecording();
}

const String _base64Encoded20x20TestImage =
    'iVBORw0KGgoAAAANSUhEUgAAABQAAAAUCAIAAAAC64paAAAACXBIWXMAAC4jAAAuIwF4pT92AAAA'
    'B3RJTUUH5AMFFBksg4i3gQAAABl0RVh0Q29tbWVudABDcmVhdGVkIHdpdGggR0lNUFeBDhcAAAAj'
    'SURBVDjLY2TAC/7jlWVioACMah4ZmhnxpyHG0QAb1UyZZgBjWAIm/clP0AAAAABJRU5ErkJggg==';

HtmlImage _createRealTestImage() {
  return HtmlImage(
    createDomHTMLImageElement()..src = 'data:text/plain;base64,$_base64Encoded20x20TestImage',
    20,
    20,
  );
}
