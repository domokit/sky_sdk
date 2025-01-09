// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart' hide BackdropFilterEngineLayer, ClipRectEngineLayer;
import 'package:ui/ui.dart';

import 'package:web_engine_tester/golden_tester.dart';

import '../../common/test_initialization.dart';

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

  // The black circle on the left should not be blurred since it is outside
  // the clip boundary around backdrop filter. However there should be only
  // one red dot since the other one should be blurred by filter.
  test('Background should only blur at ancestor clip boundary', () async {
    const Rect region = Rect.fromLTWH(0, 0, 190, 130);

    final SurfaceSceneBuilder builder = SurfaceSceneBuilder();
    final Picture backgroundPicture = _drawBackground(region);
    builder.addPicture(Offset.zero, backgroundPicture);

    builder.pushClipRect(const Rect.fromLTRB(10, 10, 180, 120));
    final Picture circles1 = _drawTestPictureWithCircles(region, 30, 30);
    builder.addPicture(Offset.zero, circles1);

    builder.pushClipRect(const Rect.fromLTRB(60, 10, 180, 120));
    builder.pushBackdropFilter(ImageFilter.blur(sigmaX: 10.0, sigmaY: 10.0));
    final Picture circles2 = _drawTestPictureWithCircles(region, 90, 30);
    builder.addPicture(Offset.zero, circles2);
    builder.pop();
    builder.pop();
    builder.pop();

    domDocument.body!.append(builder.build().webOnlyRootElement!);

    await matchGoldenFile('backdrop_filter_clip.png', region: region);
  });

  test('Background should only blur at ancestor clip boundary after move', () async {
    const Rect region = Rect.fromLTWH(0, 0, 190, 130);

    final SurfaceSceneBuilder builder = SurfaceSceneBuilder();
    final Picture backgroundPicture = _drawBackground(region);
    builder.addPicture(Offset.zero, backgroundPicture);
    final ClipRectEngineLayer clipEngineLayer = builder.pushClipRect(
      const Rect.fromLTRB(10, 10, 180, 120),
    );
    final Picture circles1 = _drawTestPictureWithCircles(region, 30, 30);
    builder.addPicture(Offset.zero, circles1);
    final ClipRectEngineLayer clipEngineLayer2 = builder.pushClipRect(
      const Rect.fromLTRB(60, 10, 180, 120),
    );
    final BackdropFilterEngineLayer oldBackdropFilterLayer = builder.pushBackdropFilter(
      ImageFilter.blur(sigmaX: 10.0, sigmaY: 10.0),
    );
    final Picture circles2 = _drawTestPictureWithCircles(region, 90, 30);
    builder.addPicture(Offset.zero, circles2);
    builder.pop();
    builder.pop();
    builder.pop();
    builder.build();

    // Now reparent filter layer in next scene.
    final SurfaceSceneBuilder builder2 = SurfaceSceneBuilder();
    builder2.addPicture(Offset.zero, backgroundPicture);
    builder2.pushClipRect(const Rect.fromLTRB(10, 10, 180, 120), oldLayer: clipEngineLayer);
    builder2.addPicture(Offset.zero, circles1);
    builder2.pushClipRect(const Rect.fromLTRB(10, 75, 180, 120), oldLayer: clipEngineLayer2);
    builder2.pushBackdropFilter(
      ImageFilter.blur(sigmaX: 10.0, sigmaY: 10.0),
      oldLayer: oldBackdropFilterLayer,
    );
    builder2.addPicture(Offset.zero, circles2);
    builder2.pop();
    builder2.pop();
    builder2.pop();

    domDocument.body!.append(builder2.build().webOnlyRootElement!);

    await matchGoldenFile('backdrop_filter_clip_moved.png', region: region);
  });

  // The blur filter should be applied to the background inside the clip even
  // though there are no children of the backdrop filter.
  test('Background should blur even if child does not paint', () async {
    const Rect region = Rect.fromLTWH(0, 0, 190, 130);

    final SurfaceSceneBuilder builder = SurfaceSceneBuilder();
    final Picture backgroundPicture = _drawBackground(region);
    builder.addPicture(Offset.zero, backgroundPicture);

    builder.pushClipRect(const Rect.fromLTRB(10, 10, 180, 120));
    final Picture circles1 = _drawTestPictureWithCircles(region, 30, 30);
    builder.addPicture(Offset.zero, circles1);

    builder.pushClipRect(const Rect.fromLTRB(60, 10, 180, 120));
    builder.pushBackdropFilter(ImageFilter.blur(sigmaX: 10.0, sigmaY: 10.0));
    builder.pop();
    builder.pop();
    builder.pop();

    domDocument.body!.append(builder.build().webOnlyRootElement!);

    await matchGoldenFile('backdrop_filter_no_child_rendering.png', region: region);
  });
  test('colorFilter as imageFilter', () async {
    const Rect region = Rect.fromLTWH(0, 0, 190, 130);

    final SurfaceSceneBuilder builder = SurfaceSceneBuilder();
    final Picture backgroundPicture = _drawBackground(region);
    builder.addPicture(Offset.zero, backgroundPicture);

    builder.pushClipRect(const Rect.fromLTRB(10, 10, 180, 120));
    final Picture circles1 = _drawTestPictureWithCircles(region, 30, 30);

    // current background color is light green, apply a light yellow colorFilter
    const ColorFilter colorFilter = ColorFilter.mode(Color(0xFFFFFFB1), BlendMode.modulate);
    builder.pushBackdropFilter(colorFilter);
    builder.addPicture(Offset.zero, circles1);
    builder.pop();

    domDocument.body!.append(builder.build().webOnlyRootElement!);

    await matchGoldenFile('backdrop_filter_colorFilter_as_imageFilter.png', region: region);
  });
}

Picture _drawTestPictureWithCircles(Rect region, double offsetX, double offsetY) {
  final EnginePictureRecorder recorder = PictureRecorder() as EnginePictureRecorder;
  final RecordingCanvas canvas = recorder.beginRecording(region);
  canvas.drawCircle(
    Offset(offsetX + 10, offsetY + 10),
    10,
    SurfacePaint()..style = PaintingStyle.fill,
  );
  canvas.drawCircle(
    Offset(offsetX + 60, offsetY + 10),
    10,
    SurfacePaint()
      ..style = PaintingStyle.fill
      ..color = const Color.fromRGBO(255, 0, 0, 1),
  );
  canvas.drawCircle(
    Offset(offsetX + 10, offsetY + 60),
    10,
    SurfacePaint()
      ..style = PaintingStyle.fill
      ..color = const Color.fromRGBO(0, 255, 0, 1),
  );
  canvas.drawCircle(
    Offset(offsetX + 60, offsetY + 60),
    10,
    SurfacePaint()
      ..style = PaintingStyle.fill
      ..color = const Color.fromRGBO(0, 0, 255, 1),
  );
  return recorder.endRecording();
}

Picture _drawBackground(Rect region) {
  final EnginePictureRecorder recorder = PictureRecorder() as EnginePictureRecorder;
  final RecordingCanvas canvas = recorder.beginRecording(region);
  canvas.drawRect(
    region.deflate(8.0),
    SurfacePaint()
      ..style = PaintingStyle.fill
      ..color = const Color(0xFFE0FFE0),
  );
  return recorder.endRecording();
}
