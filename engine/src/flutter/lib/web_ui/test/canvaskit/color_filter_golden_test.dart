// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;

import 'common.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

const ui.Rect region = ui.Rect.fromLTRB(0, 0, 500, 250);

void testMain() {
  group('ColorFilter', () {
    setUpCanvasKitTest(withImplicitView: true);

    test('ColorFilter.matrix applies a color filter', () async {
      final LayerSceneBuilder builder = LayerSceneBuilder();

      builder.pushOffset(0, 0);

      // Draw a red circle and apply it to the scene.
      final CkPictureRecorder recorder = CkPictureRecorder();
      final CkCanvas canvas = recorder.beginRecording(region);

      canvas.drawCircle(
        const ui.Offset(75, 125),
        50,
        CkPaint()..color = const ui.Color.fromARGB(255, 255, 0, 0),
      );
      final CkPicture redCircle = recorder.endRecording();

      builder.addPicture(ui.Offset.zero, redCircle);

      // Apply a "greyscale" color filter.
      builder.pushColorFilter(
        const ui.ColorFilter.matrix(<double>[
          0.2126, 0.7152, 0.0722, 0, 0, //
          0.2126, 0.7152, 0.0722, 0, 0, //
          0.2126, 0.7152, 0.0722, 0, 0, //
          0, 0, 0, 1, 0, //
        ]),
      );

      // Draw another red circle and apply it to the scene.
      // This one should be grey since we have the color filter.
      final CkPictureRecorder recorder2 = CkPictureRecorder();
      final CkCanvas canvas2 = recorder2.beginRecording(region);

      canvas2.drawCircle(
        const ui.Offset(425, 125),
        50,
        CkPaint()..color = const ui.Color.fromARGB(255, 255, 0, 0),
      );
      final CkPicture greyCircle = recorder2.endRecording();

      builder.addPicture(ui.Offset.zero, greyCircle);

      await matchSceneGolden('canvaskit_colorfilter.png', builder.build(), region: region);
    });

    test('invertColors inverts the colors', () async {
      final LayerSceneBuilder builder = LayerSceneBuilder();

      builder.pushOffset(0, 0);

      // Draw a red circle and apply it to the scene.
      final CkPictureRecorder recorder = CkPictureRecorder();
      final CkCanvas canvas = recorder.beginRecording(region);

      canvas.drawCircle(
        const ui.Offset(75, 125),
        50,
        CkPaint()..color = const ui.Color.fromARGB(255, 255, 0, 0),
      );
      final CkPicture redCircle = recorder.endRecording();

      builder.addPicture(ui.Offset.zero, redCircle);

      // Draw another red circle with invertColors.
      final CkPictureRecorder recorder2 = CkPictureRecorder();
      final CkCanvas canvas2 = recorder2.beginRecording(region);

      canvas2.drawCircle(
        const ui.Offset(425, 125),
        50,
        CkPaint()
          ..color = const ui.Color.fromARGB(255, 255, 0, 0)
          ..invertColors = true,
      );
      final CkPicture invertedCircle = recorder2.endRecording();

      builder.addPicture(ui.Offset.zero, invertedCircle);

      await matchSceneGolden('canvaskit_invertcolors.png', builder.build(), region: region);
    });

    test('ColorFilter.matrix works for inverse matrix', () async {
      final LayerSceneBuilder builder = LayerSceneBuilder();

      builder.pushOffset(0, 0);

      // Draw a red, green, and blue square with the inverted color matrix.
      builder.pushColorFilter(
        const ui.ColorFilter.matrix(<double>[
          -1, 0, 0, 0, 255, //
          0, -1, 0, 0, 255, //
          0, 0, -1, 0, 255, //
          0, 0, 0, 1, 0, //
        ]),
      );

      final CkPictureRecorder recorder = CkPictureRecorder();

      final CkCanvas canvas = recorder.beginRecording(region);
      canvas.drawRect(
        const ui.Rect.fromLTWH(50, 50, 100, 100),
        CkPaint()..color = const ui.Color.fromARGB(255, 255, 0, 0),
      );
      canvas.drawRect(
        const ui.Rect.fromLTWH(200, 50, 100, 100),
        CkPaint()..color = const ui.Color.fromARGB(255, 0, 255, 0),
      );
      canvas.drawRect(
        const ui.Rect.fromLTWH(350, 50, 100, 100),
        CkPaint()..color = const ui.Color.fromARGB(255, 0, 0, 255),
      );
      final CkPicture invertedSquares = recorder.endRecording();

      builder.addPicture(ui.Offset.zero, invertedSquares);

      await matchSceneGolden('canvaskit_inverse_colormatrix.png', builder.build(), region: region);
    });

    test('ColorFilter color with 0 opacity', () async {
      final LayerSceneBuilder builder = LayerSceneBuilder();
      builder.pushOffset(0, 0);
      final CkPictureRecorder recorder = CkPictureRecorder();
      final CkCanvas canvas = recorder.beginRecording(region);

      canvas.drawCircle(
        const ui.Offset(75, 125),
        50,
        CkPaint()..color = const ui.Color.fromARGB(255, 255, 0, 0),
      );
      final CkPicture redCircle1 = recorder.endRecording();
      builder.addPicture(ui.Offset.zero, redCircle1);

      builder.pushColorFilter(
        ui.ColorFilter.mode(const ui.Color(0x00000000).withOpacity(0), ui.BlendMode.srcOver),
      );

      // Draw another red circle and apply it to the scene.
      // This one should also be red with the color filter doing nothing
      final CkPictureRecorder recorder2 = CkPictureRecorder();
      final CkCanvas canvas2 = recorder2.beginRecording(region);
      canvas2.drawCircle(
        const ui.Offset(425, 125),
        50,
        CkPaint()..color = const ui.Color.fromARGB(255, 255, 0, 0),
      );
      final CkPicture redCircle2 = recorder2.endRecording();

      builder.addPicture(ui.Offset.zero, redCircle2);

      await matchSceneGolden(
        'canvaskit_transparent_colorfilter.png',
        builder.build(),
        region: region,
      );
    });

    test('ColorFilter with dst blend mode', () async {
      final LayerSceneBuilder builder = LayerSceneBuilder();
      builder.pushOffset(0, 0);
      final CkPictureRecorder recorder = CkPictureRecorder();
      final CkCanvas canvas = recorder.beginRecording(region);

      canvas.drawCircle(
        const ui.Offset(75, 125),
        50,
        CkPaint()..color = const ui.Color.fromARGB(255, 255, 0, 0),
      );
      final CkPicture redCircle1 = recorder.endRecording();
      builder.addPicture(ui.Offset.zero, redCircle1);

      // Push dst color filter
      builder.pushColorFilter(const ui.ColorFilter.mode(ui.Color(0xffff0000), ui.BlendMode.dst));

      // Draw another red circle and apply it to the scene.
      // This one should also be red with the color filter doing nothing
      final CkPictureRecorder recorder2 = CkPictureRecorder();
      final CkCanvas canvas2 = recorder2.beginRecording(region);
      canvas2.drawCircle(
        const ui.Offset(425, 125),
        50,
        CkPaint()..color = const ui.Color.fromARGB(255, 255, 0, 0),
      );
      final CkPicture redCircle2 = recorder2.endRecording();

      builder.addPicture(ui.Offset.zero, redCircle2);

      await matchSceneGolden('canvaskit_dst_colorfilter.png', builder.build(), region: region);
    });

    test('ColorFilter only applies to child bounds', () async {
      final LayerSceneBuilder builder = LayerSceneBuilder();

      builder.pushOffset(0, 0);

      // Draw a red circle and add it to the scene.
      final CkPictureRecorder recorder = CkPictureRecorder();
      final CkCanvas canvas = recorder.beginRecording(region);

      canvas.drawCircle(
        const ui.Offset(75, 125),
        50,
        CkPaint()..color = const ui.Color.fromARGB(255, 255, 0, 0),
      );
      final CkPicture redCircle = recorder.endRecording();

      builder.addPicture(ui.Offset.zero, redCircle);

      // Apply a green color filter.
      builder.pushColorFilter(const ui.ColorFilter.mode(ui.Color(0xff00ff00), ui.BlendMode.color));
      // Draw another red circle and apply it to the scene.
      // This one should be green since we have the color filter.
      final CkPictureRecorder recorder2 = CkPictureRecorder();
      final CkCanvas canvas2 = recorder2.beginRecording(region);

      canvas2.drawCircle(
        const ui.Offset(425, 125),
        50,
        CkPaint()..color = const ui.Color.fromARGB(255, 255, 0, 0),
      );
      final CkPicture greenCircle = recorder2.endRecording();

      builder.addPicture(ui.Offset.zero, greenCircle);

      await matchSceneGolden('canvaskit_colorfilter_bounds.png', builder.build(), region: region);
    });

    test('ColorFilter works as an ImageFilter', () async {
      final LayerSceneBuilder builder = LayerSceneBuilder();

      builder.pushOffset(0, 0);

      // Draw a red circle and add it to the scene.
      final CkPictureRecorder recorder = CkPictureRecorder();
      final CkCanvas canvas = recorder.beginRecording(region);

      canvas.drawCircle(
        const ui.Offset(75, 125),
        50,
        CkPaint()..color = const ui.Color.fromARGB(255, 255, 0, 0),
      );
      final CkPicture redCircle = recorder.endRecording();

      builder.addPicture(ui.Offset.zero, redCircle);

      // Apply a green color filter as an ImageFilter.
      builder.pushImageFilter(const ui.ColorFilter.mode(ui.Color(0xff00ff00), ui.BlendMode.color));
      // Draw another red circle and apply it to the scene.
      // This one should be green since we have the color filter.
      final CkPictureRecorder recorder2 = CkPictureRecorder();
      final CkCanvas canvas2 = recorder2.beginRecording(region);

      canvas2.drawCircle(
        const ui.Offset(425, 125),
        50,
        CkPaint()..color = const ui.Color.fromARGB(255, 255, 0, 0),
      );
      final CkPicture greenCircle = recorder2.endRecording();

      builder.addPicture(ui.Offset.zero, greenCircle);

      await matchSceneGolden(
        'canvaskit_colorfilter_as_imagefilter.png',
        builder.build(),
        region: region,
      );
    });
    // TODO(hterkelsen): https://github.com/flutter/flutter/issues/71520
  }, skip: isSafari || isFirefox);
}
