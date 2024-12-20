// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;
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
  group('ShaderMask', () {
    setUpCanvasKitTest(withImplicitView: true);

    test('Renders sweep gradient with color blend', () async {
      final LayerSceneBuilder builder = LayerSceneBuilder();

      builder.pushOffset(0, 0);

      // Draw a red circle and apply it to the scene.
      final CkPictureRecorder recorder = CkPictureRecorder();
      final CkCanvas canvas = recorder.beginRecording(region);

      canvas.drawCircle(
        const ui.Offset(425, 125),
        50,
        CkPaint()..color = const ui.Color.fromARGB(255, 255, 0, 0),
      );
      final CkPicture redCircle = recorder.endRecording();

      builder.addPicture(ui.Offset.zero, redCircle);

      final CkGradientSweep shader = CkGradientSweep(
        const ui.Offset(250, 125),
        const <ui.Color>[
          ui.Color(0xFF4285F4),
          ui.Color(0xFF34A853),
          ui.Color(0xFFFBBC05),
          ui.Color(0xFFEA4335),
          ui.Color(0xFF4285F4),
        ],
        const <double>[0.0, 0.25, 0.5, 0.75, 1.0],
        ui.TileMode.clamp,
        -(math.pi / 2),
        math.pi * 2 - (math.pi / 2),
        null,
      );

      final ui.Path clipPath = ui.Path()..addOval(const ui.Rect.fromLTWH(25, 75, 100, 100));
      builder.pushClipPath(clipPath);

      // Apply a shader mask.
      builder.pushShaderMask(shader, const ui.Rect.fromLTRB(0, 0, 200, 250), ui.BlendMode.color);

      // Draw another red circle and apply it to the scene.
      // This one should be grey since we have the color filter.
      final CkPictureRecorder recorder2 = CkPictureRecorder();
      final CkCanvas canvas2 = recorder2.beginRecording(region);

      canvas2.drawRect(
        const ui.Rect.fromLTWH(25, 75, 100, 100),
        CkPaint()..color = const ui.Color.fromARGB(255, 0, 255, 0),
      );

      final CkPicture sweepCircle = recorder2.endRecording();

      builder.addPicture(ui.Offset.zero, sweepCircle);

      await matchSceneGolden('canvaskit_shadermask_linear.png', builder.build(), region: region);
    });

    /// Regression test for https://github.com/flutter/flutter/issues/78959
    test('Renders sweep gradient with color blend translated', () async {
      final LayerSceneBuilder builder = LayerSceneBuilder();

      builder.pushOffset(0, 0);

      // Draw a red circle and apply it to the scene.
      final CkPictureRecorder recorder = CkPictureRecorder();
      final CkCanvas canvas = recorder.beginRecording(region);

      canvas.drawCircle(
        const ui.Offset(425, 125),
        50,
        CkPaint()..color = const ui.Color.fromARGB(255, 255, 0, 0),
      );
      final CkPicture redCircle = recorder.endRecording();

      builder.addPicture(ui.Offset.zero, redCircle);

      final CkGradientSweep shader = CkGradientSweep(
        const ui.Offset(250, 125),
        const <ui.Color>[
          ui.Color(0xFF4285F4),
          ui.Color(0xFF34A853),
          ui.Color(0xFFFBBC05),
          ui.Color(0xFFEA4335),
          ui.Color(0xFF4285F4),
        ],
        const <double>[0.0, 0.25, 0.5, 0.75, 1.0],
        ui.TileMode.clamp,
        -(math.pi / 2),
        math.pi * 2 - (math.pi / 2),
        null,
      );

      final ui.Path clipPath = ui.Path()..addOval(const ui.Rect.fromLTWH(25, 75, 100, 100));
      builder.pushClipPath(clipPath);

      // Apply a shader mask.
      builder.pushShaderMask(shader, const ui.Rect.fromLTRB(50, 50, 200, 250), ui.BlendMode.color);

      // Draw another red circle and apply it to the scene.
      // This one should be grey since we have the color filter.
      final CkPictureRecorder recorder2 = CkPictureRecorder();
      final CkCanvas canvas2 = recorder2.beginRecording(region);

      canvas2.drawRect(
        const ui.Rect.fromLTWH(25, 75, 100, 100),
        CkPaint()..color = const ui.Color.fromARGB(255, 0, 255, 0),
      );

      final CkPicture sweepCircle = recorder2.endRecording();

      builder.addPicture(ui.Offset.zero, sweepCircle);

      await matchSceneGolden(
        'canvaskit_shadermask_linear_translated.png',
        builder.build(),
        region: region,
      );
    });
    // TODO(hterkelsen): https://github.com/flutter/flutter/issues/71520
  }, skip: isSafari || isFirefox);
}
