// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';

import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;

import '../common/test_initialization.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  group('$LayerScene', () {
    setUpAll(() async {
      await bootstrapAndRunApp();
    });

    test('toImage returns an image', () async {
      final ui.PictureRecorder recorder = ui.PictureRecorder();
      expect(recorder, isA<CkPictureRecorder>());

      final ui.Canvas canvas = ui.Canvas(recorder);
      expect(canvas, isA<CanvasKitCanvas>());

      final ui.Paint paint = ui.Paint();
      expect(paint, isA<CkPaint>());
      paint.color = const ui.Color.fromARGB(255, 255, 0, 0);

      // Draw a red circle.
      canvas.drawCircle(const ui.Offset(20, 20), 10, paint);

      final ui.Picture picture = recorder.endRecording();
      expect(picture, isA<CkPicture>());

      final ui.SceneBuilder builder = ui.SceneBuilder();
      expect(builder, isA<LayerSceneBuilder>());

      builder.pushOffset(0, 0);
      builder.addPicture(ui.Offset.zero, picture);

      final ui.Scene scene = builder.build();

      final ui.Image sceneImage = await scene.toImage(100, 100);
      expect(sceneImage, isA<CkImage>());
    });

    test('pushColorFilter does not throw', () async {
      final ui.SceneBuilder builder = ui.SceneBuilder();
      expect(builder, isA<LayerSceneBuilder>());

      builder.pushOffset(0, 0);
      builder.pushColorFilter(const ui.ColorFilter.srgbToLinearGamma());

      final ui.Scene scene = builder.build();
      expect(scene, isNotNull);
    });
  });
}
