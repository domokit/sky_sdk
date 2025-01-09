// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;

import 'common.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  group('CanvasKit', () {
    setUpCanvasKitTest(withImplicitView: true);

    // Regression test for https://github.com/flutter/flutter/issues/63715
    test('TransformLayer prerolls correctly', () async {
      final CkPicture picture = paintPicture(const ui.Rect.fromLTRB(0, 0, 60, 60), (
        CkCanvas canvas,
      ) {
        canvas.drawRect(
          const ui.Rect.fromLTRB(0, 0, 60, 60),
          CkPaint()..style = ui.PaintingStyle.fill,
        );
      });

      final LayerSceneBuilder sb = LayerSceneBuilder();
      sb.pushClipRect(const ui.Rect.fromLTRB(15, 15, 30, 30));

      // Intentionally use a perspective transform, which triggered the
      // https://github.com/flutter/flutter/issues/63715 bug.
      sb.pushTransform(Float64List.fromList(Matrix4.identity().storage..[15] = 2));

      sb.addPicture(ui.Offset.zero, picture);
      final LayerScene scene = sb.build();
      final LayerTree layerTree = scene.layerTree;
      await renderScene(scene);
      final ClipRectEngineLayer clipRect =
          layerTree.rootLayer.debugLayers.single as ClipRectEngineLayer;
      expect(clipRect.paintBounds, const ui.Rect.fromLTRB(15, 15, 30, 30));

      final TransformEngineLayer transform = clipRect.debugLayers.single as TransformEngineLayer;
      expect(transform.paintBounds, const ui.Rect.fromLTRB(0, 0, 30, 30));
    });

    test('can push a leaf layer without a container layer', () async {
      final CkPictureRecorder recorder = CkPictureRecorder();
      recorder.beginRecording(ui.Rect.zero);
      LayerSceneBuilder().addPicture(ui.Offset.zero, recorder.endRecording());
    });

    test('null ViewEmbedder with PlatformView', () async {
      final LayerSceneBuilder sb = LayerSceneBuilder();
      const ui.Rect kDefaultRegion = ui.Rect.fromLTRB(0, 0, 200, 200);
      await createPlatformView(0, 'test-platform-view');
      sb.pushOffset(0, 0);
      sb.addPlatformView(0, width: 10, height: 10);
      sb.pushOffset(0, 0);
      final LayerScene layerScene = sb.build();
      final ui.Image testImage = await layerScene.toImage(100, 100);

      final CkPictureRecorder recorder = CkPictureRecorder();
      final CkCanvas canvas = recorder.beginRecording(kDefaultRegion);
      canvas.drawImage(testImage as CkImage, ui.Offset.zero, CkPaint());
      await matchPictureGolden(
        'canvaskit_null_viewembedder_with_platformview.png',
        recorder.endRecording(),
        region: kDefaultRegion,
      );
    });

    test('ImageFilter layer applies matrix in preroll', () async {
      final CkPicture picture = paintPicture(const ui.Rect.fromLTRB(0, 0, 100, 100), (
        CkCanvas canvas,
      ) {
        canvas.drawRect(
          const ui.Rect.fromLTRB(0, 0, 100, 100),
          CkPaint()..style = ui.PaintingStyle.fill,
        );
      });

      final LayerSceneBuilder sb = LayerSceneBuilder();
      sb.pushImageFilter(
        ui.ImageFilter.matrix(
          (Matrix4.identity()
                ..scale(0.5, 0.5)
                ..translate(20))
              .toFloat64(),
        ),
      );
      sb.addPicture(ui.Offset.zero, picture);

      final LayerScene scene = sb.build();
      final LayerTree layerTree = scene.layerTree;
      await renderScene(scene);

      final ImageFilterEngineLayer imageFilterLayer =
          layerTree.rootLayer.debugLayers.single as ImageFilterEngineLayer;
      expect(imageFilterLayer.paintBounds, const ui.Rect.fromLTRB(10, 0, 60, 50));
    });

    test('Opacity layer works correctly with Scene.toImage', () async {
      // This is a regression test for https://github.com/flutter/flutter/issues/138009
      final CkPicture picture = paintPicture(const ui.Rect.fromLTRB(0, 0, 100, 100), (
        CkCanvas canvas,
      ) {
        canvas.drawRect(
          const ui.Rect.fromLTRB(0, 0, 100, 100),
          CkPaint()..style = ui.PaintingStyle.fill,
        );
      });

      final LayerSceneBuilder sb = LayerSceneBuilder();
      sb.pushTransform(Matrix4.identity().toFloat64());
      sb.pushOpacity(97, offset: const ui.Offset(20, 20));
      sb.addPicture(ui.Offset.zero, picture);

      final LayerScene scene = sb.build();
      final ui.Image testImage = await scene.toImage(200, 200);

      final CkPictureRecorder recorder = CkPictureRecorder();
      final CkCanvas canvas = recorder.beginRecording(const ui.Rect.fromLTRB(0, 0, 200, 200));
      canvas.drawImage(testImage as CkImage, ui.Offset.zero, CkPaint());
      await matchPictureGolden(
        'canvaskit_scene_toimage_opacity_layer.png',
        recorder.endRecording(),
        region: const ui.Rect.fromLTRB(0, 0, 200, 200),
      );
    });
  });
}
