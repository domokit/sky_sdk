// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';

import 'package:ui/ui.dart' as ui;

import 'scene_builder_utils.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  setUpAll(() {
    LayerSliceBuilder.debugRecorderFactory = (ui.Rect rect) {
      final StubSceneCanvas canvas = StubSceneCanvas();
      final StubPictureRecorder recorder = StubPictureRecorder(canvas);
      return (recorder, canvas);
    };
  });

  tearDownAll(() {
    LayerSliceBuilder.debugRecorderFactory = null;
  });

  group('EngineSceneBuilder', () {
    test('single picture', () {
      final EngineSceneBuilder sceneBuilder = EngineSceneBuilder();

      const ui.Rect pictureRect = ui.Rect.fromLTRB(100, 100, 200, 200);
      sceneBuilder.addPicture(ui.Offset.zero, StubPicture(pictureRect));

      final EngineScene scene = sceneBuilder.build() as EngineScene;
      final List<LayerSlice?> slices = scene.rootLayer.slices;
      expect(slices.length, 1);
      expect(slices[0], layerSlice(withPictureRect: pictureRect));
    });

    test('two pictures', () {
      final EngineSceneBuilder sceneBuilder = EngineSceneBuilder();

      const ui.Rect pictureRect1 = ui.Rect.fromLTRB(100, 100, 200, 200);
      const ui.Rect pictureRect2 = ui.Rect.fromLTRB(300, 300, 400, 400);
      sceneBuilder.addPicture(ui.Offset.zero, StubPicture(pictureRect1));
      sceneBuilder.addPicture(ui.Offset.zero, StubPicture(pictureRect2));

      final EngineScene scene = sceneBuilder.build() as EngineScene;
      final List<LayerSlice?> slices = scene.rootLayer.slices;
      expect(slices.length, 1);
      expect(slices[0], layerSlice(withPictureRect: const ui.Rect.fromLTRB(100, 100, 400, 400)));
    });

    test('picture + platform view (overlapping)', () {
      final EngineSceneBuilder sceneBuilder = EngineSceneBuilder();

      const ui.Rect pictureRect = ui.Rect.fromLTRB(100, 100, 200, 200);
      const ui.Rect platformViewRect = ui.Rect.fromLTRB(150, 150, 250, 250);
      sceneBuilder.addPicture(ui.Offset.zero, StubPicture(pictureRect));
      sceneBuilder.addPlatformView(
        1,
        offset: platformViewRect.topLeft,
        width: platformViewRect.width,
        height: platformViewRect.height,
      );

      final EngineScene scene = sceneBuilder.build() as EngineScene;
      final List<LayerSlice?> slices = scene.rootLayer.slices;
      expect(slices.length, 1);
      expect(
        slices[0],
        layerSlice(
          withPictureRect: pictureRect,
          withPlatformViews: <PlatformView>[
            PlatformView(1, platformViewRect, const PlatformViewStyling()),
          ],
        ),
      );
    });

    test('platform view + picture (overlapping)', () {
      final EngineSceneBuilder sceneBuilder = EngineSceneBuilder();

      const ui.Rect pictureRect = ui.Rect.fromLTRB(100, 100, 200, 200);
      const ui.Rect platformViewRect = ui.Rect.fromLTRB(150, 150, 250, 250);
      sceneBuilder.addPlatformView(
        1,
        offset: platformViewRect.topLeft,
        width: platformViewRect.width,
        height: platformViewRect.height,
      );
      sceneBuilder.addPicture(ui.Offset.zero, StubPicture(pictureRect));

      final EngineScene scene = sceneBuilder.build() as EngineScene;
      final List<LayerSlice?> slices = scene.rootLayer.slices;
      expect(slices.length, 2);
      expect(
        slices[0],
        layerSlice(
          withPlatformViews: <PlatformView>[
            PlatformView(1, platformViewRect, const PlatformViewStyling()),
          ],
        ),
      );
      expect(slices[1], layerSlice(withPictureRect: pictureRect));
    });

    test('platform view sandwich (overlapping)', () {
      final EngineSceneBuilder sceneBuilder = EngineSceneBuilder();

      const ui.Rect pictureRect1 = ui.Rect.fromLTRB(100, 100, 200, 200);
      const ui.Rect platformViewRect = ui.Rect.fromLTRB(150, 150, 250, 250);
      const ui.Rect pictureRect2 = ui.Rect.fromLTRB(200, 200, 300, 300);
      sceneBuilder.addPicture(ui.Offset.zero, StubPicture(pictureRect1));
      sceneBuilder.addPlatformView(
        1,
        offset: platformViewRect.topLeft,
        width: platformViewRect.width,
        height: platformViewRect.height,
      );
      sceneBuilder.addPicture(ui.Offset.zero, StubPicture(pictureRect2));

      final EngineScene scene = sceneBuilder.build() as EngineScene;
      final List<LayerSlice?> slices = scene.rootLayer.slices;
      expect(slices.length, 2);
      expect(
        slices[0],
        layerSlice(
          withPictureRect: pictureRect1,
          withPlatformViews: <PlatformView>[
            PlatformView(1, platformViewRect, const PlatformViewStyling()),
          ],
        ),
      );
      expect(slices[1], layerSlice(withPictureRect: pictureRect2));
    });

    test('platform view sandwich (non-overlapping)', () {
      final EngineSceneBuilder sceneBuilder = EngineSceneBuilder();

      const ui.Rect pictureRect1 = ui.Rect.fromLTRB(100, 100, 200, 200);
      const ui.Rect platformViewRect = ui.Rect.fromLTRB(150, 150, 250, 250);
      const ui.Rect pictureRect2 = ui.Rect.fromLTRB(50, 50, 100, 100);
      sceneBuilder.addPicture(ui.Offset.zero, StubPicture(pictureRect1));
      sceneBuilder.addPlatformView(
        1,
        offset: platformViewRect.topLeft,
        width: platformViewRect.width,
        height: platformViewRect.height,
      );
      sceneBuilder.addPicture(ui.Offset.zero, StubPicture(pictureRect2));

      final EngineScene scene = sceneBuilder.build() as EngineScene;
      final List<LayerSlice?> slices = scene.rootLayer.slices;

      // The top picture does not overlap with the platform view, so it should
      // be grouped into the slice below it to reduce the number of canvases we
      // need.
      expect(slices.length, 1);
      expect(
        slices[0],
        layerSlice(
          withPictureRect: const ui.Rect.fromLTRB(50, 50, 200, 200),
          withPlatformViews: <PlatformView>[
            PlatformView(1, platformViewRect, const PlatformViewStyling()),
          ],
        ),
      );
    });

    test('platform view sandwich (overlapping) with offset layers', () {
      final EngineSceneBuilder sceneBuilder = EngineSceneBuilder();

      const ui.Rect pictureRect1 = ui.Rect.fromLTRB(100, 100, 200, 200);
      sceneBuilder.addPicture(ui.Offset.zero, StubPicture(pictureRect1));

      sceneBuilder.pushOffset(150, 150);
      const ui.Rect platformViewRect = ui.Rect.fromLTRB(0, 0, 100, 100);
      sceneBuilder.addPlatformView(
        1,
        offset: platformViewRect.topLeft,
        width: platformViewRect.width,
        height: platformViewRect.height,
      );
      sceneBuilder.pushOffset(50, 50);
      sceneBuilder.addPicture(ui.Offset.zero, StubPicture(const ui.Rect.fromLTRB(0, 0, 100, 100)));

      final EngineScene scene = sceneBuilder.build() as EngineScene;
      final List<LayerSlice?> slices = scene.rootLayer.slices;
      expect(slices.length, 2);
      expect(
        slices[0],
        layerSlice(
          withPictureRect: pictureRect1,
          withPlatformViews: <PlatformView>[
            PlatformView(
              1,
              platformViewRect,
              const PlatformViewStyling(position: PlatformViewPosition.offset(ui.Offset(150, 150))),
            ),
          ],
        ),
      );
      expect(slices[1], layerSlice(withPictureRect: const ui.Rect.fromLTRB(200, 200, 300, 300)));
    });

    test('grid view test', () {
      // This test case covers a grid of elements, where each element is a platform
      // view that has flutter content underneath it and on top of it.
      // See a detailed explanation of this use-case in the following flutter issue:
      // https://github.com/flutter/flutter/issues/149863
      final EngineSceneBuilder sceneBuilder = EngineSceneBuilder();

      const double padding = 10;
      const double tileSize = 50;
      final List<PlatformView> expectedPlatformViews = <PlatformView>[];
      for (int x = 0; x < 10; x++) {
        for (int y = 0; y < 10; y++) {
          final ui.Offset offset = ui.Offset(
            padding + (tileSize + padding) * x,
            padding + (tileSize + padding) * y,
          );
          sceneBuilder.pushOffset(offset.dx, offset.dy);
          sceneBuilder.addPicture(
            ui.Offset.zero,
            StubPicture(const ui.Rect.fromLTWH(0, 0, tileSize, tileSize)),
          );
          sceneBuilder.addPlatformView(
            1,
            offset: const ui.Offset(5, 5),
            width: tileSize - 10,
            height: tileSize - 10,
          );
          sceneBuilder.addPicture(
            const ui.Offset(10, 10),
            StubPicture(const ui.Rect.fromLTWH(0, 0, tileSize - 20, tileSize - 20)),
          );
          sceneBuilder.pop();
          expectedPlatformViews.add(
            PlatformView(
              1,
              const ui.Rect.fromLTRB(5.0, 5.0, tileSize - 5.0, tileSize - 5.0),
              PlatformViewStyling(position: PlatformViewPosition.offset(offset)),
            ),
          );
        }
      }

      final EngineScene scene = sceneBuilder.build() as EngineScene;
      final List<LayerSlice?> slices = scene.rootLayer.slices;

      // It is important that the optimizations of the scene builder result in
      // there only being two scene slices.
      expect(slices.length, 2);
      expect(
        slices[0],
        layerSlice(
          withPictureRect: const ui.Rect.fromLTRB(
            padding,
            padding,
            10 * (padding + tileSize),
            10 * (padding + tileSize),
          ),
          withPlatformViews: expectedPlatformViews,
        ),
      );
      expect(
        slices[1],
        layerSlice(
          withPictureRect: const ui.Rect.fromLTRB(
            padding + 10,
            padding + 10,
            10 * (padding + tileSize) - 10,
            10 * (padding + tileSize) - 10,
          ),
        ),
      );
    });
  });
}

LayerSliceMatcher layerSlice({
  ui.Rect withPictureRect = ui.Rect.zero,
  List<PlatformView> withPlatformViews = const <PlatformView>[],
}) => LayerSliceMatcher(withPictureRect, withPlatformViews);

class LayerSliceMatcher extends Matcher {
  LayerSliceMatcher(this.expectedPictureRect, this.expectedPlatformViews);

  final ui.Rect expectedPictureRect;
  final List<PlatformView> expectedPlatformViews;

  @override
  Description describe(Description description) {
    return description.add(
      '<picture slice with cullRect: $expectedPictureRect and platform views: $expectedPlatformViews>',
    );
  }

  @override
  bool matches(dynamic item, Map<dynamic, dynamic> matchState) {
    if (item is! LayerSlice) {
      return false;
    }
    final ScenePicture picture = item.picture;
    if (picture is! StubPicture) {
      return false;
    }

    if (picture.cullRect != expectedPictureRect) {
      return false;
    }

    if (item.platformViews.length != expectedPlatformViews.length) {
      return false;
    }

    for (int i = 0; i < item.platformViews.length; i++) {
      final PlatformView expectedView = expectedPlatformViews[i];
      final PlatformView actualView = item.platformViews[i];
      if (expectedView.viewId != actualView.viewId) {
        return false;
      }
      if (expectedView.bounds != actualView.bounds) {
        return false;
      }
      if (expectedView.styling != actualView.styling) {
        return false;
      }
    }

    return true;
  }
}
