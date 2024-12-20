// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

import '../common/matchers.dart';
import 'common.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  group('CanvasKit', () {
    setUpCanvasKitTest(withImplicitView: true);

    late LayerScene scene;

    setUp(() {
      // Create a scene to use in tests.
      final CkPicture picture = paintPicture(const ui.Rect.fromLTRB(0, 0, 60, 60), (
        CkCanvas canvas,
      ) {
        canvas.drawRect(
          const ui.Rect.fromLTRB(0, 0, 60, 60),
          CkPaint()..style = ui.PaintingStyle.fill,
        );
      });
      final LayerSceneBuilder sb = LayerSceneBuilder();
      sb.addPicture(ui.Offset.zero, picture);
      scene = sb.build();
    });

    test('can render into arbitrary views', () async {
      await CanvasKitRenderer.instance.renderScene(scene, implicitView);

      final EngineFlutterView anotherView = EngineFlutterView(
        EnginePlatformDispatcher.instance,
        createDomElement('another-view'),
      );
      EnginePlatformDispatcher.instance.viewManager.registerView(anotherView);

      await CanvasKitRenderer.instance.renderScene(scene, anotherView);
    });

    test('will error if trying to render into an unregistered view', () async {
      final EngineFlutterView unregisteredView = EngineFlutterView(
        EnginePlatformDispatcher.instance,
        createDomElement('unregistered-view'),
      );
      expect(
        () => CanvasKitRenderer.instance.renderScene(scene, unregisteredView),
        throwsAssertionError,
      );
    });

    test('will dispose the Rasterizer for a disposed view', () async {
      final EngineFlutterView view = EngineFlutterView(
        EnginePlatformDispatcher.instance,
        createDomElement('multi-view'),
      );
      EnginePlatformDispatcher.instance.viewManager.registerView(view);
      expect(CanvasKitRenderer.instance.debugGetRasterizerForView(view), isNotNull);

      EnginePlatformDispatcher.instance.viewManager.disposeAndUnregisterView(view.viewId);
      expect(CanvasKitRenderer.instance.debugGetRasterizerForView(view), isNull);
    });

    // Issue https://github.com/flutter/flutter/issues/142094
    test('does not reset platform view factories when disposing a view', () async {
      expect(PlatformViewManager.instance.knowsViewType('self-test'), isFalse);

      final EngineFlutterView view = EngineFlutterView(
        EnginePlatformDispatcher.instance,
        createDomElement('multi-view'),
      );
      EnginePlatformDispatcher.instance.viewManager.registerView(view);
      expect(CanvasKitRenderer.instance.debugGetRasterizerForView(view), isNotNull);

      EnginePlatformDispatcher.instance.viewManager.disposeAndUnregisterView(view.viewId);
      expect(CanvasKitRenderer.instance.debugGetRasterizerForView(view), isNull);

      expect(
        PlatformViewManager.instance.knowsViewType(
          ui_web.PlatformViewRegistry.defaultVisibleViewType,
        ),
        isTrue,
      );
      expect(
        PlatformViewManager.instance.knowsViewType(
          ui_web.PlatformViewRegistry.defaultInvisibleViewType,
        ),
        isTrue,
      );
    });
  });
}
