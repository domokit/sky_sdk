// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:js_interop';

import 'package:test/test.dart';
import 'package:ui/src/engine.dart' as engine;
import 'package:ui/src/engine/frame_service.dart';
import 'package:ui/src/engine/initialization.dart';
import 'package:ui/ui.dart' as ui;
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

import 'fake_asset_manager.dart';
import 'rendering.dart';

void setUpUnitTests({
  bool withImplicitView = false,
  bool emulateTesterEnvironment = true,
  bool setUpTestViewDimensions = true,
}) {
  late final FakeAssetScope debugFontsScope;
  setUpAll(() async {
    if (emulateTesterEnvironment) {
      ui_web.debugEmulateFlutterTesterEnvironment = true;
    }

    debugFontsScope = configureDebugFontsAssetScope(fakeAssetManager);
    debugOnlyAssetManager = fakeAssetManager;
    await bootstrapAndRunApp(withImplicitView: withImplicitView);
    engine.debugOverrideJsConfiguration(
      <String, Object?>{'fontFallbackBaseUrl': 'assets/fallback_fonts/'}.jsify()
          as engine.JsFlutterConfiguration?,
    );

    if (setUpTestViewDimensions) {
      // The following parameters are hard-coded in Flutter's test embedder. Since
      // we don't have an embedder yet this is the lowest-most layer we can put
      // this stuff in.
      const double devicePixelRatio = 3.0;
      engine.EngineFlutterDisplay.instance.debugOverrideDevicePixelRatio(devicePixelRatio);
      engine.EnginePlatformDispatcher.instance.implicitView?.debugPhysicalSizeOverride =
          const ui.Size(800 * devicePixelRatio, 600 * devicePixelRatio);
      FrameService.debugOverrideFrameService(FakeFrameService());
    }

    setUpRenderingForTests();
  });

  tearDownAll(() async {
    fakeAssetManager.popAssetScope(debugFontsScope);
  });
}

void setUpImplicitView() {
  late engine.EngineFlutterWindow myWindow;

  final engine.EnginePlatformDispatcher dispatcher = engine.EnginePlatformDispatcher.instance;

  setUp(() {
    myWindow = engine.EngineFlutterView.implicit(dispatcher, null);
    dispatcher.viewManager.registerView(myWindow);
  });

  tearDown(() async {
    dispatcher.viewManager.unregisterView(myWindow.viewId);
    await myWindow.resetHistory();
    myWindow.dispose();
  });
}

Future<void> bootstrapAndRunApp({bool withImplicitView = false}) async {
  final Completer<void> completer = Completer<void>();
  await ui_web.bootstrapEngine(runApp: () => completer.complete());
  await completer.future;
  if (!withImplicitView) {
    _disableImplicitView();
  }
}

void _disableImplicitView() {
  // TODO(mdebbar): Instead of disabling the implicit view, we should be able to
  //                initialize tests without an implicit view to begin with.
  //                https://github.com/flutter/flutter/issues/138906
  final engine.EngineFlutterWindow? implicitView =
      engine.EnginePlatformDispatcher.instance.implicitView;
  if (implicitView != null) {
    engine.EnginePlatformDispatcher.instance.viewManager.disposeAndUnregisterView(
      implicitView.viewId,
    );
  }
}

class FakeFrameService extends FrameService {
  @override
  void scheduleFrame() {}
}
