// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;
import 'package:web_engine_tester/golden_tester.dart';

/// Commit a recording canvas to a bitmap, and compare with the expected.
///
/// [region] specifies the area of the canvas that will be included in the
/// golden.
///
/// If [canvasRect] is omitted, it defaults to the value of [region].
Future<void> canvasScreenshot(
  RecordingCanvas rc,
  String fileName, {
  ui.Rect region = const ui.Rect.fromLTWH(0, 0, 500, 500),
  ui.Rect? canvasRect,
  bool setupPerspective = false,
}) async {
  canvasRect ??= region;
  final EngineCanvas engineCanvas = BitmapCanvas(canvasRect, RenderStrategy());

  rc.endRecording();
  rc.apply(engineCanvas, region);

  // Wrap in <flt-scene> so that our CSS selectors kick in.
  final DomElement sceneElement = createDomElement('flt-scene');
  if (isIosSafari) {
    // Shrink to fit on the iPhone screen.
    sceneElement.style.position = 'absolute';
    sceneElement.style.transformOrigin = '0 0 0';
    sceneElement.style.transform = 'scale(0.3)';
  }
  try {
    if (setupPerspective) {
      // iFrame disables perspective, set it explicitly for test.
      engineCanvas.rootElement.style.perspective = '400px';
      for (final DomElement element in engineCanvas.rootElement.querySelectorAll('div')) {
        element.style.perspective = '400px';
      }
    }
    sceneElement.append(engineCanvas.rootElement);
    domDocument.body!.append(sceneElement);
    await matchGoldenFile('$fileName.png', region: region);
  } finally {
    // The page is reused across tests, so remove the element after taking the
    // screenshot.
    sceneElement.remove();
  }
}

Future<void> sceneScreenshot(
  SurfaceSceneBuilder sceneBuilder,
  String fileName, {
  ui.Rect region = const ui.Rect.fromLTWH(0, 0, 600, 800),
}) async {
  DomElement? sceneElement;
  try {
    sceneElement = sceneBuilder.build().webOnlyRootElement;
    domDocument.body!.append(sceneElement!);
    await matchGoldenFile('$fileName.png', region: region);
  } finally {
    // The page is reused across tests, so remove the element after taking the
    // screenshot.
    sceneElement?.remove();
  }
}
