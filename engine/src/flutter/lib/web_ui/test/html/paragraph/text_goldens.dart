// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;

import 'package:web_engine_tester/golden_tester.dart';

import 'helper.dart';

/// Class that controls some details of how screenshotting is made.
class EngineGoldenTester {
  EngineGoldenTester(this.viewportSize);

  /// The size of the browser window used in this golden test.
  final ui.Size viewportSize;

  static Future<EngineGoldenTester> initialize({
    ui.Size viewportSize = const ui.Size(2400, 1800),
  }) async {
    assert(() {
      if (viewportSize.width.ceil() != viewportSize.width ||
          viewportSize.height.ceil() != viewportSize.height) {
        throw Exception('Gold only supports integer screen sizes, but found: $viewportSize');
      }
      return true;
    }());

    return EngineGoldenTester(viewportSize);
  }

  ui.Rect get viewportRegion => ui.Rect.fromLTWH(0, 0, viewportSize.width, viewportSize.height);

  Future<void> diffScreenshot(String fileName, {ui.Rect? region}) async {
    await matchGoldenFile('$fileName.png', region: region ?? viewportRegion);
  }

  /// Prepares the DOM and inserts all the necessary nodes, then invokes Gold's
  /// screenshot diffing.
  ///
  /// It also cleans up the DOM after itself.
  Future<void> diffCanvasScreenshot(EngineCanvas canvas, String fileName, {ui.Rect? region}) async {
    // Wrap in <flt-scene> so that our CSS selectors kick in.
    final DomElement sceneElement = createDomElement('flt-scene');
    if (isIosSafari) {
      // Shrink to fit on the iPhone screen.
      sceneElement.style.position = 'absolute';
      sceneElement.style.transformOrigin = '0 0 0';
      sceneElement.style.transform = 'scale(0.3)';
    }
    try {
      sceneElement.append(canvas.rootElement);
      domDocument.body!.append(sceneElement);
      String screenshotName = '${fileName}_${canvas.runtimeType}';
      if (canvas is BitmapCanvas) {
        screenshotName += '+canvas_measurement';
      }
      await diffScreenshot(screenshotName, region: region);
    } finally {
      // The page is reused across tests, so remove the element after taking the
      // screenshot.
      sceneElement.remove();
    }
  }
}

/// Runs the given test [body] with each type of canvas.
void testEachCanvas(String description, CanvasTest body) {
  const ui.Rect bounds = ui.Rect.fromLTWH(0, 0, 600, 800);
  test('$description (bitmap + canvas measurement)', () async {
    return body(BitmapCanvas(bounds, RenderStrategy()));
  });
  test('$description (dom)', () {
    return body(DomCanvas(domDocument.createElement('flt-picture')));
  });
}

final ui.TextStyle _defaultTextStyle = ui.TextStyle(
  color: const ui.Color(0xFF000000),
  fontFamily: 'Roboto',
  fontSize: 14,
);

CanvasParagraph paragraph(
  String text, {
  ui.ParagraphStyle? paragraphStyle,
  ui.TextStyle? textStyle,
  double maxWidth = double.infinity,
}) {
  final ui.ParagraphBuilder builder = ui.ParagraphBuilder(paragraphStyle ?? ui.ParagraphStyle());
  builder.pushStyle(textStyle ?? _defaultTextStyle);
  builder.addText(text);
  builder.pop();
  final CanvasParagraph paragraph = builder.build() as CanvasParagraph;
  paragraph.layout(ui.ParagraphConstraints(width: maxWidth));
  return paragraph;
}
