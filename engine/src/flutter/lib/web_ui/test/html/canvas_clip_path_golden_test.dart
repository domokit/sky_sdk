// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:js_util' as js_util;

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart' as engine;
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' hide TextStyle;

import '../common/test_initialization.dart';
import 'screenshot.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  setUpUnitTests(setUpTestViewDimensions: false);

  // Regression test for https://github.com/flutter/flutter/issues/48683
  // Should clip image with oval.
  test('Clips image with oval clip path', () async {
    final engine.RecordingCanvas rc = engine.RecordingCanvas(const Rect.fromLTRB(0, 0, 500, 500));
    rc.save();
    final Image testImage = createTestImage();
    final double testWidth = testImage.width.toDouble();
    final double testHeight = testImage.height.toDouble();
    final Path path = Path();
    path.addOval(Rect.fromLTWH(100, 30, testWidth, testHeight));
    rc.clipPath(path);
    rc.drawImageRect(
      testImage,
      Rect.fromLTRB(0, 0, testWidth, testHeight),
      Rect.fromLTWH(100, 30, testWidth, testHeight),
      engine.SurfacePaint(),
    );
    rc.restore();
    await canvasScreenshot(rc, 'image_clipped_by_oval');
  });

  // Regression test for https://github.com/flutter/flutter/issues/48683
  test('Clips triangle with oval clip path', () async {
    final engine.RecordingCanvas rc = engine.RecordingCanvas(const Rect.fromLTRB(0, 0, 500, 500));
    rc.save();
    const double testWidth = 200;
    const double testHeight = 150;
    final Path path = Path();
    path.addOval(const Rect.fromLTWH(100, 30, testWidth, testHeight));
    rc.clipPath(path);
    final Path paintPath = Path();
    paintPath.moveTo(testWidth / 2, 0);
    paintPath.lineTo(testWidth, testHeight);
    paintPath.lineTo(0, testHeight);
    paintPath.close();
    rc.drawPath(
      paintPath,
      engine.SurfacePaint()
        ..color = const Color(0xFF00FF00)
        ..style = PaintingStyle.fill,
    );
    rc.restore();
    await canvasScreenshot(rc, 'triangle_clipped_by_oval');
  });

  // Regression test for https://github.com/flutter/flutter/issues/78782
  test('Clips on Safari when clip bounds off screen', () async {
    final engine.RecordingCanvas rc = engine.RecordingCanvas(const Rect.fromLTRB(0, 0, 500, 500));
    rc.save();
    const double testWidth = 200;
    const double testHeight = 150;

    final Path paintPath = Path();
    paintPath.addRect(const Rect.fromLTWH(-50, 0, testWidth, testHeight));
    paintPath.close();
    rc.drawPath(
      paintPath,
      engine.SurfacePaint()
        ..color = const Color(0xFF000000)
        ..style = PaintingStyle.stroke,
    );

    final Path path = Path();
    path.moveTo(-200, 0);
    path.lineTo(100, 75);
    path.lineTo(-200, 150);
    path.close();
    rc.clipPath(path);
    rc.drawImageRect(
      createTestImage(),
      const Rect.fromLTRB(0, 0, testWidth, testHeight),
      const Rect.fromLTWH(-50, 0, testWidth, testHeight),
      engine.SurfacePaint(),
    );
    rc.restore();
    await canvasScreenshot(
      rc,
      'image_clipped_by_triangle_off_screen',
      region: const Rect.fromLTWH(0, 0, 600, 800),
    );
  });

  // Tests oval clipping using border radius 50%.
  test('Clips against oval', () async {
    final engine.RecordingCanvas rc = engine.RecordingCanvas(const Rect.fromLTRB(0, 0, 500, 500));
    rc.save();
    const double testWidth = 200;
    const double testHeight = 150;

    final Path paintPath = Path();
    paintPath.addRect(const Rect.fromLTWH(-50, 0, testWidth, testHeight));
    paintPath.close();
    rc.drawPath(
      paintPath,
      engine.SurfacePaint()
        ..color = const Color(0xFF000000)
        ..style = PaintingStyle.stroke,
    );

    final Path path = Path();
    path.addOval(const Rect.fromLTRB(-200, 0, 100, 150));
    rc.clipPath(path);
    rc.drawImageRect(
      createTestImage(),
      const Rect.fromLTRB(0, 0, testWidth, testHeight),
      const Rect.fromLTWH(-50, 0, testWidth, testHeight),
      engine.SurfacePaint(),
    );
    rc.restore();
    await canvasScreenshot(
      rc,
      'image_clipped_by_oval_path',
      region: const Rect.fromLTWH(0, 0, 600, 800),
    );
  });

  test('Clips with fillType evenOdd', () async {
    final engine.RecordingCanvas rc = engine.RecordingCanvas(const Rect.fromLTRB(0, 0, 500, 500));
    rc.save();
    const double testWidth = 400;
    const double testHeight = 350;

    // draw RGB test image
    rc.drawImageRect(
      createTestImage(),
      const Rect.fromLTRB(0, 0, testWidth, testHeight),
      const Rect.fromLTWH(0, 0, testWidth, testHeight),
      engine.SurfacePaint(),
    );

    // draw a clipping path with:
    // 1) an outside larger rectangle
    // 2) a smaller inner rectangle specified by a path
    final Path path = Path();
    path.addRect(const Rect.fromLTWH(0, 0, testWidth, testHeight));
    const double left = 25;
    const double top = 30;
    const double right = 300;
    const double bottom = 250;
    path
      ..moveTo(left, top)
      ..lineTo(right, top)
      ..lineTo(right, bottom)
      ..lineTo(left, bottom)
      ..close();
    path.fillType = PathFillType.evenOdd;
    rc.clipPath(path);

    // draw an orange paint path of size testWidth and testHeight
    final Path paintPath = Path();
    paintPath.addRect(const Rect.fromLTWH(0, 0, testWidth, testHeight));
    paintPath.close();
    rc.drawPath(
      paintPath,
      engine.SurfacePaint()
        ..color = const Color(0xFFFF9800)
        ..style = PaintingStyle.fill,
    );
    rc.restore();

    // when fillType is set to evenOdd from the clipping path, expect the inner
    // rectangle should clip some of the orange painted portion, revealing the RGB testImage
    await canvasScreenshot(
      rc,
      'clipPath_uses_fillType_evenOdd',
      region: const Rect.fromLTWH(0, 0, 600, 800),
    );
  });
}

engine.HtmlImage createTestImage({int width = 200, int height = 150}) {
  final engine.DomCanvasElement canvas = engine.createDomCanvasElement(
    width: width,
    height: height,
  );
  final engine.DomCanvasRenderingContext2D ctx = canvas.context2D;
  ctx.fillStyle = '#E04040';
  ctx.fillRect(0, 0, width / 3, height);
  ctx.fill();
  ctx.fillStyle = '#40E080';
  ctx.fillRect(width / 3, 0, width / 3, height);
  ctx.fill();
  ctx.fillStyle = '#2040E0';
  ctx.fillRect(2 * width / 3, 0, width / 3, height);
  ctx.fill();
  final engine.DomHTMLImageElement imageElement = engine.createDomHTMLImageElement();
  imageElement.src = js_util.callMethod<String>(canvas, 'toDataURL', <dynamic>[]);
  return engine.HtmlImage(imageElement, width, height);
}
