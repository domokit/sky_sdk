// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:js_util' as js_util;

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' hide TextStyle;

import '../../common/test_initialization.dart';
import '../screenshot.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  setUpUnitTests(setUpTestViewDimensions: false);

  test('Blend circles with difference and color', () async {
    final RecordingCanvas rc = RecordingCanvas(const Rect.fromLTRB(0, 0, 400, 300));
    rc.save();
    rc.drawRect(
      const Rect.fromLTRB(0, 0, 400, 400),
      SurfacePaint()
        ..style = PaintingStyle.fill
        ..color = const Color.fromARGB(255, 255, 255, 255),
    );
    rc.drawCircle(
      const Offset(100, 100),
      80.0,
      SurfacePaint()
        ..style = PaintingStyle.fill
        ..color = const Color.fromARGB(128, 255, 0, 0)
        ..blendMode = BlendMode.difference,
    );

    rc.drawCircle(
      const Offset(170, 100),
      80.0,
      SurfacePaint()
        ..style = PaintingStyle.fill
        ..blendMode = BlendMode.color
        ..color = const Color.fromARGB(128, 0, 255, 0),
    );

    rc.drawCircle(
      const Offset(135, 170),
      80.0,
      SurfacePaint()
        ..style = PaintingStyle.fill
        ..color = const Color.fromARGB(128, 255, 0, 0),
    );
    rc.restore();

    await canvasScreenshot(rc, 'canvas_blend_circle_diff_color');
  });

  test('Blend circle and text with multiply', () async {
    final RecordingCanvas rc = RecordingCanvas(const Rect.fromLTRB(0, 0, 400, 300));
    rc.save();
    rc.drawRect(
      const Rect.fromLTRB(0, 0, 400, 400),
      SurfacePaint()
        ..style = PaintingStyle.fill
        ..color = const Color.fromARGB(255, 255, 255, 255),
    );
    rc.drawCircle(
      const Offset(100, 100),
      80.0,
      SurfacePaint()
        ..style = PaintingStyle.fill
        ..color = const Color.fromARGB(128, 255, 0, 0)
        ..blendMode = BlendMode.difference,
    );
    rc.drawCircle(
      const Offset(170, 100),
      80.0,
      SurfacePaint()
        ..style = PaintingStyle.fill
        ..blendMode = BlendMode.color
        ..color = const Color.fromARGB(128, 0, 255, 0),
    );

    rc.drawCircle(
      const Offset(135, 170),
      80.0,
      SurfacePaint()
        ..style = PaintingStyle.fill
        ..color = const Color.fromARGB(128, 255, 0, 0),
    );
    rc.drawImage(
      createTestImage(),
      const Offset(135.0, 130.0),
      SurfacePaint()..blendMode = BlendMode.multiply,
    );
    rc.restore();
    await canvasScreenshot(rc, 'canvas_blend_image_multiply');
  });
}

HtmlImage createTestImage() {
  const int width = 100;
  const int height = 50;
  final DomCanvasElement canvas = createDomCanvasElement(width: width, height: height);
  final DomCanvasRenderingContext2D ctx = canvas.context2D;
  ctx.fillStyle = '#E04040';
  ctx.fillRect(0, 0, 33, 50);
  ctx.fill();
  ctx.fillStyle = '#40E080';
  ctx.fillRect(33, 0, 33, 50);
  ctx.fill();
  ctx.fillStyle = '#2040E0';
  ctx.fillRect(66, 0, 33, 50);
  ctx.fill();
  final DomHTMLImageElement imageElement = createDomHTMLImageElement();
  imageElement.src = js_util.callMethod<String>(canvas, 'toDataURL', <dynamic>[]);
  return HtmlImage(imageElement, width, height);
}
