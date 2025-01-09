// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart';

import 'package:web_engine_tester/golden_tester.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  test('rect stroke with clip', () async {
    const Rect region = Rect.fromLTWH(0, 0, 250, 250);
    // Set `hasParagraphs` to true to force DOM rendering.
    final BitmapCanvas canvas = BitmapCanvas(region, RenderStrategy()..hasParagraphs = true);

    const Rect rect = Rect.fromLTWH(0, 0, 150, 150);

    canvas.clipRect(rect.inflate(10.0), ClipOp.intersect);

    canvas.drawRect(
      rect,
      SurfacePaintData()
        ..color = 0x6fff0000
        ..strokeWidth = 20.0
        ..style = PaintingStyle.stroke,
    );

    canvas.drawRect(
      rect,
      SurfacePaintData()
        ..color = 0x6f0000ff
        ..strokeWidth = 10.0
        ..style = PaintingStyle.stroke,
    );

    canvas.drawRect(
      rect,
      SurfacePaintData()
        ..color = 0xff000000
        ..strokeWidth = 1.0
        ..style = PaintingStyle.stroke,
    );

    domDocument.body!.style.margin = '0px';
    domDocument.body!.append(canvas.rootElement);
    await matchGoldenFile('rect_clip_strokes_dom.png', region: region);
    canvas.rootElement.remove();
  });

  test('rrect stroke with clip', () async {
    const Rect region = Rect.fromLTWH(0, 0, 250, 250);
    // Set `hasParagraphs` to true to force DOM rendering.
    final BitmapCanvas canvas = BitmapCanvas(region, RenderStrategy()..hasParagraphs = true);

    final RRect rrect = RRect.fromRectAndRadius(
      const Rect.fromLTWH(0, 0, 150, 150),
      const Radius.circular(20),
    );

    canvas.clipRect(rrect.outerRect.inflate(10.0), ClipOp.intersect);

    canvas.drawRRect(
      rrect,
      SurfacePaintData()
        ..color = 0x6fff0000
        ..strokeWidth = 20.0
        ..style = PaintingStyle.stroke,
    );

    canvas.drawRRect(
      rrect,
      SurfacePaintData()
        ..color = 0x6f0000ff
        ..strokeWidth = 10.0
        ..style = PaintingStyle.stroke,
    );

    canvas.drawRRect(
      rrect,
      SurfacePaintData()
        ..color = 0xff000000
        ..strokeWidth = 1.0
        ..style = PaintingStyle.stroke,
    );

    domDocument.body!.style.margin = '0px';
    domDocument.body!.append(canvas.rootElement);
    await matchGoldenFile('rrect_clip_strokes_dom.png', region: region);
    canvas.rootElement.remove();
  });

  test('circle stroke with clip', () async {
    const Rect region = Rect.fromLTWH(0, 0, 250, 250);
    // Set `hasParagraphs` to true to force DOM rendering.
    final BitmapCanvas canvas = BitmapCanvas(region, RenderStrategy()..hasParagraphs = true);

    const Rect rect = Rect.fromLTWH(0, 0, 150, 150);

    canvas.clipRect(rect.inflate(10.0), ClipOp.intersect);

    canvas.drawCircle(
      rect.center,
      rect.width / 2,
      SurfacePaintData()
        ..color = 0x6fff0000
        ..strokeWidth = 20.0
        ..style = PaintingStyle.stroke,
    );

    canvas.drawCircle(
      rect.center,
      rect.width / 2,
      SurfacePaintData()
        ..color = 0x6f0000ff
        ..strokeWidth = 10.0
        ..style = PaintingStyle.stroke,
    );

    canvas.drawCircle(
      rect.center,
      rect.width / 2,
      SurfacePaintData()
        ..color = 0xff000000
        ..strokeWidth = 1.0
        ..style = PaintingStyle.stroke,
    );

    domDocument.body!.style.margin = '0px';
    domDocument.body!.append(canvas.rootElement);
    await matchGoldenFile('circle_clip_strokes_dom.png', region: region);
    canvas.rootElement.remove();
  });
}
