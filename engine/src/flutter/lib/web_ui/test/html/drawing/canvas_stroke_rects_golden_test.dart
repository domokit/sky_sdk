// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart';

import 'package:web_engine_tester/golden_tester.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  const Rect region = Rect.fromLTWH(0, 0, 300, 300);

  late BitmapCanvas canvas;

  setUp(() {
    canvas = BitmapCanvas(region, RenderStrategy());
  });

  tearDown(() {
    canvas.rootElement.remove();
  });

  test('draws rects side by side with fill and stroke', () async {
    paintSideBySideRects(canvas);

    domDocument.body!.append(canvas.rootElement);
    await matchGoldenFile('canvas_stroke_rects.png', region: region);
  });
}

void paintSideBySideRects(BitmapCanvas canvas) {
  canvas.drawRect(
    const Rect.fromLTRB(0, 0, 300, 300),
    SurfacePaintData()
      ..color = 0xFFFFFFFF
      ..style = PaintingStyle.fill,
  ); // white

  canvas.drawRect(
    const Rect.fromLTRB(0, 20, 40, 60),
    SurfacePaintData()
      ..style = PaintingStyle.fill
      ..color = 0x7f0000ff,
  );
  canvas.drawRect(
    const Rect.fromLTRB(40, 20, 80, 60),
    SurfacePaintData()
      ..style = PaintingStyle.stroke
      ..strokeWidth = 4
      ..color = 0x7fff0000,
  );

  // Rotate 30 degrees (in rad: deg*pi/180)
  canvas.transform(Matrix4.rotationZ(30.0 * math.pi / 180.0).storage);

  canvas.drawRect(
    const Rect.fromLTRB(100, 60, 140, 100),
    SurfacePaintData()
      ..style = PaintingStyle.fill
      ..color = 0x7fff00ff,
  );
  canvas.drawRect(
    const Rect.fromLTRB(140, 60, 180, 100),
    SurfacePaintData()
      ..style = PaintingStyle.stroke
      ..strokeWidth = 4
      ..color = 0x7fffff00,
  );
}
