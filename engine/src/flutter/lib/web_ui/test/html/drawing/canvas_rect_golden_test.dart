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
  const Rect region = Rect.fromLTWH(0, 0, 150, 420);

  late BitmapCanvas canvas;

  setUp(() {
    canvas = BitmapCanvas(region, RenderStrategy());
  });

  tearDown(() {
    canvas.rootElement.remove();
  });

  test('draws rect with flipped coordinates L > R, T > B', () async {
    paintRects(canvas);

    domDocument.body!.append(canvas.rootElement);
    await matchGoldenFile('canvas_rect_flipped.png', region: region);
  });
}

void paintRects(BitmapCanvas canvas) {
  canvas.drawRect(
    const Rect.fromLTRB(30, 40, 100, 50),
    SurfacePaintData()
      ..color =
          0xFF4CAF50 //Colors.green
      ..strokeWidth = 1.0
      ..style = PaintingStyle.stroke,
  );

  // swap left and right.
  canvas.drawRect(
    const Rect.fromLTRB(100, 150, 30, 140),
    SurfacePaintData()
      ..color =
          0xFFF44336 //Colors.red
      ..strokeWidth = 1.0
      ..style = PaintingStyle.stroke,
  );

  // Repeat above for fill
  canvas.drawRect(
    const Rect.fromLTRB(30, 240, 100, 250),
    SurfacePaintData()
      ..color =
          0xFF4CAF50 //Colors.green
      ..style = PaintingStyle.fill,
  );

  // swap left and right.
  canvas.drawRect(
    const Rect.fromLTRB(100, 350, 30, 340),
    SurfacePaintData()
      ..color =
          0xFFF44336 //Colors.red
      ..style = PaintingStyle.fill,
  );
}
