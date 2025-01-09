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
  const Rect region = Rect.fromLTWH(0, 0, 500, 500);

  late BitmapCanvas canvas;

  setUp(() {
    canvas = BitmapCanvas(region, RenderStrategy());
  });

  tearDown(() {
    canvas.rootElement.remove();
  });

  test('draws paths using nonzero and evenodd winding rules', () async {
    paintPaths(canvas);
    domDocument.body!.append(canvas.rootElement);
    await matchGoldenFile('canvas_path_winding.png', region: region);
  });
}

void paintPaths(BitmapCanvas canvas) {
  canvas.drawRect(
    const Rect.fromLTRB(0, 0, 500, 500),
    SurfacePaintData()
      ..color = 0xFFFFFFFF
      ..style = PaintingStyle.fill,
  ); // white

  final SurfacePaint paintFill =
      SurfacePaint()
        ..style = PaintingStyle.fill
        ..color = const Color(0xFF00B0FF);
  final SurfacePaint paintStroke =
      SurfacePaint()
        ..style = PaintingStyle.stroke
        ..strokeWidth = 2
        ..color = const Color(0xFFE00000);
  final Path path1 =
      Path()
        ..fillType = PathFillType.evenOdd
        ..moveTo(50, 0)
        ..lineTo(21, 90)
        ..lineTo(98, 35)
        ..lineTo(2, 35)
        ..lineTo(79, 90)
        ..close()
        ..addRect(const Rect.fromLTWH(20, 100, 200, 50))
        ..addRect(const Rect.fromLTWH(40, 120, 160, 10));
  final Path path2 =
      Path()
        ..fillType = PathFillType.nonZero
        ..moveTo(50, 200)
        ..lineTo(21, 290)
        ..lineTo(98, 235)
        ..lineTo(2, 235)
        ..lineTo(79, 290)
        ..close()
        ..addRect(const Rect.fromLTWH(20, 300, 200, 50))
        ..addRect(const Rect.fromLTWH(40, 320, 160, 10));
  canvas.drawPath(path1, paintFill.paintData);
  canvas.drawPath(path2, paintFill.paintData);
  canvas.drawPath(path1, paintStroke.paintData);
  canvas.drawPath(path2, paintStroke.paintData);
}
