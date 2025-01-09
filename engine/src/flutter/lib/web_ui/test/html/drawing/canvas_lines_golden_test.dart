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
  const Rect region = Rect.fromLTWH(0, 0, 300, 300);

  late BitmapCanvas canvas;
  late BitmapCanvas domCanvas;

  setUp(() {
    canvas = BitmapCanvas(region, RenderStrategy());
    // setting isInsideSvgFilterTree true forces use of DOM canvas
    domCanvas = BitmapCanvas(region, RenderStrategy()..isInsideSvgFilterTree = true);
  });

  tearDown(() {
    canvas.rootElement.remove();
    domCanvas.rootElement.remove();
  });

  test('draws lines with varying strokeWidth', () async {
    paintLines(canvas);

    domDocument.body!.append(canvas.rootElement);
    await matchGoldenFile('canvas_lines_thickness.png', region: region);
  });
  test('draws lines with varying strokeWidth with dom canvas', () async {
    paintLines(domCanvas);

    domDocument.body!.append(domCanvas.rootElement);
    await matchGoldenFile('canvas_lines_thickness_dom_canvas.png', region: region);
  });
  test('draws lines with negative Offset values with dom canvas', () async {
    // test rendering lines correctly with negative offset when using DOM
    final SurfacePaintData paintWithStyle =
        SurfacePaintData()
          ..color =
              0xFFE91E63 // Colors.pink
          ..style = PaintingStyle.stroke
          ..strokeWidth = 16
          ..strokeCap = StrokeCap.round;

    // canvas.drawLine ignores paint.style (defaults to fill) according to api docs.
    // expect lines are rendered the same regardless of the set paint.style
    final SurfacePaintData paintWithoutStyle =
        SurfacePaintData()
          ..color =
              0xFF4CAF50 // Colors.green
          ..strokeWidth = 16
          ..strokeCap = StrokeCap.round;

    // test vertical, horizontal, and diagonal lines
    final List<Offset> points = <Offset>[
      const Offset(-25, 50),
      const Offset(45, 50),
      const Offset(100, -25),
      const Offset(100, 200),
      const Offset(-150, -145),
      const Offset(100, 200),
    ];
    final List<Offset> shiftedPoints =
        points.map((Offset point) => point.translate(20, 20)).toList();

    paintLinesFromPoints(domCanvas, paintWithStyle, points);
    paintLinesFromPoints(domCanvas, paintWithoutStyle, shiftedPoints);

    domDocument.body!.append(domCanvas.rootElement);
    await matchGoldenFile('canvas_lines_with_negative_offset.png', region: region);
  });

  test('drawLines method respects strokeCap with dom canvas', () async {
    final SurfacePaintData paintStrokeCapRound =
        SurfacePaintData()
          ..color =
              0xFFE91E63 // Colors.pink
          ..strokeWidth = 16
          ..strokeCap = StrokeCap.round;

    final SurfacePaintData paintStrokeCapSquare =
        SurfacePaintData()
          ..color =
              0xFF4CAF50 // Colors.green
          ..strokeWidth = 16
          ..strokeCap = StrokeCap.square;

    final SurfacePaintData paintStrokeCapButt =
        SurfacePaintData()
          ..color =
              0xFFFF9800 // Colors.orange
          ..strokeWidth = 16
          ..strokeCap = StrokeCap.butt;

    // test vertical, horizontal, and diagonal lines
    final List<Offset> points = <Offset>[
      const Offset(5, 50),
      const Offset(45, 50),
      const Offset(100, 5),
      const Offset(100, 200),
      const Offset(5, 10),
      const Offset(100, 200),
    ];
    final List<Offset> shiftedPoints =
        points.map((Offset point) => point.translate(50, 50)).toList();
    final List<Offset> twiceShiftedPoints =
        shiftedPoints.map((Offset point) => point.translate(50, 50)).toList();

    paintLinesFromPoints(domCanvas, paintStrokeCapRound, points);
    paintLinesFromPoints(domCanvas, paintStrokeCapSquare, shiftedPoints);
    paintLinesFromPoints(domCanvas, paintStrokeCapButt, twiceShiftedPoints);

    domDocument.body!.append(domCanvas.rootElement);
    await matchGoldenFile('canvas_lines_with_strokeCap.png', region: region);
  });
}

void paintLines(BitmapCanvas canvas) {
  final SurfacePaintData nullPaint =
      SurfacePaintData()
        ..strokeWidth = 1.0
        ..style = PaintingStyle.stroke;
  final SurfacePaintData paint1 =
      SurfacePaintData()
        ..color =
            0xFF9E9E9E // Colors.grey
        ..strokeWidth = 1.0
        ..style = PaintingStyle.stroke;
  final SurfacePaintData paint2 =
      SurfacePaintData()
        ..color = 0x7fff0000
        ..strokeWidth = 1.0
        ..style = PaintingStyle.stroke;
  final SurfacePaintData paint3 =
      SurfacePaintData()
        ..color =
            0xFF4CAF50 //Colors.green
        ..strokeWidth = 1.0
        ..style = PaintingStyle.stroke;
  // Draw markers around 100x100 box
  canvas.drawLine(const Offset(50, 40), const Offset(148, 40), nullPaint);
  canvas.drawLine(const Offset(50, 50), const Offset(52, 50), paint1);
  canvas.drawLine(const Offset(150, 50), const Offset(148, 50), paint1);
  canvas.drawLine(const Offset(50, 150), const Offset(52, 150), paint1);
  canvas.drawLine(const Offset(150, 150), const Offset(148, 150), paint1);
  // Draw diagonal
  canvas.drawLine(const Offset(50, 50), const Offset(150, 150), paint2);
  // Draw horizontal
  paint3.strokeWidth = 1.0;
  paint3.color = 0xFFFF0000;
  canvas.drawLine(const Offset(50, 55), const Offset(150, 55), paint3);
  paint3.strokeWidth = 2.0;
  paint3.color = 0xFF2196F3; // Colors.blue;
  canvas.drawLine(const Offset(50, 60), const Offset(150, 60), paint3);
  paint3.strokeWidth = 4.0;
  paint3.color = 0xFFFF9800; // Colors.orange;
  canvas.drawLine(const Offset(50, 70), const Offset(150, 70), paint3);
}

void paintLinesFromPoints(BitmapCanvas canvas, SurfacePaintData paint, List<Offset> points) {
  // points list contains pairs of Offset points, so for loop step is 2
  for (int i = 0; i < points.length - 1; i += 2) {
    canvas.drawLine(points[i], points[i + 1], paint);
  }
}
