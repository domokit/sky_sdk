// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/ui.dart';
import 'package:web_engine_tester/golden_tester.dart';

import '../common/test_initialization.dart';
import 'utils.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  setUpUnitTests(withImplicitView: true, setUpTestViewDimensions: false);

  const Rect region = Rect.fromLTWH(0, 0, 300, 300);

  test('draws lines with varying strokeWidth', () async {
    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder, region);
    paintLines(canvas);

    await drawPictureUsingCurrentRenderer(recorder.endRecording());

    await matchGoldenFile('canvas_lines_thickness.png', region: region);
  });

  test('draws lines with negative Offset values', () async {
    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder, region);

    // test rendering lines correctly with negative offset when using DOM
    final Paint paintWithStyle =
        Paint()
          ..color = const Color(0xFFE91E63) // Colors.pink
          ..style = PaintingStyle.stroke
          ..strokeWidth = 16
          ..strokeCap = StrokeCap.round;

    // canvas.drawLine ignores paint.style (defaults to fill) according to api docs.
    // expect lines are rendered the same regardless of the set paint.style
    final Paint paintWithoutStyle =
        Paint()
          ..color = const Color(0xFF4CAF50) // Colors.green
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

    paintLinesFromPoints(canvas, paintWithStyle, points);
    paintLinesFromPoints(canvas, paintWithoutStyle, shiftedPoints);

    await drawPictureUsingCurrentRenderer(recorder.endRecording());
    await matchGoldenFile('canvas_lines_with_negative_offset.png', region: region);
  });

  test('drawLines method respects strokeCap', () async {
    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder, region);

    final Paint paintStrokeCapRound =
        Paint()
          ..color = const Color(0xFFE91E63) // Colors.pink
          ..strokeWidth = 16
          ..strokeCap = StrokeCap.round;

    final Paint paintStrokeCapSquare =
        Paint()
          ..color = const Color(0xFF4CAF50) // Colors.green
          ..strokeWidth = 16
          ..strokeCap = StrokeCap.square;

    final Paint paintStrokeCapButt =
        Paint()
          ..color = const Color(0xFFFF9800) // Colors.orange
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

    paintLinesFromPoints(canvas, paintStrokeCapRound, points);
    paintLinesFromPoints(canvas, paintStrokeCapSquare, shiftedPoints);
    paintLinesFromPoints(canvas, paintStrokeCapButt, twiceShiftedPoints);

    await drawPictureUsingCurrentRenderer(recorder.endRecording());
    await matchGoldenFile('canvas_lines_with_strokeCap.png', region: region);
  });
}

void paintLines(Canvas canvas) {
  final Paint nullPaint =
      Paint()
        ..strokeWidth = 1.0
        ..style = PaintingStyle.stroke;
  final Paint paint1 =
      Paint()
        ..color = const Color(0xFF9E9E9E) // Colors.grey
        ..strokeWidth = 1.0
        ..style = PaintingStyle.stroke;
  final Paint paint2 =
      Paint()
        ..color = const Color(0x7fff0000)
        ..strokeWidth = 1.0
        ..style = PaintingStyle.stroke;
  final Paint paint3 =
      Paint()
        ..color = const Color(0xFF4CAF50) //Colors.green
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
  paint3.color = const Color(0xFFFF0000);
  canvas.drawLine(const Offset(50, 55), const Offset(150, 55), paint3);
  paint3.strokeWidth = 2.0;
  paint3.color = const Color(0xFF2196F3); // Colors.blue;
  canvas.drawLine(const Offset(50, 60), const Offset(150, 60), paint3);
  paint3.strokeWidth = 4.0;
  paint3.color = const Color(0xFFFF9800); // Colors.orange;
  canvas.drawLine(const Offset(50, 70), const Offset(150, 70), paint3);
}

void paintLinesFromPoints(Canvas canvas, Paint paint, List<Offset> points) {
  // points list contains pairs of Offset points, so for loop step is 2
  for (int i = 0; i < points.length - 1; i += 2) {
    canvas.drawLine(points[i], points[i + 1], paint);
  }
}
