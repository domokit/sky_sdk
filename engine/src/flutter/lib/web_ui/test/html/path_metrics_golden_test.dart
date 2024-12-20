// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' hide TextStyle;

import '../common/matchers.dart';
import '../common/test_initialization.dart';
import 'screenshot.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  const double screenWidth = 600.0;
  const double screenHeight = 800.0;
  const Rect screenRect = Rect.fromLTWH(0, 0, screenWidth, screenHeight);
  const Color black12Color = Color(0x1F000000);
  const Color redAccentColor = Color(0xFFFF1744);
  const double kDashLength = 5.0;

  setUpUnitTests(setUpTestViewDimensions: false);

  test('Should calculate tangent on line', () async {
    final Path path = Path();
    path.moveTo(50, 130);
    path.lineTo(150, 20);

    final PathMetric metric = path.computeMetrics().first;
    final Tangent t = metric.getTangentForOffset(50.0)!;
    expect(t.position.dx, within(from: 83.633, distance: 0.01));
    expect(t.position.dy, within(from: 93.0, distance: 0.01));
    expect(t.vector.dx, within(from: 0.672, distance: 0.01));
    expect(t.vector.dy, within(from: -0.739, distance: 0.01));
  });

  test('Should calculate tangent on cubic curve', () async {
    final Path path = Path();
    const double p1x = 240;
    const double p1y = 120;
    const double p2x = 320;
    const double p2y = 25;
    path.moveTo(150, 20);
    path.quadraticBezierTo(p1x, p1y, p2x, p2y);
    final PathMetric metric = path.computeMetrics().first;
    final Tangent t = metric.getTangentForOffset(50.0)!;
    expect(t.position.dx, within(from: 187.25, distance: 0.01));
    expect(t.position.dy, within(from: 53.33, distance: 0.01));
    expect(t.vector.dx, within(from: 0.82, distance: 0.01));
    expect(t.vector.dy, within(from: 0.56, distance: 0.01));
  });

  test('Should calculate tangent on quadratic curve', () async {
    final Path path = Path();
    const double p0x = 150;
    const double p0y = 20;
    const double p1x = 320;
    const double p1y = 25;
    path.moveTo(150, 20);
    path.quadraticBezierTo(p0x, p0y, p1x, p1y);
    final PathMetric metric = path.computeMetrics().first;
    final Tangent t = metric.getTangentForOffset(50.0)!;
    expect(t.position.dx, within(from: 199.82, distance: 0.01));
    expect(t.position.dy, within(from: 21.46, distance: 0.01));
    expect(t.vector.dx, within(from: 0.99, distance: 0.01));
    expect(t.vector.dy, within(from: 0.02, distance: 0.01));
  });

  // Test for extractPath to draw 5 pixel length dashed line using quad curve.
  test('Should draw dashed line on quadratic curve.', () async {
    final RecordingCanvas rc = RecordingCanvas(const Rect.fromLTRB(0, 0, 500, 500));

    final SurfacePaint paint =
        SurfacePaint()
          ..style = PaintingStyle.stroke
          ..strokeWidth = 3
          ..color = black12Color;
    final SurfacePaint redPaint =
        SurfacePaint()
          ..style = PaintingStyle.stroke
          ..strokeWidth = 1
          ..color = redAccentColor;

    final SurfacePath path = SurfacePath();
    path.moveTo(50, 130);
    path.lineTo(150, 20);
    const double p1x = 240;
    const double p1y = 120;
    const double p2x = 320;
    const double p2y = 25;
    path.quadraticBezierTo(p1x, p1y, p2x, p2y);

    rc.drawPath(path, paint);

    const double t0 = 0.2;
    const double t1 = 0.7;

    final List<PathMetric> metrics = path.computeMetrics().toList();
    double totalLength = 0;
    for (final PathMetric m in metrics) {
      totalLength += m.length;
    }
    final Path dashedPath = Path();
    for (final PathMetric measurePath in path.computeMetrics()) {
      double distance = totalLength * t0;
      bool draw = true;
      while (distance < measurePath.length * t1) {
        const double length = kDashLength;
        if (draw) {
          dashedPath.addPath(measurePath.extractPath(distance, distance + length), Offset.zero);
        }
        distance += length;
        draw = !draw;
      }
    }
    rc.drawPath(dashedPath, redPaint);
    await canvasScreenshot(rc, 'path_dash_quadratic', canvasRect: screenRect);
  });

  // Test for extractPath to draw 5 pixel length dashed line using cubic curve.
  test('Should draw dashed line on cubic curve.', () async {
    final RecordingCanvas rc = RecordingCanvas(const Rect.fromLTRB(0, 0, 500, 500));

    final SurfacePaint paint =
        SurfacePaint()
          ..style = PaintingStyle.stroke
          ..strokeWidth = 3
          ..color = black12Color;
    final SurfacePaint redPaint =
        SurfacePaint()
          ..style = PaintingStyle.stroke
          ..strokeWidth = 1
          ..color = redAccentColor;

    final Path path = Path();
    path.moveTo(50, 130);
    path.lineTo(150, 20);
    const double p1x = 40;
    const double p1y = 120;
    const double p2x = 300;
    const double p2y = 130;
    const double p3x = 320;
    const double p3y = 25;
    path.cubicTo(p1x, p1y, p2x, p2y, p3x, p3y);

    rc.drawPath(path, paint);

    const double t0 = 0.2;
    const double t1 = 0.7;

    final List<PathMetric> metrics = path.computeMetrics().toList();
    double totalLength = 0;
    for (final PathMetric m in metrics) {
      totalLength += m.length;
    }
    final Path dashedPath = Path();
    for (final PathMetric measurePath in path.computeMetrics()) {
      double distance = totalLength * t0;
      bool draw = true;
      while (distance < measurePath.length * t1) {
        const double length = kDashLength;
        if (draw) {
          dashedPath.addPath(measurePath.extractPath(distance, distance + length), Offset.zero);
        }
        distance += length;
        draw = !draw;
      }
    }
    rc.drawPath(dashedPath, redPaint);
    await canvasScreenshot(rc, 'path_dash_cubic', canvasRect: screenRect);
  });
}
