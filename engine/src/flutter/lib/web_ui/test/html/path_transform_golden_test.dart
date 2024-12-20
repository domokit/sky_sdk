// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' hide TextStyle;

import '../common/test_initialization.dart';
import 'screenshot.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  const double screenWidth = 600.0;
  const double screenHeight = 800.0;
  const Rect screenRect = Rect.fromLTWH(0, 0, screenWidth, screenHeight);

  setUpUnitTests(setUpTestViewDimensions: false);

  test('Should draw transformed line.', () async {
    final RecordingCanvas rc = RecordingCanvas(const Rect.fromLTRB(0, 0, 500, 500));
    final Path path = Path();
    path.moveTo(0, 0);
    path.lineTo(300, 200);
    rc.drawPath(
      path,
      SurfacePaint()
        ..style = PaintingStyle.stroke
        ..strokeWidth = 2.0
        ..color = const Color(0xFF404000),
    );
    final Path transformedPath = Path();
    final Matrix4 testMatrixTranslateRotate = Matrix4.rotationZ(math.pi * 30.0 / 180.0)
      ..translate(100, 20);
    transformedPath.addPath(path, Offset.zero, matrix4: testMatrixTranslateRotate.toFloat64());
    rc.drawPath(
      transformedPath,
      SurfacePaint()
        ..style = PaintingStyle.stroke
        ..strokeWidth = 2.0
        ..color = const Color.fromRGBO(0, 128, 255, 1.0),
    );
    await canvasScreenshot(rc, 'path_transform_with_line', canvasRect: screenRect);
  });

  test('Should draw transformed line.', () async {
    final RecordingCanvas rc = RecordingCanvas(const Rect.fromLTRB(0, 0, 500, 500));
    final Path path = Path();
    path.addRect(const Rect.fromLTRB(50, 40, 300, 100));
    rc.drawPath(
      path,
      SurfacePaint()
        ..style = PaintingStyle.stroke
        ..strokeWidth = 2.0
        ..color = const Color(0xFF404000),
    );
    final Path transformedPath = Path();
    final Matrix4 testMatrixTranslateRotate = Matrix4.rotationZ(math.pi * 30.0 / 180.0)
      ..translate(100, 20);
    transformedPath.addPath(path, Offset.zero, matrix4: testMatrixTranslateRotate.toFloat64());
    rc.drawPath(
      transformedPath,
      SurfacePaint()
        ..style = PaintingStyle.stroke
        ..strokeWidth = 2.0
        ..color = const Color.fromRGBO(0, 128, 255, 1.0),
    );
    await canvasScreenshot(rc, 'path_transform_with_rect', canvasRect: screenRect);
  });

  test('Should draw transformed quadratic curve.', () async {
    final RecordingCanvas rc = RecordingCanvas(const Rect.fromLTRB(0, 0, 500, 500));
    final Path path = Path();
    path.moveTo(100, 100);
    path.quadraticBezierTo(100, 300, 400, 300);
    rc.drawPath(
      path,
      SurfacePaint()
        ..style = PaintingStyle.stroke
        ..strokeWidth = 2.0
        ..color = const Color(0xFF404000),
    );
    final Path transformedPath = Path();
    final Matrix4 testMatrixTranslateRotate = Matrix4.rotationZ(math.pi * 30.0 / 180.0)
      ..translate(100, -80);
    transformedPath.addPath(path, Offset.zero, matrix4: testMatrixTranslateRotate.toFloat64());
    rc.drawPath(
      transformedPath,
      SurfacePaint()
        ..style = PaintingStyle.stroke
        ..strokeWidth = 2.0
        ..color = const Color.fromRGBO(0, 128, 255, 1.0),
    );
    await canvasScreenshot(rc, 'path_transform_with_quadratic_curve', canvasRect: screenRect);
  });

  test('Should draw transformed conic.', () async {
    final RecordingCanvas rc = RecordingCanvas(const Rect.fromLTRB(0, 0, 500, 500));
    const double yStart = 20;

    const Offset p0 = Offset(25, yStart + 25);
    const Offset pc = Offset(60, yStart + 150);
    const Offset p2 = Offset(100, yStart + 50);

    final Path path = Path();
    path.moveTo(p0.dx, p0.dy);
    path.conicTo(pc.dx, pc.dy, p2.dx, p2.dy, 0.5);
    path.close();
    path.moveTo(p0.dx, p0.dy + 100);
    path.conicTo(pc.dx, pc.dy + 100, p2.dx, p2.dy + 100, 10);
    path.close();

    rc.drawPath(
      path,
      SurfacePaint()
        ..style = PaintingStyle.stroke
        ..strokeWidth = 2.0
        ..color = const Color(0xFF404000),
    );
    final Path transformedPath = Path();
    final Matrix4 testMatrixTranslateRotate = Matrix4.rotationZ(math.pi * 30.0 / 180.0)
      ..translate(100, -80);
    transformedPath.addPath(path, Offset.zero, matrix4: testMatrixTranslateRotate.toFloat64());
    rc.drawPath(
      transformedPath,
      SurfacePaint()
        ..style = PaintingStyle.stroke
        ..strokeWidth = 2.0
        ..color = const Color.fromRGBO(0, 128, 255, 1.0),
    );
    await canvasScreenshot(rc, 'path_transform_with_conic', canvasRect: screenRect);
  });

  test('Should draw transformed arc.', () async {
    final RecordingCanvas rc = RecordingCanvas(const Rect.fromLTRB(0, 0, 500, 500));

    final Path path = Path();
    path.moveTo(350, 280);
    path.arcToPoint(
      const Offset(450, 90),
      radius: const Radius.elliptical(200, 50),
      rotation: -math.pi / 6.0,
      largeArc: true,
    );
    path.close();

    rc.drawPath(
      path,
      SurfacePaint()
        ..style = PaintingStyle.stroke
        ..strokeWidth = 2.0
        ..color = const Color(0xFF404000),
    );

    final Path transformedPath = Path();
    final Matrix4 testMatrixTranslateRotate = Matrix4.rotationZ(math.pi * 30.0 / 180.0)
      ..translate(100, 10);
    transformedPath.addPath(path, Offset.zero, matrix4: testMatrixTranslateRotate.toFloat64());
    rc.drawPath(
      transformedPath,
      SurfacePaint()
        ..style = PaintingStyle.stroke
        ..strokeWidth = 2.0
        ..color = const Color.fromRGBO(0, 128, 255, 1.0),
    );
    await canvasScreenshot(rc, 'path_transform_with_arc', canvasRect: screenRect);
  });

  test('Should draw transformed rrect.', () async {
    final RecordingCanvas rc = RecordingCanvas(const Rect.fromLTRB(0, 0, 500, 500));

    final Path path = Path();
    path.addRRect(RRect.fromLTRBR(50, 50, 300, 200, const Radius.elliptical(4, 8)));

    rc.drawPath(
      path,
      SurfacePaint()
        ..style = PaintingStyle.stroke
        ..strokeWidth = 2.0
        ..color = const Color(0xFF404000),
    );

    final Path transformedPath = Path();
    final Matrix4 testMatrixTranslateRotate = Matrix4.rotationZ(math.pi * 30.0 / 180.0)
      ..translate(100, -80);
    transformedPath.addPath(path, Offset.zero, matrix4: testMatrixTranslateRotate.toFloat64());
    rc.drawPath(
      transformedPath,
      SurfacePaint()
        ..style = PaintingStyle.stroke
        ..strokeWidth = 2.0
        ..color = const Color.fromRGBO(0, 128, 255, 1.0),
    );
    await canvasScreenshot(rc, 'path_transform_with_rrect', canvasRect: screenRect);
  });
}
