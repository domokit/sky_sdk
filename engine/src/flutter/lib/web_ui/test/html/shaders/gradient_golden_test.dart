// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;
import 'dart:typed_data';

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart';

import '../../common/test_initialization.dart';
import '../screenshot.dart';

// TODO(yjbanov): unskip Firefox tests when Firefox implements WebGL in headless mode.
// https://github.com/flutter/flutter/issues/86623

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  const double screenWidth = 600.0;
  const double screenHeight = 800.0;
  const Rect screenRect = Rect.fromLTWH(0, 0, screenWidth, screenHeight);
  const Rect region = Rect.fromLTWH(0, 0, 500, 240);

  setUpUnitTests(withImplicitView: true);

  test('Paints sweep gradient rectangles', () async {
    final RecordingCanvas canvas = RecordingCanvas(const Rect.fromLTRB(0, 0, 400, 300));
    canvas.save();

    final SurfacePaint borderPaint =
        SurfacePaint()
          ..style = PaintingStyle.stroke
          ..strokeWidth = 1
          ..color = const Color(0xFF000000);

    const List<Color> colors = <Color>[
      Color(0xFF000000),
      Color(0xFFFF3C38),
      Color(0xFFFF8C42),
      Color(0xFFFFF275),
      Color(0xFF6699CC),
      Color(0xFF656D78),
    ];
    const List<double> stops = <double>[0.0, 0.05, 0.4, 0.6, 0.9, 1.0];

    GradientSweep sweepGradient = GradientSweep(
      const Offset(0.5, 0.5),
      colors,
      stops,
      TileMode.clamp,
      0,
      360.0 / 180.0 * math.pi,
      null,
    );

    final GradientSweep sweepGradientRotated = GradientSweep(
      const Offset(0.5, 0.5),
      colors,
      stops,
      TileMode.clamp,
      0,
      360.0 / 180.0 * math.pi,
      Matrix4.rotationZ(math.pi / 6.0).storage,
    );

    const double kBoxWidth = 150;
    const double kBoxHeight = 80;
    // Gradient with default center.
    Rect rectBounds = const Rect.fromLTWH(10, 20, kBoxWidth, kBoxHeight);
    canvas.drawRect(
      rectBounds,
      SurfacePaint()..shader = engineGradientToShader(sweepGradient, rectBounds),
    );
    canvas.drawRect(rectBounds, borderPaint);

    // Gradient with shifted center and rotation.
    rectBounds = rectBounds.translate(kBoxWidth + 10, 0);
    canvas.drawRect(
      rectBounds,
      SurfacePaint()
        ..shader = engineGradientToShader(
          sweepGradientRotated,
          Rect.fromLTWH(
            rectBounds.center.dx,
            rectBounds.top,
            rectBounds.width / 2,
            rectBounds.height,
          ),
        ),
    );
    canvas.drawRect(rectBounds, borderPaint);

    // Gradient with start/endangle.
    sweepGradient = GradientSweep(
      const Offset(0.5, 0.5),
      colors,
      stops,
      TileMode.clamp,
      math.pi / 6,
      3 * math.pi / 4,
      null,
    );

    rectBounds = rectBounds.translate(kBoxWidth + 10, 0);
    canvas.drawRect(
      rectBounds,
      SurfacePaint()..shader = engineGradientToShader(sweepGradient, rectBounds),
    );
    canvas.drawRect(rectBounds, borderPaint);

    // Tile mode repeat
    rectBounds = const Rect.fromLTWH(10, 110, kBoxWidth, kBoxHeight);
    sweepGradient = GradientSweep(
      const Offset(0.5, 0.5),
      colors,
      stops,
      TileMode.repeated,
      math.pi / 6,
      3 * math.pi / 4,
      null,
    );

    canvas.drawRect(
      rectBounds,
      SurfacePaint()..shader = engineGradientToShader(sweepGradient, rectBounds),
    );
    canvas.drawRect(rectBounds, borderPaint);

    // Tile mode mirror
    rectBounds = rectBounds.translate(kBoxWidth + 10, 0);
    sweepGradient = GradientSweep(
      const Offset(0.5, 0.5),
      colors,
      stops,
      TileMode.mirror,
      math.pi / 6,
      3 * math.pi / 4,
      null,
    );
    canvas.drawRect(
      rectBounds,
      SurfacePaint()..shader = engineGradientToShader(sweepGradient, rectBounds),
    );
    canvas.drawRect(rectBounds, borderPaint);

    canvas.restore();
    await canvasScreenshot(canvas, 'sweep_gradient_rect', canvasRect: screenRect, region: region);
  }, skip: isFirefox);

  test('Paints sweep gradient ovals', () async {
    final RecordingCanvas canvas = RecordingCanvas(const Rect.fromLTRB(0, 0, 400, 300));
    canvas.save();

    final SurfacePaint borderPaint =
        SurfacePaint()
          ..style = PaintingStyle.stroke
          ..strokeWidth = 1
          ..color = const Color(0xFF000000);

    const List<Color> colors = <Color>[
      Color(0xFF000000),
      Color(0xFFFF3C38),
      Color(0xFFFF8C42),
      Color(0xFFFFF275),
      Color(0xFF6699CC),
      Color(0xFF656D78),
    ];
    final List<double> stops = <double>[0.0, 0.05, 0.4, 0.6, 0.9, 1.0];

    GradientSweep sweepGradient = GradientSweep(
      const Offset(0.5, 0.5),
      colors,
      stops,
      TileMode.clamp,
      0,
      360.0 / 180.0 * math.pi,
      null,
    );

    final GradientSweep sweepGradientRotated = GradientSweep(
      const Offset(0.5, 0.5),
      colors,
      stops,
      TileMode.clamp,
      0,
      360.0 / 180.0 * math.pi,
      Matrix4.rotationZ(math.pi / 6.0).storage,
    );

    const double kBoxWidth = 150;
    const double kBoxHeight = 80;
    // Gradient with default center.
    Rect rectBounds = const Rect.fromLTWH(10, 20, kBoxWidth, kBoxHeight);
    canvas.drawOval(
      rectBounds,
      SurfacePaint()..shader = engineGradientToShader(sweepGradient, rectBounds),
    );
    canvas.drawRect(rectBounds, borderPaint);

    // Gradient with shifted center and rotation.
    rectBounds = rectBounds.translate(kBoxWidth + 10, 0);
    canvas.drawOval(
      rectBounds,
      SurfacePaint()
        ..shader = engineGradientToShader(
          sweepGradientRotated,
          Rect.fromLTWH(
            rectBounds.center.dx,
            rectBounds.top,
            rectBounds.width / 2,
            rectBounds.height,
          ),
        ),
    );
    canvas.drawRect(rectBounds, borderPaint);

    // Gradient with start/endangle.
    sweepGradient = GradientSweep(
      const Offset(0.5, 0.5),
      colors,
      stops,
      TileMode.clamp,
      math.pi / 6,
      3 * math.pi / 4,
      null,
    );

    rectBounds = rectBounds.translate(kBoxWidth + 10, 0);
    canvas.drawOval(
      rectBounds,
      SurfacePaint()..shader = engineGradientToShader(sweepGradient, rectBounds),
    );
    canvas.drawRect(rectBounds, borderPaint);

    // Tile mode repeat
    rectBounds = const Rect.fromLTWH(10, 110, kBoxWidth, kBoxHeight);
    sweepGradient = GradientSweep(
      const Offset(0.5, 0.5),
      colors,
      stops,
      TileMode.repeated,
      math.pi / 6,
      3 * math.pi / 4,
      null,
    );

    canvas.drawOval(
      rectBounds,
      SurfacePaint()..shader = engineGradientToShader(sweepGradient, rectBounds),
    );
    canvas.drawRect(rectBounds, borderPaint);

    // Tile mode mirror
    rectBounds = rectBounds.translate(kBoxWidth + 10, 0);
    sweepGradient = GradientSweep(
      const Offset(0.5, 0.5),
      colors,
      stops,
      TileMode.mirror,
      math.pi / 6,
      3 * math.pi / 4,
      null,
    );
    canvas.drawOval(
      rectBounds,
      SurfacePaint()..shader = engineGradientToShader(sweepGradient, rectBounds),
    );
    canvas.drawRect(rectBounds, borderPaint);

    canvas.restore();
    await canvasScreenshot(canvas, 'sweep_gradient_oval', canvasRect: screenRect, region: region);
  }, skip: isFirefox);

  test('Paints sweep gradient paths', () async {
    final RecordingCanvas canvas = RecordingCanvas(const Rect.fromLTRB(0, 0, 400, 300));
    canvas.save();

    final SurfacePaint borderPaint =
        SurfacePaint()
          ..style = PaintingStyle.stroke
          ..strokeWidth = 1
          ..color = const Color(0xFF000000);

    const List<Color> colors = <Color>[
      Color(0xFF000000),
      Color(0xFFFF3C38),
      Color(0xFFFF8C42),
      Color(0xFFFFF275),
      Color(0xFF6699CC),
      Color(0xFF656D78),
    ];
    const List<double> stops = <double>[0.0, 0.05, 0.4, 0.6, 0.9, 1.0];

    GradientSweep sweepGradient = GradientSweep(
      const Offset(0.5, 0.5),
      colors,
      stops,
      TileMode.clamp,
      0,
      360.0 / 180.0 * math.pi,
      null,
    );

    final GradientSweep sweepGradientRotated = GradientSweep(
      const Offset(0.5, 0.5),
      colors,
      stops,
      TileMode.clamp,
      0,
      360.0 / 180.0 * math.pi,
      Matrix4.rotationZ(math.pi / 6.0).storage,
    );

    const double kBoxWidth = 150;
    const double kBoxHeight = 80;
    // Gradient with default center.
    Rect rectBounds = const Rect.fromLTWH(10, 20, kBoxWidth, kBoxHeight);
    Path path = samplePathFromRect(rectBounds);
    canvas.drawPath(
      path,
      SurfacePaint()..shader = engineGradientToShader(sweepGradient, rectBounds),
    );
    canvas.drawRect(rectBounds, borderPaint);

    // Gradient with shifted center and rotation.
    rectBounds = rectBounds.translate(kBoxWidth + 10, 0);
    path = samplePathFromRect(rectBounds);
    canvas.drawPath(
      path,
      SurfacePaint()
        ..shader = engineGradientToShader(
          sweepGradientRotated,
          Rect.fromLTWH(
            rectBounds.center.dx,
            rectBounds.top,
            rectBounds.width / 2,
            rectBounds.height,
          ),
        ),
    );
    canvas.drawRect(rectBounds, borderPaint);

    // Gradient with start/endangle.
    sweepGradient = GradientSweep(
      const Offset(0.5, 0.5),
      colors,
      stops,
      TileMode.clamp,
      math.pi / 6,
      3 * math.pi / 4,
      null,
    );

    rectBounds = rectBounds.translate(kBoxWidth + 10, 0);
    path = samplePathFromRect(rectBounds);
    canvas.drawPath(
      path,
      SurfacePaint()..shader = engineGradientToShader(sweepGradient, rectBounds),
    );
    canvas.drawRect(rectBounds, borderPaint);

    // Tile mode repeat
    rectBounds = const Rect.fromLTWH(10, 110, kBoxWidth, kBoxHeight);
    sweepGradient = GradientSweep(
      const Offset(0.5, 0.5),
      colors,
      stops,
      TileMode.repeated,
      math.pi / 6,
      3 * math.pi / 4,
      null,
    );

    path = samplePathFromRect(rectBounds);
    canvas.drawPath(
      path,
      SurfacePaint()..shader = engineGradientToShader(sweepGradient, rectBounds),
    );
    canvas.drawRect(rectBounds, borderPaint);

    // Tile mode mirror
    rectBounds = rectBounds.translate(kBoxWidth + 10, 0);
    sweepGradient = GradientSweep(
      const Offset(0.5, 0.5),
      colors,
      stops,
      TileMode.mirror,
      math.pi / 6,
      3 * math.pi / 4,
      null,
    );
    path = samplePathFromRect(rectBounds);
    canvas.drawPath(
      path,
      SurfacePaint()..shader = engineGradientToShader(sweepGradient, rectBounds),
    );
    canvas.drawRect(rectBounds, borderPaint);

    canvas.restore();
    await canvasScreenshot(canvas, 'sweep_gradient_path', canvasRect: screenRect, region: region);
  }, skip: isFirefox);

  /// Regression test for https://github.com/flutter/flutter/issues/74137.
  test('Paints rotated and shifted linear gradient', () async {
    final RecordingCanvas canvas = RecordingCanvas(const Rect.fromLTRB(0, 0, 400, 300));
    canvas.save();

    final SurfacePaint borderPaint =
        SurfacePaint()
          ..style = PaintingStyle.stroke
          ..strokeWidth = 1
          ..color = const Color(0xFF000000);

    const List<Color> colors = <Color>[
      Color(0xFF000000),
      Color(0xFFFF3C38),
      Color(0xFFFF8C42),
      Color(0xFFFFF275),
      Color(0xFF6699CC),
      Color(0xFF656D78),
    ];
    const List<double> stops = <double>[0.0, 0.05, 0.4, 0.6, 0.9, 1.0];

    GradientLinear linearGradient = GradientLinear(
      const Offset(50, 50),
      const Offset(200, 130),
      colors,
      stops,
      TileMode.clamp,
      Matrix4.identity().storage,
    );

    const double kBoxWidth = 150;
    const double kBoxHeight = 80;
    // Gradient with default center.
    Rect rectBounds = const Rect.fromLTWH(10, 20, kBoxWidth, kBoxHeight);
    canvas.drawRect(
      rectBounds,
      SurfacePaint()..shader = engineLinearGradientToShader(linearGradient, rectBounds),
    );
    canvas.drawRect(rectBounds, borderPaint);

    // Tile mode repeat
    rectBounds = const Rect.fromLTWH(10, 110, kBoxWidth, kBoxHeight);
    linearGradient = GradientLinear(
      const Offset(50, 50),
      const Offset(200, 130),
      colors,
      stops,
      TileMode.repeated,
      Matrix4.identity().storage,
    );

    canvas.drawRect(
      rectBounds,
      SurfacePaint()..shader = engineLinearGradientToShader(linearGradient, rectBounds),
    );
    canvas.drawRect(rectBounds, borderPaint);

    canvas.restore();
    await canvasScreenshot(
      canvas,
      'linear_gradient_rect_shifted',
      canvasRect: screenRect,
      region: region,
    );
  }, skip: isFirefox);

  /// Regression test for https://github.com/flutter/flutter/issues/82748.
  test('Paints gradient with gradient stop outside range', () async {
    final RecordingCanvas canvas = RecordingCanvas(const Rect.fromLTRB(0, 0, 400, 300));
    canvas.save();

    final SurfacePaint borderPaint =
        SurfacePaint()
          ..style = PaintingStyle.stroke
          ..strokeWidth = 1
          ..color = const Color(0xFF000000);

    const List<Color> colors = <Color>[Color(0xFF000000), Color(0xFFFF3C38)];
    const List<double> stops = <double>[0.0, 10.0];

    final GradientLinear linearGradient = GradientLinear(
      const Offset(50, 50),
      const Offset(200, 130),
      colors,
      stops,
      TileMode.clamp,
      Matrix4.identity().storage,
    );

    const double kBoxWidth = 150;
    const double kBoxHeight = 80;
    // Gradient with default center.
    const Rect rectBounds = Rect.fromLTWH(10, 20, kBoxWidth, kBoxHeight);
    canvas.drawRect(
      rectBounds,
      SurfacePaint()..shader = engineLinearGradientToShader(linearGradient, rectBounds),
    );
    canvas.drawRect(rectBounds, borderPaint);
    canvas.restore();

    final EngineCanvas engineCanvas = BitmapCanvas(screenRect, RenderStrategy());
    canvas.endRecording();
    canvas.apply(engineCanvas, screenRect);
  }, skip: isFirefox);

  test("Creating lots of gradients doesn't create too many webgl contexts", () async {
    final DomCanvasElement sideCanvas = createDomCanvasElement(width: 5, height: 5);
    final DomCanvasRenderingContextWebGl? context =
        sideCanvas.getContext('webgl') as DomCanvasRenderingContextWebGl?;
    expect(context, isNotNull);

    final EngineCanvas engineCanvas = BitmapCanvas(
      const Rect.fromLTRB(0, 0, 100, 100),
      RenderStrategy(),
    );
    for (double x = 0; x < 100; x += 10) {
      for (double y = 0; y < 100; y += 10) {
        const List<Color> colors = <Color>[Color(0xFFFF0000), Color(0xFF0000FF)];

        final GradientLinear linearGradient = GradientLinear(
          Offset.zero,
          const Offset(10, 10),
          colors,
          null,
          TileMode.clamp,
          Matrix4.identity().storage,
        );
        engineCanvas.drawRect(
          Rect.fromLTWH(x, y, 10, 10),
          SurfacePaintData()..shader = linearGradient,
        );
      }
    }

    expect(context!.isContextLost(), isFalse);
  }, skip: isFirefox);

  test('Paints clamped, rotated and shifted linear gradient', () async {
    final RecordingCanvas canvas = RecordingCanvas(const Rect.fromLTRB(0, 0, 400, 300));
    canvas.save();

    final SurfacePaint borderPaint =
        SurfacePaint()
          ..style = PaintingStyle.stroke
          ..strokeWidth = 1
          ..color = const Color(0xFF000000);

    const List<Color> colors = <Color>[
      Color(0xFF000000),
      Color(0xFFFF3C38),
      Color(0xFFFF8C42),
      Color(0xFFFFF275),
      Color(0xFF6699CC),
      Color(0xFF656D78),
    ];
    const List<double> stops = <double>[0.0, 0.05, 0.4, 0.6, 0.9, 1.0];

    GradientLinear linearGradient = GradientLinear(
      const Offset(50, 50),
      const Offset(200, 130),
      colors,
      stops,
      TileMode.clamp,
      Matrix4.identity().storage,
    );

    const double kBoxWidth = 150;
    const double kBoxHeight = 80;
    // Gradient with default center.
    Rect rectBounds = const Rect.fromLTWH(10, 20, kBoxWidth, kBoxHeight);
    canvas.drawRect(
      rectBounds,
      SurfacePaint()..shader = engineLinearGradientToShader(linearGradient, rectBounds),
    );
    canvas.drawRect(rectBounds, borderPaint);

    // Tile mode repeat
    rectBounds = const Rect.fromLTWH(10, 110, kBoxWidth, kBoxHeight);
    linearGradient = GradientLinear(
      const Offset(50, 50),
      const Offset(200, 130),
      colors,
      stops,
      TileMode.clamp,
      Matrix4.identity().storage,
    );

    canvas.drawRect(
      rectBounds,
      SurfacePaint()..shader = engineLinearGradientToShader(linearGradient, rectBounds),
    );
    canvas.drawRect(rectBounds, borderPaint);

    canvas.restore();
    await canvasScreenshot(
      canvas,
      'linear_gradient_rect_clamp_rotated',
      canvasRect: screenRect,
      region: region,
    );
  }, skip: isFirefox);

  test('Paints linear gradient properly when within svg context', () async {
    final RecordingCanvas canvas = RecordingCanvas(const Rect.fromLTRB(0, 0, 500, 240));
    canvas.save();

    canvas.renderStrategy.isInsideSvgFilterTree = true;

    final SurfacePaint borderPaint =
        SurfacePaint()
          ..style = PaintingStyle.stroke
          ..strokeWidth = 1
          ..color = const Color(0xFF000000);

    const List<Color> colors = <Color>[Color(0xFFFF0000), Color(0xFF0000FF)];

    final GradientLinear linearGradient = GradientLinear(
      const Offset(125, 75),
      const Offset(175, 125),
      colors,
      null,
      TileMode.clamp,
      Matrix4.identity().storage,
    );

    const double kBoxWidth = 150;
    const double kBoxHeight = 100;
    // Gradient with default center.
    const Rect rectBounds = Rect.fromLTWH(100, 50, kBoxWidth, kBoxHeight);
    canvas.drawRect(
      rectBounds,
      SurfacePaint()..shader = engineLinearGradientToShader(linearGradient, rectBounds),
    );
    canvas.drawRect(rectBounds, borderPaint);

    canvas.restore();
    await canvasScreenshot(
      canvas,
      'linear_gradient_in_svg_context',
      canvasRect: screenRect,
      region: region,
    );
  }, skip: isFirefox);

  test('Paints transformed linear gradient', () async {
    final RecordingCanvas canvas = RecordingCanvas(const Rect.fromLTRB(0, 0, 400, 300));
    canvas.save();

    const List<Color> colors = <Color>[
      Color(0xFF000000),
      Color(0xFFFF3C38),
      Color(0xFFFF8C42),
      Color(0xFFFFF275),
      Color(0xFF6699CC),
      Color(0xFF656D78),
    ];

    const List<double> stops = <double>[0.0, 0.05, 0.4, 0.6, 0.9, 1.0];

    final Matrix4 transform =
        Matrix4.identity()
          ..translate(50, 50)
          ..scale(0.3, 0.7)
          ..rotateZ(0.5);

    final GradientLinear linearGradient = GradientLinear(
      const Offset(5, 5),
      const Offset(200, 130),
      colors,
      stops,
      TileMode.clamp,
      transform.storage,
    );

    const double kBoxWidth = 150;
    const double kBoxHeight = 80;

    Rect rectBounds = const Rect.fromLTWH(10, 20, kBoxWidth, kBoxHeight);
    canvas.drawRect(
      rectBounds,
      SurfacePaint()..shader = engineLinearGradientToShader(linearGradient, rectBounds),
    );

    rectBounds = const Rect.fromLTWH(10, 110, kBoxWidth, kBoxHeight);
    canvas.drawOval(
      rectBounds,
      SurfacePaint()..shader = engineLinearGradientToShader(linearGradient, rectBounds),
    );

    canvas.restore();
    await canvasScreenshot(
      canvas,
      'linear_gradient_clamp_transformed',
      canvasRect: screenRect,
      region: region,
    );
  }, skip: isFirefox);

  test('Paints transformed sweep gradient', () async {
    final RecordingCanvas canvas = RecordingCanvas(const Rect.fromLTRB(0, 0, 400, 300));
    canvas.save();

    const List<Color> colors = <Color>[
      Color(0xFF000000),
      Color(0xFFFF3C38),
      Color(0xFFFF8C42),
      Color(0xFFFFF275),
      Color(0xFF6699CC),
      Color(0xFF656D78),
    ];

    const List<double> stops = <double>[0.0, 0.05, 0.4, 0.6, 0.9, 1.0];

    final Matrix4 transform =
        Matrix4.identity()
          ..translate(100, 150)
          ..scale(0.3, 0.7)
          ..rotateZ(0.5);

    final GradientSweep sweepGradient = GradientSweep(
      const Offset(0.5, 0.5),
      colors,
      stops,
      TileMode.clamp,
      0.0,
      2 * math.pi,
      transform.storage,
    );

    const double kBoxWidth = 150;
    const double kBoxHeight = 80;

    Rect rectBounds = const Rect.fromLTWH(10, 20, kBoxWidth, kBoxHeight);
    canvas.drawRect(
      rectBounds,
      SurfacePaint()..shader = engineGradientToShader(sweepGradient, rectBounds),
    );

    rectBounds = const Rect.fromLTWH(10, 110, kBoxWidth, kBoxHeight);
    canvas.drawOval(
      rectBounds,
      SurfacePaint()..shader = engineGradientToShader(sweepGradient, rectBounds),
    );

    canvas.restore();
    await canvasScreenshot(
      canvas,
      'sweep_gradient_clamp_transformed',
      canvasRect: screenRect,
      region: region,
    );
  }, skip: isFirefox);

  test('Paints transformed radial gradient', () async {
    final RecordingCanvas canvas = RecordingCanvas(const Rect.fromLTRB(0, 0, 400, 300));
    canvas.save();

    const List<Color> colors = <Color>[
      Color(0xFF000000),
      Color(0xFFFF3C38),
      Color(0xFFFF8C42),
      Color(0xFFFFF275),
      Color(0xFF6699CC),
      Color(0xFF656D78),
    ];

    const List<double> stops = <double>[0.0, 0.05, 0.4, 0.6, 0.9, 1.0];

    final Matrix4 transform =
        Matrix4.identity()
          ..translate(50, 50)
          ..scale(0.3, 0.7)
          ..rotateZ(0.5);

    final GradientRadial radialGradient = GradientRadial(
      const Offset(0.5, 0.5),
      400,
      colors,
      stops,
      TileMode.clamp,
      transform.storage,
    );

    const double kBoxWidth = 150;
    const double kBoxHeight = 80;

    Rect rectBounds = const Rect.fromLTWH(10, 20, kBoxWidth, kBoxHeight);
    canvas.drawRect(
      rectBounds,
      SurfacePaint()..shader = engineRadialGradientToShader(radialGradient, rectBounds),
    );

    rectBounds = const Rect.fromLTWH(10, 110, kBoxWidth, kBoxHeight);
    canvas.drawOval(
      rectBounds,
      SurfacePaint()..shader = engineRadialGradientToShader(radialGradient, rectBounds),
    );

    canvas.restore();
    await canvasScreenshot(
      canvas,
      'radial_gradient_clamp_transformed',
      canvasRect: screenRect,
      region: region,
    );
  }, skip: isFirefox);

  test('Paints two gradient with same width and different height', () async {
    final RecordingCanvas canvas = RecordingCanvas(const Rect.fromLTRB(0, 0, 400, 300));
    canvas.save();

    const List<Color> colors = <Color>[
      Color(0xFF000000),
      Color(0xFFFF3C38),
      Color(0xFFFF8C42),
      Color(0xFFFFF275),
      Color(0xFF6699CC),
      Color(0xFF656D78),
    ];

    const List<double> stops = <double>[0.0, 0.05, 0.4, 0.6, 0.9, 1.0];

    final Matrix4 transform =
        Matrix4.identity()
          ..translate(100, 150)
          ..scale(0.3, 0.7)
          ..rotateZ(0.5);

    final GradientSweep sweepGradient = GradientSweep(
      const Offset(0.5, 0.5),
      colors,
      stops,
      TileMode.clamp,
      0.0,
      2 * math.pi,
      transform.storage,
    );

    const double kBoxWidth = 150;
    const double kBoxHeight1 = 40;
    const double kBoxHeight2 = 80;

    const Rect rectBounds1 = Rect.fromLTWH(10, 20, kBoxWidth, kBoxHeight1);
    const Rect rectBounds2 = Rect.fromLTWH(10, 80, kBoxWidth, kBoxHeight2);
    canvas.drawRect(
      rectBounds1,
      SurfacePaint()..shader = engineGradientToShader(sweepGradient, rectBounds1),
    );
    canvas.drawRect(
      rectBounds2,
      SurfacePaint()..shader = engineGradientToShader(sweepGradient, rectBounds2),
    );

    canvas.restore();
    await canvasScreenshot(
      canvas,
      'radial_gradient_double_item',
      canvasRect: screenRect,
      region: region,
    );
  }, skip: isFirefox);
}

Shader engineGradientToShader(GradientSweep gradient, Rect rect) {
  return Gradient.sweep(
    Offset(
      rect.left + gradient.center.dx * rect.width,
      rect.top + gradient.center.dy * rect.height,
    ),
    gradient.colors,
    gradient.colorStops,
    gradient.tileMode,
    gradient.startAngle,
    gradient.endAngle,
    gradient.matrix4 == null ? null : Float64List.fromList(gradient.matrix4!),
  );
}

Shader engineLinearGradientToShader(GradientLinear gradient, Rect rect) {
  return Gradient.linear(
    gradient.from,
    gradient.to,
    gradient.colors,
    gradient.colorStops,
    gradient.tileMode,
    gradient.matrix4 == null ? null : Float64List.fromList(gradient.matrix4!.matrix),
  );
}

Shader engineRadialGradientToShader(GradientRadial gradient, Rect rect) {
  return Gradient.radial(
    Offset(
      rect.left + gradient.center.dx * rect.width,
      rect.top + gradient.center.dy * rect.height,
    ),
    gradient.radius,
    gradient.colors,
    gradient.colorStops,
    gradient.tileMode,
    gradient.matrix4 == null ? null : Float64List.fromList(gradient.matrix4!),
  );
}

Path samplePathFromRect(Rect rectBounds) =>
    Path()
      ..moveTo(rectBounds.center.dx, rectBounds.top)
      ..lineTo(rectBounds.left, rectBounds.bottom)
      ..quadraticBezierTo(
        rectBounds.center.dx + 20,
        rectBounds.bottom - 40,
        rectBounds.right,
        rectBounds.bottom,
      )
      ..close();
