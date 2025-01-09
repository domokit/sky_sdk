// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;
import 'dart:typed_data';

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart' hide ColorSpace;
import 'package:ui/ui.dart' hide TextStyle;
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;
import 'package:web_engine_tester/golden_tester.dart';

import '../common/matchers.dart';
import '../common/test_initialization.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  setUpUnitTests(withImplicitView: true, setUpTestViewDimensions: false);

  const double screenWidth = 600.0;
  const double screenHeight = 800.0;
  const Rect screenRect = Rect.fromLTWH(0, 0, screenWidth, screenHeight);
  final SurfacePaint testPaint = SurfacePaint()..color = const Color(0xFFFF0000);

  // Commit a recording canvas to a bitmap, and compare with the expected
  Future<void> checkScreenshot(
    RecordingCanvas rc,
    String fileName, {
    Rect region = const Rect.fromLTWH(0, 0, 500, 500),
  }) async {
    final EngineCanvas engineCanvas = BitmapCanvas(screenRect, RenderStrategy());

    // Draws the estimated bounds so we can spot the bug in Gold.
    engineCanvas
      ..save()
      ..drawRect(
        rc.pictureBounds!,
        SurfacePaintData()
          ..color = const Color.fromRGBO(0, 0, 255, 1.0).value
          ..style = PaintingStyle.stroke
          ..strokeWidth = 1.0,
      )
      ..restore();

    rc.apply(engineCanvas, screenRect);

    // Wrap in <flt-scene> so that our CSS selectors kick in.
    final DomElement sceneElement = createDomElement('flt-scene');
    if (isIosSafari) {
      // Shrink to fit on the iPhone screen.
      sceneElement.style.position = 'absolute';
      sceneElement.style.transformOrigin = '0 0 0';
      sceneElement.style.transform = 'scale(0.3)';
    }
    try {
      sceneElement.append(engineCanvas.rootElement);
      domDocument.body!.append(sceneElement);
      await matchGoldenFile('paint_bounds_for_$fileName.png', region: region);
    } finally {
      // The page is reused across tests, so remove the element after taking the
      // screenshot.
      sceneElement.remove();
    }
  }

  test('Empty canvas reports correct paint bounds', () async {
    final RecordingCanvas rc = RecordingCanvas(const Rect.fromLTWH(1, 2, 300, 400));
    rc.endRecording();
    expect(rc.pictureBounds, Rect.zero);
    await checkScreenshot(rc, 'empty_canvas');
  });

  test('Computes paint bounds for draw line', () async {
    final RecordingCanvas rc = RecordingCanvas(screenRect);
    rc.drawLine(const Offset(50, 100), const Offset(120, 140), testPaint);
    rc.endRecording();
    // The off by one is due to the minimum stroke width of 1.
    expect(rc.pictureBounds, const Rect.fromLTRB(49, 99, 121, 141));
    await checkScreenshot(rc, 'draw_line');
  });

  test('Computes paint bounds for draw line when line exceeds limits', () async {
    // Uses max bounds when computing paint bounds
    final RecordingCanvas rc = RecordingCanvas(screenRect);
    rc.drawLine(const Offset(50, 100), const Offset(screenWidth + 100.0, 140), testPaint);
    rc.endRecording();
    // The off by one is due to the minimum stroke width of 1.
    expect(rc.pictureBounds, const Rect.fromLTRB(49.0, 99.0, screenWidth, 141.0));
    await checkScreenshot(rc, 'draw_line_exceeding_limits');
  });

  test('Computes paint bounds for draw rect', () async {
    final RecordingCanvas rc = RecordingCanvas(screenRect);
    rc.drawRect(const Rect.fromLTRB(10, 20, 30, 40), testPaint);
    rc.endRecording();
    expect(rc.pictureBounds, const Rect.fromLTRB(10, 20, 30, 40));
    await checkScreenshot(rc, 'draw_rect');
  });

  test('Computes paint bounds for draw rect when exceeds limits', () async {
    // Uses max bounds when computing paint bounds
    RecordingCanvas rc = RecordingCanvas(screenRect);
    rc.drawRect(const Rect.fromLTRB(10, 20, 30 + screenWidth, 40 + screenHeight), testPaint);
    rc.endRecording();
    expect(rc.pictureBounds, const Rect.fromLTRB(10, 20, screenWidth, screenHeight));

    rc = RecordingCanvas(screenRect);
    rc.drawRect(const Rect.fromLTRB(-200, -100, 30, 40), testPaint);
    rc.endRecording();
    expect(rc.pictureBounds, const Rect.fromLTRB(0, 0, 30, 40));
    await checkScreenshot(rc, 'draw_rect_exceeding_limits');
  });

  test('Computes paint bounds for translate', () async {
    final RecordingCanvas rc = RecordingCanvas(screenRect);
    rc.translate(5, 7);
    rc.drawRect(const Rect.fromLTRB(10, 20, 30, 40), testPaint);
    rc.endRecording();
    expect(rc.pictureBounds, const Rect.fromLTRB(15, 27, 35, 47));
    await checkScreenshot(rc, 'translate');
  });

  test('Computes paint bounds for scale', () async {
    final RecordingCanvas rc = RecordingCanvas(screenRect);
    rc.scale(2, 2);
    rc.drawRect(const Rect.fromLTRB(10, 20, 30, 40), testPaint);
    rc.endRecording();
    expect(rc.pictureBounds, const Rect.fromLTRB(20, 40, 60, 80));
    await checkScreenshot(rc, 'scale');
  });

  test('Computes paint bounds for rotate', () async {
    final RecordingCanvas rc = RecordingCanvas(screenRect);
    rc.rotate(math.pi / 4.0);
    rc.drawLine(const Offset(1, 0), Offset(50 * math.sqrt(2) - 1, 0), testPaint);
    rc.endRecording();
    // The extra 0.7 is due to stroke width of 1 rotated by 45 degrees.
    expect(rc.pictureBounds, within(distance: 0.1, from: const Rect.fromLTRB(0, 0, 50.7, 50.7)));
    await checkScreenshot(rc, 'rotate');
  });

  test('Computes paint bounds for horizontal skew', () async {
    final RecordingCanvas rc = RecordingCanvas(screenRect);
    rc.skew(1.0, 0.0);
    rc.drawRect(const Rect.fromLTRB(20, 20, 40, 40), testPaint);
    rc.endRecording();
    expect(
      rc.pictureBounds,
      within(distance: 0.1, from: const Rect.fromLTRB(40.0, 20.0, 80.0, 40.0)),
    );
    await checkScreenshot(rc, 'skew_horizontally');
  });

  test('Computes paint bounds for vertical skew', () async {
    final RecordingCanvas rc = RecordingCanvas(screenRect);
    rc.skew(0.0, 1.0);
    rc.drawRect(const Rect.fromLTRB(20, 20, 40, 40), testPaint);
    rc.endRecording();
    expect(
      rc.pictureBounds,
      within(distance: 0.1, from: const Rect.fromLTRB(20.0, 40.0, 40.0, 80.0)),
    );
    await checkScreenshot(rc, 'skew_vertically');
  });

  test('Computes paint bounds for a complex transform', () async {
    final RecordingCanvas rc = RecordingCanvas(screenRect);
    final Float32List matrix = Float32List(16);
    // translate(210, 220) , scale(2, 3), rotate(math.pi / 4.0)
    matrix[0] = 1.4;
    matrix[1] = 2.12;
    matrix[2] = 0.0;
    matrix[3] = 0.0;
    matrix[4] = -1.4;
    matrix[5] = 2.12;
    matrix[6] = 0.0;
    matrix[7] = 0.0;
    matrix[8] = 0.0;
    matrix[9] = 0.0;
    matrix[10] = 2.0;
    matrix[11] = 0.0;
    matrix[12] = 210.0;
    matrix[13] = 220.0;
    matrix[14] = 0.0;
    matrix[15] = 1.0;
    rc.transform(matrix);
    rc.drawRect(const Rect.fromLTRB(10, 20, 30, 40), testPaint);
    rc.endRecording();
    expect(
      rc.pictureBounds,
      within(distance: 0.001, from: const Rect.fromLTRB(168.0, 283.6, 224.0, 368.4)),
    );
    await checkScreenshot(rc, 'complex_transform');
  });

  test('drawPaint should cover full size', () async {
    final RecordingCanvas rc = RecordingCanvas(screenRect);
    rc.drawPaint(testPaint);
    rc.drawRect(const Rect.fromLTRB(10, 20, 30, 40), testPaint);
    rc.endRecording();
    expect(rc.pictureBounds, screenRect);
    await checkScreenshot(rc, 'draw_paint');
  });

  test('drawColor should cover full size', () async {
    final RecordingCanvas rc = RecordingCanvas(screenRect);
    final SurfacePaint testPaint = SurfacePaint()..color = const Color(0xFF80FF00);
    rc.drawRect(const Rect.fromLTRB(10, 20, 30, 40), testPaint);
    rc.drawColor(const Color(0xFFFF0000), BlendMode.multiply);
    rc.drawRect(const Rect.fromLTRB(10, 60, 30, 80), testPaint);
    rc.endRecording();
    expect(rc.pictureBounds, screenRect);
    await checkScreenshot(rc, 'draw_color');
  });

  test('Computes paint bounds for draw oval', () async {
    final RecordingCanvas rc = RecordingCanvas(screenRect);
    rc.drawOval(const Rect.fromLTRB(10, 20, 30, 40), testPaint);
    rc.endRecording();
    expect(rc.pictureBounds, const Rect.fromLTRB(10, 20, 30, 40));
    await checkScreenshot(rc, 'draw_oval');
  });

  test('Computes paint bounds for draw round rect', () async {
    final RecordingCanvas rc = RecordingCanvas(screenRect);
    rc.drawRRect(
      RRect.fromRectAndRadius(const Rect.fromLTRB(10, 20, 30, 40), const Radius.circular(5.0)),
      testPaint,
    );
    rc.endRecording();
    expect(rc.pictureBounds, const Rect.fromLTRB(10, 20, 30, 40));
    await checkScreenshot(rc, 'draw_round_rect');
  });

  test('Computes empty paint bounds when inner rect outside of outer rect for '
      'drawDRRect', () async {
    final RecordingCanvas rc = RecordingCanvas(screenRect);
    rc.drawDRRect(
      RRect.fromRectAndCorners(const Rect.fromLTRB(10, 20, 30, 40)),
      RRect.fromRectAndCorners(const Rect.fromLTRB(1, 2, 3, 4)),
      testPaint,
    );
    rc.endRecording();
    expect(rc.pictureBounds, Rect.zero);
    await checkScreenshot(rc, 'draw_drrect_empty');
  });

  test('Computes paint bounds using outer rect for drawDRRect', () async {
    final RecordingCanvas rc = RecordingCanvas(screenRect);
    rc.drawDRRect(
      RRect.fromRectAndCorners(const Rect.fromLTRB(10, 20, 30, 40)),
      RRect.fromRectAndCorners(const Rect.fromLTRB(12, 22, 28, 38)),
      testPaint,
    );
    rc.endRecording();
    expect(rc.pictureBounds, const Rect.fromLTRB(10, 20, 30, 40));
    await checkScreenshot(rc, 'draw_drrect');
  });

  test('Computes paint bounds for draw circle', () async {
    // Paint bounds of one circle.
    RecordingCanvas rc = RecordingCanvas(screenRect);
    rc.drawCircle(const Offset(20, 20), 10.0, testPaint);
    rc.endRecording();
    expect(rc.pictureBounds, const Rect.fromLTRB(10.0, 10.0, 30.0, 30.0));

    // Paint bounds of a union of two circles.
    rc = RecordingCanvas(screenRect);
    rc.drawCircle(const Offset(20, 20), 10.0, testPaint);
    rc.drawCircle(const Offset(200, 300), 100.0, testPaint);
    rc.endRecording();
    expect(rc.pictureBounds, const Rect.fromLTRB(10.0, 10.0, 300.0, 400.0));
    await checkScreenshot(rc, 'draw_circle');
  });

  test('Computes paint bounds for draw image', () {
    final RecordingCanvas rc = RecordingCanvas(screenRect);
    rc.drawImage(TestImage(), const Offset(50, 100), SurfacePaint());
    rc.endRecording();
    expect(rc.pictureBounds, const Rect.fromLTRB(50.0, 100.0, 70.0, 110.0));
  });

  test('Computes paint bounds for draw image rect', () {
    final RecordingCanvas rc = RecordingCanvas(screenRect);
    rc.drawImageRect(
      TestImage(),
      const Rect.fromLTRB(1, 1, 20, 10),
      const Rect.fromLTRB(5, 6, 400, 500),
      SurfacePaint(),
    );
    rc.endRecording();
    expect(rc.pictureBounds, const Rect.fromLTRB(5.0, 6.0, 400.0, 500.0));
  });

  test(
    'Computes paint bounds for single-line draw paragraph',
    () async {
      final RecordingCanvas rc = RecordingCanvas(screenRect);
      final Paragraph paragraph = createTestParagraph();
      const double textLeft = 5.0;
      const double textTop = 7.0;
      const double widthConstraint = 300.0;
      paragraph.layout(const ParagraphConstraints(width: widthConstraint));
      rc.drawParagraph(paragraph, const Offset(textLeft, textTop));
      rc.endRecording();
      expect(
        rc.pictureBounds!.width,
        lessThan(widthConstraint),
        reason:
            'The given width constraint $widthConstraint is more than the '
            'test string needs, so the width of the visible text is actually '
            'smaller than the given width.',
      );
      expect(
        rc.pictureBounds,
        Rect.fromLTRB(textLeft, textTop, textLeft + paragraph.maxIntrinsicWidth, 21.0),
      );
      await checkScreenshot(rc, 'draw_paragraph');
    }, // TODO(mdebbar): https://github.com/flutter/flutter/issues/65789
    skip:
        ui_web.browser.browserEngine == ui_web.BrowserEngine.webkit &&
        ui_web.browser.operatingSystem == ui_web.OperatingSystem.iOs,
  );

  test(
    'Computes paint bounds for multi-line draw paragraph',
    () async {
      final RecordingCanvas rc = RecordingCanvas(screenRect);
      final Paragraph paragraph = createTestParagraph();
      const double textLeft = 5.0;
      const double textTop = 7.0;
      // Do not go lower than the shortest word.
      const double widthConstraint = 130.0;
      paragraph.layout(const ParagraphConstraints(width: widthConstraint));
      rc.drawParagraph(paragraph, const Offset(textLeft, textTop));
      rc.endRecording();

      const double fontWidth = 14;
      const int lettersInLongestWord = 9;
      const double longestLineWidth = lettersInLongestWord * fontWidth;
      expect(
        rc.pictureBounds!.width,
        lessThan(widthConstraint),
        reason:
            'The test string "A short sentence." is broken up into two lines, '
            '"A short" and "sentence.". The longest line contains '
            '$lettersInLongestWord characters, each  ${fontWidth}px wide. '
            'That line is ${longestLineWidth}px wide, which is less than '
            '$widthConstraint.',
      );
      expect(
        rc.pictureBounds,
        const Rect.fromLTRB(textLeft, textTop, textLeft + longestLineWidth, 35.0),
      );
      await checkScreenshot(rc, 'draw_paragraph_multi_line');
    }, // TODO(mdebbar): https://github.com/flutter/flutter/issues/65789
    skip:
        ui_web.browser.browserEngine == ui_web.BrowserEngine.webkit &&
        ui_web.browser.operatingSystem == ui_web.OperatingSystem.iOs,
  );

  test('Should exclude painting outside simple clipRect', () async {
    // One clipped line.
    RecordingCanvas rc = RecordingCanvas(screenRect);
    rc.clipRect(const Rect.fromLTRB(50, 50, 100, 100), ClipOp.intersect);
    rc.drawLine(const Offset(10, 11), const Offset(20, 21), testPaint);
    rc.endRecording();
    expect(rc.pictureBounds, Rect.zero);

    // Two clipped lines.
    rc = RecordingCanvas(screenRect);
    rc.clipRect(const Rect.fromLTRB(50, 50, 100, 100), ClipOp.intersect);
    rc.drawLine(const Offset(10, 11), const Offset(20, 21), testPaint);
    rc.drawLine(const Offset(52, 53), const Offset(55, 56), testPaint);
    rc.endRecording();

    // Extra pixel due to default line length
    expect(rc.pictureBounds, const Rect.fromLTRB(51, 52, 56, 57));
    await checkScreenshot(rc, 'clip_rect_simple');
  });

  test('Should include intersection of clipRect and painting', () async {
    RecordingCanvas rc = RecordingCanvas(screenRect);
    rc.clipRect(const Rect.fromLTRB(50, 50, 100, 100), ClipOp.intersect);
    rc.drawRect(const Rect.fromLTRB(20, 60, 120, 70), testPaint);
    rc.endRecording();
    expect(rc.pictureBounds, const Rect.fromLTRB(50, 60, 100, 70));
    await checkScreenshot(rc, 'clip_rect_intersects_paint_left_to_right');

    rc = RecordingCanvas(screenRect);
    rc.clipRect(const Rect.fromLTRB(50, 50, 100, 100), ClipOp.intersect);
    rc.drawRect(const Rect.fromLTRB(60, 20, 70, 200), testPaint);
    rc.endRecording();
    expect(rc.pictureBounds, const Rect.fromLTRB(60, 50, 70, 100));
    await checkScreenshot(rc, 'clip_rect_intersects_paint_top_to_bottom');
  });

  test('Should intersect rects for multiple clipRect calls', () async {
    final RecordingCanvas rc = RecordingCanvas(screenRect);
    rc.clipRect(const Rect.fromLTRB(50, 50, 100, 100), ClipOp.intersect);
    rc.scale(2.0, 2.0);
    rc.clipRect(const Rect.fromLTRB(30, 30, 45, 45), ClipOp.intersect);
    rc.drawRect(const Rect.fromLTRB(10, 30, 60, 35), testPaint);
    rc.endRecording();

    expect(rc.pictureBounds, const Rect.fromLTRB(60, 60, 90, 70));
    await checkScreenshot(rc, 'clip_rects_intersect');
  });

  // drawShadow
  test('Computes paint bounds for drawShadow', () async {
    final RecordingCanvas rc = RecordingCanvas(screenRect);
    final Path path = Path();
    path.addRect(const Rect.fromLTRB(20, 30, 100, 110));
    rc.drawShadow(path, const Color(0xFFFF0000), 2.0, true);
    rc.endRecording();

    expect(
      rc.pictureBounds,
      within(distance: 0.05, from: const Rect.fromLTRB(0.0, 8.5, 123.5, 134.1)),
    );
    await checkScreenshot(rc, 'path_with_shadow');
  });

  test('Clip with negative scale reports correct paint bounds', () async {
    // The following draws a filled rectangle that occupies the bottom half of
    // the canvas. Notice that both the clip and the rectangle are drawn
    // forward. What makes them appear at the bottom is the translation and a
    // vertical flip via a negative scale. This replicates the Material
    // overscroll glow effect at the bottom of a list, where it is drawn upside
    // down.
    final RecordingCanvas rc = RecordingCanvas(const Rect.fromLTRB(0, 0, 100, 100));
    rc
      ..translate(0, 100)
      ..scale(1, -1)
      ..clipRect(const Rect.fromLTRB(0, 0, 100, 50), ClipOp.intersect)
      ..drawRect(const Rect.fromLTRB(0, 0, 100, 100), SurfacePaint());
    rc.endRecording();

    expect(rc.pictureBounds, const Rect.fromLTRB(0.0, 50.0, 100.0, 100.0));
    await checkScreenshot(rc, 'scale_negative');
  });

  test('Clip with a rotation reports correct paint bounds', () async {
    final RecordingCanvas rc = RecordingCanvas(const Rect.fromLTRB(0, 0, 100, 100));
    rc
      ..translate(50, 50)
      ..rotate(math.pi / 4.0)
      ..clipRect(const Rect.fromLTWH(-20, -20, 40, 40), ClipOp.intersect)
      ..drawRect(const Rect.fromLTWH(-80, -80, 160, 160), SurfacePaint());
    rc.endRecording();

    expect(
      rc.pictureBounds,
      within(
        distance: 0.001,
        from: Rect.fromCircle(center: const Offset(50, 50), radius: 20 * math.sqrt(2)),
      ),
    );
    await checkScreenshot(rc, 'clip_rect_rotated');
  });

  test('Rotated line reports correct paint bounds', () async {
    final RecordingCanvas rc = RecordingCanvas(const Rect.fromLTRB(0, 0, 100, 100));
    rc
      ..translate(50, 50)
      ..rotate(math.pi / 4.0)
      ..drawLine(Offset.zero, const Offset(20, 20), SurfacePaint());
    rc.endRecording();

    expect(
      rc.pictureBounds,
      within(distance: 0.1, from: const Rect.fromLTRB(34.4, 48.6, 65.6, 79.7)),
    );
    await checkScreenshot(rc, 'line_rotated');
  });

  // Regression test for https://github.com/flutter/flutter/issues/46339.
  test('Should draw a Rect for straight line when strokeWidth is zero.', () async {
    final RecordingCanvas rc = RecordingCanvas(screenRect);

    final Path path = Path();
    final SurfacePaint paint =
        SurfacePaint()
          ..style = PaintingStyle.stroke
          ..strokeWidth = 0.0
          ..color = const Color(0xFFFF0000);
    path.moveTo(10, 10);
    path.lineTo(90, 10);
    rc.drawPath(path, paint);
    rc.endRecording();
    // Should draw a Rect
    final List<PaintCommand> commands = rc.debugPaintCommands;
    expect(commands.length, 1);
    expect(commands.first, isA<PaintDrawRect>());
    // Should inflate picture bounds
    expect(rc.pictureBounds, within(distance: 0.1, from: const Rect.fromLTRB(10, 10, 90, 11)));
    await checkScreenshot(rc, 'path_straight_line_with_zero_stroke_width');
  });

  test('Should support reusing path and reset when drawing into canvas.', () async {
    final RecordingCanvas rc = RecordingCanvas(const Rect.fromLTRB(0, 0, 100, 100));

    final Path path = Path();
    path.moveTo(3, 0);
    path.lineTo(100, 97);
    rc.drawPath(
      path,
      SurfacePaint()
        ..style = PaintingStyle.stroke
        ..strokeWidth = 2.0
        ..color = const Color(0xFFFF0000),
    );
    path.reset();
    path.moveTo(0, 3);
    path.lineTo(97, 100);
    rc.drawPath(
      path,
      SurfacePaint()
        ..style = PaintingStyle.stroke
        ..strokeWidth = 2.0
        ..color = const Color(0xFF00FF00),
    );
    rc.endRecording();
    await checkScreenshot(rc, 'reuse_path');
  });

  test('Should draw RRect after line when beginning new path.', () async {
    final RecordingCanvas rc = RecordingCanvas(const Rect.fromLTRB(0, 0, 200, 400));
    rc.save();
    rc.translate(50.0, 100.0);
    final Path path = Path();
    // Draw a vertical small line (caret).
    path.moveTo(8, 4);
    path.lineTo(8, 24);
    // Draw round rect below caret.
    path.addRRect(RRect.fromLTRBR(0.5, 100.5, 80.7, 150.7, const Radius.circular(10)));
    rc.drawPath(
      path,
      SurfacePaint()
        ..style = PaintingStyle.stroke
        ..strokeWidth = 2.0
        ..color = const Color(0xFF404000),
    );
    rc.restore();
    rc.endRecording();
    await checkScreenshot(rc, 'path_with_line_and_roundrect');
  });

  // Regression test for https://github.com/flutter/flutter/issues/64371.
  test('Should draw line following a polygon without closing path.', () async {
    final RecordingCanvas rc = RecordingCanvas(const Rect.fromLTRB(0, 0, 200, 400));
    rc.save();
    rc.translate(50.0, 100.0);
    final Path path = Path();
    // Draw a vertical small line (caret).
    path.addPolygon(const <Offset>[Offset(0, 10), Offset(20, 5), Offset(50, 10)], false);
    path.lineTo(60, 80);
    path.lineTo(0, 80);
    path.close();
    rc.drawPath(
      path,
      SurfacePaint()
        ..style = PaintingStyle.stroke
        ..strokeWidth = 2.0
        ..color = const Color(0xFF404000),
    );
    rc.restore();
    rc.endRecording();
    await checkScreenshot(rc, 'path_with_addpolygon');
  });

  test('should include paint spread in bounds estimates', () async {
    final SurfaceSceneBuilder sb = SurfaceSceneBuilder();

    final List<PaintSpreadPainter> painters = <PaintSpreadPainter>[
      (RecordingCanvas canvas, SurfacePaint paint) {
        canvas.drawLine(Offset.zero, const Offset(20.0, 20.0), paint);
      },
      (RecordingCanvas canvas, SurfacePaint paint) {
        canvas.drawRect(const Rect.fromLTRB(0.0, 0.0, 20.0, 20.0), paint);
      },
      (RecordingCanvas canvas, SurfacePaint paint) {
        canvas.drawRRect(RRect.fromLTRBR(0.0, 0.0, 20.0, 20.0, const Radius.circular(7.0)), paint);
      },
      (RecordingCanvas canvas, SurfacePaint paint) {
        canvas.drawDRRect(
          RRect.fromLTRBR(0.0, 0.0, 20.0, 20.0, const Radius.circular(5.0)),
          RRect.fromLTRBR(4.0, 4.0, 16.0, 16.0, const Radius.circular(5.0)),
          paint,
        );
      },
      (RecordingCanvas canvas, SurfacePaint paint) {
        canvas.drawOval(const Rect.fromLTRB(0.0, 5.0, 20.0, 15.0), paint);
      },
      (RecordingCanvas canvas, SurfacePaint paint) {
        canvas.drawCircle(const Offset(10.0, 10.0), 10.0, paint);
      },
      (RecordingCanvas canvas, SurfacePaint paint) {
        final SurfacePath path =
            SurfacePath()
              ..moveTo(10, 0)
              ..lineTo(20, 10)
              ..lineTo(10, 20)
              ..lineTo(0, 10)
              ..close();
        canvas.drawPath(path, paint);
      },

      // Images are not affected by mask filter or stroke width. They use image
      // filter instead.
      (RecordingCanvas canvas, SurfacePaint paint) {
        canvas.drawImage(_createRealTestImage(), Offset.zero, paint);
      },
      (RecordingCanvas canvas, SurfacePaint paint) {
        canvas.drawImageRect(
          _createRealTestImage(),
          const Rect.fromLTRB(0, 0, 20, 20),
          const Rect.fromLTRB(5, 5, 15, 15),
          paint,
        );
      },
    ];

    Picture drawBounds(Rect bounds) {
      final EnginePictureRecorder recorder = EnginePictureRecorder();
      final RecordingCanvas canvas = recorder.beginRecording(Rect.largest);
      canvas.drawRect(
        bounds,
        SurfacePaint()
          ..style = PaintingStyle.stroke
          ..strokeWidth = 1.0
          ..color = const Color.fromARGB(255, 0, 255, 0),
      );
      return recorder.endRecording();
    }

    for (int i = 0; i < painters.length; i++) {
      sb.pushOffset(0.0, 20.0 + 60.0 * i);
      final PaintSpreadPainter painter = painters[i];

      // Paint with zero paint spread.
      {
        sb.pushOffset(20.0, 0.0);
        final EnginePictureRecorder recorder = EnginePictureRecorder();
        final RecordingCanvas canvas = recorder.beginRecording(Rect.largest);
        final SurfacePaint zeroSpreadPaint = SurfacePaint();
        painter(canvas, zeroSpreadPaint);
        sb.addPicture(Offset.zero, recorder.endRecording());
        sb.addPicture(Offset.zero, drawBounds(canvas.pictureBounds!));
        sb.pop();
      }

      // Paint with a thick stroke paint.
      {
        sb.pushOffset(80.0, 0.0);
        final EnginePictureRecorder recorder = EnginePictureRecorder();
        final RecordingCanvas canvas = recorder.beginRecording(Rect.largest);
        final SurfacePaint thickStrokePaint =
            SurfacePaint()
              ..style = PaintingStyle.stroke
              ..strokeWidth = 5.0;
        painter(canvas, thickStrokePaint);
        sb.addPicture(Offset.zero, recorder.endRecording());
        sb.addPicture(Offset.zero, drawBounds(canvas.pictureBounds!));
        sb.pop();
      }

      // Paint with a mask filter blur.
      {
        sb.pushOffset(140.0, 0.0);
        final EnginePictureRecorder recorder = EnginePictureRecorder();
        final RecordingCanvas canvas = recorder.beginRecording(Rect.largest);
        final SurfacePaint maskFilterBlurPaint =
            SurfacePaint()..maskFilter = const MaskFilter.blur(BlurStyle.normal, 5.0);
        painter(canvas, maskFilterBlurPaint);
        sb.addPicture(Offset.zero, recorder.endRecording());
        sb.addPicture(Offset.zero, drawBounds(canvas.pictureBounds!));
        sb.pop();
      }

      // Paint with a thick stroke paint and a mask filter blur.
      {
        sb.pushOffset(200.0, 0.0);
        final EnginePictureRecorder recorder = EnginePictureRecorder();
        final RecordingCanvas canvas = recorder.beginRecording(Rect.largest);
        final SurfacePaint thickStrokeAndBlurPaint =
            SurfacePaint()
              ..style = PaintingStyle.stroke
              ..strokeWidth = 5.0
              ..maskFilter = const MaskFilter.blur(BlurStyle.normal, 5.0);
        painter(canvas, thickStrokeAndBlurPaint);
        sb.addPicture(Offset.zero, recorder.endRecording());
        sb.addPicture(Offset.zero, drawBounds(canvas.pictureBounds!));
        sb.pop();
      }

      sb.pop();
    }

    final DomElement sceneElement = sb.build().webOnlyRootElement!;
    domDocument.body!.append(sceneElement);
    try {
      await matchGoldenFile('paint_spread_bounds.png', region: const Rect.fromLTRB(0, 0, 250, 600));
    } finally {
      sceneElement.remove();
    }
  });
}

typedef PaintSpreadPainter = void Function(RecordingCanvas canvas, SurfacePaint paint);

const String _base64Encoded20x20TestImage =
    'iVBORw0KGgoAAAANSUhEUgAAABQAAAAUC'
    'AIAAAAC64paAAAACXBIWXMAAC4jAAAuIwF4pT92AAAA'
    'B3RJTUUH5AMFFBksg4i3gQAAABl0RVh0Q29tbWVudABDcmVhdGVkIHdpdGggR0lNUFeBDhcAAAAj'
    'SURBVDjLY2TAC/7jlWVioACMah4ZmhnxpyHG0QAb1UyZZgBjWAIm/clP0AAAAABJRU5ErkJggg==';

HtmlImage _createRealTestImage() {
  return HtmlImage(
    createDomHTMLImageElement()..src = 'data:text/plain;base64,$_base64Encoded20x20TestImage',
    20,
    20,
  );
}

class TestImage implements Image {
  @override
  int get width => 20;

  @override
  int get height => 10;

  @override
  Future<ByteData> toByteData({ImageByteFormat format = ImageByteFormat.rawRgba}) async {
    throw UnsupportedError('Cannot encode test image');
  }

  @override
  String toString() => '[$width\u00D7$height]';

  @override
  void dispose() {}

  @override
  bool get debugDisposed => false;

  @override
  Image clone() => this;

  @override
  bool isCloneOf(Image other) => other == this;

  @override
  List<StackTrace> /*?*/ debugGetOpenHandleStackTraces() => <StackTrace>[];

  @override
  ColorSpace get colorSpace => ColorSpace.sRGB;
}

Paragraph createTestParagraph() {
  final ParagraphBuilder builder = ParagraphBuilder(
    ParagraphStyle(
      fontFamily: 'Ahem',
      fontStyle: FontStyle.normal,
      fontWeight: FontWeight.normal,
      fontSize: 14.0,
    ),
  );
  builder.addText('A short sentence.');
  return builder.build();
}
