// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;

import 'package:test/bootstrap/browser.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart';

import '../../common/test_initialization.dart';
import 'text_goldens.dart';

typedef PaintTest = void Function(RecordingCanvas recordingCanvas);

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  final EngineGoldenTester goldenTester = await EngineGoldenTester.initialize(
    viewportSize: const Size(600, 600),
  );

  setUpUnitTests(
    withImplicitView: true,
    emulateTesterEnvironment: false,
    setUpTestViewDimensions: false,
  );

  void paintTest(EngineCanvas canvas, PaintTest painter) {
    const Rect screenRect = Rect.fromLTWH(0, 0, 600, 600);
    final RecordingCanvas recordingCanvas = RecordingCanvas(screenRect);
    painter(recordingCanvas);
    recordingCanvas.endRecording();
    recordingCanvas.apply(canvas, screenRect);
  }

  testEachCanvas('clips multiline text against rectangle', (EngineCanvas canvas) {
    // [DomCanvas] doesn't support clip commands.
    if (canvas is! DomCanvas) {
      paintTest(canvas, paintTextWithClipRect);
      return goldenTester.diffCanvasScreenshot(canvas, 'multiline_text_clipping_rect');
    }
    return null;
  });

  testEachCanvas('clips multiline text against rectangle with transform', (EngineCanvas canvas) {
    // [DomCanvas] doesn't support clip commands.
    if (canvas is! DomCanvas) {
      paintTest(canvas, paintTextWithClipRectTranslated);
      return goldenTester.diffCanvasScreenshot(canvas, 'multiline_text_clipping_rect_translate');
    }
    return null;
  });

  testEachCanvas('clips multiline text against round rectangle', (EngineCanvas canvas) {
    // [DomCanvas] doesn't support clip commands.
    if (canvas is! DomCanvas) {
      paintTest(canvas, paintTextWithClipRoundRect);
      return goldenTester.diffCanvasScreenshot(canvas, 'multiline_text_clipping_roundrect');
    }
    return null;
  });

  testEachCanvas('clips multiline text against path', (EngineCanvas canvas) {
    // [DomCanvas] doesn't support clip commands.
    if (canvas is! DomCanvas) {
      paintTest(canvas, paintTextWithClipPath);
      return goldenTester.diffCanvasScreenshot(canvas, 'multiline_text_clipping_path');
    }
    return null;
  });

  testEachCanvas('clips multiline text against stack of rects', (EngineCanvas canvas) {
    // [DomCanvas] doesn't support clip commands.
    if (canvas is! DomCanvas) {
      paintTest(canvas, paintTextWithClipStack);
      return goldenTester.diffCanvasScreenshot(canvas, 'multiline_text_clipping_stack1');
    }
    return null;
  });
}

const Rect testBounds = Rect.fromLTRB(50, 50, 230, 220);

void drawBackground(RecordingCanvas canvas) {
  canvas.drawRect(
    testBounds,
    SurfacePaint()
      ..style = PaintingStyle.fill
      ..color = const Color(0xFF9E9E9E),
  );
  canvas.drawRect(
    testBounds.inflate(-40),
    SurfacePaint()
      ..strokeWidth = 1
      ..style = PaintingStyle.stroke
      ..color = const Color(0xFF009688),
  );
}

void drawQuickBrownFox(RecordingCanvas canvas) {
  canvas.drawParagraph(
    paragraph(
      'The quick brown fox jumps over the lazy dog',
      textStyle: TextStyle(
        color: const Color(0xFF000000),
        decoration: TextDecoration.none,
        fontFamily: 'Roboto',
        fontSize: 30,
        background: Paint()..color = const Color.fromRGBO(50, 255, 50, 1.0),
      ),
      maxWidth: 180,
    ),
    Offset(testBounds.left, testBounds.top),
  );
}

void paintTextWithClipRect(RecordingCanvas canvas) {
  drawBackground(canvas);
  canvas.clipRect(testBounds.inflate(-40), ClipOp.intersect);
  drawQuickBrownFox(canvas);
}

void paintTextWithClipRectTranslated(RecordingCanvas canvas) {
  drawBackground(canvas);
  canvas.clipRect(testBounds.inflate(-40), ClipOp.intersect);
  canvas.translate(30, 10);
  drawQuickBrownFox(canvas);
}

const Color deepOrange = Color(0xFFFF5722);

void paintTextWithClipRoundRect(RecordingCanvas canvas) {
  final RRect roundRect = RRect.fromRectAndCorners(
    testBounds.inflate(-40),
    topRight: const Radius.elliptical(45, 40),
    bottomLeft: const Radius.elliptical(50, 40),
    bottomRight: const Radius.circular(30),
  );
  drawBackground(canvas);
  canvas.drawRRect(
    roundRect,
    SurfacePaint()
      ..color = deepOrange
      ..style = PaintingStyle.fill,
  );
  canvas.clipRRect(roundRect);
  drawQuickBrownFox(canvas);
}

void paintTextWithClipPath(RecordingCanvas canvas) {
  drawBackground(canvas);
  final Path path = Path();
  const double delta = 40.0;
  final Rect clipBounds = testBounds.inflate(-delta);
  final double midX = (clipBounds.left + clipBounds.right) / 2.0;
  final double midY = (clipBounds.top + clipBounds.bottom) / 2.0;
  path.moveTo(clipBounds.left - delta, midY);
  path.quadraticBezierTo(midX, midY, midX, clipBounds.top - delta);
  path.quadraticBezierTo(midX, midY, clipBounds.right + delta, midY);
  path.quadraticBezierTo(midX, midY, midX, clipBounds.bottom + delta);
  path.quadraticBezierTo(midX, midY, clipBounds.left - delta, midY);
  path.close();
  canvas.drawPath(
    path,
    SurfacePaint()
      ..color = deepOrange
      ..style = PaintingStyle.fill,
  );
  canvas.clipPath(path);
  drawQuickBrownFox(canvas);
}

void paintTextWithClipStack(RecordingCanvas canvas) {
  drawBackground(canvas);
  final Rect inflatedRect = testBounds.inflate(-40);
  canvas.clipRect(inflatedRect, ClipOp.intersect);
  canvas.rotate(math.pi / 8.0);
  canvas.translate(40, -40);
  canvas.clipRect(inflatedRect, ClipOp.intersect);
  canvas.drawRect(
    inflatedRect,
    SurfacePaint()
      ..color = deepOrange
      ..style = PaintingStyle.fill,
  );
  drawQuickBrownFox(canvas);
}
