// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart';

import '../common/test_initialization.dart';
import 'paragraph/helper.dart';
import 'screenshot.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  setUpUnitTests(
    withImplicitView: true,
    emulateTesterEnvironment: false,
    setUpTestViewDimensions: false,
  );

  /// Regression test for https://github.com/flutter/flutter/issues/64734.
  test('Clips using difference', () async {
    const Offset shift = Offset(8, 8);
    const Rect region = Rect.fromLTRB(0, 0, 400, 300);
    final RecordingCanvas canvas = RecordingCanvas(region);
    final Rect titleRect = const Rect.fromLTWH(20, 0, 50, 20).shift(shift);
    final SurfacePaint paint =
        SurfacePaint()
          ..style = PaintingStyle.stroke
          ..color = const Color(0xff000000)
          ..strokeWidth = 1;
    canvas.save();
    try {
      final Rect borderRect = Rect.fromLTRB(0, 10, region.width, region.height).shift(shift);
      canvas.clipRect(titleRect, ClipOp.difference);
      canvas.drawRect(borderRect, paint);
    } finally {
      canvas.restore();
    }
    canvas.drawRect(titleRect, paint);
    await canvasScreenshot(
      canvas,
      'clip_op_difference',
      region: const Rect.fromLTRB(0, 0, 420, 360),
    );
  });

  /// Regression test for https://github.com/flutter/flutter/issues/86345
  test('Clips with zero width or height', () async {
    const Rect region = Rect.fromLTRB(0, 0, 400, 300);
    final RecordingCanvas canvas = RecordingCanvas(region);

    final SurfacePaint paint =
        SurfacePaint()
          ..style = PaintingStyle.fill
          ..color = const Color(0xff00ff00);
    final SurfacePaint borderPaint =
        SurfacePaint()
          ..style = PaintingStyle.stroke
          ..color = const Color(0xffff0000)
          ..strokeWidth = 1;

    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        final double x = 10 + i * 70;
        final double y = 10 + j * 70;
        canvas.save();
        // Clip.
        canvas.clipRect(Rect.fromLTWH(x, y, i * 25, j * 25), ClipOp.intersect);
        // Draw the blue (clipped) rect.
        canvas.drawRect(Rect.fromLTWH(x, y, 50, 50), paint);
        final Paragraph p = plain(
          EngineParagraphStyle(fontFamily: 'Roboto', fontSize: 34),
          '23',
          textStyle: EngineTextStyle.only(color: const Color(0xff0000ff)),
        );
        p.layout(const ParagraphConstraints(width: double.infinity));
        canvas.drawParagraph(p, Offset(x, y));
        canvas.restore();
        // Draw the red border.
        canvas.drawRect(Rect.fromLTWH(x, y, 50, 50), borderPaint);
      }
    }
    await canvasScreenshot(canvas, 'clip_zero_width_height', region: region);
  });
}
