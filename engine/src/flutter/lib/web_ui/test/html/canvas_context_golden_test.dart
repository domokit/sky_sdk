// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart' as engine;
import 'package:ui/ui.dart' hide TextStyle;

import '../common/test_initialization.dart';
import 'screenshot.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

/// Tests context save/restore.
Future<void> testMain() async {
  const double screenWidth = 600.0;
  const double screenHeight = 800.0;
  const Rect screenRect = Rect.fromLTWH(0, 0, screenWidth, screenHeight);

  setUpUnitTests(setUpTestViewDimensions: false);

  // Regression test for https://github.com/flutter/flutter/issues/49429
  // Should clip with correct transform.
  test('Clips image with oval clip path', () async {
    final engine.RecordingCanvas rc = engine.RecordingCanvas(const Rect.fromLTRB(0, 0, 400, 300));
    final engine.SurfacePaint paint =
        Paint() as engine.SurfacePaint
          ..color = const Color(0xFF00FF00)
          ..style = PaintingStyle.fill;
    rc.save();
    final Path ovalPath = Path();
    ovalPath.addOval(const Rect.fromLTWH(100, 30, 200, 100));
    rc.clipPath(ovalPath);
    rc.translate(-500, -500);
    rc.save();
    rc.translate(500, 500);
    rc.drawPath(ovalPath, paint);
    // The line below was causing SaveClipStack to incorrectly set
    // transform before path painting.
    rc.translate(-1000, -1000);
    rc.save();
    rc.restore();
    rc.restore();
    rc.restore();
    // The rectangle should paint without clipping since we restored
    // context.
    rc.drawRect(const Rect.fromLTWH(0, 0, 4, 200), paint);
    await canvasScreenshot(rc, 'context_save_restore_transform', canvasRect: screenRect);
  });

  test('Should restore clip path', () async {
    final engine.RecordingCanvas rc = engine.RecordingCanvas(const Rect.fromLTRB(0, 0, 400, 300));
    final Paint goodPaint =
        Paint()
          ..color = const Color(0x8000FF00)
          ..style = PaintingStyle.fill;
    final Paint badPaint =
        Paint()
          ..color = const Color(0xFFFF0000)
          ..style = PaintingStyle.fill;
    rc.save();
    final Path ovalPath = Path();
    ovalPath.addOval(const Rect.fromLTWH(100, 30, 200, 100));
    rc.clipPath(ovalPath);
    rc.translate(-500, -500);
    rc.save();
    rc.restore();
    // The rectangle should be clipped against oval.
    rc.drawRect(const Rect.fromLTWH(0, 0, 300, 300), badPaint as engine.SurfacePaint);
    rc.restore();
    // The rectangle should paint without clipping since we restored
    // context.
    rc.drawRect(const Rect.fromLTWH(0, 0, 200, 200), goodPaint as engine.SurfacePaint);
    await canvasScreenshot(rc, 'context_save_restore_clip', canvasRect: screenRect);
  });
}
