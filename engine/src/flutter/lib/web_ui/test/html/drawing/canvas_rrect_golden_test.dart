// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart';

import '../screenshot.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  late RecordingCanvas rc;
  const Rect canvasRect = Rect.fromLTWH(0, 0, 500, 100);

  const Rect region = Rect.fromLTWH(8, 8, 500, 100); // Compensate for old golden tester padding

  final SurfacePaint niceRRectPaint =
      SurfacePaint()
        ..color = const Color.fromRGBO(250, 186, 218, 1.0) // #fabada
        ..style = PaintingStyle.fill;

  // Some values to see how the algo behaves as radius get absurdly large
  const List<double> rRectRadii = <double>[0, 10, 20, 80, 8000];

  const Radius someFixedRadius = Radius.circular(10);

  setUp(() {
    rc = RecordingCanvas(const Rect.fromLTWH(0, 0, 500, 100));
    rc.translate(10, 10); // Center
  });

  test('round square with big (equal) radius ends up as a circle', () async {
    for (int i = 0; i < 5; i++) {
      rc.drawRRect(
        RRect.fromRectAndRadius(
          Rect.fromLTWH(100 * i.toDouble(), 0, 80, 80),
          Radius.circular(rRectRadii[i]),
        ),
        niceRRectPaint,
      );
    }
    await canvasScreenshot(rc, 'canvas_rrect_round_square', canvasRect: canvasRect, region: region);
  });

  /// Regression test for https://github.com/flutter/flutter/issues/62631
  test('round square with flipped left/right coordinates', () async {
    rc.translate(35, 320);
    rc.drawRRect(
      RRect.fromRectAndRadius(const Rect.fromLTRB(-30, -100, 30, -300), const Radius.circular(30)),
      niceRRectPaint,
    );
    rc.drawPath(
      Path()
        ..moveTo(0, 0)
        ..lineTo(20, 0),
      niceRRectPaint,
    );
    await canvasScreenshot(
      rc,
      'canvas_rrect_flipped',
      canvasRect: canvasRect,
      region: const Rect.fromLTWH(0, 0, 100, 200),
    );
  });

  test('round rect with big radius scale down smaller radius', () async {
    for (int i = 0; i < 5; i++) {
      final Radius growingRadius = Radius.circular(rRectRadii[i]);
      final RRect rrect = RRect.fromRectAndCorners(
        Rect.fromLTWH(100 * i.toDouble(), 0, 80, 80),
        bottomRight: someFixedRadius,
        topRight: growingRadius,
        bottomLeft: growingRadius,
      );

      rc.drawRRect(rrect, niceRRectPaint);
    }
    await canvasScreenshot(
      rc,
      'canvas_rrect_overlapping_radius',
      canvasRect: canvasRect,
      region: region,
    );
  });

  test('diff round rect with big radius scale down smaller radius', () async {
    for (int i = 0; i < 5; i++) {
      final Radius growingRadius = Radius.circular(rRectRadii[i]);
      final RRect outerRRect = RRect.fromRectAndCorners(
        Rect.fromLTWH(100 * i.toDouble(), 0, 80, 80),
        bottomRight: someFixedRadius,
        topRight: growingRadius,
        bottomLeft: growingRadius,
      );

      // Inner is half of outer, but offset a little so it looks nicer
      final RRect innerRRect = RRect.fromRectAndCorners(
        Rect.fromLTWH(100 * i.toDouble() + 5, 5, 40, 40),
        bottomRight: someFixedRadius / 2,
        topRight: growingRadius / 2,
        bottomLeft: growingRadius / 2,
      );

      rc.drawDRRect(outerRRect, innerRRect, niceRRectPaint);
    }

    await canvasScreenshot(
      rc,
      'canvas_drrect_overlapping_radius',
      canvasRect: canvasRect,
      region: region,
    );
  });
}
