// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;

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

  test('draw arc', () async {
    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder, region);
    canvas.drawArc(
      const Rect.fromLTRB(100, 100, 200, 200),
      math.pi / 3.0,
      4.0 * math.pi / 3.0,
      false,
      Paint()
        ..style = PaintingStyle.stroke
        ..strokeWidth = 3.0
        ..color = const Color(0xFFFF00FF),
    );

    await drawPictureUsingCurrentRenderer(recorder.endRecording());

    await matchGoldenFile('ui_canvas_draw_arc.png', region: region);
  });

  test('draw circle', () async {
    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder, region);
    canvas.drawCircle(
      const Offset(150, 150),
      50,
      Paint()
        ..style = PaintingStyle.stroke
        ..strokeWidth = 3.0
        ..color = const Color(0xFFFF0000),
    );

    await drawPictureUsingCurrentRenderer(recorder.endRecording());

    await matchGoldenFile('ui_canvas_draw_circle.png', region: region);
  });

  test('draw oval', () async {
    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder, region);
    canvas.drawOval(
      const Rect.fromLTRB(100, 125, 200, 175),
      Paint()
        ..style = PaintingStyle.stroke
        ..strokeWidth = 3.0
        ..color = const Color(0xFF00FFFF),
    );

    await drawPictureUsingCurrentRenderer(recorder.endRecording());

    await matchGoldenFile('ui_canvas_draw_oval.png', region: region);
  });
}
