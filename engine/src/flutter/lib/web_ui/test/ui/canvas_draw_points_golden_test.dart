// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;

import 'package:web_engine_tester/golden_tester.dart';

import '../common/test_initialization.dart';
import 'utils.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  setUpUnitTests(withImplicitView: true, setUpTestViewDimensions: false);

  const ui.Rect region = ui.Rect.fromLTWH(0, 0, 400, 600);

  late ui.PictureRecorder recorder;
  late ui.Canvas canvas;

  setUp(() {
    recorder = ui.PictureRecorder();
    canvas = ui.Canvas(recorder, region);
  });

  tearDown(() {});

  test('draws points in all 3 modes', () async {
    final ui.Paint paint = ui.Paint();
    paint.strokeWidth = 2.0;
    paint.color = const ui.Color(0xFF00FF00);
    const List<ui.Offset> points = <ui.Offset>[
      ui.Offset(10, 10),
      ui.Offset(50, 10),
      ui.Offset(70, 70),
      ui.Offset(170, 70),
    ];
    canvas.drawPoints(ui.PointMode.points, points, paint);
    const List<ui.Offset> points2 = <ui.Offset>[
      ui.Offset(10, 110),
      ui.Offset(50, 110),
      ui.Offset(70, 170),
      ui.Offset(170, 170),
    ];
    canvas.drawPoints(ui.PointMode.lines, points2, paint);
    const List<ui.Offset> points3 = <ui.Offset>[
      ui.Offset(10, 210),
      ui.Offset(50, 210),
      ui.Offset(70, 270),
      ui.Offset(170, 270),
    ];
    canvas.drawPoints(ui.PointMode.polygon, points3, paint);

    await drawPictureUsingCurrentRenderer(recorder.endRecording());
    await matchGoldenFile('ui_canvas_draw_points.png', region: region);
  });

  test('draws raw points in all 3 modes', () async {
    final ui.Paint paint = ui.Paint();
    paint.strokeWidth = 2.0;
    paint.color = const ui.Color(0xFF0000FF);
    final Float32List points = offsetListToFloat32List(const <ui.Offset>[
      ui.Offset(10, 10),
      ui.Offset(50, 10),
      ui.Offset(70, 70),
      ui.Offset(170, 70),
    ]);
    canvas.drawRawPoints(ui.PointMode.points, points, paint);
    final Float32List points2 = offsetListToFloat32List(const <ui.Offset>[
      ui.Offset(10, 110),
      ui.Offset(50, 110),
      ui.Offset(70, 170),
      ui.Offset(170, 170),
    ]);
    canvas.drawRawPoints(ui.PointMode.lines, points2, paint);
    final Float32List points3 = offsetListToFloat32List(const <ui.Offset>[
      ui.Offset(10, 210),
      ui.Offset(50, 210),
      ui.Offset(70, 270),
      ui.Offset(170, 270),
    ]);
    canvas.drawRawPoints(ui.PointMode.polygon, points3, paint);

    await drawPictureUsingCurrentRenderer(recorder.endRecording());
    await matchGoldenFile('ui_canvas_draw_raw_points.png', region: region);
  });

  test('Should draw points with strokeWidth', () async {
    final ui.Paint nullStrokePaint = ui.Paint()..color = const ui.Color(0xffff0000);
    canvas.drawRawPoints(
      ui.PointMode.lines,
      Float32List.fromList(<double>[30.0, 20.0, 200.0, 20.0]),
      nullStrokePaint,
    );
    final ui.Paint strokePaint1 =
        ui.Paint()
          ..strokeWidth = 1.0
          ..color = const ui.Color(0xff0000ff);
    canvas.drawRawPoints(
      ui.PointMode.lines,
      Float32List.fromList(<double>[30.0, 30.0, 200.0, 30.0]),
      strokePaint1,
    );
    final ui.Paint strokePaint3 =
        ui.Paint()
          ..strokeWidth = 3.0
          ..color = const ui.Color(0xff00a000);
    canvas.drawRawPoints(
      ui.PointMode.lines,
      Float32List.fromList(<double>[30.0, 40.0, 200.0, 40.0]),
      strokePaint3,
    );
    canvas.drawRawPoints(
      ui.PointMode.points,
      Float32List.fromList(<double>[30.0, 50.0, 40.0, 50.0, 50.0, 50.0]),
      strokePaint3,
    );
    await drawPictureUsingCurrentRenderer(recorder.endRecording());
    await matchGoldenFile('ui_canvas_draw_points_stroke.png', region: region);
  });
}
